/**
 * @file    turtle_sim.c
 * @brief   Multi-day sea-turtle deployment simulation for the UW_DOPPLER tag.
 *          Models dive/surface behaviour, SWS surface detection, the TX
 *          scheduler (real floors + serialization), MC / wear-counter, and LB
 *          low-battery mode, then emits a self-contained HTML report with inline
 *          SVG charts (no external assets — opens in any browser).
 *
 * This is a BEHAVIOUR ILLUSTRATION, not a bit-exact firmware replay: it uses the
 * real constants and decision rules (kns_app_uw_doppler.c) so the report shows
 * how the shipped logic behaves over a >7-day deployment. The battery model is
 * deliberately accelerated so the LB transition is visible within the window.
 *
 * Build: gcc -Wall -Wextra -O2 -std=c99 turtle_sim.c -o turtle_sim
 * Run:   ./turtle_sim [output.html]   (default: turtle_report.html)
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

/* ---- Firmware constants (mirrored from kns_app_uw_doppler.c) ---- */
#define TX_MAX_COUNT            3
#define TX_INITIAL_INTERVAL_S   10
#define TX_GROWTH_PCT           50
#define TX_MAX_INTERVAL_S       180
#define MIN_INTER_TX_S          5      /* MAC-ready -> first TX floor */
#define TX_HARD_MIN_S           2      /* absolute floor between two TX */
#define TX_DURATION_S           1      /* a real Argos frame is < 1 s on the wire */
#define WL_SIZE                 1024   /* wear-leveling area (MC/WKU) */
#define ONAIR_MC_MOD            512    /* on-air MC is 9-bit */
#define LB_ENTER_MV             2900
#define LB_EXIT_MV              3100
#define LB_TX_MAX_COUNT         3
#define LB_TX_INTERVAL_S        60

/* ---- Simulation parameters ---- */
#define SIM_DAYS                30
#define SIM_SECONDS             ((uint32_t)SIM_DAYS * 86400u)
#define SAMPLE_DAY_INDEX        3      /* which day to show as the dive/surface strip */

/* ---- Deterministic PRNG (LCG) — no Math.random, reproducible reports ---- */
static uint32_t rng_state = 0x1234abcdu;
static uint32_t rnd(void) { rng_state = rng_state * 1664525u + 1013904223u; return rng_state; }
static uint32_t rnd_range(uint32_t lo, uint32_t hi) { return lo + (rnd() % (hi - lo + 1u)); }

/* ---- Battery model (accelerated, illustrative) ----
 * Start healthy, sag roughly linearly with a per-TX dip, so LB engages part-way
 * through the window. Real Li-SOCl2 lasts years — this is compressed to show the
 * behaviour, clearly labelled as such in the report. */
static int battery_mV(uint32_t t_s, uint32_t total_tx)
{
	double frac = (double)t_s / (double)SIM_SECONDS;
	int base = (int)(3600.0 - 850.0 * frac);          /* 3600 -> 2750 over the run */
	int tx_dip = (int)(total_tx / 40);                /* cumulative TX wear on Vloaded */
	int v = base - tx_dip;
	if (v < 2600) v = 2600;
	return v;
}

/* ---- Aggregates ---- */
static int   day_tx[SIM_DAYS];
static int   day_seq[SIM_DAYS];
static int   day_surfacings[SIM_DAYS];
static int   day_short_windows[SIM_DAYS];   /* surfaced but dived before full seq */
static int   batt_samples[SIM_DAYS + 1];    /* daily battery snapshot (mV) */
static int   lb_day[SIM_DAYS];              /* 1 if LB active at any point that day */

/* ---- Event log ---- */
typedef struct { uint32_t t_s; const char *kind; char detail[64]; } Event;
static Event events[256];
static int   n_events = 0;
static void log_event(uint32_t t_s, const char *kind, const char *detail)
{
	if (n_events >= (int)(sizeof(events)/sizeof(events[0]))) return;
	events[n_events].t_s = t_s;
	events[n_events].kind = kind;
	snprintf(events[n_events].detail, sizeof(events[n_events].detail), "%s", detail);
	n_events++;
}

/* ---- Sample-day dive/surface strip ---- */
typedef struct { uint32_t start_s; uint32_t end_s; int surface; int tx; } Segment;
static Segment strip[512];
static int     n_strip = 0;

/* ---- On-air MC trace (sampled every N sequences) ---- */
static int mc_trace[2048];
static int n_mc_trace = 0;

int main(int argc, char **argv)
{
	const char *out_path = (argc > 1) ? argv[1] : "turtle_report.html";

	uint32_t t = 0;                 /* wall-clock seconds */
	uint64_t mc_hw = 0;             /* wear-leveled MC high-water (per sequence) */
	uint32_t total_tx = 0, total_seq = 0, total_surf = 0, total_short = 0;
	uint32_t wear_overflows = 0;
	int lb_active = 0, lb_enter_count = 0, lb_exit_count = 0;
	uint32_t sample_day_lo = (uint32_t)SAMPLE_DAY_INDEX * 86400u;
	uint32_t sample_day_hi = sample_day_lo + 86400u;

	batt_samples[0] = battery_mV(0, 0);

	while (t < SIM_SECONDS) {
		/* ---- DIVE ---- active turtle: mostly 8-40 min foraging dives, with an
		 * occasional long rest dive up to 6 h. */
		uint32_t dive_s;
		if ((rnd() % 100u) < 5u) dive_s = rnd_range(2u*3600u, 6u*3600u);  /* long rest dive */
		else                     dive_s = rnd_range(8u*60u, 40u*60u);
		uint32_t dive_start = t;
		t += dive_s;
		if (t >= SIM_SECONDS) t = SIM_SECONDS;
		if (dive_start < sample_day_hi && t > sample_day_lo && n_strip < 512) {
			strip[n_strip].start_s = dive_start; strip[n_strip].end_s = t;
			strip[n_strip].surface = 0; strip[n_strip].tx = 0; n_strip++;
		}
		if (t >= SIM_SECONDS) break;

		/* ---- SURFACE ---- (30 s - 3 min breathing window) */
		uint32_t surf_s = rnd_range(30u, 180u);
		uint32_t surf_start = t;
		uint32_t surf_end = t + surf_s;
		int day = (int)(surf_start / 86400u);
		if (day >= SIM_DAYS) break;
		total_surf++; day_surfacings[day]++;

		/* Battery reading at surface -> LB hysteresis (the ONLY battery reaction;
		 * there is no hard TX veto anymore). */
		int vbat = battery_mV(surf_start, total_tx);
		if (!lb_active && vbat > 0 && vbat < LB_ENTER_MV) {
			lb_active = 1; lb_enter_count++;
			log_event(surf_start, "LB-ENTER", "battery sag -> reduced cadence");
		} else if (lb_active && vbat > LB_EXIT_MV) {
			lb_active = 0; lb_exit_count++;
			log_event(surf_start, "LB-EXIT", "battery recovered");
		}
		if (lb_active) lb_day[day] = 1;

		/* ---- TX SEQUENCE (serialized, floored, capped) ---- */
		int cap = lb_active ? LB_TX_MAX_COUNT : TX_MAX_COUNT;
		int interval0 = lb_active ? LB_TX_INTERVAL_S : TX_INITIAL_INTERVAL_S;
		total_seq++; day_seq[day]++;
		mc_hw++;                                  /* one MC per SEQUENCE (high-water) */
		if ((mc_hw % WL_SIZE) == 0) {             /* wear area wrapped -> overflow */
			wear_overflows++;
			char d[64]; snprintf(d, sizeof(d), "MC hw=%llu (on-air %llu)",
				(unsigned long long)mc_hw, (unsigned long long)(mc_hw % ONAIR_MC_MOD));
			log_event(surf_start, "WEAR-OVF", d);
		}
		if (n_mc_trace < (int)(sizeof(mc_trace)/sizeof(mc_trace[0])))
			mc_trace[n_mc_trace++] = (int)(mc_hw % ONAIR_MC_MOD);

		uint32_t tx_t = surf_start;
		int tx_done_in_seq = 0;
		int interval = interval0;
		for (int i = 0; i < cap; i++) {
			/* first TX: MIN_INTER floor from surface; later: interval, hard-min */
			uint32_t gap = (i == 0) ? (uint32_t)MIN_INTER_TX_S : (uint32_t)interval;
			if (gap < (uint32_t)TX_HARD_MIN_S) gap = TX_HARD_MIN_S;
			uint32_t next_tx = (i == 0) ? surf_start + MIN_INTER_TX_S : tx_t + gap;
			/* Serialization: the previous TX must have completed (TX_DURATION). */
			if (next_tx + TX_DURATION_S > surf_end) break;   /* dived before this TX */
			tx_t = next_tx;
			total_tx++; day_tx[day]++; tx_done_in_seq++;
			if (surf_start >= sample_day_lo && surf_start < sample_day_hi && n_strip < 512) {
				strip[n_strip].start_s = tx_t; strip[n_strip].end_s = tx_t + TX_DURATION_S;
				strip[n_strip].surface = 1; strip[n_strip].tx = 1; n_strip++;
			}
			/* interval growth for the next TX in this sequence */
			interval = interval + (interval * TX_GROWTH_PCT) / 100;
			if (interval > TX_MAX_INTERVAL_S) interval = TX_MAX_INTERVAL_S;
		}
		if (tx_done_in_seq < cap) { total_short++; day_short_windows[day]++; }

		/* record the surface window in the sample-day strip */
		if (surf_start >= sample_day_lo && surf_start < sample_day_hi && n_strip < 512) {
			strip[n_strip].start_s = surf_start; strip[n_strip].end_s = surf_end;
			strip[n_strip].surface = 1; strip[n_strip].tx = 0; n_strip++;
		}

		t = surf_end;
		int cur_day = (int)(surf_start / 86400u);
		if (cur_day + 1 <= SIM_DAYS) batt_samples[cur_day + 1] = battery_mV(t, total_tx);
	}

	/* fill any missing daily battery snapshots */
	for (int d = 1; d <= SIM_DAYS; d++)
		if (batt_samples[d] == 0) batt_samples[d] = battery_mV((uint32_t)d*86400u, total_tx);

	/* first-TX event marker */
	log_event(0, "DEPLOY", "tag sealed, first dive");

	/* =================== HTML REPORT =================== */
	FILE *f = fopen(out_path, "w");
	if (!f) { fprintf(stderr, "cannot open %s\n", out_path); return 1; }

	fprintf(f,
"<!doctype html><html lang=\"en\"><head><meta charset=\"utf-8\">"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
"<title>UW_DOPPLER Turtle Deployment Simulation</title><style>"
":root{--bg:#0b1020;--card:#151c30;--ink:#e8edf7;--mut:#93a1c0;--acc:#4fd1c5;"
"--tx:#f6ad55;--uw:#2b3a67;--surf:#4fd1c5;--lb:#f56565;--grid:#26314f}"
"@media(prefers-color-scheme:light){:root{--bg:#f4f6fb;--card:#fff;--ink:#12203a;"
"--mut:#5a6b8c;--grid:#e2e8f4;--uw:#c3d0f0}}"
"*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);"
"font:15px/1.5 system-ui,Segoe UI,Roboto,sans-serif}"
".wrap{max-width:1000px;margin:0 auto;padding:28px}"
"h1{font-size:24px;margin:0 0 4px}h2{font-size:17px;margin:30px 0 10px;color:var(--acc)}"
".sub{color:var(--mut);margin:0 0 20px}"
".kpis{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:12px}"
".kpi{background:var(--card);border-radius:12px;padding:14px 16px}"
".kpi .v{font-size:26px;font-weight:700}.kpi .l{color:var(--mut);font-size:12px;text-transform:uppercase;letter-spacing:.04em}"
".card{background:var(--card);border-radius:12px;padding:16px;margin-top:12px;overflow-x:auto}"
"svg{display:block;max-width:100%%;height:auto}"
"table{border-collapse:collapse;width:100%%;font-size:13px}"
"th,td{text-align:left;padding:6px 10px;border-bottom:1px solid var(--grid)}"
"th{color:var(--mut);font-weight:600}"
".tag{display:inline-block;padding:1px 8px;border-radius:20px;font-size:12px;font-weight:600}"
".note{color:var(--mut);font-size:12px;margin-top:8px}"
".legend{display:flex;gap:16px;flex-wrap:wrap;color:var(--mut);font-size:12px;margin-top:8px}"
".sw{display:inline-block;width:12px;height:12px;border-radius:3px;vertical-align:-1px;margin-right:5px}"
"</style></head><body><div class=\"wrap\">");

	fprintf(f, "<h1>&#128034; UW_DOPPLER &mdash; Turtle Deployment Simulation</h1>");
	fprintf(f, "<p class=\"sub\">%d-day sealed deployment &middot; dive/surface behaviour, SWS detection, "
	           "serialized TX scheduling, MC + wear-counter, and LB low-battery mode &middot; "
	           "seed 0x1234abcd (reproducible)</p>", SIM_DAYS);

	/* ---- KPI cards ---- */
	fprintf(f, "<div class=\"kpis\">");
	fprintf(f, "<div class=\"kpi\"><div class=\"v\">%u</div><div class=\"l\">Surfacings</div></div>", total_surf);
	fprintf(f, "<div class=\"kpi\"><div class=\"v\">%u</div><div class=\"l\">TX sequences</div></div>", total_seq);
	fprintf(f, "<div class=\"kpi\"><div class=\"v\">%u</div><div class=\"l\">Argos frames</div></div>", total_tx);
	fprintf(f, "<div class=\"kpi\"><div class=\"v\">%llu</div><div class=\"l\">MC high-water</div></div>", (unsigned long long)mc_hw);
	fprintf(f, "<div class=\"kpi\"><div class=\"v\">%u</div><div class=\"l\">Wear overflows</div></div>", wear_overflows);
	fprintf(f, "<div class=\"kpi\"><div class=\"v\">%d</div><div class=\"l\">LB enter/exit</div></div>", lb_enter_count);
	fprintf(f, "<div class=\"kpi\"><div class=\"v\">%d mV</div><div class=\"l\">Final battery</div></div>", batt_samples[SIM_DAYS]);
	fprintf(f, "<div class=\"kpi\"><div class=\"v\">%u</div><div class=\"l\">Short windows</div></div>", total_short);
	fprintf(f, "</div>");

	/* ---- Battery chart (line) with LB band + thresholds ---- */
	{
		int W = 920, H = 220, ml = 46, mr = 14, mt = 12, mb = 24;
		int pw = W - ml - mr, ph = H - mt - mb;
		int vmin = 2600, vmax = 3650;
		fprintf(f, "<h2>Battery voltage &amp; LB mode</h2><div class=\"card\">");
		fprintf(f, "<svg viewBox=\"0 0 %d %d\" role=\"img\">", W, H);
		/* LB enter/exit threshold lines */
		for (int k = 0; k < 2; k++) {
			int mv = k ? LB_EXIT_MV : LB_ENTER_MV;
			int y = mt + ph - (int)((double)(mv - vmin) / (vmax - vmin) * ph);
			fprintf(f, "<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke=\"%s\" "
			        "stroke-dasharray=\"4 4\" stroke-width=\"1\" opacity=\".6\"/>",
			        ml, y, W - mr, y, k ? "#68d391" : "var(--lb)");
			fprintf(f, "<text x=\"%d\" y=\"%d\" font-size=\"10\" fill=\"var(--mut)\">%d</text>",
			        4, y + 3, mv);
		}
		/* LB-active shaded bands (per day) */
		for (int d = 0; d < SIM_DAYS; d++) if (lb_day[d]) {
			int x = ml + (int)((double)d / SIM_DAYS * pw);
			int w = (int)(1.0 / SIM_DAYS * pw) + 1;
			fprintf(f, "<rect x=\"%d\" y=\"%d\" width=\"%d\" height=\"%d\" fill=\"var(--lb)\" opacity=\".10\"/>",
			        x, mt, w, ph);
		}
		/* battery polyline */
		fprintf(f, "<polyline fill=\"none\" stroke=\"var(--acc)\" stroke-width=\"2\" points=\"");
		for (int d = 0; d <= SIM_DAYS; d++) {
			int x = ml + (int)((double)d / SIM_DAYS * pw);
			int y = mt + ph - (int)((double)(batt_samples[d] - vmin) / (vmax - vmin) * ph);
			fprintf(f, "%d,%d ", x, y);
		}
		fprintf(f, "\"/>");
		/* axes labels */
		fprintf(f, "<text x=\"%d\" y=\"%d\" font-size=\"10\" fill=\"var(--mut)\">day 0</text>", ml, H - 6);
		fprintf(f, "<text x=\"%d\" y=\"%d\" font-size=\"10\" fill=\"var(--mut)\" text-anchor=\"end\">day %d</text>", W - mr, H - 6, SIM_DAYS);
		fprintf(f, "</svg>");
		fprintf(f, "<div class=\"legend\"><span><span class=\"sw\" style=\"background:var(--acc)\"></span>VBAT</span>"
		           "<span><span class=\"sw\" style=\"background:var(--lb);opacity:.4\"></span>LB active</span>"
		           "<span>dashed = LB enter/exit thresholds</span></div>");
		fprintf(f, "<div class=\"note\">Battery model is accelerated for illustration (a real 19 Ah Li-SOCl&#8322; "
		           "pack lasts years). Note: LB only reduces cadence &mdash; TX is never battery-vetoed.</div></div>");
	}

	/* ---- TX per day (bars) ---- */
	{
		int W = 920, H = 200, ml = 46, mr = 14, mt = 12, mb = 24;
		int pw = W - ml - mr, ph = H - mt - mb;
		int maxtx = 1; for (int d = 0; d < SIM_DAYS; d++) if (day_tx[d] > maxtx) maxtx = day_tx[d];
		fprintf(f, "<h2>Argos frames per day</h2><div class=\"card\">");
		fprintf(f, "<svg viewBox=\"0 0 %d %d\" role=\"img\">", W, H);
		int bw = pw / SIM_DAYS;
		for (int d = 0; d < SIM_DAYS; d++) {
			int h = (int)((double)day_tx[d] / maxtx * ph);
			int x = ml + d * bw;
			int y = mt + ph - h;
			const char *col = lb_day[d] ? "var(--lb)" : "var(--tx)";
			fprintf(f, "<rect x=\"%d\" y=\"%d\" width=\"%d\" height=\"%d\" rx=\"2\" fill=\"%s\"/>",
			        x, y, bw - 2 > 1 ? bw - 2 : 1, h, col);
		}
		fprintf(f, "<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke=\"var(--grid)\"/>", ml, mt + ph, W - mr, mt + ph);
		fprintf(f, "<text x=\"%d\" y=\"%d\" font-size=\"10\" fill=\"var(--mut)\">max %d/day</text>", ml, mt + 10, maxtx);
		fprintf(f, "</svg>");
		fprintf(f, "<div class=\"legend\"><span><span class=\"sw\" style=\"background:var(--tx)\"></span>normal</span>"
		           "<span><span class=\"sw\" style=\"background:var(--lb)\"></span>LB day (reduced cadence)</span></div></div>");
	}

	/* ---- On-air MC (mod 512 sawtooth) ---- */
	{
		int W = 920, H = 170, ml = 46, mr = 14, mt = 12, mb = 20;
		int pw = W - ml - mr, ph = H - mt - mb;
		fprintf(f, "<h2>On-air message counter (MC mod 512)</h2><div class=\"card\">");
		fprintf(f, "<svg viewBox=\"0 0 %d %d\" role=\"img\">", W, H);
		fprintf(f, "<polyline fill=\"none\" stroke=\"var(--acc)\" stroke-width=\"1.3\" points=\"");
		for (int i = 0; i < n_mc_trace; i++) {
			int x = ml + (int)((double)i / (n_mc_trace > 1 ? n_mc_trace - 1 : 1) * pw);
			int y = mt + ph - (int)((double)mc_trace[i] / (ONAIR_MC_MOD - 1) * ph);
			fprintf(f, "%d,%d ", x, y);
		}
		fprintf(f, "\"/>");
		fprintf(f, "<text x=\"%d\" y=\"%d\" font-size=\"10\" fill=\"var(--mut)\">0</text>", 30, mt + ph);
		fprintf(f, "<text x=\"%d\" y=\"%d\" font-size=\"10\" fill=\"var(--mut)\">511</text>", 24, mt + 8);
		fprintf(f, "</svg>");
		fprintf(f, "<div class=\"note\">The high-water MC (%llu) is wear-leveled in flash; the on-air field is 9-bit "
		           "so it saw-tooths every 512 sequences. A wear-overflow (+1024) is absorbed mod-512, so the on-air "
		           "counter is crash-safe (see test_wear_overflow_torn.c).</div></div>", (unsigned long long)mc_hw);
	}

	/* ---- Sample-day dive/surface/TX strip ---- */
	{
		int W = 920, H = 70, ml = 8, mr = 8, mt = 10, mb = 18;
		int pw = W - ml - mr, ph = H - mt - mb;
		fprintf(f, "<h2>Day %d &mdash; dive / surface / TX timeline</h2><div class=\"card\">", SAMPLE_DAY_INDEX);
		fprintf(f, "<svg viewBox=\"0 0 %d %d\" role=\"img\">", W, H);
		fprintf(f, "<rect x=\"%d\" y=\"%d\" width=\"%d\" height=\"%d\" fill=\"var(--uw)\" rx=\"3\"/>", ml, mt, pw, ph);
		for (int i = 0; i < n_strip; i++) {
			double a = (double)(strip[i].start_s - sample_day_lo) / 86400.0;
			double b = (double)(strip[i].end_s   - sample_day_lo) / 86400.0;
			if (b < 0 || a > 1) continue;
			if (a < 0) a = 0;
			if (b > 1) b = 1;
			int x = ml + (int)(a * pw); int w = (int)((b - a) * pw); if (w < 1) w = 1;
			if (strip[i].tx)
				fprintf(f, "<rect x=\"%d\" y=\"%d\" width=\"%d\" height=\"%d\" fill=\"var(--tx)\"/>", x, mt - 3, w < 2 ? 2 : w, ph + 6);
			else if (strip[i].surface)
				fprintf(f, "<rect x=\"%d\" y=\"%d\" width=\"%d\" height=\"%d\" fill=\"var(--surf)\"/>", x, mt, w, ph);
		}
		fprintf(f, "<text x=\"%d\" y=\"%d\" font-size=\"10\" fill=\"var(--mut)\">00:00</text>", ml, H - 5);
		fprintf(f, "<text x=\"%d\" y=\"%d\" font-size=\"10\" fill=\"var(--mut)\" text-anchor=\"end\">24:00</text>", W - mr, H - 5);
		fprintf(f, "</svg>");
		fprintf(f, "<div class=\"legend\"><span><span class=\"sw\" style=\"background:var(--uw)\"></span>underwater</span>"
		           "<span><span class=\"sw\" style=\"background:var(--surf)\"></span>surface</span>"
		           "<span><span class=\"sw\" style=\"background:var(--tx)\"></span>Argos TX</span></div></div>");
	}

	/* ---- Event log ---- */
	fprintf(f, "<h2>Notable events</h2><div class=\"card\"><table>"
	           "<tr><th>Day</th><th>Time</th><th>Event</th><th>Detail</th></tr>");
	for (int i = 0; i < n_events; i++) {
		uint32_t d = events[i].t_s / 86400u;
		uint32_t rem = events[i].t_s % 86400u;
		const char *col = "var(--acc)";
		if (!strcmp(events[i].kind, "LB-ENTER") || !strcmp(events[i].kind, "WEAR-OVF")) col = "var(--tx)";
		if (!strcmp(events[i].kind, "LB-EXIT")) col = "#68d391";
		fprintf(f, "<tr><td>%u</td><td>%02u:%02u:%02u</td>"
		           "<td><span class=\"tag\" style=\"background:%s;color:#0b1020\">%s</span></td><td>%s</td></tr>",
		        d, rem/3600u, (rem%3600u)/60u, rem%60u, col, events[i].kind, events[i].detail);
	}
	fprintf(f, "</table></div>");

	fprintf(f, "<p class=\"note\">Generated by Tests/sim/turtle_sim.c &mdash; uses the shipped UW_DOPPLER "
	           "constants and decision rules. Illustrative behaviour model, not a bit-exact firmware replay.</p>");
	fprintf(f, "</div></body></html>");
	fclose(f);

	/* console summary */
	printf("Turtle sim: %d days, %u surfacings, %u sequences, %u frames, MC hw=%llu, "
	       "%u wear-overflows, LB enter=%d exit=%d, final batt=%dmV\n",
	       SIM_DAYS, total_surf, total_seq, total_tx, (unsigned long long)mc_hw,
	       wear_overflows, lb_enter_count, lb_exit_count, batt_samples[SIM_DAYS]);
	printf("Report written to %s\n", out_path);
	return 0;
}
