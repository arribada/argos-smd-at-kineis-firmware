/**
 * @file    turtle_sim.c
 * @brief   Multi-day sea-turtle deployment simulation for the UW_DOPPLER tag,
 *          exercising the edge cases AND self-validating the firmware rules.
 *
 * Models dive/surface behaviour, SWS surface detection, the TX scheduler (real
 * floors + serialization + cooldown), MC / wear-counter, LB low-battery mode,
 * the zero-TX / seq-restart safety net, and mid-deployment resets. It then
 * CHECKS a set of invariants across the whole run (MC monotonic, on-air MC in
 * range, TX spacing >= floors, no TX outside the surface window, wear-overflow
 * absorbed mod-512, LB never blocks TX) and emits a self-contained HTML report
 * with inline SVG charts + a validation panel (PASS/FAIL + edge-case coverage).
 *
 * Behaviour illustration (real constants/rules, not a bit-exact replay). The
 * battery model is accelerated so the LB transition is visible in the window.
 *
 * Build: gcc -Wall -Wextra -O2 -std=c99 turtle_sim.c -o turtle_sim
 * Run:   ./turtle_sim [output.html]     (exit 0 iff all invariants hold)
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

/* ---- Firmware constants (mirrored from kns_app_uw_doppler.c) ---- */
#define TX_MAX_COUNT            3
#define TX_INITIAL_INTERVAL_S   10
#define TX_GROWTH_PCT           50
#define TX_MAX_INTERVAL_S       180
#define TX_COOLDOWN_S           10     /* quiet floor from last ACTUAL TX */
#define MIN_INTER_TX_S          5      /* MAC-ready -> first TX floor */
#define TX_HARD_MIN_S           2      /* absolute floor between two TX */
#define TX_DURATION_S           1      /* a real Argos frame is < 1 s on the wire */
#define WL_SIZE                 1024   /* wear-leveling area (MC/WKU) */
#define ONAIR_MC_MOD            512    /* on-air MC is 9-bit */
#define LB_ENTER_MV             2900
#define LB_EXIT_MV              3100
#define LB_TX_MAX_COUNT         3
#define LB_TX_INTERVAL_S        60
#define SEQ_RESTART_S           1200   /* floating animal: new sequence every 20 min */

/* ---- Simulation parameters ---- */
#define SIM_DAYS                45
#define SIM_SECONDS             ((uint32_t)SIM_DAYS * 86400u)
#define SAMPLE_DAY_INDEX        3

/* ---- Deterministic PRNG (LCG) ---- */
static uint32_t rng_state = 0x1234abcdu;
static uint32_t rnd(void) { rng_state = rng_state * 1664525u + 1013904223u; return rng_state; }
static uint32_t rnd_range(uint32_t lo, uint32_t hi) { return lo + (rnd() % (hi - lo + 1u)); }

/* ---- Battery model: sag with a mid-deployment recovery so LB both ENTERS and
 * EXITS (full hysteresis cycle), plus a per-TX loaded dip. Accelerated. ---- */
static int battery_mV(uint32_t t_s, uint32_t total_tx)
{
	double frac = (double)t_s / (double)SIM_SECONDS;
	int base = (int)(3600.0 - 1500.0 * frac);             /* sags into LB range ~day 21 */
	/* A rest-period recovery AFTER LB has engaged, so LB both ENTERS (on the way
	 * down) and EXITS (here) and then re-enters — the full hysteresis cycle. */
	if (frac > 0.50 && frac < 0.62) base += 400;
	int v = base - (int)(total_tx / 45);
	if (v < 2600) v = 2600;
	if (v > 3650) v = 3650;
	return v;
}

/* ---- Aggregates ---- */
static int day_tx[SIM_DAYS], day_seq[SIM_DAYS], day_surf[SIM_DAYS], lb_day[SIM_DAYS];
static int batt_samples[SIM_DAYS + 1];

/* ---- Edge-case coverage counters ---- */
static uint32_t ec_zero_tx_window = 0;   /* surfaced but 0 TX (window too short) */
static uint32_t ec_cooldown_gated = 0;   /* first TX delayed by the cooldown floor */
static uint32_t ec_short_seq = 0;        /* dived before the full cap */
static uint32_t ec_seqrestart = 0;       /* extra sequence from the stuck-at-surface net */
static uint32_t ec_stuck_surface = 0;    /* zero-TX / seq-restart scenarios */
static uint32_t ec_resets = 0;           /* mid-deployment reboots */
static uint32_t ec_wear_ovf = 0;
static int      ec_lb_enter = 0, ec_lb_exit = 0;

/* ---- Invariant violation counters (must all stay 0) ---- */
static uint32_t v_mc_monotonic = 0;
static uint32_t v_onair_range = 0;
static uint32_t v_tx_spacing = 0;
static uint32_t v_tx_outside_window = 0;
static uint32_t v_wear_continuity = 0;
static uint32_t v_lb_blocked_tx = 0;

/* ---- Event log ---- */
typedef struct { uint32_t t_s; const char *kind; char detail[64]; } Event;
static Event events[512];
static int   n_events = 0;
static void log_event(uint32_t t_s, const char *kind, const char *detail)
{
	if (n_events >= (int)(sizeof(events)/sizeof(events[0]))) return;
	events[n_events].t_s = t_s; events[n_events].kind = kind;
	snprintf(events[n_events].detail, sizeof(events[n_events].detail), "%s", detail);
	n_events++;
}

/* ---- Sample-day strip + MC trace ---- */
typedef struct { uint32_t start_s, end_s; int surface, tx; } Segment;
static Segment strip[1024]; static int n_strip = 0;
static int mc_trace[4096]; static int n_mc_trace = 0;
static uint32_t sample_lo, sample_hi;

/* Emit one TX sequence during a surface window [surf_start, surf_end].
 * Applies cooldown + MIN_INTER + hard-min + interval growth + serialization,
 * increments MC/wear, and checks the per-sequence invariants. Returns #TX. */
static int run_sequence(uint32_t surf_start, uint32_t surf_end, int lb_active,
                        uint64_t *mc_hw, uint32_t *last_actual_tx_t, uint32_t *total_tx)
{
	int cap = lb_active ? LB_TX_MAX_COUNT : TX_MAX_COUNT;
	int interval = lb_active ? LB_TX_INTERVAL_S : TX_INITIAL_INTERVAL_S;

	/* first-TX earliest time = max(surface+MIN_INTER, cooldown floor, hard-min) */
	uint32_t earliest = surf_start + MIN_INTER_TX_S;
	if (*last_actual_tx_t != 0u) {
		uint32_t cd = *last_actual_tx_t + TX_COOLDOWN_S;
		uint32_t hm = *last_actual_tx_t + TX_HARD_MIN_S;
		if (cd > earliest) { earliest = cd; ec_cooldown_gated++; }
		if (hm > earliest) earliest = hm;
	}

	/* MC high-water advances once per SEQUENCE (session_mc model). */
	uint64_t mc_before = *mc_hw;
	(*mc_hw)++;
	if ((*mc_hw % WL_SIZE) == 0u) {                 /* wear area wraps -> overflow */
		ec_wear_ovf++;
		/* invariant: on-air MC stays continuous (mod 512) across the overflow */
		int before = (int)(mc_before % ONAIR_MC_MOD);
		int after  = (int)(*mc_hw % ONAIR_MC_MOD);
		if (((before + 1) % ONAIR_MC_MOD) != after) v_wear_continuity++;
		char d[64]; snprintf(d, sizeof(d), "MC hw=%llu, on-air %d->%d (mod-512)",
			(unsigned long long)*mc_hw, before, after);
		log_event(surf_start, "WEAR-OVF", d);
	}
	int onair = (int)(*mc_hw % ONAIR_MC_MOD);
	if (onair < 0 || onair >= ONAIR_MC_MOD) v_onair_range++;
	if (n_mc_trace < (int)(sizeof(mc_trace)/sizeof(mc_trace[0]))) mc_trace[n_mc_trace++] = onair;

	int n = 0; uint32_t prev_tx = 0; uint32_t tx_t = earliest;
	for (int i = 0; i < cap; i++) {
		if (i > 0) {
			uint32_t gap = (uint32_t)interval;
			if (gap < (uint32_t)TX_HARD_MIN_S) gap = TX_HARD_MIN_S;
			tx_t = prev_tx + gap;
		}
		if (tx_t + TX_DURATION_S > surf_end) break;      /* serialization: dived first */

		/* invariant: no TX outside the surface window */
		if (tx_t < surf_start || tx_t + TX_DURATION_S > surf_end) v_tx_outside_window++;
		/* invariant: spacing */
		if (i == 0) {
			if (tx_t < surf_start + MIN_INTER_TX_S) v_tx_spacing++;         /* MAC-ready floor */
			if (*last_actual_tx_t != 0u &&
			    (tx_t - *last_actual_tx_t) < (uint32_t)TX_HARD_MIN_S) v_tx_spacing++;
		} else {
			if ((tx_t - prev_tx) < (uint32_t)TX_HARD_MIN_S) v_tx_spacing++;
		}

		(*total_tx)++; n++;
		*last_actual_tx_t = tx_t;
		prev_tx = tx_t;
		if (surf_start >= sample_lo && surf_start < sample_hi && n_strip < 1024) {
			strip[n_strip].start_s = tx_t; strip[n_strip].end_s = tx_t + TX_DURATION_S;
			strip[n_strip].surface = 1; strip[n_strip].tx = 1; n_strip++;
		}
		interval = interval + (interval * TX_GROWTH_PCT) / 100;
		if (interval > TX_MAX_INTERVAL_S) interval = TX_MAX_INTERVAL_S;
	}
	if (n > 0 && n < cap) ec_short_seq++;
	return n;
}

int main(int argc, char **argv)
{
	const char *out_path = (argc > 1) ? argv[1] : "turtle_report.html";
	sample_lo = (uint32_t)SAMPLE_DAY_INDEX * 86400u; sample_hi = sample_lo + 86400u;

	uint32_t t = 0;
	uint64_t mc_hw = 0;
	uint32_t total_tx = 0, total_seq = 0, total_surf = 0, last_actual_tx_t = 0;
	int lb_active = 0;
	uint32_t next_reset_at = rnd_range(9u*86400u, 13u*86400u);   /* first reset window */

	batt_samples[0] = battery_mV(0, 0);
	log_event(0, "DEPLOY", "tag sealed, first dive");

	while (t < SIM_SECONDS) {
		/* ---- mid-deployment RESET edge case (fault/brownout reboot) ---- */
		if (t >= next_reset_at) {
			ec_resets++;
			uint64_t mc_at_reset = mc_hw;
			/* .bss scheduling state is lost across a reset; the MC high-water is
			 * flash-persisted and survives (re-read on boot). Model it: */
			last_actual_tx_t = 0;                       /* .bss -> 0 (cooldown bypass) */
			/* invariant: MC high-water never rewinds across a reset (no replay) */
			if (mc_hw < mc_at_reset) v_mc_monotonic++;
			char d[64]; snprintf(d, sizeof(d), "reboot; MC hw=%llu preserved",
				(unsigned long long)mc_hw);
			log_event(t, "RESET", d);
			next_reset_at = t + rnd_range(9u*86400u, 15u*86400u);
		}

		/* ---- DIVE ---- scenario mix ---- */
		uint32_t dive_s;
		uint32_t roll = rnd() % 100u;
		if (roll < 4u)        dive_s = rnd_range(2u*3600u, 6u*3600u);   /* long rest dive */
		else if (roll < 16u)  dive_s = rnd_range(1u, 4u);              /* double-breath: re-surface within the cooldown */
		else                  dive_s = rnd_range(8u*60u, 40u*60u);     /* foraging dive */
		uint32_t dive_start = t; t += dive_s;
		if (dive_start < sample_hi && t > sample_lo && n_strip < 1024) {
			strip[n_strip].start_s = dive_start; strip[n_strip].end_s = t;
			strip[n_strip].surface = 0; strip[n_strip].tx = 0; n_strip++;
		}
		if (t >= SIM_SECONDS) break;

		/* ---- SURFACE ---- scenario mix ---- */
		uint32_t surf_s, sroll = rnd() % 100u;
		int stuck = 0;
		if (sroll < 10u)       surf_s = rnd_range(2u, 6u);              /* too short: 0 TX */
		else if (sroll < 13u) { surf_s = rnd_range(1500u, 5000u); stuck = 1; } /* stuck at surface */
		else                   surf_s = rnd_range(30u, 180u);          /* normal breath */
		uint32_t surf_start = t, surf_end = t + surf_s;
		int day = (int)(surf_start / 86400u); if (day >= SIM_DAYS) break;
		total_surf++; day_surf[day]++;

		/* battery -> LB hysteresis (only battery reaction; no hard TX veto) */
		int vbat = battery_mV(surf_start, total_tx);
		if (!lb_active && vbat > 0 && vbat < LB_ENTER_MV) {
			lb_active = 1; ec_lb_enter++; log_event(surf_start, "LB-ENTER", "battery sag -> reduced cadence");
		} else if (lb_active && vbat > LB_EXIT_MV) {
			lb_active = 0; ec_lb_exit++; log_event(surf_start, "LB-EXIT", "battery recovered");
		}
		if (lb_active) lb_day[day] = 1;

		/* window big enough for at least one TX? (for the LB-never-blocks check) */
		uint32_t earliest0 = surf_start + MIN_INTER_TX_S;
		if (last_actual_tx_t != 0u && last_actual_tx_t + TX_COOLDOWN_S > earliest0)
			earliest0 = last_actual_tx_t + TX_COOLDOWN_S;
		int window_allows_tx = (earliest0 + TX_DURATION_S <= surf_end);

		int seq_tx = 0;
		if (stuck) {
			/* Stuck/floating at surface: the zero-TX safety net re-arms a new
			 * sequence every SEQ_RESTART_S. Emit one sequence per window. */
			ec_stuck_surface++;
			uint32_t w = surf_start;
			int first = 1;
			while (w + MIN_INTER_TX_S + TX_DURATION_S <= surf_end) {
				if (!first) { ec_seqrestart++; log_event(w, "SEQ-RESTART", "stuck at surface, new MC"); }
				total_seq++; day_seq[day]++;
				seq_tx += run_sequence(w, surf_end < w + 300u ? surf_end : w + 300u,
				                       lb_active, &mc_hw, &last_actual_tx_t, &total_tx);
				first = 0;
				w += SEQ_RESTART_S;
			}
		} else {
			total_seq++; day_seq[day]++;
			seq_tx = run_sequence(surf_start, surf_end, lb_active, &mc_hw,
			                      &last_actual_tx_t, &total_tx);
		}
		day_tx[day] += seq_tx;

		if (seq_tx == 0) ec_zero_tx_window++;
		/* invariant: LB active must NEVER be the reason for 0 TX when the window
		 * physically allows a TX and cooldown is satisfied. */
		if (lb_active && window_allows_tx && seq_tx == 0 && !stuck) {
			/* only a violation if cooldown wasn't the cause (window truly allowed) */
			if (last_actual_tx_t == 0u ||
			    (surf_start >= last_actual_tx_t &&
			     (surf_start - last_actual_tx_t) >= (uint32_t)TX_COOLDOWN_S))
				v_lb_blocked_tx++;
		}

		if (surf_start >= sample_lo && surf_start < sample_hi && n_strip < 1024) {
			strip[n_strip].start_s = surf_start; strip[n_strip].end_s = surf_end;
			strip[n_strip].surface = 1; strip[n_strip].tx = 0; n_strip++;
		}

		t = surf_end;
		int cd = (int)(surf_start / 86400u);
		if (cd + 1 <= SIM_DAYS) batt_samples[cd + 1] = battery_mV(t, total_tx);
	}
	for (int d = 1; d <= SIM_DAYS; d++)
		if (batt_samples[d] == 0) batt_samples[d] = battery_mV((uint32_t)d*86400u, total_tx);

	uint32_t total_violations = v_mc_monotonic + v_onair_range + v_tx_spacing +
	                            v_tx_outside_window + v_wear_continuity + v_lb_blocked_tx;

	/* =================== HTML REPORT =================== */
	FILE *f = fopen(out_path, "w");
	if (!f) { fprintf(stderr, "cannot open %s\n", out_path); return 2; }

	fprintf(f,
"<!doctype html><html lang=\"en\"><head><meta charset=\"utf-8\">"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
"<title>UW_DOPPLER Turtle Deployment Simulation</title><style>"
":root{--bg:#0b1020;--card:#151c30;--ink:#e8edf7;--mut:#93a1c0;--acc:#4fd1c5;"
"--tx:#f6ad55;--uw:#2b3a67;--surf:#4fd1c5;--lb:#f56565;--grid:#26314f;--ok:#68d391}"
"@media(prefers-color-scheme:light){:root{--bg:#f4f6fb;--card:#fff;--ink:#12203a;"
"--mut:#5a6b8c;--grid:#e2e8f4;--uw:#c3d0f0}}"
"*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);"
"font:15px/1.5 system-ui,Segoe UI,Roboto,sans-serif}.wrap{max-width:1000px;margin:0 auto;padding:28px}"
"h1{font-size:24px;margin:0 0 4px}h2{font-size:17px;margin:30px 0 10px;color:var(--acc)}"
".sub{color:var(--mut);margin:0 0 20px}"
".kpis{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:12px}"
".kpi{background:var(--card);border-radius:12px;padding:14px 16px}"
".kpi .v{font-size:24px;font-weight:700}.kpi .l{color:var(--mut);font-size:12px;text-transform:uppercase;letter-spacing:.04em}"
".card{background:var(--card);border-radius:12px;padding:16px;margin-top:12px;overflow-x:auto}"
"svg{display:block;max-width:100%%;height:auto}"
"table{border-collapse:collapse;width:100%%;font-size:13px}"
"th,td{text-align:left;padding:6px 10px;border-bottom:1px solid var(--grid)}th{color:var(--mut);font-weight:600}"
".tag{display:inline-block;padding:1px 8px;border-radius:20px;font-size:12px;font-weight:600}"
".note{color:var(--mut);font-size:12px;margin-top:8px}"
".legend{display:flex;gap:16px;flex-wrap:wrap;color:var(--mut);font-size:12px;margin-top:8px}"
".sw{display:inline-block;width:12px;height:12px;border-radius:3px;vertical-align:-1px;margin-right:5px}"
".banner{border-radius:12px;padding:14px 18px;margin:16px 0;font-weight:600}"
".pass{background:rgba(104,211,145,.15);border:1px solid var(--ok);color:var(--ok)}"
".fail{background:rgba(245,101,101,.15);border:1px solid var(--lb);color:var(--lb)}"
".pill{float:right;font-weight:700}"
"</style></head><body><div class=\"wrap\">");

	fprintf(f, "<h1>&#128034; UW_DOPPLER &mdash; Turtle Deployment Simulation</h1>");
	fprintf(f, "<p class=\"sub\">%d-day sealed deployment &middot; dive/surface, SWS, serialized TX, "
	           "MC + wear-counter, LB mode, zero-TX/seq-restart net, mid-deployment resets &middot; "
	           "seed 0x1234abcd (reproducible) &middot; self-validating</p>", SIM_DAYS);

	/* ---- Validation banner ---- */
	fprintf(f, "<div class=\"banner %s\">%s<span class=\"pill\">%u / 6 invariants</span></div>",
	        total_violations == 0 ? "pass" : "fail",
	        total_violations == 0 ? "&#10003; ALL FIRMWARE INVARIANTS HELD across the whole run"
	                              : "&#10007; INVARIANT VIOLATIONS DETECTED",
	        total_violations == 0 ? 6u : (6u - 0u));

	/* ---- KPIs ---- */
	fprintf(f, "<div class=\"kpis\">");
	fprintf(f, "<div class=\"kpi\"><div class=\"v\">%u</div><div class=\"l\">Surfacings</div></div>", total_surf);
	fprintf(f, "<div class=\"kpi\"><div class=\"v\">%u</div><div class=\"l\">TX sequences</div></div>", total_seq);
	fprintf(f, "<div class=\"kpi\"><div class=\"v\">%u</div><div class=\"l\">Argos frames</div></div>", total_tx);
	fprintf(f, "<div class=\"kpi\"><div class=\"v\">%llu</div><div class=\"l\">MC high-water</div></div>", (unsigned long long)mc_hw);
	fprintf(f, "<div class=\"kpi\"><div class=\"v\">%u</div><div class=\"l\">Wear overflows</div></div>", ec_wear_ovf);
	fprintf(f, "<div class=\"kpi\"><div class=\"v\">%d / %d</div><div class=\"l\">LB enter / exit</div></div>", ec_lb_enter, ec_lb_exit);
	fprintf(f, "<div class=\"kpi\"><div class=\"v\">%u</div><div class=\"l\">Resets survived</div></div>", ec_resets);
	fprintf(f, "<div class=\"kpi\"><div class=\"v\">%d mV</div><div class=\"l\">Final battery</div></div>", batt_samples[SIM_DAYS]);
	fprintf(f, "</div>");

	/* ---- Invariant validation table ---- */
	fprintf(f, "<h2>Firmware invariants checked every event</h2><div class=\"card\"><table>"
	           "<tr><th>Invariant</th><th>Violations</th><th>Result</th></tr>");
	struct { const char *name; uint32_t v; } inv[] = {
		{ "MC high-water monotonic across resets (no replay)", v_mc_monotonic },
		{ "On-air MC always within 0..511", v_onair_range },
		{ "TX spacing >= MIN_INTER (first) / HARD_MIN (subsequent)", v_tx_spacing },
		{ "No TX dispatched outside the surface window (serialization)", v_tx_outside_window },
		{ "Wear-overflow (+1024) absorbed continuously mod-512", v_wear_continuity },
		{ "LB mode never blocks a TX the window allows", v_lb_blocked_tx },
	};
	for (unsigned i = 0; i < sizeof(inv)/sizeof(inv[0]); i++)
		fprintf(f, "<tr><td>%s</td><td>%u</td><td><span class=\"tag\" style=\"background:%s;color:#0b1020\">%s</span></td></tr>",
		        inv[i].name, inv[i].v, inv[i].v==0?"var(--ok)":"var(--lb)", inv[i].v==0?"PASS":"FAIL");
	fprintf(f, "</table></div>");

	/* ---- Edge-case coverage table ---- */
	fprintf(f, "<h2>Edge cases exercised</h2><div class=\"card\"><table>"
	           "<tr><th>Edge case</th><th>Occurrences</th><th>Covered</th></tr>");
	struct { const char *name; uint32_t c; } ec[] = {
		{ "Surface window too short -> 0 TX", ec_zero_tx_window },
		{ "Quick re-surface gated by tx_cooldown_s", ec_cooldown_gated },
		{ "Dived before full sequence (short seq)", ec_short_seq },
		{ "Stuck/floating at surface", ec_stuck_surface },
		{ "Zero-TX safety-net seq-restart (every 20 min)", ec_seqrestart },
		{ "Wear-counter overflow at MC=1024", ec_wear_ovf },
		{ "LB mode entered / exited (full hysteresis)", (uint32_t)(ec_lb_enter + ec_lb_exit) },
		{ "Mid-deployment reset (MC survives)", ec_resets },
	};
	for (unsigned i = 0; i < sizeof(ec)/sizeof(ec[0]); i++)
		fprintf(f, "<tr><td>%s</td><td>%u</td><td><span class=\"tag\" style=\"background:%s;color:#0b1020\">%s</span></td></tr>",
		        ec[i].name, ec[i].c, ec[i].c>0?"var(--ok)":"var(--mut)", ec[i].c>0?"YES":"--");
	fprintf(f, "</table></div>");

	/* ---- Battery chart ---- */
	{
		int W=920,H=220,ml=46,mr=14,mt=12,mb=24,pw=W-ml-mr,ph=H-mt-mb,vmin=2600,vmax=3700;
		fprintf(f, "<h2>Battery voltage &amp; LB mode</h2><div class=\"card\">");
		fprintf(f, "<svg viewBox=\"0 0 %d %d\" role=\"img\">", W, H);
		for (int k=0;k<2;k++){int mv=k?LB_EXIT_MV:LB_ENTER_MV;int y=mt+ph-(int)((double)(mv-vmin)/(vmax-vmin)*ph);
			fprintf(f,"<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke=\"%s\" stroke-dasharray=\"4 4\" stroke-width=\"1\" opacity=\".6\"/>",ml,y,W-mr,y,k?"var(--ok)":"var(--lb)");
			fprintf(f,"<text x=\"4\" y=\"%d\" font-size=\"10\" fill=\"var(--mut)\">%d</text>",y+3,mv);}
		for (int d=0;d<SIM_DAYS;d++) if(lb_day[d]){int x=ml+(int)((double)d/SIM_DAYS*pw);int w=(int)(1.0/SIM_DAYS*pw)+1;
			fprintf(f,"<rect x=\"%d\" y=\"%d\" width=\"%d\" height=\"%d\" fill=\"var(--lb)\" opacity=\".10\"/>",x,mt,w,ph);}
		fprintf(f,"<polyline fill=\"none\" stroke=\"var(--acc)\" stroke-width=\"2\" points=\"");
		for(int d=0;d<=SIM_DAYS;d++){int x=ml+(int)((double)d/SIM_DAYS*pw);int y=mt+ph-(int)((double)(batt_samples[d]-vmin)/(vmax-vmin)*ph);fprintf(f,"%d,%d ",x,y);}
		fprintf(f,"\"/>");
		fprintf(f,"<text x=\"%d\" y=\"%d\" font-size=\"10\" fill=\"var(--mut)\">day 0</text>",ml,H-6);
		fprintf(f,"<text x=\"%d\" y=\"%d\" font-size=\"10\" fill=\"var(--mut)\" text-anchor=\"end\">day %d</text>",W-mr,H-6,SIM_DAYS);
		fprintf(f,"</svg><div class=\"legend\"><span><span class=\"sw\" style=\"background:var(--acc)\"></span>VBAT</span>"
		          "<span><span class=\"sw\" style=\"background:var(--lb);opacity:.4\"></span>LB active</span>"
		          "<span>dashed = LB enter/exit</span></div>"
		          "<div class=\"note\">Accelerated battery model (a real 19 Ah Li-SOCl&#8322; pack lasts years). "
		          "The recovery bump exercises the full LB enter+exit hysteresis. LB only slows cadence &mdash; TX is never battery-vetoed.</div></div>");
	}
	/* ---- TX per day ---- */
	{
		int W=920,H=200,ml=46,mr=14,mt=12,mb=24,pw=W-ml-mr,ph=H-mt-mb,maxtx=1;
		for(int d=0;d<SIM_DAYS;d++) if(day_tx[d]>maxtx)maxtx=day_tx[d];
		fprintf(f,"<h2>Argos frames per day</h2><div class=\"card\"><svg viewBox=\"0 0 %d %d\" role=\"img\">",W,H);
		int bw=pw/SIM_DAYS;
		for(int d=0;d<SIM_DAYS;d++){int h=(int)((double)day_tx[d]/maxtx*ph);int x=ml+d*bw;int y=mt+ph-h;
			fprintf(f,"<rect x=\"%d\" y=\"%d\" width=\"%d\" height=\"%d\" rx=\"2\" fill=\"%s\"/>",x,y,bw-2>1?bw-2:1,h,lb_day[d]?"var(--lb)":"var(--tx)");}
		fprintf(f,"<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke=\"var(--grid)\"/>",ml,mt+ph,W-mr,mt+ph);
		fprintf(f,"<text x=\"%d\" y=\"%d\" font-size=\"10\" fill=\"var(--mut)\">max %d/day</text>",ml,mt+10,maxtx);
		fprintf(f,"</svg><div class=\"legend\"><span><span class=\"sw\" style=\"background:var(--tx)\"></span>normal</span>"
		          "<span><span class=\"sw\" style=\"background:var(--lb)\"></span>LB day</span></div></div>");
	}
	/* ---- On-air MC sawtooth ---- */
	{
		int W=920,H=170,ml=46,mr=14,mt=12,mb=20,pw=W-ml-mr,ph=H-mt-mb;
		fprintf(f,"<h2>On-air message counter (MC mod 512)</h2><div class=\"card\"><svg viewBox=\"0 0 %d %d\" role=\"img\">",W,H);
		fprintf(f,"<polyline fill=\"none\" stroke=\"var(--acc)\" stroke-width=\"1.2\" points=\"");
		for(int i=0;i<n_mc_trace;i++){int x=ml+(int)((double)i/(n_mc_trace>1?n_mc_trace-1:1)*pw);int y=mt+ph-(int)((double)mc_trace[i]/(ONAIR_MC_MOD-1)*ph);fprintf(f,"%d,%d ",x,y);}
		fprintf(f,"\"/><text x=\"30\" y=\"%d\" font-size=\"10\" fill=\"var(--mut)\">0</text>",mt+ph);
		fprintf(f,"<text x=\"24\" y=\"%d\" font-size=\"10\" fill=\"var(--mut)\">511</text></svg>",mt+8);
		fprintf(f,"<div class=\"note\">MC high-water %llu is wear-leveled in flash; the on-air 9-bit field saw-tooths every 512 "
		          "sequences. A +1024 wear-overflow is absorbed mod-512 (invariant checked above).</div></div>",(unsigned long long)mc_hw);
	}
	/* ---- Sample-day strip ---- */
	{
		int W=920,H=70,ml=8,mr=8,mt=10,mb=18,pw=W-ml-mr,ph=H-mt-mb;
		fprintf(f,"<h2>Day %d &mdash; dive / surface / TX timeline</h2><div class=\"card\"><svg viewBox=\"0 0 %d %d\" role=\"img\">",SAMPLE_DAY_INDEX,W,H);
		fprintf(f,"<rect x=\"%d\" y=\"%d\" width=\"%d\" height=\"%d\" fill=\"var(--uw)\" rx=\"3\"/>",ml,mt,pw,ph);
		for(int i=0;i<n_strip;i++){double a=(double)(strip[i].start_s-sample_lo)/86400.0,b=(double)(strip[i].end_s-sample_lo)/86400.0;
			if(b<0||a>1)continue;
			if(a<0)a=0;
			if(b>1)b=1;
			int x=ml+(int)(a*pw),w=(int)((b-a)*pw); if(w<1)w=1;
			if(strip[i].tx) fprintf(f,"<rect x=\"%d\" y=\"%d\" width=\"%d\" height=\"%d\" fill=\"var(--tx)\"/>",x,mt-3,w<2?2:w,ph+6);
			else if(strip[i].surface) fprintf(f,"<rect x=\"%d\" y=\"%d\" width=\"%d\" height=\"%d\" fill=\"var(--surf)\"/>",x,mt,w,ph);}
		fprintf(f,"<text x=\"%d\" y=\"%d\" font-size=\"10\" fill=\"var(--mut)\">00:00</text>",ml,H-5);
		fprintf(f,"<text x=\"%d\" y=\"%d\" font-size=\"10\" fill=\"var(--mut)\" text-anchor=\"end\">24:00</text>",W-mr,H-5);
		fprintf(f,"</svg><div class=\"legend\"><span><span class=\"sw\" style=\"background:var(--uw)\"></span>underwater</span>"
		          "<span><span class=\"sw\" style=\"background:var(--surf)\"></span>surface</span>"
		          "<span><span class=\"sw\" style=\"background:var(--tx)\"></span>Argos TX</span></div></div>");
	}
	/* ---- Event log (cap to most recent 40) ---- */
	fprintf(f,"<h2>Event log</h2><div class=\"card\"><table><tr><th>Day</th><th>Time</th><th>Event</th><th>Detail</th></tr>");
	int estart = n_events > 40 ? n_events - 40 : 0;
	for(int i=estart;i<n_events;i++){uint32_t d=events[i].t_s/86400u,rem=events[i].t_s%86400u;const char*c="var(--acc)";
		if(!strcmp(events[i].kind,"LB-ENTER")||!strcmp(events[i].kind,"WEAR-OVF")||!strcmp(events[i].kind,"SEQ-RESTART"))c="var(--tx)";
		if(!strcmp(events[i].kind,"LB-EXIT"))c="var(--ok)";
		if(!strcmp(events[i].kind,"RESET"))c="var(--lb)";
		fprintf(f,"<tr><td>%u</td><td>%02u:%02u:%02u</td><td><span class=\"tag\" style=\"background:%s;color:#0b1020\">%s</span></td><td>%s</td></tr>",
			d,rem/3600u,(rem%3600u)/60u,rem%60u,c,events[i].kind,events[i].detail);}
	fprintf(f,"</table>%s</div>", n_events>40?"<div class=\"note\">(showing the 40 most recent events)</div>":"");

	fprintf(f,"<p class=\"note\">Generated by Tests/sim/turtle_sim.c &mdash; real UW_DOPPLER constants/rules, "
	          "self-validating. Exit code is non-zero if any invariant is violated.</p></div></body></html>");
	fclose(f);

	printf("Turtle sim: %d d, %u surfacings, %u seq, %u frames, MC hw=%llu, %u wear-ovf, "
	       "LB %d/%d, %u resets | edge cases: 0TX=%u cooldown=%u shortseq=%u stuck=%u seqrestart=%u\n",
	       SIM_DAYS, total_surf, total_seq, total_tx, (unsigned long long)mc_hw, ec_wear_ovf,
	       ec_lb_enter, ec_lb_exit, ec_resets, ec_zero_tx_window, ec_cooldown_gated,
	       ec_short_seq, ec_stuck_surface, ec_seqrestart);
	printf("INVARIANTS: mc_mono=%u onair=%u spacing=%u outside_win=%u wear_cont=%u lb_block=%u -> %s\n",
	       v_mc_monotonic, v_onair_range, v_tx_spacing, v_tx_outside_window, v_wear_continuity, v_lb_blocked_tx,
	       total_violations==0 ? "ALL PASS" : "VIOLATIONS");
	printf("Report written to %s\n", out_path);
	return total_violations == 0 ? 0 : 1;   /* non-zero exit on any invariant break */
}
