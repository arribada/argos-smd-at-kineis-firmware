# HW integration test plan — SMD_STDALONE × UW_DOPPLER

Sequence of validation steps to run **when the HW PA-brownout fix is in place**. Each step lists the AT command to run, the expected UART trace, the EVTLOG event to check, and the failure signature.

Read each section in order — earlier failures often explain later ones.

---

## Prerequisites

- Board with the PA brownout HW fix applied (bigger cap, fix soldering, etc.).
- Chip recovered from any deep-LPM state (`Tools/recover_chip.ps1 -Reflash`).
- Production binary flashed: `SMD_STDALONE × UW_DOPPLER × UART × BASIC`, `MCU_PA_GPIO_ENABLE=1`, `SMPS_BYPASS_TX=1`, `UW_DOPPLER_VERBOSE_TRACE=1`.
- COM3 or COM12 @ **9600 baud** open.

---

## T1. Boot sanity (no AT needed)

Press NRST and observe the UART for the first 25 seconds. Expected sequence:

```
==== BOOT cause=[P] CSR=0x0401C600 SR1=0x00000000 ====
[ST] 0->0 t=1283        (BOOT)
hb t=1302 s=0           ... continues every 1 s
[ST] 0->1 t=5083        (BOOT_DEPLOY_LED, 4 s after init blink ends)
hb t=5302 s=1           ... continues
[ST] 1->2 t=10102       (INIT_MAC, 5 s after entering BOOT_DEPLOY_LED)
[ST] 2->3 t=10122       (WAIT_MAC_READY)
[ST] 3->4 t=10142       (MONITORING — TARGET)
hb t=10302 s=4          ... heartbeats forever
```

**Visual** : RED → GREEN → BLUE LED cycle (~1.2 s), then BLUE blink 10× (4 s, boot indicator), then steady GREEN or BLUE (deploy/no-deploy indicator) for 5 s, then LED off.

**FAIL signatures :**
- `s=4` never appears → MAC init failed (run T2 with longer timeout).
- `[ST] 3->3` repeatedly → MAC retry loop (HW radio issue or libkineis state issue).
- `==== BOOT cause=[B] ====` (Brown-out) → PSU sag at boot, not at TX. Inspect VBAT during cold-start.

---

## T2. AT command surface

```
AT+PING=?     → +OK
AT+VERSION=? → +VERSION=v0.8 +OK
AT+ADDR=?    → +ADDR=11223344 +OK
AT+STATUS=?  → +STATUS=4,<elapsed_s>,...,<batt_mV>,<deploy>,... +OK
AT+SWS=?     → +SWS=<state>,<raw>,<hysteresis>,<threshold>,<dive_cnt>,<surf_cnt> +OK
AT+LED=?     → +LED=1 +OK
AT+DEPLOY=?  → +DEPLOY=1 +OK
AT+LOG=?     → +LOG=<N> followed by event entries +OK
```

**FAIL :** `+ERROR=1203` on a known command → command not registered for this APP. Verify build was UW_DOPPLER.

---

## T3. SWS forced measurement

```
AT+SWSFORCE  → +SWS=<state>,<adc> +OK
```

`<adc>` should be 200-400 in air, 700-1000 underwater (per the SWS_LBCFG thresholds). Issue 3 calls in a row and verify the ADC value drifts ±10 LSB max (jitter check).

**FAIL :** `<adc> = 0` or `4095` → ADC stuck/dead, or SWS power pin not driving. Check `+SWSCFG=?` for sane params.

---

## T4. Magnet 6 s gesture → SHUTDOWN

Apply magnet to the reed switch, hold for **at least 6 s**. Watch UART :

```
[GST] ON t=<tick>                       (immediate, white LED on)
[GST] ask SHUTDOWN hold=6000ms          (after 6 s)
[GST] CONFIRMED shutdown t=<tick>       (after OFF→ON within 2 s)
[ST] 4->7 t=<tick>                      (SHUTDOWN_BLINK)
[UW_DPL] Entering SHUTDOWN...
```

Then board **physically loses power** (VBAT to TPS63901 drops to 0 if the PWR_LATCH circuit works correctly).

**Recovery :** approach magnet again → reed circuit re-latches → board boots, run T1 again.

**FAIL :**
- Magnet detected but gesture not confirmed → debounce too aggressive, check `MGR_REED_*_MS` constants.
- LPM_shutdownNow doesn't actually cut power → BSP_HAS_PWR_LATCH may not be defined for this board, or PB7 pull-down not effective. Measure PB7 voltage just before SHUTDOWN.

---

## T5. AT+SHUTDOWN (test SHUTDOWN without magnet)

This command was added 2026-06-06 specifically for testing the SHUTDOWN path without needing a magnet. Identical to T4 effect.

```
AT+SHUTDOWN  → +OK
[ST] 4->7 ...
[UW_DPL] Entering SHUTDOWN...
```

Power cut + reed-switch wake same as T4.

---

## T6. TX path (after HW brownout fix)

Trigger a forced TX :

```
AT+TEST=1
```

Expected (was failing before the HW fix) :

```
+TEST=1
+OK
[ST] 4->5 t=<tick>                     (SURFACE_TX)
[TRACE] SURF_TX enter t=<tick>
[TRACE] SURF_TX LED set t=<tick>       (LED VIOLET)
[TRACE] TCXO warmup 2000ms
[TRACE] TCXO ready t=<tick>
[TRACE] KNS_Q_push start t=<tick>
[TRACE] KNS_Q_push done t=<tick>
[TRACE] >>> WAIT_TX_DONE t=<tick>
[ST] 5->6 t=<tick>                     (WAIT_TX_DONE)

[PA-TRACE] turn_on_pa ENTER
[<                                     (just before PA_PSU_EN HIGH)
[>                                     (just after — CRITICAL marker)
[UW_DPL] TX done (#1)                  (KNS_MAC_TX_DONE event)
[ST] 6->4 t=<tick>                     (back to MONITORING)
```

**Critical check :** `[>` must appear after `[<`. If `[<` appears but `[>` doesn't, the brownout HW fix is insufficient → scope VSYS through PA enable.

**Endurance :** `AT+TEST=5` to fire 5 TX cycles back-to-back.

---

## T7. Battery monitor

```
AT+STATUS=?
```

Look at field index 7 (zero-based): the battery mV. Should match a multimeter reading on VBAT ±50 mV.

```
AT+BATCFG=?
```

Verify thresholds match the spec (low-bat enter / exit voltages).

---

## T8. Deploy mode toggle

```
AT+DEPLOY=0       → +OK, LED indicator changes to BLUE during BOOT_DEPLOY_LED
AT+DEPLOY=1       → +OK, LED indicator returns to GREEN
AT+SAVE           → persists to NVM
NRST              → re-verify the new setting survives reset (AT+DEPLOY=?)
```

---

## T9. LPM (skip until HW debug session)

LPM STOP is intentionally disabled in this build (`uw_doppler_lpmReq` returns NONE in MONITORING). The aggregator code is wired and ready but the chip locks irrecoverably on second STOP entry — root cause not yet identified.

**To attempt** : edit `kns_app_uw_doppler.c:uw_doppler_lpmReq()` and flip the `return LOW_POWER_MODE_NONE` to `return LOW_POWER_MODE_STOP`. Rebuild + reflash. Be ready to run `Tools/recover_chip.ps1 -Reflash` to recover when chip locks.

---

## T10. AT+LPM=4 (STANDBY) — optional

```
AT+LPM=4
```

Triggers STANDBY via the AT_LPM command path. Chip enters STANDBY (deeper than STOP but lighter than SHUTDOWN — SRAM2 retained, RTC running). Wake via PB3 EXTI or RTC alarm.

**Not exercised by the production app flow** — only useful for power-consumption measurements.

---

## T11. Endurance — 10 min cycle

Loop the following sequence 10 times :

1. `AT+TEST=1` → wait for `TX done`
2. `AT+STATUS=?` → battery doesn't drop > 20 mV per TX
3. `AT+SWS=?` → state stable
4. Wait 50 s (let backoff settle)

Expect 0 reboots, 0 HardFaults, battery drain ~50-100 mV total. Variations could indicate residual brownout or backoff misbehavior.

---

## T12. EVTLOG forensics

After all tests, dump :

```
AT+LOG=?     → list of events with timestamps + states + data
AT+TXSTATS=? → cumulative TX counters (done / timeout / error / backoff_used)
AT+PMLOG=?   → post-mortem flash log (error-severity events survive reset)
```

Expected event types in a happy-path log :
```
e=00 (EVT_BOOT)
e=01 (EVT_STATE_CHANGE) × N
e=09 (EVT_MAC_READY)
e=06 (EVT_TX_START)
e=07 (EVT_TX_DONE)
```

Unexpected types to investigate :
- e=08 (EVT_TX_TIMEOUT) → MAC stuck, likely radio issue
- e=0E (EVT_ERROR) → check the data field for the error code
- e=14 (EVT_BOOT_FAIL) → boot loop counter incremented (now force-cleared)
- e=15 (EVT_FACTORY_RESET) → recovery triggered
