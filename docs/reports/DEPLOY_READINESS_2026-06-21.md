# Deployment readiness — v2 (2026-06-21)

Multi-agent evaluation (12 dimensions x boards) of branch `v2` after the 2026-06-20/21 total audit
and all fixes. Primary target = **APP=UW_DOPPLER BOARD=SMD_STDALONE COMM=UART DEBUG=0**, epoxy-sealed
~12-month ocean. Secondary products = STDLN / GUI / DOPPLER on SMD_PA/NOPA/STDALONE/OP (bench-accessible).

## Verdict: 87/100 — GO-WITH-GATES — "CONDITIONAL YES"

The firmware is **code-ready** for the sealed UW_DOPPLER deployment: all 3 CRITICAL sealed-brick
fixes are verified live, the recovery tree is coherent with **no provable code-level brick**, and
519 unit tests pass with regression locks. **No NO-GO conditions.** Deploy is approved subject to the
bench gates below. Do NOT seal until the STOP2 µA floor and the LSE crystal-death behaviour are
bench-confirmed; everything else is provably ready now.

## Dimension scores (avg 87.5)

| Dimension | Score | Verdict |
|---|---:|---|
| backward-compat (main→v2) | 94 | GO |
| build-config / all boards | 91 | GO |
| robustness / recovery | 89 | GO-WITH-GATES |
| code quality / dead code | 88 | GO |
| memory safety | 88 | GO-WITH-GATES |
| reed / gesture / magnet | 88 | GO-WITH-GATES |
| MAC / TX / payload | 88 | GO-WITH-GATES |
| concurrency / ISR | 86 | GO-WITH-GATES |
| flash / credential durability | 86 | GO-WITH-GATES |
| LPM / wake | 86 | GO-WITH-GATES |
| battery / power | 84 | GO-WITH-GATES |
| SWS / dive-surface | 82 | GO-WITH-GATES |

## Per-config readiness
- **UW_DOPPLER / SMD_STDALONE / UART (sealed): GO-WITH-GATES (87).** Only config carrying sealed-brick
  weight; all 3 CRITICAL fixes confirmed live; triple compile-time lock (board + COMM #error).
- **STDLN × all boards: GO.** Bench-accessible, not sealed.
- **GUI × all boards: GO-WITH-GATES.** STANDBY radio de-init + SPI re-arm fixed; verify SPI-after-STOP on bench.
- **DOPPLER × all boards: GO.** DOPPLER+SPI now compiles; SPI-only over-reads are bench-recoverable.
- **UW_DOPPLER on non-STDALONE or COMM=SPI: refused at compile (`#error`)** — not a deployable combo.

## Biggest risk
**LSE crystal death DURING an active STOP2/SHUTDOWN sleep on the sealed unit.** The RTC liveness gate
only validates the crystal at sleep ENTRY; once asleep, IWDG is frozen (STOP2 by design) and the RTC
WUT is the sole armed wake. A marginal 32.768 kHz crystal that passes entry but stops mid-sleep =
no wake, no rescue (magnet/reed auto-wake also routes through the dead-RTC cold-boot path). Probability
low; consequence total. **HW failure mode, not a code defect** — the only firmware mitigation (an
LSI-clocked in-sleep watchdog) would defeat the µA budget. Mitigate via crystal screening/derating at
manufacturing and the shortest-acceptable STOP2 period.

## Gates
### Code-blocking: NONE for the sealed UW_DOPPLER deployment.

### Bench — must pass before seal
1. Measure STOP2 idle floor µA on real sealed SMD_STDALONE vs the ~12-month budget (paper-only ~23 µA).
2. Read the new `[ERR] true reset cause masked by RMVF` WARN: if silent IWDG hangs appear, move
   crash-counting onto the pre-RMVF snapshot (deferred #2).
3. LSE crystal-death: validate the LSE→LSI recovery boot on a deliberately-stalled crystal; accept the
   mid-sleep-death residual or screen crystals.
4. Hall/reed thresholds in real ocean salinity/temperature.
5. SWS dive/surface + the `max_dive_time_s` backstop underwater on live HW.
6. (CI) Wire the APP×BOARD×COMM compile matrix as a committed harness (currently manual).
7. Pre-seal: dump + verify the persisted NVM config (no operator mis-config sealed in).

### Accepted residuals
- LSE-death-mid-STOP2 (HW, irreducible in firmware, very low probability after a passing entry gate).
- Deferred #2 (silent-IWDG not counted; conservative always-wipe kept; observability WARN added).
- Deferred #3 (LSE→LSI reclassified benign; normal paths byte-identical; no change).
- SPI-path over-reads are structurally unreachable in the sealed UART build (UW+SPI `#error`).
- `increment_wear_counter` 2-transaction window is recoverable via the MGR_CRED mirror (creds cannot
  be irrevocably corrupted).
- Latent dead `bl_flash_write_bl_state` (no live caller) — add a static_assert/linker guard (not seal-blocking).
