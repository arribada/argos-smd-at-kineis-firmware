# Kineis/Extdep/Conf — integration config + glue the Kineis lib requires

## Purpose

The small set of **integrator-provided** primitives and compile-time
configuration that the closed-source `libkineis` (see [../../Lib](../../Lib))
depends on. The library is built against these symbols/headers; changing them
changes how the MAC stack asserts, takes critical sections, and lays out its
event queues.

## Files

| File | Role |
|------|------|
| [kineis_sw_conf.h](kineis_sw_conf.h) | Software config knobs for the lib: points `KINEIS_SW_ASSERT_H` at `kns_assert.h`, and defines `enum ERROR_RETURN_T` (the AT/SPI error codes — `ERROR_PARAMETER_FORMAT`, `ERROR_FEATURE_NOT_AVAILABLE`, …) shared across the app. |
| [kns_assert.c](kns_assert.c) / [.h](kns_assert.h) | `kns_assert(cond)` runtime check. On failure → `kns_assert_failed()` → `Error_Handler()` → `MGR_ERR_logAndReset()` (device reset). **Always active — there is no `NDEBUG` gate**, so an assert ships as a reset. |
| [kns_cs.c](kns_cs.c) / [.h](kns_cs.h) | Critical section: `KNS_CS_enter()` / `KNS_CS_exit()` (disable/enable IRQs, nestable). Used by `KNS_Q` and any code touching shared/ISR state. |
| [kns_q_conf.c](kns_q_conf.c) / [.h](kns_q_conf.h) | Queue configuration consumed by `KNS_Q` ([../../App/Kineis_os](../../App/Kineis_os)): the `enum KNS_Q_handle_t` (5 FIFOs, priority-ordered), the static `qPool[]` descriptors, and `qIdx2Str()` for logs. |

## Integration

- `libkineis` is compiled expecting `KINEIS_SW_ASSERT_H`, the critical-section
  API, the error enum and the queue layout defined here — this folder is part of
  the **library contract**, alongside [../Mcu](../Mcu) (MCU abstraction) and
  [../../App/Kineis_os](../../App/Kineis_os) (OS/queue backend).
- Build flags do not alter these (they are always compiled, all apps/boards).

## Gotchas

- **`qPool[]` order MUST match the `KNS_Q_handle_t` enum** and the priority
  expectation in `kns_q_baremetal.c`; a mismatch silently misroutes events. (A
  prior audit caught a missing comma here; covered by `Tests/unit/` regression.)
- **`kns_assert` resets the device.** Use it only for truly impossible states,
  never for recoverable/expected conditions (see the LPM-02 hardening in
  [../../Lpm](../../Lpm), which replaced an assert-on-veto with a log).
- `kns_cs` nests by IRQ-disable depth; every `enter` needs a matching `exit`.
