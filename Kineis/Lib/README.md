# Kineis/Lib — closed-source Kineis MAC stack (prebuilt) + public API

## Purpose

The **closed-source** Kineis protocol stack, shipped as prebuilt static
archives plus the public headers that form the integration contract. There are
**no `.c` sources here** for the stack itself — you link the `.a` and implement
the symbols it leaves undefined (provided by [../Extdep/Mcu](../Extdep/Mcu),
[../Extdep/Conf](../Extdep/Conf) and [../App/Kineis_os](../App/Kineis_os)).

## Files

### Archives (do not edit)

| File | Role |
|------|------|
| [libkineis.a](libkineis.a) | The Kineis MAC stack (framing, AES auth, message counter, TX/RX scheduling, sat detection). |
| [libknsrf_wl.a](libknsrf_wl.a) | STM32WL SubGHz RF backend used by the MAC. |
| `libkineis_undefined_symbols.txt`, `libknsrf_wl_undefined_symbols.txt` | The exact symbols each archive expects the integrator to provide (the `mcu_*` wrappers, `KNS_CS_*`, `kns_assert`, queue glue). **This is the contract** — if a build fails to link, check these. |
| [libkineis_info.c](libkineis_info.c) / [.h](libkineis_info.h), [libknsrf_wl_info.c](libknsrf_wl_info.c) / [.h](libknsrf_wl_info.h) | Tiny version-string accessors (compiled into the app, surfaced in `build_info`). |

### Public API headers (the contract you call)

| File | Role |
|------|------|
| [kns_mac.h](kns_mac.h) | MAC service API: `KNS_MAC_*` calls, the `KNS_Q_UL_MAC2APP` event ids (`KNS_MAC_TX_DONE`, `KNS_MAC_TX_TIMEOUT`, `KNS_MAC_OK`, …) and `KNS_MAC_getRsrcStatus()` (drives LPM gating). |
| [kns_mac_evt.h](kns_mac_evt.h), [kns_mac_prfl_cfg.h](kns_mac_prfl_cfg.h) | MAC event structs and profile config (BASIC / BLIND / …). |
| [kns_cfg.h](kns_cfg.h) | Credential & radio config: `KNS_CFG_getId/getAddr/getSN/getMC/getRadioInfo`, `setRadioInfo`, `saveRadioInfo`. |
| [kns_rf.h](kns_rf.h) | RF-layer interface. |
| [kns_types.h](kns_types.h) | Shared types incl. `enum KNS_status_t`. |
| [kns_srvc_common.h](kns_srvc_common.h), [kns_glossary.h](kns_glossary.h) | Common service defs / glossary. |

## Integration

- Linked in [../../Makefile](../../Makefile) via
  `-Wl,--whole-archive -lkineis -lknsrf_wl -Wl,--no-whole-archive`.
- The app supplies everything the archives leave undefined: MCU abstraction
  ([../Extdep/Mcu](../Extdep/Mcu)), assert/critical-section/queue config
  ([../Extdep/Conf](../Extdep/Conf)), the OS scheduler + queues
  ([../App/Kineis_os](../App/Kineis_os)), and the app loops drive the MAC purely
  through the `KNS_Q_DL_APP2MAC` / `KNS_Q_UL_MAC2APP` queues.
- The current lib version string (e.g. `v11.1.0_…`) is emitted in `build_info`
  at build time.

## Gotchas

- **Closed source** — you cannot read/patch the MAC internals; behaviour is
  inferred from these headers + the local Doxygen under `Kineis/Doc/`.
- The undefined-symbols `.txt` files are authoritative: a new lib drop can add a
  required symbol and break the link until the app provides it.
- Credentials (ID/ADDR/SECKEY/RCONF) flow through `kns_cfg.h` into NVM/flash; the
  message counter is a 9-bit field (0..511) — exceeding it desyncs the AES IV
  (see `Tests/unit/test_mc_9bit.c`).
