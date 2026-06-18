# Credential durability (sealed-deploy brick #4) — design + verification

**Branch:** v2-fix-lpm · **Status:** implemented, code-verified, unit-tested. Bench
validation (#4 protocol) pending — to run together.

## The brick

Page 0 of FLASH_USER (`0x0803B000`) holds **both** the immutable credentials and
the mutable wear-counter overflow words:

| offset | field | mutable? |
|---|---|---|
| 0  | ID (8)        | no (provisioned) |
| 8  | ADDR (8)      | no |
| 16 | SECKEY (16)   | no |
| 32 | RADIOCONF (16)| no (static fallback if blank) |
| 48 | APP_VARS (8)  | rare |
| 56 | MSG_COUNTER_OF (overflow) | **every ~1024 TX** |
| 64 | WKU_COUNTER_OF (overflow) | **every ~1024 wakes** |

`MCU_FLASH_write` (mcu_flash.c:84) is a whole-page **read-modify-write** (backup →
erase → reprogram). Every counter-overflow write transits page 0 through a blank
state (~25 ms, IRQ off). A brownout in that window blanks page 0 → `MCU_NVM_getID`
(mcu_nvm.c:305) / `getAddr` (:337) silently fall back to **TEST credentials**
(ID 123456 / ADDR 11:22:33:44) → the tag TXes under a wrong identity forever.

The 9-bit MC clamp had **doubled** the exposure: the 511→0 wrap was mis-classified
as an operator jump → a destructive page-0 rewrite every **512** TX instead of the
legitimate overflow every 1024.

## The fix — three layers (backward compatible, credential offsets unchanged)

### Layer 1 — `MCU_NVM_setMC` 9-bit wrap (mcu_nvm.c)
Wear-level distances computed on the **9-bit ring** (`& 0x1FFu`): the 511→0 rolling
wrap is now a normal forward WL step (one slot program, **no** page-0 erase). The
genuine far-jump branch realigns the shadow to `(shadow & ~0x1FF) + 0x200 + mcTmp`
— strictly monotonic, residue == mcTmp. Restores the 1/1024 cadence + saves flash.

### Layer 2A — `MGR_CRED` mirror + atomic restore (new module)
CRC-protected 64-byte mirror of `{ID,ADDR,SECKEY,RADIOCONF}` at **`0x08032800`**
(page 101, outside FLASH_USER, formerly the linker-reserved BAT_LOG sibling — no
`.ld` MEMORY-region change). `MGR_CRED_syncAndRestore()` (boot + CRED_FAIL tick):

| page 0 | mirror | action |
|---|---|---|
| provisioned | current   | **OK_NOOP** (zero flash writes) |
| provisioned | stale/none| **MIRROR_WRITTEN** (seed/update) |
| blank/partial | valid   | **RESTORED** — atomic single-RMW page-0 rewrite |
| blank/partial | none    | **FAIL_LOUD** |

Key properties: raw `MCU_FLASH_read` (never the substituting getters) + 48-byte
**payload-only** compare → no per-boot wear; restore is **one** `MCU_FLASH_write`
(never a half-populated page); a not-fully-provisioned page **never** overwrites a
good mirror; mirror programmed **magic-dword-last** (torn write → CRC-invalid).

### Layer 2B — fail-loud terminal state (kns_app_uw_doppler.c)
`g_cred_result` set at boot. On `FAIL_LOUD`, `UW_DOPPLER_INIT_MAC` routes to a new
terminal **`UW_DOPPLER_CRED_FAIL`** (solid red LED + `ERR_CREDS_BLANK`) **instead of
starting the MAC** → never TX under test identity. The state is excluded from both
reset deadmen (state-hang loop guard + MAC-timeout), keeps the WDG fed, services AT,
and returns to `INIT_MAC` automatically when re-provisioned over AT.

## Adversarial verification (multi-agent, against live source)

Two review passes (design, then implementation). The design pass found 5 holes
(boot-loop on fail-loud, mirror-clobber on torn restore, per-boot wear) — all folded
into the implementation. The implementation pass: **7/8 properties HOLD** with
file:line proof (atomic restore, never-clobber, fail-loud terminal, steady-state
no-write, Layer 1 correctness, flash/UB safety, backward-compat).

**1 documented residual (benign):** the page-0 restore heals credentials but not the
counter overflow words blanked by the same brownout, so the internal 64-bit
high-water restarts its overflow tally. **No on-air effect:** on-air MC = `(overflow*1024 + WL_index) & 0x1FF`; since `1024 = 2*512` the residue equals the
**surviving** `WL_index & 0x1FF`, so the next-boot on-air sequence is byte-identical
(proven, `test_mc_wrap.c:test_of_loss_preserves_onair_residue`). The only full-shadow
consumer is the operator far-jump (bench). Restoring the overflow is impossible
(unrecoverable) and a mirror-stored shadow would be stale; accepted by design.

## Tests
- `Tests/unit/test_cred_mirror.c` (12): CRC vector, first-boot seed, brownout
  restore, partial-no-clobber, fail-loud, CRC/version/torn reject, reprovision,
  steady-state-no-write, RADIOCONF-blank-no-rewrite.
- `Tests/unit/test_mc_wrap.c` (10): wrap-is-forward, post-wrap cadence, legacy high
  shadow, rollback RAM-only, far-jump residue/monotonic (low+high shadow), +1 from
  every residue, boundaries, OF-loss on-air invariance.
- Full suite **39/39 OK**; firmware `DEBUG=0` builds clean.

## Bench validation (#4) — to define together
1. Provision a unit → reboot → log `MIRROR_WRITTEN` then `OK_NOOP` (no re-write).
2. JLink-erase page 0 only (mirror intact) → reboot → `RESTORED` + TX under real ID.
3. Erase page 0 **and** mirror → reboot → solid red, no TX, `AT+VERSION` alive;
   provision over AT → resumes to MONITORING without power cycle.
4. (stretch) cut power mid page-0 RMW under load → confirm self-heal across reboots.

## Future
When a coordinated provisioning-tool update is possible, move credentials to their
own **immutable** page (never co-located with RMW counters) — makes this mirror
redundant. Until then the mirror is the safety net.
