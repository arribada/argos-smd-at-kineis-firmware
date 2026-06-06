# Master Audit — argos-smd-at-kineis-firmware

**Date:** 2026-06-05
**Scope:** STANDALONE board + UW_DOPPLER app (turtle tracker)
**Context:** Régression MAC stack ne devient jamais READY → reboot loop. Investigation suite à demande utilisateur "reprendre proprement".

---

## TL;DR — Top 5 actions prioritaires

1. **P0 ADC** — `LPM_standby_enter` et `LPM_shutdown_enter` n'appellent pas `MX_ADC_DeInit()` → ADC + clock laissés actifs avant reset → re-init wakeup en état indéterminé.
2. **P0 ADC** — `__HAL_ADC_ENABLE_VREFINT()` jamais appelé → `raw_vref` peut être 0 → cache batterie falsifié → décisions LB / shutdown corrompues.
3. **P0 SWS** — Constante `L4_DROP_PERCENT = 8%` vs **15% en linkit-v4 main**. Trop sensible → fausses détections surface.
4. **P1 SWS** — Plage `pulse_on_max_us` cappée à 1000µs alors que linkit-v4 va jusqu'à 10000µs → biofouling sévère mal géré.
5. **P0 Architecture** — Protocole 2-gesture magnet (POWER_OFF / OPERATIONAL / CONFIG / SHUTDOWN avec confirmation) **n'existe pas du tout** dans le firmware actuel. `MGR_REED` ne fait que ON/OFF brut. À écrire entièrement.

---

## 1. Stack Kineis — Architecture confirmée

(Source: `.claude/kineis/kineis_stack_integration.txt`)

### Tasks scheduled par KNS_OS
- Kineis stack task (gère MAC L1/L2)
- LPM task
- Customer app task (UW_DOPPLER)

### Layout mémoire (validé contre notre LD)
| Section | RAM/ROM | Survit à |
|---|---|---|
| `.knsCtxtData` / `.knsCtxtBss` | SRAM2 | Soft reset (mais reinit chaque boot via Sram2_Init) |
| `.retentionRamData` / `.retentionRamBss` | SRAM2 | NRST / IWDG / Soft (via SRAM_RST option byte) |
| `.lpmSection` / `.msgCntSectionData` | TAMP BKP regs | Tout sauf VBAT off |

### MAC init flow (KNS_MAC_init)
1. `MCU_NVM_getRadioConfZonePtr()` → pointeur vers buffer 16 bytes
2. `MCU_AES_init(specific_key)` puis `MCU_AES_decrypt()` → radio config décryptée
3. Stack configure SubGHz avec freq/modulation extraite
4. MAC ready quand `KNS_MAC_evt_t::id == KNS_MAC_INIT_DONE`

### Wrappers MCU critiques à vérifier
- `MCU_MISC_getSettingsHwRf()` — DOIT retourner correct RF level
- `MCU_MISC_turn_on_pa()` / `turn_off_pa()` — PA control
- `MCU_AES_init/decrypt()` — RADIO CONFIG DECRYPT
- `MCU_NVM_getMC()/setMC()` — message counter
- `MCU_TIM_*` — timer/RTC

### Radio config zone (16 bytes AES)
- Source 1: Hardcoded statique `radioConfZone[16]` dans `mcu_nvm.c:65-102`
- Source 2: Flash zone à `FLASH_USER_START_ADDR + FLASH_RADIOCONF_OFFSET` (si valide)
- Fallback automatique → `MGR_NVM_reset()` ne peut pas casser ça (touche que NVM_Config app)

---

## 2. Kineis Doppler — Lecture key findings

(Source: `.claude/kineis/Kineis_doppler_loc.txt`)

### Pour le firmware tracker tortue
- **Clustering**: la constellation Kinéis groupe les mesures intelligement → notre job = TX **régulier** dans une fenêtre surface (pas besoin TX optimisé pour 1 sat précis)
- **Frequency drift**: géré côté satellite → TCXO stable suffit, pas besoin compensation firmware
- **Visibility**: 10-15 min entre clusters dans la constellation 2024+ → notre cooldown TX cohérent avec ça
- **Performance**: -50% erreur position vs ancien Argos. Notre job c'est juste de **TX quand on est en surface**.

### Implication pour notre design
- First-TX-on-surface ASAP (déjà fait via cached battery + cached radio cfg + FIRST_TX_RANDOM_WINDOW_MS)
- Repetitions selon TX_GROWTH_PERCENT (déjà fait)
- Pas besoin de TX pendant les passages SAT spécifiques — le clustering gère

---

## 3. ADC — Audit complet (vs DS13105 datasheet)

### Pinout VÉRIFIÉ contre datasheet
- **PA11 = ADC_IN7** (SWS) ✓ correct dans `adc.c:45`
- **PB13 = ADC_IN0** (BAT_SENSE via divider 120k/300k) ✓ correct dans `mgr_bat.c:102`
- L'agent audit s'est planté en affirmant `PB13 = ADC_IN5`. La vraie réf datasheet ligne 3359: `PB13 ... ADC_IN0`.

### Bugs RÉELLEMENT présents

#### P0 — Pas de DeInit ADC avant STANDBY/SHUTDOWN
**Fichier:** `Kineis/Lpm/Src/lpm.c` (LPM_standby_enter, LPM_shutdown_enter)
**Problème:** Seul `LPM_stop_enter` appelle `MX_ADC_DeInit()`. Les paths STANDBY/SHUTDOWN laissent ADC actif → clock toujours fournie → consommation parasitaire + état indéterminé au prochain boot.
**Fix:** Ajouter `MX_ADC_DeInit()` dans les 2 callbacks LPM concernés.

#### P0 — VREFINT internal path jamais activé
**Fichier:** `adc.c` MspInit (manque `__HAL_ADC_ENABLE_VREFINT()` ou écriture `ADC_CCR.VREFEN=1`)
**Problème:** `mgr_bat.c:87` lit `ADC_CHANNEL_VREFINT` mais le buffer interne VREFINT n'est pas connecté → `raw_vref` = 0 ou random → calcul VBAT faux.
**Fix:** Activer VREFINT dans MspInit OU avant chaque read VREFINT.

#### P1 — Returns ignorés
- `adc.c:42` `HAL_ADCEx_Calibration_Start(&hadc)` return non checké
- `adc.c:49-51` `HAL_ADC_ConfigChannel` return ignoré (return silencieux du wrapper)
- **Fix:** Logger ou `Error_Handler()` (à choisir selon politique).

#### P2 — Sampling time channel-specific manquant
Common1/Common2 réglés à 160.5 cycles. OK pour SWS (PA11 source impedance basse), mais BAT_SENSE via R4=120k/R5=300k a impédance source ~80kΩ → besoin sampling >>160.5 cycles à 12-bit. Risque: lecture VBAT inexacte.

### Le wrapper `Core/Src/adc.c` ajouté manuellement par l'utilisateur — verdict
- HAL ADC driver existe bien dans `Drivers/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_adc.c` (toujours présent dans le bundle ST)
- Ce qui manquait par défaut dans le projet généré Kineis: `Core/Src/adc.c` / `Core/Inc/adc.h` (le wrapper applicatif au-dessus du HAL)
- L'ajout par l'utilisateur est **correct dans le principe** mais avec les 3 P0/P1 ci-dessus à corriger.

---

## 4. SWS — Comparaison vs linkit-v4-core latest

(Source: `docs/reports/linkit_v4_sws_magnet.md` + `.claude/sws_analog_implementation.md`)

### État du port linkit-v4 dans `mgr_sws.c`
- ✅ 5-level detection (L1-L5) déjà porté
- ✅ AIR_BASELINE_FLOOR, stuck-state recovery, anti-spike, coherence, adaptive delay, etc.
- ✅ Toutes les hardenings Apr-2026 présentes
- ⚠️ Refactor upstream: linkit a éclaté en 4 fichiers (`sws_analog_{constants,detection,calibration,service}.{hpp,cpp}`)

### Divergences numériques
| Constante | mgr_sws.c | linkit-v4 main | Action |
|---|---|---|---|
| `L4_DROP_PERCENT` | **8%** | **15%** | Aligner à 15% (notre version est sur-sensible) |
| `pulse_on_max_us` | 1000µs | **10000µs** | Étendre pour biofouling sévère |
| `L1_DROP_PERCENT` | 4 | 4 | OK |
| `L3_DROP_PERCENT` | 4 | 4 | OK |
| `L5_DROP_PERCENT` | 10 | 10 | OK |

### Fonctionnalités linkit-v4 ABSENTES côté Kineis
- **Guided calibration** : machine `CalibPhase` 8 états avec LED feedback (GREEN/BLUE patterns)
- **Test mode** : `start/stop/timeout` API pour validation HW
- **Diagnostics struct** CRC-protégée: 7 compteurs (stuck_recovery, coherence_recalib, dive_timeout, force_surface, spike_reject, peak_incoherent, saadc_init_retry)
- **Callbacks**: `set_status_notify`, `set_heartbeat_thresholds_sec`
- **Hints opérateur**: `CAL_OFFSET_HINT_AIR/WATER` exposés dans commande `SWS.CAL`

---

## 5. Magnet/Reed protocole 2-gesture — ABSENT

(Source: `docs/reports/linkit_v4_sws_magnet.md`)

### État actuel firmware (mgr_reed.c)
- EXTI ON/OFF brut
- Hold duration mesurée
- Classification basique SHORT/MEDIUM/LONG_SHUTDOWN

### Ce qui manque (et que linkit-v4 implémente)
- **FSM ReedSwitchGesture**: `{ENGAGE, SHORT_HOLD@3s, LONG_HOLD@6s, RELEASE}`
- **Enum ConfirmationPending**: `{NONE, ENTER_CONFIG, EXIT_CONFIG, POWEROFF}`
- **Window confirmation** 2000ms via `post_task_prio`
- **Re-ENGAGE** dans la fenêtre = confirme
- **Patterns LED** flash 50ms (BLUE pour CONFIG question, GREEN pour OPERATIONAL question, RED pour SHUTDOWN question)

### Protocole cible (validé par utilisateur)
```
POWER_OFF (SHUTDOWN, only reed wakes)
  → Magnet ON → LED blanche solide → 5 blinks verts (slow) → OPERATIONAL

OPERATIONAL
  → Magnet 3s → fast blink BLEU (question CONFIG?)
     → user OFF→ON dans 2s → slow blink BLEU → CONFIG
     → sinon → retour OPERATIONAL

CONFIG
  → Magnet 3s → fast blink VERT (question OPERATIONAL?)
     → user OFF→ON dans 2s → slow blink VERT → OPERATIONAL
  → Magnet 6s+ → fast blink ROUGE (question SHUTDOWN?)
     → user OFF→ON dans 2s → LED off → SHUTDOWN
```

---

## 6. État des "safety patches" debug actuellement actifs

(Ces patches sont des HACKS posés pour stabiliser pendant debug. À retirer après vrai fix.)

| Patch | File | Raison | À supprimer une fois? |
|---|---|---|---|
| `boot_loop_handle` force-clear | kns_app_uw_doppler.c:192 | Empêche SHUTDOWN perma-off | Fix MAC + auto-clear sur cause BOR/PIN |
| `MGR_WDG_init()` commenté | kns_app_uw_doppler.c | IWDG_STOP option byte not set | Set IWDG_STOP option byte OU refresh from SysTick |
| `MGR_ERR_checkCrashLoop` force-clear | mgr_err.c:175 | Évite STOP2 1h | Fix root cause crashes |
| `LPM_setForcedMode(STOP)` commenté | kns_app_uw_doppler.c | Empêchait MAC init | Laisser MAC manage LPM (déjà fait par défaut HEAD) |
| TCXO restoré HEAD (warmup 2000ms, FORCE_STATE=1) | mcu_misc.c | Mon changement avait cassé MAC | Garder, c'est le bon état |

---

## 7. Plan de fix proposé (par ordre)

### Phase 0 — Stabiliser et nettoyer
1. Réécrire l'ADC report propre dans `docs/reports/adc_audit.md` (l'actuel a des erreurs)
2. Garder les safety patches actifs jusqu'à Phase 2 complétée
3. Écrire `CLAUDE.md` (règles agent) + `docs/PROJECT_RULES.md`

### Phase 1 — ADC fix (devrait débloquer les reboots)
1. Ajouter `MX_ADC_DeInit()` dans `LPM_standby_enter` et `LPM_shutdown_enter`
2. Activer VREFINT internal path dans MspInit
3. Logger les returns ADC ignorés
4. Tests unitaires: mock HAL_ADC, vérifier séquence init/deinit
5. Build + flash + tester si plus de reboot mystérieux

### Phase 2 — SWS aligner sur linkit-v4
1. Bumper `L4_DROP_PERCENT` 8→15%
2. Étendre `pulse_on_max_us` 1000→10000µs
3. Ajouter Diagnostics struct CRC-protégée
4. Tests unitaires correspondants

### Phase 3 — Restaurer guards proprement
1. Re-activer `MGR_WDG_init` mais SEULEMENT après transition MONITORING
2. `boot_loop_handle` clear sur cause BOR/PIN (NRST manuel = user reset OK)
3. `MGR_ERR_checkCrashLoop` re-activer (devrait être inutile avec MAC fixé)

### Phase 4 — 2-gesture magnet
1. Créer `MGR_GESTURE` (nouveau module) au-dessus de `MGR_REED`
2. Implémenter FSM + LED patterns
3. États OPERATIONAL/CONFIG persistés en TAMP BKP
4. Tests unitaires: simulation EXTI events + tick

### Phase 5 — Reporter Kineis Doppler firmware-side
- Vérifier FIRST_TX strategy alignement Doppler timing
- Pas grand chose à changer (déjà OK)

---

## 8. Questions ouvertes pour utilisateur

1. **L4_DROP_PERCENT**: tu valides l'alignement à 15%? L'utilisateur précédent a peut-être délibérément descendu à 8%.
2. **2-gesture magnet sortie OPERATIONAL → SHUTDOWN**: dispo aussi depuis OPERATIONAL ou seulement depuis CONFIG?
3. **MGR_GESTURE** : nouveau module séparé OU intégration dans `MGR_REED` directement?
4. **Logs flash quand DEPLOYED**: PMLOG actuel suffit ou besoin EVTLOG aussi mirroré en flash?
5. **Test framework**: garder `test_framework.h` minimaliste + ajouter génération HTML, OU migrer vers Unity (qui a un rapport HTML built-in)?

---

**Fin du master audit.** Tous les rapports détaillés:
- `docs/reports/adc_audit.md` (à réécrire)
- `docs/reports/linkit_v4_sws_magnet.md`
- `docs/reports/kineis_pdfs_summary.md` (à réécrire)
- `.claude/linkit-uw-behavior.md`
- `.claude/sws_analog_implementation.md`
