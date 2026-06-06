# Comparaison linkit-v4-core (main, juin 2026) vs firmware Kineis

Source : `https://github.com/arribada/linkit-v4-core` (branche `main`, public).
Note : clone Git impossible dans cet environnement (Bash et PowerShell désactivés).
Les fichiers ont été récupérés via `raw.githubusercontent.com` et analysés à
distance. Comparaison faite avec :

- `Kineis/App/Managers/MGR_SWS/Src/mgr_sws.c`
- `Kineis/App/Managers/MGR_REED/{Inc,Src}/mgr_reed.{h,c}`
- `.claude/sws_analog_implementation.md`
- `.claude/linkit-uw-behavior.md`

---

## 1. SWS — Analyse algorithmique (~400 mots)

### Refactor structurel (nouveau)

Le service SWS a été éclaté en trois fichiers :

- `core/services/sws_analog_constants.hpp` (toutes les `#define`)
- `core/services/sws_analog_calibration.cpp` (calibration / persistance flash)
- `core/services/sws_analog_detection.cpp` (algorithme 5 niveaux)
- `core/services/sws_analog_service.cpp` (orchestration, test-mode, diagnostics)

Le `.claude/sws_analog_implementation.md` local décrit encore l’ancienne
architecture mono-fichier (`sws_analog_service.cpp` ~856 lignes) avec 3 tiers
T1/T2/T3. **Cette doc est obsolète.** Le mgr_sws.c (en-tête « 5-Level Surface
Detection ») reflète déjà la nouvelle architecture L1–L5, donc le portage de
base est aligné.

### Différences de constantes (importantes)

| Constante | doc / mgr_sws.c local | linkit-v4 `main` actuel | Impact |
|---|---|---|---|
| `DEFAULT_HYSTERESIS_PERCENT` | 14 % (doc obsolète) / 4 % (mgr_sws.c) | **4 %** | OK, mgr_sws déjà aligné |
| `L4_DROP_PERCENT` | **8 %** (mgr_sws.c) | **15 %** | **À porter** : seuil L4 plus tolérant côté linkit |
| `L1_DROP_PERCENT` | 4 % | 4 % | OK |
| `L2_DROP_PERCENT` / step | 3 % / 2 % | 3 % / 2 % | OK |
| `MIN_WATER_AIR_RATIO` | 3 (mgr_sws) | 3 | OK (doc dit 5 — obsolète) |
| `AIR_RECALIB_EMA_WEIGHT` | 15 % | **0.15f (15 %)** | OK |
| `AIR_RECALIB_MAX_RATIO` | 70 % | **0.70f (70 %)** | OK |

### Mécanismes nouveaux côté linkit-v4 absents/partiels chez nous

1. **Guided calibration** : machine à états `CalibPhase` (IDLE → AIR_WAITING →
   AIR_SAMPLING → AIR_DONE_PAUSE → WATER_WAITING → WATER_SAMPLING →
   WAIT_RESURFACE_FOR_ACK → COMPLETION_PAUSE → DONE) avec feedback LED
   (GREEN=air, BLUE=eau) et timeout `GUIDED_CALIB_TIMEOUT_TICKS = 300`.
   **Absent côté mgr_sws** (le firmware Kineis n’a qu’une calibration auto).
2. **Test mode** : `start_test_mode/stop_test_mode/set_test_timeout_ms`,
   période fixe `SWS_TEST_MODE_SAMPLE_MS = 100 ms`, auto-stop pour éviter la
   décharge batterie. Une variante existe (`test_interval_*_ms` dans
   `MGR_SWS_Config_t`) mais pas l’API démarrage/arrêt explicite.
3. **Diagnostics compteurs CRC-protégés** : `stuck_recovery_count`,
   `coherence_recalib_count`, `dive_timeout_count`, `force_surface_count`,
   `spike_reject_count`, `peak_incoherent_count`, `saadc_init_retry_count`.
   Le mgr_sws compte certains événements internes mais n’expose pas la
   structure de diagnostics complète.
4. **Heartbeat thresholds** (`set_heartbeat_thresholds_sec(uw, surf)`) pour
   notifier un superviseur si aucun changement d’état pendant N secondes.
   Absent côté Kineis.
5. **Notifications externes** (`set_status_notify`,
   `set_guided_calib_notify`, `set_on_test_stop`) — callbacks vers BLE/DTE.
6. **Persistance Calibration class via fichier `SWS.CAL`** avec offsets
   `CAL_OFFSET_HINT_WATER/HINT_AIR/RUN_WATER/RUN_AIR/PEAK`. mgr_sws sauvegarde
   en NVM mais sans hints opérateur.

### Sécurités identiques (déjà portées correctement)

`AIR_BASELINE_FLOOR`, `AIR_BASELINE_RECOVER`, `THRESHOLD_MIN_ABOVE_AIR`,
`PROXIMITY_GUARD_PERCENT` (95 / 99 biofouling), `OVERRIDE_MIN_TIME_SEC = 1`,
`SURFACE_LOCKOUT_DURATION_SEC = 30`, `MAX_CONSECUTIVE_DIVE_TIMEOUTS = 3`,
`AIR_COLLAPSE_RECOVERY_SAMPLES = 5`, contrast-adaptive sample delay
(200 µs – 10 ms côté linkit, 200 µs – 1 ms chez nous → **plage max plus
courte chez Kineis**, à monter à 10 ms si biofouling sévère).

---

## 2. Magnet / Reed — Protocole 2-gestes (~300 mots)

### Architecture linkit-v4

- `core/hardware/reed.hpp` : classe `ReedSwitch` avec enum
  `ReedSwitchGesture { ENGAGE, SHORT_HOLD, LONG_HOLD, RELEASE }`.
  Constructeur prend `(Switch&, short_hold_ms=3000, long_hold_ms=6000)`.
- À l’ENGAGE, deux tâches scheduler sont armées :
  `SHORT_HOLD` après `short_hold_ms`, puis `LONG_HOLD` après
  `(long_hold_ms - short_hold_ms)` supplémentaires. À RELEASE, les tâches
  pendantes sont annulées (`system_scheduler->cancel_task(m_task)`) et un
  RELEASE est émis. Callback : `m_user_callback(gesture)`.

### State machine `GenTracker` (core/sm/gentracker.{hpp,cpp})

Enum `ConfirmationPending { NONE, ENTER_CONFIG, EXIT_CONFIG, POWEROFF }`.
Constante clé : `CONFIRMATION_TIMEOUT_MS = 2000` (fenêtre de re-engage).

Protocole 2-gestes (handler `react(ReedSwitchEvent)`) :

```
SHORT_HOLD (3 s) → m_confirmation_pending = ENTER_CONFIG (si Operational)
                                          ou EXIT_CONFIG (si Configuration)
LONG_HOLD  (6 s) → m_confirmation_pending = POWEROFF
RELEASE          → si pending != NONE, démarre la fenêtre 2 s
                   (m_awaiting_re_engage = true, post_task_prio 2000 ms)
ENGAGE (<2 s)    → si m_awaiting_re_engage : transition exécutée
                                              (transit ConfigurationState /
                                               PreOperationalState / OffState)
Timeout (>2 s)   → cancel_confirmation() : pending = NONE,
                                            LED restaurée selon état FSM
```

### Feedback LED (`core/sm/ledsm.cpp`)

Trois patterns « question » (clignotement rapide 50 ms on/off, hardcodé) :
- `SetLEDConfirmConfig` : **BLUE flash 50 ms** (entrée config)
- `SetLEDConfirmExitConfig` : **GREEN flash 50 ms** (sortie config)
- `SetLEDConfirmPowerOff` : **RED flash 50 ms** (power-off)

Le magnet engagé/désengagé hors confirmation affiche un solide WHITE
(`m_is_magnet_engaged`).

Note : la doc demandait un « fast blink = question / slow blink = confirm »
— en réalité linkit-v4 utilise **uniquement le fast blink 50 ms pour
poser la question**, la confirmation est silencieuse côté LED mais déclenche
un buzzer (`SetBuzzPowerDown`, etc.). Aucun « slow blink » 120 ms n’est
défini comme constante (50 ms est codé en dur dans chaque `entry()`).

### État côté Kineis (`MGR_REED`)

`mgr_reed.{c,h}` n’expose que 2 événements bruts (`MAGNET_ON`/`OFF`) +
`getLastHoldDuration_ms()`. **Tout le découpage ENGAGE/SHORT_HOLD/LONG_HOLD,
la fenêtre de confirmation 2 s, la machine ConfirmationPending et le mapping
LED restent à implémenter.** L’usage actuel (cf. commentaire mgr_reed.c)
n’est qu’un seuil « hold > 10 s → shutdown », sans 2-gestes ni feedback
question/confirmation.

---

## 3. Références fichiers

- `c:/Users/fourn/Programmation/CProjects/argos-smd-at-kineis-firmware/Kineis/App/Managers/MGR_SWS/Src/mgr_sws.c`
- `c:/Users/fourn/Programmation/CProjects/argos-smd-at-kineis-firmware/Kineis/App/Managers/MGR_REED/Src/mgr_reed.c`
- `c:/Users/fourn/Programmation/CProjects/argos-smd-at-kineis-firmware/Kineis/App/Managers/MGR_REED/Inc/mgr_reed.h`
- `c:/Users/fourn/Programmation/CProjects/argos-smd-at-kineis-firmware/.claude/sws_analog_implementation.md` (à mettre à jour : L1–L5 au lieu de T1–T3)
- `c:/Users/fourn/Programmation/CProjects/argos-smd-at-kineis-firmware/.claude/linkit-uw-behavior.md` (idem)

Upstream consulté :
- `core/services/sws_analog_constants.hpp`
- `core/services/sws_analog_detection.cpp`
- `core/services/sws_analog_calibration.cpp`
- `core/services/sws_analog_service.hpp`
- `core/hardware/reed.{hpp,cpp}`
- `core/sm/gentracker.{hpp,cpp}`
- `core/sm/ledsm.cpp`
