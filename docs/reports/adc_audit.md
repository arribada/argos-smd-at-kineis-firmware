# Audit ADC — argos-smd-at-kineis-firmware

Branche : `v2-fix-lpm`. Auditeur : Claude (Opus 4.7 1M). Cible : STM32WL55 (Cortex-M4).

Périmètre : `Core/Src/adc.c`, `Core/Inc/adc.h`, `Kineis/App/Managers/MGR_BAT/Src/mgr_bat.c`,
`Kineis/App/Managers/MGR_SWS/Src/mgr_sws.c`, `Kineis/Lpm/Src/lpm.c`, `Core/Src/main.c`.

## A. Séquence d'init

| Item | Statut | Localisation |
|------|--------|--------------|
| RCC ADC clock enable | PASS | `Core/Src/adc.c:64` (`__HAL_RCC_ADC_CLK_ENABLE`) |
| GPIO ANALOG avant init | PASS | `Core/Src/adc.c:67-71` (PA11 dans MspInit) |
| Résolution 12 bits | PASS | `Core/Src/adc.c:19` |
| Calibration (`HAL_ADCEx_Calibration_Start`) | WARNING | `Core/Src/adc.c:42` — appelée mais **valeur de retour ignorée** et **paramètre `SingleEnded` manquant** (la signature WL est `(ADC_HandleTypeDef*)` donc OK, mais l'absence de check masque tout échec) |
| Common config / VREFINT path | **FAIL** | `Core/Src/adc.c` — aucun appel à `__HAL_RCC_SYSCFG_CLK_ENABLE` + `HAL_ADCEx_EnableVREFINT`/`ADC_CCR.VREFEN`. `mgr_bat.c:87` configure `ADC_CHANNEL_VREFINT` mais le chemin interne n'est **jamais activé** → lecture vrefint à 0 ou aléatoire → `vdda_mV` divisé par ~0 → overflow puis `last_good_vbat_mV` retourné en boucle |
| Sampling time channel | WARNING | 160.5 cycles OK pour le diviseur 120k/300k (Zsource ≈ 86 kΩ) côté VBAT, mais marginal pour VREFINT (recommandé ≥ 4 µs = 160 cy @ 16 MHz → OK ; mais ADC est cadencé `PCLK/4`, soit possiblement plus rapide → revérifier `RCC_PCLK1`) |

## B. Réentrance / LPM

| Item | Statut | Localisation |
|------|--------|--------------|
| `MX_ADC_DeInit` avant STOP | PASS (partiel) | `Kineis/Lpm/Src/lpm.c:332` — appelé sous `USE_UW_DOPPLER_APP` |
| Re-init après wakeup STOP | PASS | `lpm.c:397` |
| DeInit avant STANDBY/SHUTDOWN | **FAIL** | `lpm.c:422-465` (STANDBY) et `lpm.c:469-492` (SHUTDOWN) — `MX_ADC_DeInit()` **n'est jamais appelé**. ADC reste activé → courant résiduel + risque de pin PA11 non basculée en analog propre |
| GPIO PA11 / PA12 / PB13 / PB9 vers ANALOG en LPM | **FAIL** | `lpm.c:566-568, 580-582` — PA11/PA12 sont **exclus** de la mise en analog (`#if !defined(USE_UW_DOPPLER_APP)`). `lpm.c:611` ne met pas PB13 en analog (déjà fait), mais **PB9 (VBAT_EN) reste en sortie push-pull** → si MOSFET Q2 reste actif, divider consomme `VBAT/(R4+R5) = 3.7V/420k ≈ 9 µA` en STOP/STANDBY (gros leak) |
| Conflit VREFINT vs SubGHz | WARNING | Le SubGHz utilise sa propre VREFBUF interne, pas de conflit direct, mais VREFINT_CAL est lu sans avoir activé le chemin interne — voir §A |
| Double init de l'ADC | **FAIL** | `Core/Src/main.c:736` (UW_DOPPLER) + `main.c:905` (DOPPLER) — appels exclusifs par #if, OK ; mais `MX_ADC_Init` est aussi appelé au wakeup STOP **sans avoir déinit** sur le chemin SLEEP (HAL refuse silencieusement si déjà init, mais le handle peut être laissé dans un état incohérent) |

## C. Multi-canal

| Item | Statut | Localisation |
|------|--------|--------------|
| Canaux : SWS=PA11=IN7, VBAT=PB13 | **FAIL CRITIQUE** | `mgr_bat.c:8, 102` annonce et utilise **`ADC_CHANNEL_0`** pour PB13. **Sur STM32WL55, PB13 = ADC_IN5**, pas IN0. IN0 = PB1. → La lecture VBAT échantillonne en réalité une **broche floating ou autre signal** (selon ce qu'est PB1 sur la board). C'est aussi pourquoi le commentaire schématique (2.91 V max attendu sur PB13) ne correspond probablement pas aux valeurs lues. À corriger en `ADC_CHANNEL_5`. |
| Switch de canal entre BAT et SWS | PASS | `mgr_bat.c:117-120` restaure canal 7 même sur erreur |
| Attente ADRDY entre switch | WARNING | `HAL_ADC_ConfigChannel` ne provoque pas d'attente ADRDY ; HAL_ADC_Start le fait implicitement, OK |
| Timeout / gestion erreur | PASS | `adc.c:91` (100 ms), `mgr_bat.c:94,109` (100 ms) |

## D. Chemin de lecture

| Item | Statut | Localisation |
|------|--------|--------------|
| Start → Poll EOC → GetValue → Stop | PASS | `adc.c:87-99` |
| Timeout raisonnable | PASS | 100 ms (largement suffisant : 1 conv ≈ 12 µs @ 16 MHz) |
| Gestion ADC busy / calib en cours | **FAIL** | Aucune vérification de `HAL_ADC_STATE_BUSY_INTERNAL` ni de `__HAL_ADC_GET_FLAG(ADRDY)`. Si la calibration retourne une erreur (ignorée), le handle reste en `HAL_ADC_STATE_ERROR_*` et les appels suivants retournent silencieusement 0 |
| `HAL_ADC_Init` valeur de retour | **FAIL** | `adc.c:37-39` : `return` muet sur erreur, suivi d'un `HAL_ADCEx_Calibration_Start` sur un handle non-init → potentiel hard fault / écriture registre invalide → **cause plausible de reboot** |

## E. Math diviseur VBAT

Schéma : `VBAT * 300k / (120k+300k) = VBAT * 0.714` → 4.2 V → 3.0 V (cap), 3.2 V → 2.28 V.

`mgr_bat.c:142-145` :
```
vbat_mV = raw_vbat * vdda_mV * (R4+R5) / (4095 * R5)
        = raw_vbat * 3300 * 420 / (4095 * 300)
```

| Item | Statut | Note |
|------|--------|------|
| Ratio diviseur | PASS | 420/300 = 1.4 ✓ |
| Calcul VDDA via VREFINT | **FAIL** | Voir §A : chemin VREFINT non activé → `raw_vref` peu fiable, donc `vdda_mV` faux ; même si OK, le canal IN0 utilisé n'est pas PB13 → résultat doublement faux |
| Activation BAT_SENSE_EN (PB9) | PASS | `mgr_bat.c:83, 123` HIGH avant / LOW après |
| Délai de settling 2 ms | PASS | Suffisant pour C parasite × 86 kΩ Thévenin |
| Cache "last_good" sur échec | PASS | `mgr_bat.c:125-129` ; mais masque les bugs ci-dessus en production |

## F. Comparaison vs HAL ADC référence

Manquent par rapport à un exemple `ADC_RegularConversion_Polling` Cube :
- `HAL_ADC_GetState()` non vérifié.
- `HAL_ADC_DeInit` avant ré-init n'est pas systématique.
- Pas de gestion ADC overrun (le flag ADC_OVR_DATA_OVERWRITTEN est OK mais aucun log si EOC manqué).
- Pas d'`HAL_ADCEx_DisableVREFINT()` au DeInit → si on l'active un jour, leak interne.

---

## Liste hiérarchisée des correctifs

### P0 — provoque reboots / corruption MAC
1. **`adc.c:37-42` — `HAL_ADC_Init` échec ignoré puis `HAL_ADCEx_Calibration_Start` sur handle invalide.** Provoque accès registre sans clock garantie → bus fault → reboot (compatible avec la suspicion utilisateur). Correctif : `if (HAL_ADC_Init(...) != HAL_OK) Error_Handler();` et idem pour calibration.
2. **`mgr_bat.c:102` — `ADC_CHANNEL_0` au lieu de `ADC_CHANNEL_5` pour PB13.** Lecture sur une broche non destinée → valeur erratique ; si PB1 est laissée flottante par `MX_GPIO_Init`, le bus interne ADC peut transitoirement laisser passer du couplage SubGHz/PA → glitches de MAC (le MAC consomme la valeur batterie pour gating TX).
3. **`lpm.c:422-492` — Pas de `MX_ADC_DeInit()` ni de mise à zéro de `__HAL_RCC_ADC_CLK_DISABLE` avant STANDBY/SHUTDOWN.** L'ADC peut maintenir une référence active, et au wakeup STANDBY (qui passe par reset complet), `MX_ADC_Init` est rappelé sur un peripheral éventuellement en état indéterminé → calibration faux positif → reboot crash-loop.

### P1 — valeurs fausses
4. **`mgr_bat.c:87` — VREFINT lu sans `HAL_ADCEx_EnableVREFINT()` ni `ADC_CCR.VREFEN`.** Le canal interne retourne 0/random → `vdda_mV` aberrant.
5. **`adc.c:32-33` — Sampling time identique pour les deux registres `SamplingTimeCommon1/2`.** Pas un bug, mais empêche d'avoir un sampling court (SWS) et long (VBAT haute impédance) ; conseillé : Common2 = max (640.5 cy) pour VBAT.
6. **`adc.h:30` doc — annonce "0 on error" mais ne distingue pas erreur vs lecture nulle valide.** Préférer `bool ADC_ReadValue(uint32_t *out)`.

### P2 — non optimal côté énergie
7. **`lpm.c:611` — PB9 (VBAT_EN) n'est jamais désactivée en STANDBY/SHUTDOWN par `HAL_PWREx_EnableGPIOPullDown`.** Si l'état push-pull précédent était HIGH (race condition à l'entrée STANDBY), Q2 conduit pendant tout le STANDBY → ~9 µA continu.
8. **`lpm.c:566-583` — PA11/PA12 exclus de la passage en analog en LPM sous `USE_UW_DOPPLER_APP`.** PA12 reste en push-pull (consommation négligeable si LOW, mais à vérifier) ; PA11 reste configurée comme analog par MspInit, OK — mais clock ADC pas coupée (§B).
9. **`mgr_bat.c:84` `HAL_Delay(2)` bloquant** au sein d'une mesure batterie potentiellement appelée depuis idle/SWS task → +2 ms de wake/jitter.

---

*Fin du rapport — ~600 mots.*
