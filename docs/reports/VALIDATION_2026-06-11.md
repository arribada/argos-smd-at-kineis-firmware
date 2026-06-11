# Validation autonome 2026-06-11 — UW_DOPPLER livrable

Session bench autonome (nuit). Carte SMD_STDALONE sur COM3, JLink, firmware
`v0.8.1` / lib Kineis `v11.1.0_e9373c5_0x6Tx`, branche `v2-fix-lpm`.

## 1. Power-off (gesture / AT+SHUTDOWN) — redessiné et validé

Symptômes corrigés (rapportés 2026-06-10) : 25 µA au lieu de 5 µA, reed
inopérant, off/on bloqué en shutdown.

Root cause : SHUTDOWN n'est réveillable QUE par les pins WKUP câblées en
dur (PA0/PC13/PB3) — le reed est sur PB6 → EXTI mort en SHUTDOWN. En plus
le pulldown PWR sur PB7 (40 kΩ) se battait contre la pull-up externe du
latch : diviseur ≈ 25 µA permanent + latch jamais relâché + `PWR_PDCRB`
persistant à travers NRST (= "off/on reste en shutdown").

Nouveau design (`MGR_LPM_UW_enterShutdownReed`, mgr_lpm_uw.c) :
- PB7 drivé LOW activement (STOP2 retient le drive, pas de diviseur).
- Phase 1 : attente en STOP2 du retrait de l'aimant de confirmation.
- Sur batterie : le régulateur coupe au retrait → vrai off (plancher HW),
  réveil = re-latch reed → POR → OPERATIONAL.
- Si VDD survit (bench/USB) : PB7 relâché, boucle STOP2 (~1-2 µA MCU),
  réveil = EXTI reed PB6 → reset → OPERATIONAL + wake blink (marqueur
  TAMP BKP11R).
- Hygiène boot : `PWR->PDCRB &= ~(1<<7)` dans main.c (purge le piège).
- **Validé au bench** : magnet wake fonctionnel (confirmé par l'utilisateur).
- 25 µA résiduels mesurés = plancher STOP2 carte entière (quiescent
  TPS63901 + LSE/RTC) quand VDD est maintenu — incompressible par firmware.

Option **`make REED_WKUP3=1`** (+ fil reed→PB3) : vrai SHUTDOWN réveillable
par WKUP3, plancher sub-µA MCU. Hex prêt : `c:/tmp/fw_uw_reed_wkup3.hex`.
À tester quand le fil est posé.

## 2. Latence détection SURFACE ("ping rapide") — MESURÉE, requirement tenu

Instrumentation on-target `[LAT]` (ticks HAL) :

| Étape | Mesure |
|---|---|
| Réveil STOP2 → sample SWS → détection SURFACE → push MAC | **48 ms** |
| Push MAC → TX radio terminé | **1.66 s** |
| Cadence échantillonnage sous l'eau (STOP2 cyclé) | 1 s (uw_interval=500 ms arrondi au RTC 1 Hz) |
| **Surfaçage physique → message en l'air (pire cas)** | **≈ 2.5 s** |

LPM OFF (mesure host) : commande surface → TX done = 2.5 s.
La séquence est correctement AVORTÉE si replongée détectée pendant le TX.

## 3. Séquences TX + timers — VALIDÉ bench (simulation dive/surface)

Méthode : seuils SWS déplacés par AT+SWSCFG (l'ADC à sec ~245 devient
UNDERWATER avec th=55, SURFACE avec th=400). Limite connue : le moteur
adaptatif reconverge en ~2-3 samples (state-flap) — les fenêtres courtes
sont fiables, les longues reconvergent. En conditions réelles l'ADC bouge
physiquement, pas de flap.

- UW→SURFACE → `New sequence, MC=133` en 1.7 s, 3 TX espacés 10 s/15 s
  (interval=10, growth=50%, jitter=0), cap à 3 → arrêt. ✓
- `tx_seq_restart_s=45` → "Seq restart timer fired" à exactement 45 s du
  dernier TX → nouvelle séquence. ✓
- Replongée pendant séquence → abort. ✓

## 4. Message Counter per-sequence — VALIDÉ bench

- Séquence 1 : 3 TX, tous MC=133 ; AT+MC après = 134. ✓
- Séquence 2 (seq-restart) : MC=134 ; après = 135. ✓ (+1 par séquence,
  pas par message — incrément naturel via la lib, pas de double add)
- Boot : seed depuis NVM (KNS_CFG_getMC). ✓
- Couverture unitaire : `Tests/unit/test_seq_mc.c` (13 checks).

## 5. Boot = premier event surface — NOUVEAU, validé bench

`boot_first_seq_pending` (kns_app_uw_doppler.c) : à tout boot OPERATIONAL
frais (power-on, magnet, NRST) avec SWS=SURFACE → séquence TX immédiate
sans attendre une transition plongée/surface. Consommé une fois ; exclu
sur wake STANDBY (le duty-cycle garde la main). Observé au boot :
TX #1..#3 envoyés. ✓

## 6. Commandes AT — régression complète OK

VERSION=v0.8.1, SWSCFG, TXCFG (7 champs), LPMTHR, DUTYCFG, MC, SWS,
STATUS, KMAC (=1 BASIC, refus 2/3/4), BATCFG, RATECFG, LBCFG, DEPLOY,
SAVE, SWSFORCE, PING — toutes répondent et persistent comme attendu.

**Point UX majeur identifié et corrigé** : avec LPMTHR activé, la carte
était sourde aux AT ~98 % du temps (LPUART 115200 sur HSI16 = pas de
réception en STOP2 ; fenêtre ~50 ms par réveil ; mesuré : 1 commande
passée sur ~40 tentatives). C'est la "casse" ressentie après les fix LPM.
**Fix : fenêtre de grâce console** — toute commande AT décodée tient le
sommeil profond à distance pendant 30 s (`LPM_UW_AT_GRACE_MS`,
mgr_at_cmd.c `MGR_AT_CMD_getLastActivityTick` + gate dans
`MGR_LPM_UW_idleTick`). Zéro impact énergie en déploiement scellé (pas de
trafic AT). En bench : première commande à répéter jusqu'à atterrissage
(ou reset → 30 s de fenêtre), ensuite session interactive.

## 7. Low-power auto — vérifié

- Surface idle : `STOP2 5s` (= surf_interval NVM). ✓
- Underwater : `STOP2 1s` (uw_interval 500 ms arrondi). ✓
- SLEEP tier LPTIM pour < 500 ms, spin < 10 ms (AT+LPMTHR=10,500,1). ✓
- Pendant séquence TX : pas de sleep (timers MAC actifs). ✓
- Mesure courant : non accessible depuis le bench (pas d'ampèremètre
  pilotable) — à confirmer à la prochaine mesure manuelle.

## 8. Suite de tests + build matrix

- `Tests/scripts/run_tests.sh` : **26/26 suites, 395 checks OK** (dont
  nouveau test_seq_mc).
- Build matrix : **13/13 combos valides OK** — STDLN/GUI/DOPPLER ×
  {SMD_PA, SMD_NOPA, SMD_STDALONE, SMD_OP} + UW_DOPPLER/SMD_STDALONE
  (126 632 o). UW_DOPPLER/SMD_PA = refus `#error` intentionnel (board
  sans reed/LED/latch). ✓

## 9. Incident de nuit (résolu)

Flash interrompu (script JLink manquant après `make clean`) → chip effacé
+ DAP muet. **Récupéré sans intervention** : pulse NRST matériel via JLink
(`r0`/`sleep`/`r1` puis connect, script `c:/tmp/recover_nrst.jlink`).
La ligne NRST est câblée sur le header JLink — plus besoin du bouton.

## 10. Scheduler deadline-based (suite à revue utilisateur, nuit 2)

Demande : « STOP2/SLEEP en fonction du temps jusqu'à la prochaine action,
pas de façon périodique ». L'analyse a révélé 3 défauts structurels :

1. **Périodique au lieu d'événementiel** : delta = intervalle SWS fixe,
   ignorait l'échéance du prochain TX et du seq-restart.
2. **Tick non compensé** : HAL_GetTick gelé en STOP1/STOP2 → tous les
   timers logiciels comptaient en temps CPU-actif. Un interval TX de 10 s
   entrecoupé de STOP2 mettait des centaines de secondes réelles à
   « s'écouler ». Le watchdog max_dive_time (7200 s, remontée forcée) ne
   pouvait quasiment jamais tirer. Latent, jamais vu au bench car les
   séquences testées tombaient dans la fenêtre de stabilisation (30 s sans
   sleep) ou tournaient LPM OFF.
3. **Arrondi plafond** : réveil STOP2 jusqu'à 1 s APRÈS l'échéance.

Fix (commit `38be6a0`) : `uw_ms_until_next_action()` agrège min(prochain
sample SWS, prochain TX de séquence, échéance seq-restart) ;
`LPM_saveRtcTime/LPM_compensateTick` exportés de lpm.c et appliqués aux
réveils STOP1/STOP2 (tick = temps réel) ; `MGR_LPM_UW_enterStop2TimedMs`
arme le WUT en RTCCLK/16 (pas de 0.49 ms) sous 29 s et en secondes
PLANCHER au-dessus — ne dort jamais au-delà d'une échéance.

Unitaire : `test_lpm_deadline.c` (13 checks) — agrégation, math DIV16/SPRE,
clamps. **Validé au bench** (carte réalimentée, TXCFG=10,50,180,3,0,10,45,
LPM ON, 2 cycles seq-restart complets observés hors stabilisation) :
- Intervalles TX wall-clock EXACTS avec STOP2 entrecalé : TX à +10.0 s et
  +15.0 s pile malgré 2-3 cycles STOP2 entre chaque TX (compensation tick).
- Sommeils tronqués à l'échéance : `STOP2 4971ms / 4162ms / 3330ms /
  4793ms` — réveil sur la prochaine action, plus de grille périodique.
- Seq-restart à 45 s pile du dernier TX, MC +1 par séquence (139→140),
  pattern reproductible.

## 11. Incident fin de session : carte hors tension

Vers la fin de la nuit, VTref = 0.000 V, COM3 muet — la carte n'est plus
alimentée. Cause la plus probable : batterie vidée par la session (longues
phases LPM désactivé à plusieurs mA + TX + spam AT). Le firmware
deadline-scheduler est compilé/testé mais PAS flashé. À la réalimentation :
`JLink -CommanderScript c:/tmp/recover_nrst.jlink` flashe le hex courant.

## 12. Campagne robustesse (nuit 2 / matin) + 2 bugs P0 trouvés et fixés

### Bug P0 pérennité : usure flash MC (commit `ce218d0`)
La lib persiste le MC par `getMC + setMC(mc+1)` à CHAQUE TX, et
`MCU_NVM_setMC` traversait vers `set_wear_counter` = 5 erases de pages par
appel (page 0 via MCU_FLASH_write + 4 pages WL). À 100 TX/jour : endurance
flash (10k cycles) consommée en 3-9 semaines. Bug PRÉEXISTANT à cette
session (la lib le déclenchait seule) ; le per-sequence MC le doublait.
Fix : cache RAM + high-water mark dans le wrapper (1 slot programmé par
séquence, ~12 erases/an, jamais de répétition MC après crash). Validé
bench (0 écriture destructrice par séquence) + test_mc_wear.c (8 checks,
budget 1 an). Audit usure des autres zones : EVTLOG=SRAM2 (zéro flash),
PMLOG=erreurs seulement, NVM config=AT+SAVE seulement. SAIN.

### Bug P0 robustesse : wedge du tier SLEEP STOP1+LPTIM (commit `d72ddfb`)
Pendant la campagne, la carte s'est retrouvée muette + sourde à ~700 µA,
récupérable uniquement par NRST. Reproduit DÉTERMINISTIQUEMENT : première
entrée dans le tier SLEEP (STOP1+LPTIM) → le WFI ne se réveille jamais ;
IWDG gelé en STOP (option byte) → aucun sauvetage. Piège latent depuis
AT+LPMTHR (mémoire projet : une régression SLEEP similaire avait déjà été
roulée back) ; le scheduler deadline rendait la bande atteignable (restes
< 500 ms). Fix : tier retiré du chemin de production, bande routée vers
STOP2-DIV16 (précision ms + teardown complet + chemin de réveil prouvé).
Re-validé avec le même scénario : cycles `STOP2 499ms` sous l'eau (cadence
SWS 500 ms désormais tenue à la ms — avant arrondie à 1 s), AT réactif.
Bonus : le plancher de courant du band sub-seconde passe de ~700 µA
(STOP1, radio armée) à ~25 µA (STOP2).

### Résultats campagne
| Phase | Résultat |
|---|---|
| R2 — Torture resets (6× NRST, timings variés) | 6/6 boots OK, AT répond, NVM/MC cohérents |
| R3 — Cyclage dive/surface ×6 | 6/6, SWS sain, zéro crash |
| R4 — Long-run 12 min LPM ON | 0 HardFault, 0 CRASH-REPLAY, 0 boot-loop ; séquences + seq-restart nominaux |
| R1 — Fuzzing AT | Non concluant (commandes non parvenues : carte en STOP2, surdité d'accès à froid) — À REFAIRE dans la fenêtre de grâce |
| Wedge SLEEP tier | Trouvé pendant la campagne, root-causé, fixé, re-validé |

### Explication du « 700 µA entre les TX » observé
Trois couches : (1) trafic série de la campagne → grâce 30 s renouvelée →
sommeil bloqué par design (idle WFI Kineis ≈ 700 µA) ; (2) tier SLEEP
wedgé (cf. bug P0 ci-dessus) ; (3) DEPLOY=1 + seq_restart=120 s = séquence
de 3 TX toutes les 2 min en continu à la surface (~25 % duty actif). Le
« 50 µA d'avant » correspondait à un tag silencieux après cap (sans
seq-restart) et sans trafic série.

## 13. Évaluation de confiance (demandée)

### Mise à jour (après-midi) — programme « vers 10/10 »

Travail additionnel exécuté (commit `9d59c47`) :
- **Fuzzing complété dans la fenêtre de grâce : 18/18 vecteurs rejetés
  proprement**, survie au blast binaire 256 octets + 40 commandes
  rafale. Deux vrais bugs trouvés et fixés : `AT+SHUTDOWN=<garbage>`
  EXÉCUTAIT le power-off (sscanf non vérifié — le fuzz a éteint la
  carte !) → rejet strict ; `AT+MC=-1` accepté (wrap %hu→65535) → parse
  signé + bornes.
- **Wake-on-RX console** : PA3 en EXTI falling pendant chaque fenêtre
  STOP2 → 1 octet de réveil + 250 ms + commande = **5/5 accès au premier
  essai** (avant : 1/40-118). Loterie d'accès à froid éliminée. Pull-up
  interne → zéro réveil fantôme en déploiement scellé.
- **AT+LPMSTAT** : télémétrie duty-cycle on-target (uptime compensé RTC,
  ms cumulées STOP2, nb entrées, duty éveillé ×0.1 %) — le budget énergie
  se mesure désormais sur cible sans ampèremètre.
- **Soak 6 h lancé** (capture passive + probes 30 min) — résultat en fin
  de journée.

### Pérennité — confiance 9/10
- Usure flash auditée et fixée sur toutes les zones (cf. §12). MC: ~12
  erases/an. Le point était mortel il y a 24 h ; il est instrumenté et
  testé unitairement aujourd'hui.
- Timers wrap-safe (arithmétique unsigned), MC wrap u16 testé, retention
  versionnée par magic, NVM migré par version (v7), configs blindées
  contre les entrées invalides (fuzz 18/18).
- Pour 10/10 : soak ≥ 24 h propre (6 h en cours) + un cycle VBAT-loss
  complet vérifié (utilisateur, 10 min).

### Robustesse — confiance 9/10
- Filets en place et testés : IWDG 16 s (gelé en STOP, actif au run),
  boot-loop guard avec escalade factory-reset, crash forensics SRAM2,
  rate limiter RTC, backoff erreurs, gate batterie + mode LB, détection
  fautes capteur SWS, validation callbacks mcu_tim, queues v11 blindées.
- 6/6 resets torture, long-run propre, fuzzing 18/18 + survie au blast
  binaire, 28/28 suites (416 checks).
- LE wedge non-récupérable (SLEEP tier) trouvé, reproduit, éliminé,
  re-validé. Le power-off-sur-typo (AT+SHUTDOWN=zz) trouvé et fixé.
  L'accès console à froid garanti (wake-on-RX 5/5).
- Pour 10/10 : soak 6 h en cours sans anomalie (résultat ce soir) ; un
  test gesture power-down/wake complet par l'utilisateur sur ce build.

### Autonomie — confiance 7/10 — duty mesuré on-target, µA à confirmer
Hypothèses : plancher STOP2 25 µA (mesuré), réveil ~40 ms à ~6 mA
(build production sans traces UART), TX ~1 s.
- Sous l'eau (réveil/s) : ≈ 265 µA moyen → 6.4 mAh/j
- Surface idle (réveil/5 s) : ≈ 73 µA moyen
- Profil 90 % UW / 10 % surface + 60 TX/j : ≈ 8-9 mAh/j ≈ **3.2 Ah/an**
- Batterie LSH20 (13 Ah) ≈ 4 ans ; 2×AA lithium (5 Ah) ≈ 18 mois.
- LEVIER n°1 : uw_interval 500 ms → 2 s divise le coût UW par ~4
  (≈ 1.3 Ah/an → 10 ans sur LSH20) contre 2 s de latence max de détection.
- NOUVEAU : `AT+LPMSTAT=?` mesure le duty-cycle réel sur cible (uptime,
  ms STOP2 cumulées, nb d'entrées). Le soak en donnera la valeur
  déploiement-représentative ; le modèle ci-dessus devient alors
  duty_mesuré × courants.
- Pour 10/10 (matériel utilisateur requis, ~30 min au mA-mètre) :
  (1) plancher STOP2 carte au repos, (2) pic + durée d'un réveil SWS en
  build production (DEBUG=0 VERBOSE=0), (3) un TX complet. Ces 3 mesures
  × LPMSTAT = budget énergie factuel.

## Restant / recommandations

1. Mesure courant power-off avec le fil PB3 + build REED_WKUP3 (attendu
   sub-5 µA) — matériel requis, utilisateur.
2. Mesure courant STOP2 surface/UW pour chiffrer le budget énergie réel.
3. Test en eau réelle (calibration SWS adaptative non testable à sec).
4. SPI Option 3 (frm_hdlr matching) — différé, bench SPI requis.
5. La config NVM surf_interval=5000 ms est conservée (cohérente énergie) ;
   descendre à 1000 ms si la réactivité AT à la surface prime.
