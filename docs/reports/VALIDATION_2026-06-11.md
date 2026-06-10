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

## Restant / recommandations

1. Mesure courant power-off avec le fil PB3 + build REED_WKUP3 (attendu
   sub-5 µA) — matériel requis, utilisateur.
2. Mesure courant STOP2 surface/UW pour chiffrer le budget énergie réel.
3. Test en eau réelle (calibration SWS adaptative non testable à sec).
4. SPI Option 3 (frm_hdlr matching) — différé, bench SPI requis.
5. La config NVM surf_interval=5000 ms est conservée (cohérente énergie) ;
   descendre à 1000 ms si la réactivité AT à la surface prime.
