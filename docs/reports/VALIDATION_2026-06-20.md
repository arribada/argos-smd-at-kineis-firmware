# Validation livrable — consolidation v2 + re-validation robustesse (2026-06-20)

**Scope :** STM32WL55 / SMD_STDALONE / APP=UW_DOPPLER / COMM=UART / MAC=BASIC.
Déploiement scellé océanique ~12 mois (entrée terrain = aimant uniquement).

**Source de vérité :** ce document + [MASTER_AUDIT.md](MASTER_AUDIT.md).

---

## 1. Consolidation des branches (merge v2-fix-lpm → v2)

Deux lignes de travail avaient divergé depuis la base `b22fa69` (2 fév 2026) :

- **v2** (35 commits) — protocole SPI, bootloader DFU UART+SPI, doppler-sans-UW,
  PA board OP, fixes GUI, tentative LPM/SLEEP côté SPI.
- **v2-fix-lpm** (136 commits) — tout le durcissement scellé : LPM/STOP2/SHUTDOWN,
  4 brick fixes, SWS, reed debounce+blanking, gesture, NVM v8, MGR_ERR/EVTLOG/
  WDG/LED/BAT, payload F.6.

**Analyse (46 conflits, 1 agent/fichier + recoupement déterministe) :** v2-fix-lpm
est un **sur-ensemble strict** de v2. v2-fix-lpm avait déjà absorbé le travail
SPI/DFU/bootloader/doppler de v2 via des merges antérieurs, puis construit dessus.
Seul fichier unique à v2 = un `.js` Doxygen généré (bruit). Tous les 46 conflits
résolus vers v2-fix-lpm ; le **tree mergé est byte-identique au tree validé**
`84c27b3`. Vérifié dimension par dimension (ISR/clock/ADC/SPI : v2-fix-lpm préserve
et améliore la fonctionnalité de v2, jamais de perte).

- Merge : `b0299c3` (parents `f4883ef` v2 + `84c27b3` v2-fix-lpm).

## 2. Gardes flash bootloader (commit `071faed`)

Deux landmines flash identifiées (audit pre-seal) + 1 résiduel fermés :

- **G1 — BL_STATE vs creds.** La persistance flash de l'état bootloader
  (`bl_flash_read/write_bl_state`, `bl_flash_set_dfu_request`) visait
  `BL_STATE_FLASH_ADDR == FLASH_USER_START == 0x0803B000` = page 0 des
  credentials. Code **mort** (aucun appelant ; la prod signale le DFU via
  `TAMP_BKP0R` + SRAM). Compilé hors-build derrière `BL_STATE_PERSIST_ENABLED 0`
  + `_Static_assert(BL_STATE_FLASH_ADDR != FLASH_USER_START)` qui casse le build
  à tout ré-armement naïf.
- **G2 erase — `APP_FLASH_SIZE 0x33000 → 0x32000` (200K).** `bl_flash_erase_app()`
  et le plafond de taille DFU s'arrêtent à `0x08031FFF` : FLASH_PMLOG
  (`0x08032000-0x08032FFF`, dont le **miroir CRED @0x08032800**) n'est plus
  jamais effacé. Cohérent avec le linker app (ROM=200K).
- **G2 write — borne de fin d'écriture DFU.** Le handler write ne validait que
  l'adresse de départ ; ajout d'une borne `address+len ≤ APP_FLASH_END` pour
  qu'un chunk en limite ne déborde pas dans FLASH_PMLOG.

## 3. Brick scellé surfacé + corrigé (commit `6cf7c7a`)

**SWS `max_dive_time_s == 0` désactivait le seul échappatoire d'un détecteur
bloqué-immergé.** L'escalade dive-timeout (qui arme le degraded beacon et force
SURFACE) est gatée sur `max_dive_time_s > 0` (mgr_sws.c:1019). À 0, une unité
scellée avec un SWS bloqué-UW devient **muette en TX de façon permanente**, sans
recours aimant. Les 3 chemins de config acceptaient 0. Fermé en défense en
profondeur : AT+SWSCFG rejette 0, `MGR_SWS_setConfig` et NVM `apply_config`
soignent un 0 → défaut 7200 s. + test de régression.

## 4. Validation

| Élément | Résultat |
|---|---|
| Tests unitaires | **39/39 suites, 508 checks OK** (+2 SWS heal) |
| Build UW_DOPPLER/SMD_STDALONE DEBUG=0 `full` | OK — app 123.5 KB + BL 27.5 KB |
| Build STDLN/GUI/DOPPLER × STDALONE | OK |
| Build GUI/SMD_PA, DOPPLER/SMD_OP | OK |
| UW_DOPPLER × SMD_PA / SMD_NOPA | `#error` **intentionnel** (UW = STDALONE-only, kns_app_uw_doppler.c:94) |

## 5. Re-validation robustesse — nouvelle note

Audit adversarial multi-agents (8 dimensions → vérification adverse des findings
high/critical → synthèse) sur le tree mergé+gardé.

**Scores par dimension :** mac-tx 90 · flash/creds 88 · lpm-wake 88 · reed-gesture
88 · nvm 88 · watchdog 86 · sws 82 · battery 82.

**Note audit (état trouvé) : 84/100 — GO-WITH-GATES.** Le léger retrait vs 85
antérieur venait de 2 déductions, **toutes corrigées dans cette session** :
1. gardes G1/G2 non commitées → **commitées** (`071faed`) ;
2. brick SWS `max_dive=0` ouvert → **corrigé** (`6cf7c7a`) ;
3. résiduel write-path G2 → **fermé** (borne de fin d'écriture).

G1/G2 confirmées **correctes & complètes** par plusieurs agents indépendants →
gates flash (a) et (b) **fermées**.

**Note effective post-remédiation : ~88/100 — GO, sous réserve des gates BANC/HW
ci-dessous** (risque cred-brick réellement réduit, nouveau brick SWS fermé,
gate process fermée).

### Gates restantes (BANC/HW, hors-code)
- **STOP2 µA** mesuré vs budget batterie ~12 mois (`enterStop2TimedMs`) — papier seulement.
- **Biofouling** : balayage proximité/seuil SWS sur électrode encrassée — jamais exercé.
- **LSE mort PENDANT STOP2** (gate c, résiduel connu) : si le cristal meurt après
  armement WUT en sommeil, RTC ne réveille plus, IWDG gelé → reset-loop lent
  auto-limité (drain, pas strand) ; fallback LSI récupère au boot. Confirmer HW.
- **Audit config pré-scellage** : dumper le NVM persistant (AT+SWSCFG, AT+SWS≠0,
  AT+BATCFG min_tx, AT+DUTYCFG) — vérifier aucune mis-config scellée.

### Résiduels acceptés
- Weak-cell PA-brownout = drain borné fin-de-vie (le rate limiter survit au reset
  SFT mesuré ; gate 2800 mV termine la TX) — pas de strand, pas de page cred touchée.
- Fallback LSI : prescalers RTC figés 32768 Hz → ~2.3% lent. Bénin.
- ADC mort/bloqué-haut maintient UNDERWATER (fail-safe = hold-last) ; échappatoire =
  degraded-beacon dive-timeout — **couvert** par le fix §3 + la gate audit-config.

---

*Tag de déploiement posé sur ce point. Build scellé à régénérer depuis le tag
(build_info.c embarque date+commit).*
