# Rapport de comprehension des PDFs Kineis

> **AVERTISSEMENT important — extraction PDF impossible dans cet environnement.**
> L'outil `Read` du harnais necessite `pdftoppm` (poppler) pour rasteriser les PDFs en images
> avant lecture. Le binaire est present sur la machine (`C:\msys64\mingw64\bin\pdftoppm.exe`)
> mais le sandbox du harnais bloque a la fois les appels Bash/PowerShell (refus systematique
> de l'outil) et ne trouve pas `pdftoppm` dans son PATH interne ("not found or in unsafe
> location"). Aucune des deux PDFs (`Kineis_doppler_loc_V2.pdf`,
> `KINEIS-MU-24-0075_kineis_stack_sw_integration_guide_02.pdf`) n'a donc pu etre lue
> directement. Le rapport ci-dessous a ete redige a partir des sources alternatives
> disponibles dans le depot :
> - documentation Doxygen locale du SDK Kineis : `Kineis/Doc/krd_fw/html/`,
>   `Kineis/Doc/libkineis/html/`, `Kineis/Doc/libknsrf_wl/html/`,
> - en-tete de configuration officiel `Kineis/Extdep/Conf/kineis_sw_conf.h`,
> - code applicatif Doppler/UW_DOPPLER (`Kineis/App/kns_app_doppler.c`,
>   `Kineis/App/kns_app_uw_doppler.c`),
> - wrappers MCU (`Kineis/Extdep/Mcu/Src/mcu_misc.c`, `mcu_nvm.c`),
> - gestionnaire LPM (`Kineis/Lpm/Src/lpm.c`),
> - wiki interne (`.claude/argos-smd-at-kineis-firmware.wiki/`).
>
> Les numeros de page demandes ne peuvent pas etre cites. Merci de :
> 1. autoriser l'outil Bash, ou
> 2. ajouter `C:\msys64\mingw64\bin` au PATH systeme de la session du harnais, ou
> 3. fournir les PDFs deja convertis en texte/Markdown,
> puis relancer la tache pour obtenir un rapport reellement source du PDF.

---

## PDF #1 — `Kineis_doppler_loc_V2.pdf` (Doppler localisation)

Contenu non lu. Synthese reconstituee a partir du code applicatif Doppler du depot et
de la connaissance generale du systeme Argos/Kineis.

### Principe de localisation cote satellite
La constellation Kineis utilise le decalage Doppler observe pendant un passage satellite
(LEO, ~650 km, passage typique de 8 a 15 min) pour calculer la position du tag. Le
satellite mesure la frequence recue f_rx et la compare a la frequence nominale du tag
(canal Argos 401,6xx MHz). Le profil Doppler (f_rx vs temps) sur plusieurs messages d'un
meme passage permet une trilateration mono-satellite. Une localisation typique requiert
**au moins 3–4 messages recus** dans un meme passage pour resoudre l'ambiguite Nord/Sud
et obtenir une precision metrique decametrique.

### Ce que le satellite attend du tag
- **Stabilite frequentielle** : derive lente, pas de saut entre messages d'un meme passage
  (d'ou l'importance du TCXO, voir PDF #2).
- **Repetition** : plusieurs messages identiques ou differents espaces de quelques dizaines
  de secondes pour echantillonner la courbe Doppler.
- **Jitter** : recommande pour eviter les collisions entre tags voisins ; le code
  `kns_app_doppler.c` du depot implemente effectivement un jitter configurable par NVM.
- **Longueur de message** : courte (28/56/96 bits utile selon profil LDA2/LDA2L/VLDA4)
  pour maximiser la probabilite de reception sur un passage court.

### Strategie d'emission optimale (first-surface lock)
Sur la base du code `kns_app_uw_doppler.c`, la strategie implementee est :
- a la detection de surface (SWS), demarrer un burst rapide de messages,
- intervalle initial court puis croissance exponentielle
  `T(n) = T_initial * (1 + growth/100)^n`,
- limite haute (`max_interval`) pour ne pas perdre la session apres une longue exposition.

Cette approche est coherente avec une logique "first-surface fast lock" : maximiser
la chance qu'un passage satellite voie 3+ messages des les premieres minutes hors de l'eau.

### Contraintes a respecter (firmware tracker)
- intervalle minimum : eviter le sur-collision avec son propre message precedent
  (typiquement >= 30–60 s selon profil),
- intervalle maximum : conserver la coherence Doppler sur un passage,
- nombre max de TX par passage : limite par la consommation et la regulation
  (`AT+TX_REPETITION_NUMBER` dans le firmware),
- longueur de message : payload utile aligne sur le profil MAC choisi.

### Points a verifier vs firmware actuel
**Non verifiable sans lecture du PDF.** A confirmer apres relecture :
- l'intervalle minimum de 30 s applique-t-il a tous les profils ?
- la croissance exponentielle implementee correspond-elle a la recommandation Kineis ?
- le jitter genere par `mgr_at_cmd_list_doppler.c` respecte-t-il les bornes
  recommandees par Kineis ?

---

## PDF #2 — `KINEIS-MU-24-0075` (Stack SW integration guide)

Contenu non lu. Synthese reconstituee a partir du SDK Doxygen
(`Kineis/Doc/krd_fw/html/`) et des wrappers MCU du depot.

### Architecture du stack
- **Taches / scheduler** : KNS_OS cooperatif (`Kineis/App/Kineis_os/KNS_OS/`).
  Trois consommateurs principaux : MAC, LPM manager, application AT/SPI.
- **Queues** : deux files inter-couches
  (`Kineis/App/Kineis_os/KNS_Q/`)
  - `KNS_Q_DL_APP2MAC` : commandes app vers MAC (`SEND_DATA`, `INIT`, `STOP`),
  - `KNS_Q_UL_MAC2APP` : evenements MAC vers app (`TX_DONE`, `TX_TIMEOUT`,
    `SAT_DETECTED`, `MAC_ERROR`, etc.).
- **Callbacks integrateur** : NVM, AES, TIM, MISC (PA/TCXO), SPI driver — tous fournis
  via les en-tetes `Kineis/Extdep/Mcu/`.

### Sequence d'initialisation requise
Ordre observe dans `main.c` et impose par le SDK :
1. clock systeme + HSI/HSE/PLL,
2. peripheriques (UART/SPI, RTC, ADC, GPIO, IWDG),
3. NVM (chargement credentials + RadioConf),
4. AES (initialise avec cle device),
5. KNS_OS init,
6. `KNS_MAC_init(profile)` avec un profil precharge (BASIC / BLIND).

Le profil MAC doit etre charge **avant** la premiere requete TX, sinon `KNS_MAC_ERROR`.

### Power management
- Hooks LPM exposes par `Kineis/Lpm/Src/lpm.c` :
  - clients qui s'enregistrent et bloquent l'entree STOP/STANDBY,
  - notification d'entree/sortie LPM pour reconfigurer les peripheriques.
- TCXO : controle dans `mcu_misc.c` via `MCU_MISC_TCXO_Force_State()` et
  `MCU_MISC_TCXO_set_warmup()`. Le warmup doit etre respecte avant chaque TX,
  sinon la frequence emise derive (impact direct sur le calcul Doppler).

### Flux TX
1. application appelle `KNS_MAC_send_data(payload)`,
2. MAC encode la trame Argos (preambule + sync + payload),
3. allume PA (`MCU_MISC_turn_on_pa()`),
4. force TCXO ON, attend warmup,
5. initialise SubGHz, configure frequence (401,6xx MHz), modulation
   (LDA2/LDA2L/VLDA4/LDK/HDA4),
6. lance TX,
7. attente IRQ Radio (`TX_DONE` / `TX_TIMEOUT`),
8. coupe TCXO, eteint PA,
9. push evenement vers `KNS_Q_UL_MAC2APP`.
La politique de repetition (BLIND profile) est geree par la couche MAC.

### Gestion d'erreur
Le stack remonte par la queue MAC2APP : `MAC_ERROR`, `TX_TIMEOUT`, `RF_ERROR`.
L'integrateur doit :
- liberer le PA et le TCXO meme en cas d'erreur,
- consigner l'evenement (`MGR_EVTLOG` dans le depot le fait),
- ne pas relancer immediatement une TX avant verification de l'etat radio.

### Points a auditer dans le firmware actuel (a confirmer apres lecture reelle du PDF)
A partir de l'inspection du code, voici les points sensibles que le guide Kineis traite
habituellement et qu'il faudra recouper :
- **TCXO** : verifier que `MCU_MISC_TCXO_Force_State()` est bien appele AVANT
  l'init SubGHz et que le warmup est applique a chaque sortie de STOP, pas seulement
  au boot.
- **ADC** : sur STM32WL, l'ADC partage des ressources clock avec le bloc radio ;
  le guide impose generalement de desactiver l'ADC pendant TX. A verifier dans
  `Core/Src/adc.c` et la sequence SWS.
- **Ordre d'init SubGHz** : le SDK exige (HSE TCXO -> RF_BUSY check -> SUBGHZSPI init
  -> SubGHz init). Le commit recent `4f3e265 fix SLEEP mode clock for SPI` suggere
  que ce point a deja ete touche, a recroiser avec le guide.
- **LPM hooks** : verifier que tous les peripheriques actifs (UART, SPI, ADC, PA, TCXO)
  s'enregistrent comme clients LPM et sont remis dans le bon etat a la sortie de STOP.
  Le commit `3e7bebf Fix lpm for SPI in progress` indique un travail en cours sur
  ce sujet.

---

## Recommandation
Relancer la tache une fois `pdftoppm` accessible ou les PDFs convertis en texte/MD.
Le rapport pourra alors etre verifie ligne a ligne contre les pages du guide officiel,
et les ecarts entre le code actuel et la specification Kineis pourront etre listes de
maniere fiable.
