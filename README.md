<!-- README.md — projet obd_can_bridge
     Auteur : Eric (F1OCM)
     Aligné firmware v8.4 -->

# obd_can_bridge

Pont de diagnostic **OBD-II autonome sur ESP32-S3**, avec interface web intégrée accessible en WiFi depuis un smartphone ou un PC.

Le firmware est contenu dans un **seul fichier `.ino`**, page HTML/CSS/JavaScript comprise (chaîne littérale embarquée dans le programme). **Aucune bibliothèque CAN externe** : le MCP2515 est piloté en registres SPI bruts, écrits à la main d'après la datasheet Microchip.

Développé et validé sur :

- **Citroën Nemo 1.3 HDi**
- moteur Fiat **199A2000 / 1.3 Multijet** (PSA F13DTE5)
- calculateur **Marelli MJD 8F3.F6**
- bus C-CAN moteur, **500 kbit/s**
- diagnostic **ISO 15765-4, adressage étendu 29 bits**

**Firmware actuel : v8.4**

## Matériel

| Élément | Référence |
|---|---|
| MCU | Seeed XIAO ESP32-S3 |
| CAN | Seeed CAN Bus Breakout 713-105100001 |
| Contrôleur CAN | MCP2515, quartz 16 MHz |
| Transceiver | SN65HVD230 |
| CS | D7 |
| INT | D6 (câblé, non utilisé — le firmware fonctionne en polling) |
| Bus | CAN moteur, 500 kbit/s |

## Interface WiFi

Le XIAO ESP32-S3 crée son propre point d'accès :

- **SSID : `NEMO-OBD`**
- **Mot de passe : `nemo1234`**
- **IP : `192.168.1.1`**
- interface : `http://192.168.1.1/`

Aucune connexion Internet requise, aucune ressource externe (pas de CDN, pas de dépendance JS).

## Ce que la norme impose sur ce véhicule

ISO 15765-4 limite l'adressage diagnostic à deux valeurs de PDU Format :

- `0x18DA<TA><SA>` — adressage physique (un ECU précis)
- `0x18DB33<SA>` — adressage fonctionnel (broadcast, tous les ECU OBD)

**Constat empirique sur ce véhicule** : la passerelle BSI ne route vers le bus moteur que les requêtes en adressage **fonctionnel**. L'adressage physique unicast (`0x18DA10F1`) n'obtient jamais de réponse, alors que le broadcast (`0x18DB33F1`) fonctionne systématiquement. Le firmware envoie donc **toutes** ses requêtes Mode 01 en broadcast, et filtre les réponses sur l'adresse ECU connue.

**ECU répondant identifié** : `0x10` (moteur, Marelli MJD 8F3.F6). C'est le seul calculateur du véhicule qui répond à l'OBD générique légal — ABS/ESP, airbag, BSI sont sur un bus séparé, non câblé sur ce montage.

**PID Mode 01 réellement supportés par cet ECU** : la chaîne d'index s'arrête à `0x40` (bitmap `80 C0 00 00`, dernier bit à 0) — aucun PID au-delà de `0x4A` n'est disponible en OBD générique sur ce véhicule (pas de couple moteur, pas de pression de suralimentation directe, pas de position de rapport engagé).

## Filtrage matériel

Le MCP2515 est configuré avec un masque/filtre matériel qui n'accepte que les trames `0x18DAF1xx` (réponse OBD physique, quel que soit l'ECU répondant) — tout le reste du trafic bus (qui sature vite les 2 buffers de réception une fois le moteur tournant) est éliminé par le silicium avant même de remonter en SPI.

## Fonctionnalités de la page web

### Log continu (jauges)

- 9 jauges circulaires (SVG, anneau vert sur fond noir) : **Pédale D, Pédale E, Vitesse, Régime, Charge, MAF, P admission, P rampe, EGR**.
- Bouton **LOG** / **STOP LOG**.
- Cadence normale : environ **1 Hz** (un cycle complet des 9 PID, calé sur la seconde).
- Détection automatique de perte de puissance — passage en **RAFALE** (15 s, cycles enchaînés avec une pause courte de 100 ms, sans calage 1 Hz) si :
  - pédale en hausse **et** vitesse en baisse par rapport au cycle précédent, **ou**
  - pédale > 75 %.
- Ordre de **lecture** CAN (fixe, pour la synchronisation) : `0x49→0x0D→0x0C→0x04→0x10→0x0B→0x23→0x2C→0x4A` — pédale et vitesse consécutives en tête de cycle, écart temporel minimal entre les deux valeurs qui déclenchent la détection.
- Ordre d'**affichage** des jauges indépendant de l'ordre de lecture (regroupement visuel logique).
- Journal horodaté (`t_ms` relatif au démarrage) écrit dans `/drivelog.csv` sur LittleFS.
- Écriture flash **tamponnée en RAM**, au plus une fois toutes les **5 s**, **jamais pendant une rafale** (une écriture flash peut geler le WiFi le temps de l'opération — c'est tout le tampon qui part d'un coup au retour en mode normal).
- Barre de remplissage LittleFS (rouge au-delà de 85 %).
- Bouton **PURGER LOG** (supprime le journal sans passer par le téléchargement ; refusé si un log est en cours).
- Téléchargement séparé (`/download_log`), fichier `nemo_drivelog.csv`.

### Scan PID (découverte des PID supportés)

- Bouton **CALL**.
- Suit la chaîne d'index standard `0x00 → 0x20 → 0x40 → …` (bit de poids faible de chaque bitmap = "le groupe suivant existe"), s'arrête dès qu'un bitmap dit non ou qu'un maillon de la chaîne ne répond pas.
- N'interroge que les PID réellement annoncés supportés par les bitmaps décodés (pas de balayage aveugle 0x00-0xFF).
- Résultats affichés en direct (Server-Sent Events) : PID, réponse OUI/NON, buffer MCP2515 (RXB0/RXB1), données brutes, valeur décodée.
- Résultats écrits dans `/result.csv`, téléchargeable (`/download`).

### Test PID isolé

- Bouton **TEST PID 0x0C** — sonde un seul PID fixe (`TEST_PID`, modifiable dans le code), pour vérifier rapidement la liaison sans lancer un scan complet.

## Décodage Mode 01

Table de correspondance (~155 entrées) couvrant les PID Mode 01 à formule linéaire simple (`valeur = mul × brut + add`, sur 1, 2 ou 4 octets, signé ou non, à un décalage donné dans la trame — gère aussi les PID à plusieurs valeurs comme `0x66` ou `0x9A`). Formules sourcées SAE J1979 / Wikipédia.

**Non décodés (affichés en "Brut")** : les PID à champs encodés bit à bit avec table de correspondance énumérée — `0x00, 0x01, 0x03, 0x12, 0x13, 0x1C, 0x1D, 0x1E, 0x41, 0x51, 0x5F` et assimilés. Mécanisme différent (extraction de bits + table de libellés), non implémenté à ce stade.

**Anomalie documentée, pas un bug** : le PID `0x01` (octet B, bit 3 — type d'allumage) répond "essence" sur ce véhicule diesel. Le bit est lu conformément à la norme (vérifié par triangulation de sources) ; c'est l'ECU Marelli qui répond ainsi, pas une erreur de décodage.

## Fiabilité — problèmes rencontrés et corrections

| Symptôme observé | Cause réelle | Correction |
|---|---|---|
| ~45 % de PID sans réponse | Requête suivante envoyée trop tôt après une réponse rapide (violation d'un délai de repos minimal) | Cycle de durée fixe **80 ms** par PID (≥ N_Bs=75ms, ISO 15765-4 tableau 6), qu'il y ait réponse rapide ou non |
| STOP LOG et rafraîchissement de page bloqués pendant le log | `server.handleClient()` appelé une seule fois par PID (~80-720ms d'écart) | Appelé à chaque itération de l'attente CAN |
| Blocage total, y compris hors connexion (sans véhicule) | Sans accusé de réception (ACK) d'un autre nœud CAN, le MCP2515 retransmet indéfiniment en interne ; le bit TXREQ ne redescend jamais, bloquant tout envoi suivant | Attente bornée à 10 ms puis abandon forcé de l'émission précédente |
| Blocage progressif pendant un trajet long | Écriture flash (`flush()`) à chaque cycle, pouvant geler le WiFi le temps de l'opération | Tampon RAM, écriture flash espacée (5 s), jamais en rafale |

Une instrumentation de debug (`Serial.print`, 115200 bauds) reste en place dans le code — peu coûteuse, déjà utile une fois pour diagnostiquer le point de blocage ci-dessus.

## Compilation

./obd_build.sh flash ( puis ./obd_build.sh mon )

ou
```bash
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3 obd_can_bridge.ino
arduino-cli upload  --fqbn esp32:esp32:XIAO_ESP32S3 -p /dev/ttyACM0 obd_can_bridge.ino
```


Aucune bibliothèque tierce à installer.

## Reste à faire ( fait en 8.11, a tester... )

- Décodage niveau 2 (PID à bits énumérés : `0x01`, `0x1C`, `0x41`, etc.)
- Bouton **CLEAR** (Mode 04 — effacement défauts), validé faisable, pas encore codé. Attention : réinitialise aussi les moniteurs de conformité ("readiness"), nécessite un cycle de conduite complet pour redevenir "prêt".
- Lecture DTC (Mode 03/07/0A)

## Licence

Usage personnel.

**Auteur : Eric Perret (F1OCM)**
Projet développé et maintenu pour le diagnostic du Citroën Nemo 1.3 HDi.
