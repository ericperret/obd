<!-- README.md — projet obd_can_bridge Auteur : Eric (F1OCM) Date : 2026-09-05 Version doc : 1.1 (alignée firmware v6.3) -->

# obd_can_bridge

Pont de diagnostic **OBD-II autonome sur ESP32**, avec interface web intégrée accessible en WiFi depuis un smartphone.

Le firmware est contenu dans un **seul fichier `.ino`**, avec une page HTML/CSS/JavaScript embarquée directement dans le programme. Aucune librairie Arduino tierce n'est nécessaire.

Développé et validé sur :

- **Citroën Nemo 1.3 HDi**
- moteur Fiat **199A2000 / 1.3 Multijet**
- calculateur **Marelli MJD 8F3.F6**
- CAN moteur **500 kbit/s**
- diagnostic **ISO 15765-4, adressage étendu 29 bits**

**Firmware actuel : v6.3 — 05/09/2026**

## Matériel

| Élément | Référence |
|---|---|
| MCU | Seeed XIAO ESP32-S3 |
| CAN | Seeed CAN Bus Breakout 713-105100001 |
| Contrôleur CAN | MCP2515, quartz 16 MHz |
| Transceiver | SN65HVD230 |
| Bus | CAN moteur, 500 kbit/s |
| OBD | broche 6 CAN-H / broche 14 CAN-L |
| SPI | MCP2515 |

## Interface WiFi

Le XIAO ESP32-S3 crée son propre point d'accès WiFi :

- **SSID : `NEMO-OBD`**
- **Mot de passe : `nemo1234`**
- **IP : `192.168.1.1`**
- interface : `http://192.168.1.1/`

Aucune connexion Internet n'est nécessaire.

L'interface web est entièrement embarquée dans le firmware : aucune ressource externe, aucun CDN et aucune dépendance JavaScript.

## Principe de fonctionnement

Une session de mesure suit une séquence déterministe.

### 1. START LOG

Le bouton **START LOG** crée ou reprend le journal et lance le premier scan avec le moteur arrêté.

Le firmware :

1. inventorie les adresses diagnostiques présentes sur le bus ;
2. recherche les PID disponibles ;
3. enregistre le résultat du scan dans le même fichier CSV.

Lorsque le scan est terminé, l'interface affiche :

> **SCAN 1 OK - DEMARREZ LE MOTEUR**

### 2. Démarrage du moteur

Le démarrage du moteur provoque normalement le redémarrage électrique de l'ESP32.

L'état de la session est conservé dans LittleFS afin que le firmware puisse reprendre après ce redémarrage.

Le firmware attend ensuite la détection du régime moteur et valide le démarrage sur plusieurs mesures cohérentes avant de lancer le second scan.

### 3. SCAN RUN

Le second scan est réalisé moteur tournant.

À la fin :

> **SCAN 2 OK - LOG EN COURS**

L'acquisition commence immédiatement.

### 4. Acquisition asynchrone

Les requêtes PID sont émises séquentiellement, mais le traitement des réponses est **asynchrone**.

Une réponse CAN/ISO-TP reste exploitable indépendamment du délai écoulé depuis l'émission de la requête. Les réponses tardives sont associées à leur PID et mettent à jour la dernière valeur connue ; elles ne sont pas rejetées uniquement en raison de leur temps d'arrivée.

Une seule requête est active côté émission à la fois, mais la réception CAN continue indépendamment de l'ordonnancement des requêtes.

Les réponses positives et négatives sont traitées. Les codes NRC sont conservés comme informations de diagnostic ; `0x78 Response Pending` n'est pas considéré comme une erreur.

Les cinq valeurs principales sont suivies :

| PID | Donnée | Colonne CSV |
|---|---|---|
| `0C` | régime moteur | `regime_trmin` |
| `23` | pression rail | `rail_kPa` |
| `0B` | pression MAP / turbo | `map_kPa` |
| `49` | position pédale | `pedale_pct` |
| `0D` | vitesse véhicule | `vitesse_kmh` |

Une **seule piste de pédale**, PID `49`, est utilisée.

La cadence cible du journal est de **100 ms**, soit environ **10 lignes/s**.

## Journal CSV

Le journal est stocké dans la mémoire flash **LittleFS** de l'ESP32.

Le fichier est :

```text
/log.csv
```

Le fichier contient à la fois les informations de session, les résultats des scans et les données de conduite.

En-tête des données :

```csv
horodatage,ms,regime_trmin,rail_kPa,map_kPa,pedale_pct,vitesse_kmh,pid_brut,val_brut
```

Les informations de diagnostic et les résultats des scans sont conservés sous forme de lignes commençant par `#`.

L'écriture LittleFS est **tamponnée** afin de limiter les écritures flash et de ne pas bloquer inutilement le traitement CAN.

Le firmware surveille également l'espace disponible et arrête le journal si la mémoire devient insuffisante.

## Interface de conduite

La page web affiche :

- régime moteur ;
- vitesse ;
- pédale ;
- pression rail ;
- MAP / pression turbo ;
- état de la session ;
- occupation de la mémoire flash ;
- nombre de lignes enregistrées.

Les valeurs sont rafraîchies en continu depuis `/stat`.

Les contrôles disponibles sont :

- **START LOG**
- **ARRET LOG**
- **TELECHARGER CSV**
- **CHECK DEFAUTS**
- **RESET DEFAUTS**
- **PURGE**

Les commandes techniques CAN ne sont pas exposées dans l'interface de conduite.

L'en-tête affiche la version du firmware et la date de release, par exemple :

> **NEMO-OBD V6.3 — 05 SEP 2026**

## Téléchargement du journal

Le bouton **TELECHARGER CSV** récupère le fichier directement depuis l'ESP32.

Le fichier téléchargé est :

```text
log_nemo.csv
```

La purge n'est effectuée **qu'après réception effective du fichier**.

Après téléchargement réussi :

1. le journal est fermé ;
2. le fichier est supprimé ;
3. l'état de session est supprimé ;
4. l'ESP32 redémarre.

Un téléchargement interrompu ne détruit donc pas le journal.

## Diagnostic des défauts

Les fonctions de diagnostic sont accessibles depuis la page web ou la console série lorsque le journal est arrêté.

### CHECK DEFAUTS

Lecture des défauts :

- mode `03` : défauts mémorisés ;
- mode `07` : défauts en attente ;
- mode `0A` : défauts permanents.

### RESET DEFAUTS

La séquence `RESET!` :

1. relève les défauts ;
2. relève les données gelées disponibles ;
3. écrit les informations dans le journal ;
4. exécute l'effacement OBD mode `04` ;
5. effectue une nouvelle lecture.

## Console série

La console USB fonctionne à :

```text
115200 bauds
```

Quelques commandes disponibles :

| Commande | Effet |
|---|---|
| `HELP?` | aide des commandes |
| `PING?` | test de vie |
| `INFO?` | informations ESP32 / MCP2515 |
| `INIT?` | réinitialisation MCP2515 |
| `LOG ON` | démarre une session |
| `LOG OFF` | arrête une session |
| `LOG LAST` | dernière ligne mémorisée |
| `STAT?` | état du système et du journal |
| `BUS?` | compteurs CAN |
| `DTC` | défauts 03 + 07 + 0A |
| `DTCP` | défauts en attente |
| `DTCX` | défauts permanents |
| `FRZ` | données gelées |
| `RESET!` | lecture, effacement puis nouvelle lecture des défauts |
| `PID xx` | requête d'une PID OBD |
| `PURGE!` | suppression du journal |
| `ECHO ON/OFF` | active/désactive l'écho du journal sur la console |
| `TIME! <epoch>` | fournit la base d'horodatage au module |

## ISO-TP et réponses diagnostiques

Le transport diagnostic utilise ISO-TP pour les messages dépassant une trame CAN simple.

Le firmware prend en charge :

- Single Frame ;
- First Frame ;
- Consecutive Frames ;
- Flow Control ;
- réassemblage des réponses multi-trames.

Une réponse diagnostique est identifiée par son contenu et non par un délai arbitraire.

Exemple :

```text
requête       01 0C
réponse       41 0C AA BB
```

La réponse `41 0C AA BB` met à jour la valeur du PID `0C`, même si elle arrive après le passage de l'ordonnanceur à une autre requête.

Les réponses négatives de forme :

```text
7F <service> <NRC>
```

sont également traitées.

Le NRC `0x78` (**Response Pending**) indique que la requête a été reçue et que le calculateur poursuit son traitement ; il n'est pas traité comme une absence de réponse.

## Scan OBD

Le scan :

1. sonde les adresses diagnostiques disponibles ;
2. identifie les calculateurs qui répondent ;
3. balaie les PID du mode `01` ;
4. conserve les réponses reçues ;
5. écrit les résultats dans le journal.

Le scan est traité de manière asynchrone afin que la communication WiFi et le traitement CAN continuent à fonctionner pendant l'opération.

Les réponses inattendues ou tardives restent exploitables.

## PID supplémentaires

Les PID détectées par le scan mais qui ne font pas partie des cinq grandeurs principales peuvent être conservées dans un ensemble de PID supplémentaires.

Pendant l'acquisition, elles sont interrogées à tour de rôle.

Elles apparaissent dans :

```csv
pid_brut,val_brut
```

Leur valeur est enregistrée sous forme brute, sans interprétation constructeur arbitraire.

## Horodatage

Le téléphone fournit au module une base de temps Unix au début de l'utilisation de l'interface.

Les lignes de mesure comportent :

- une date/heure ;
- un temps milliseconde.

Le temps milliseconde permet notamment de reconstruire précisément la chronologie des événements lors de l'analyse du journal.

## Compilation et flash

Environnement utilisé pour le développement :

```text
arduino-cli 1.5.1
esp32:esp32 3.3.10
```

Compilation :

```bash
./obd_build.sh compile
```

Compilation et flash :

```bash
./obd_build.sh flash
```

Moniteur série :

```bash
./obd_build.sh mon
```

Cible :

```text
esp32:esp32:XIAO_ESP32S3
```

Aucune librairie Arduino tierce n'est nécessaire.

## Historique des versions

| Version | Évolution |
|---|---|
| v2.1 | 986 codes P0xxx FR embarqués, WiFi AP, base MCP2515 |
| v2.2 | SNIF filtre delta |
| v2.3 | SNIF masque automatique des compteurs roulants |
| v2.4 | SNIF apprentissage de jeu de payloads |
| v2.5 | requêtes OBD, repli 7DF → 7E0 |
| v2.6 | commande PROBE?, driver 29 bits |
| v2.7 | diagnostic 29 bits natif Marelli MJD8 |
| v2.8 | décodage J1979 + SCAN? |
| v2.9 | décodage PID 01 / 1C / 4A |
| v3.0 | CLOCK! + log CSV horodaté |
| v4.x | refonte du diagnostic 29 bits, scans et gestion des calculateurs |
| v5.x | logger de conduite, scans exhaustifs, journal LittleFS et interface web |
| v5.4 | acquisition multi-PID et optimisation du logger |
| v6.0 | refonte du logger : acquisition déterministe, CSV à 100 ms, reprise après redémarrage |
| v6.2 | stabilisation du logger et de l'interface |
| **v6.3** | **refonte de l'acquisition : réception asynchrone, réponses tardives conservées, gestion des NRC/Response Pending, acquisition physique moteur et validation du démarrage sur RPM cohérent** |

## Licence

Usage personnel.

**Auteur : Eric Perret (F1OCM)**

Projet développé et maintenu pour le diagnostic du Citroën Nemo 1.3 HDi.
