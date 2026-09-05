# obd_can_bridge

Pont de diagnostic **OBD-II autonome sur ESP32**, avec interface web intégrée accessible en WiFi depuis un smartphone.

Le firmware est contenu dans un **seul fichier `.ino`**, avec une page HTML/CSS/JavaScript embarquée directement dans le programme. Aucune librairie Arduino tierce n'est nécessaire.

Développé et validé sur :

* **Citroën Nemo 1.3 HDi**
* moteur Fiat **199A2000 / PSA F13DTE5**
* calculateur **Marelli MJD 8F3.F6**
* CAN moteur **500 kbit/s**
* diagnostic **ISO 15765-4, adressage étendu 29 bits**

**Firmware actuel : v6.2 — 05/09/2026**

## Matériel

| Élément        | Référence                            |
| -------------- | ------------------------------------ |
| MCU            | Seeed XIAO ESP32-S3                  |
| CAN            | Seeed CAN Bus Breakout 713-105100001 |
| Contrôleur CAN | MCP2515, quartz 16 MHz               |
| Transceiver    | SN65HVD230                           |
| Bus            | CAN moteur, 500 kbit/s               |
| OBD            | broche 6 CAN-H / broche 14 CAN-L     |
| SPI            | MCP2515                              |

## Interface WiFi

Le XIAO ESP32-S3 crée son propre point d'accès WiFi :

* **SSID : `NEMO-OBD`**
* **Mot de passe : `nemo1234`**
* **IP : `192.168.1.1`**
* interface : `http://192.168.1.1/`

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

Il attend ensuite la détection du régime moteur.

### 3. SCAN RUN

Le second scan est réalisé moteur tournant.

À la fin :

> **SCAN 2 OK - LOG EN COURS**

L'acquisition commence immédiatement.

### 4. Acquisition

Les cinq valeurs principales sont acquises à une cadence cible de **100 ms**, soit environ **10 échantillons/s** :

| PID  | Donnée               | Colonne CSV    |
| ---- | -------------------- | -------------- |
| `0C` | régime moteur        | `regime_trmin` |
| `23` | pression rail        | `rail_kPa`     |
| `0B` | pression MAP / turbo | `map_kPa`      |
| `49` | position pédale      | `pedale_pct`   |
| `0D` | vitesse véhicule     | `vitesse_kmh`  |

Une **seule piste de pédale**, PID `49`, est volontairement utilisée.

Les PID supplémentaires détectées pendant le scan peuvent également être interrogées périodiquement et enregistrées sous forme brute.

L'acquisition utilise une requête PID à la fois : une nouvelle requête est envoyée après réception de la réponse ou expiration d'un délai court.

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

Exemple de structure :

```text
# LOG ON v6.2
# SCAN OFF
# BUS ...
# ADR ...
# RAW ...
# FIN SCAN OFF
...
# SCAN RUN
...
# FIN SCAN RUN
# PID BRUTES SUIVIES ...
horodatage,ms,regime_trmin,rail_kPa,map_kPa,pedale_pct,vitesse_kmh,pid_brut,val_brut
...
```

L'écriture LittleFS est **tamponnée** afin de limiter les écritures flash et de ne pas bloquer inutilement le traitement CAN.

Le firmware surveille également l'espace disponible et arrête le journal si la mémoire devient insuffisante.

## Interface de conduite

La page web affiche uniquement les informations utiles pendant la conduite :

* régime moteur ;
* vitesse ;
* pédale ;
* pression rail ;
* MAP / pression turbo ;
* état de la session ;
* occupation de la mémoire flash ;
* nombre de lignes enregistrées.

Les valeurs sont rafraîchies en continu depuis `/stat`.

Les contrôles disponibles sont :

* **START LOG**
* **ARRET LOG**
* **TELECHARGER CSV**
* **CHECK DEFAUTS**
* **RESET DEFAUTS**
* **PURGE**

Les commandes techniques CAN ne sont pas exposées dans l'interface de conduite.

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

* mode `03` : défauts mémorisés ;
* mode `07` : défauts en attente ;
* mode `0A` : défauts permanents.

Les données de trame et les informations de défaut sont conservées dans le journal lorsque les fonctions de diagnostic sont utilisées pendant une session appropriée.

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

| Commande        | Effet                                                 |
| --------------- | ----------------------------------------------------- |
| `HELP?`         | aide des commandes                                    |
| `PING?`         | test de vie                                           |
| `INFO?`         | informations ESP32 / MCP2515                          |
| `INIT?`         | réinitialisation MCP2515                              |
| `LOG ON`        | démarre une session                                   |
| `LOG OFF`       | arrête une session                                    |
| `LOG LAST`      | dernière ligne mémorisée                              |
| `STAT?`         | état du système et du journal                         |
| `BUS?`          | compteurs CAN                                         |
| `DTC`           | défauts 03 + 07 + 0A                                  |
| `DTCP`          | défauts en attente                                    |
| `DTCX`          | défauts permanents                                    |
| `FRZ`           | données gelées                                        |
| `RESET!`        | lecture, effacement puis nouvelle lecture des défauts |
| `PID xx`        | requête d'une PID OBD                                 |
| `PURGE!`        | suppression du journal                                |
| `ECHO ON/OFF`   | active/désactive l'écho du journal sur la console     |
| `TIME! <epoch>` | fournit la base d'horodatage au module                |

Exemple :

```text
PID 0C
```

## Horodatage

Le téléphone fournit au module une base de temps Unix au début de l'utilisation de l'interface.

Les lignes de mesure comportent :

* une date/heure ;
* un temps milliseconde.

Exemple :

```text
2026-09-05 19:10:23.417
```

En l'absence d'horloge fournie par le téléphone, le firmware utilise le temps depuis le démarrage de l'ESP32.

## Scan OBD

Le scan ne suppose pas que toutes les informations nécessaires sont connues à l'avance.

Le firmware :

1. sonde les adresses physiques disponibles ;
2. identifie les calculateurs qui répondent ;
3. balaie les PID du mode `01` ;
4. conserve les réponses reçues ;
5. écrit les résultats dans le journal.

Le scan est réalisé de manière asynchrone afin que la communication WiFi et le traitement CAN continuent à fonctionner pendant l'opération.

Les réponses inattendues ou tardives restent exploitables : les données reçues sont associées à la PID portée par la réponse plutôt qu'à la seule requête qui était en cours d'émission.

## PID supplémentaires

Les PID détectées par le scan mais qui ne font pas partie des cinq grandeurs principales peuvent être conservées dans un ensemble de PID « brutes ».

Pendant l'acquisition, elles sont interrogées à tour de rôle.

Elles apparaissent dans :

```csv
pid_brut,val_brut
```

Leur valeur est enregistrée sous forme brute, sans interprétation constructeur arbitraire.

Cette méthode permet notamment de rechercher ultérieurement une grandeur intéressante en comparant sa série temporelle avec le régime, la pédale, la pression rail, la MAP et la vitesse.

## Compilation et flash

Le projet utilise le core Arduino officiel ESP32.

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

Le script utilise la cible :

```text
esp32:esp32:XIAO_ESP32S3
```

Aucune librairie Arduino tierce n'est nécessaire.

Les composants utilisés proviennent du core ESP32 et de l'environnement Arduino :

```cpp
SPI.h
WiFi.h
WebServer.h
LittleFS.h
```

## Architecture logicielle

Le firmware est organisé autour de plusieurs traitements coopératifs :

* pompe CAN MCP2515 ;
* décodage ISO-TP ;
* scan des adresses ;
* scan des PID ;
* ordonnanceur d'acquisition ;
* journal LittleFS ;
* serveur HTTP ;
* interface HTML/CSS/JavaScript embarquée ;
* gestion persistante de l'état de session.

Les opérations longues sont traitées sans bloquer le serveur HTTP.

Le fichier de journal est conservé lors d'un redémarrage de l'ESP32 afin de permettre la reprise d'une session interrompue par le démarrage du moteur.

## Historique récent

| Version | Évolution                                                                             |
| ------- | ------------------------------------------------------------------------------------- |
| v4.5    | séquence SCAN OFF → démarrage → SCAN RUN ; diagnostic des défauts                     |
| v4.6    | correction de l'adressage du scan et du téléchargement avec purge après transfert     |
| v4.7    | journal de scan exhaustif des PID                                                     |
| v4.8    | inventaire du bus avant balayage                                                      |
| v4.9    | scan asynchrone et écriture différée du journal                                       |
| v5.0    | prise en compte de l'adresse source réelle des réponses CAN                           |
| v5.1    | suppression des temporisations cachées de lecture série                               |
| v5.2    | jauge d'occupation flash dans l'interface                                             |
| v5.4    | correction du timing du scan et ajout des statistiques CAN                            |
| v6.0    | refonte du logger : acquisition déterministe, CSV à 100 ms, reprise après redémarrage |
| v6.2    | stabilisation du logger et de l'interface de conduite                                 |

## Licence

Usage personnel.

**Auteur : Eric Perret (F1OCM)**

Projet développé et maintenu pour le diagnostic du Citroën Nemo 1.3 HDi.
