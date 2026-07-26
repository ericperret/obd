<!-- README.md — projet obd_can_bridge Auteur : Eric (F1OCM) Date : 2026-07-26 Version doc : 1.0 (aligne firmware v3.0) -->
obd_can_bridge

Pont de diagnostic OBD-II autonome sur ESP32, avec interface web intégrée accessible en WiFi depuis un smartphone. Aucune librairie externe, aucune dépendance : tout (firmware, page HTML/CSS/JS) tient dans un seul .ino.

Développé et validé sur Citroën Nemo 1.3 HDi / Fiat Fiorino 225 (moteur Fiat 1.3 Multijet 199A2000, calculateur Marelli MJD8F3), diagnostic en CAN ISO 15765-4 adressage étendu 29 bits.

Matériel
Élément	Référence
MCU	Seeed XIAO ESP32-S3
CAN	Seeed CAN Bus Breakout (MCP2515 + SN65HVD230, quartz 16 MHz)
Bus	500 kbit/s, prise OBD broches 6 (CAN-H) / 14 (CAN-L)
SPI	2 MHz (stabilité)
Interface
WiFi AP NEMO-OBD / nemo1234 (WPA2), IP fixe 192.168.1.1, DHCP intégré.
Console USB série, 115200 bauds — seule voie pour SNIF et LOG ON.
Page web embarquée : lecture défauts (traduction FR des 986 codes P0xxx), scan PID en clair, log de conduite, réglage de l'horloge.
Commandes
Commande	Effet
PING?	test de vie → PONG
INFO?	identité ESP32 + MCP2515 + test SPI
INIT?	(ré)initialise le MCP2515
PROBE?	sonde les 4 adressages diag (11b/29b) et liste les répondants
DTC? / PEND?	défauts confirmés (mode 03) / en attente (mode 07)
CLEAR!	efface défauts + voyant (mode 04)
EGR?	consigne et erreur EGR
SCAN?	tous les PIDs supportés, décodés en clair
PID <m> <p>	requête libre hexa + décodage, ex. PID 01 0C
LOG ON [sec] / LOG OFF	log CSV horodaté en console (défaut 30 s)
LOG?	une ligne CSV (utilisé par la page web)
CLOCK! hh mm ss JJ MM AA	règle l'horloge du bus
SNIF ON / SNIF OFF	sniffer CAN brut, filtre delta (USB uniquement)
Log de conduite

Deux voies, format CSV identique (horodatage lu sur la trame horloge du bus) :

WiFi : bouton Log conduite → la page relève une ligne toutes les 30 s, accumule en mémoire, bouton Télécharger CSV pour récupérer le fichier sur le téléphone.
Console : LOG ON 30 → une ligne CSV toutes les 30 s sur le port série, à capturer côté PC (tee, minicom log, etc.).

Colonnes : horodatage, regime, eau, air, rail, map, maf, egr, charge, pedale, vitesse, niveau.

Horloge

CLOCK! réémet la trame horloge (C28A000 sur le Nemo, BCD hh mm ss MM JJ AA). Forme générique CLOCK! <idhex29> hh mm ss JJ MM AA pour un autre véhicule après sniff de son ID horloge. Le bouton Régler l'horloge de la page envoie l'heure courante du navigateur.

Note : si le calculateur de bord réémet cette trame en continu, il peut réécraser la valeur. Comportement à vérifier par véhicule.

Compatibilité multi-véhicules
Commutation 11 / 29 bits automatique : l'adressage qui répond est mémorisé et rejoué en priorité ; repli sur les 4 modes.
Mitsubishi Space Star et autres : sniffer d'abord (SNIF ON) pour relever l'ID et l'ordre d'octets de la trame horloge, puis utiliser CLOCK! <idhex> ....
Compilation / flash
bash
./obd_build.sh flash

Environnement Arduino ESP32 (core natif : SPI.h, WiFi.h, WebServer.h, StreamString.h). Aucune librairie tierce à installer.

Historique des versions

Voir les tags Git. Résumé :

Tag	Apport
v2.1	986 codes P0xxx FR embarqués, WiFi AP, base MCP2515
v2.2	SNIF filtre delta (anti-rafales réseau)
v2.3	SNIF masque auto des compteurs roulants
v2.4	SNIF apprentissage de jeu de payloads (trames multiplexées)
v2.5	requêtes OBD : repli 7DF → 7E0, dump erreurs TX
v2.6	commande PROBE?, driver 29 bits
v2.7	diagnostic 29 bits natif (Marelli MJD8), filtres ouverts
v2.8	décodage J1979 en clair + commande SCAN?
v2.9	décodage PID 01 / 1C / 4A
v3.0	CLOCK! + log CSV horodaté (console & web)
Licence

Usage personnel.
