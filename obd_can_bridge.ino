/* ============================================================
 * obd_can_bridge.ino
 * ------------------------------------------------------------
 * Pont WiFi <-> Bus CAN OBD-II - logger haute cadence
 * Cible   : Seeed XIAO ESP32-S3 + Seeed CAN Bus Breakout
 *           (713-105100001) MCP2515 @ 16 MHz + SN65HVD230
 * Vehicule: Citroen Nemo 1.3 HDi (Fiat 199A2000 / PSA F13DTE5)
 *           VIN VF7AAFHZ0B8126615 - Marelli MJD 8F3.F6
 *           C-CAN 500 kbit/s - ISO 15765-4 adressage 29 bits
 * Auteur  : Eric Perret (F1OCM) - assistance Claude
 * Version : 6.0 - 2026-09-05
 * Licence : usage personnel
 * v6.0: refonte logger. Suppression de l'arbitrage A/B et du tick 1 Hz.
 *       Header OFF -> reboot demarreur -> Header RUN -> acquisition.
 *       CSV a cadence cible 100 ms, ecriture LittleFS tamponnee.
 *       Live limite aux cinq mesures demandees : RPM, vitesse, pedale
 *       (une seule piste PID 49), rail et MAP/turbo. Telechargement
 *       reussi -> purge -> reset ESP32.
 * ------------------------------------------------------------
 * ------------------------------------------------------------
 * v5.4: correction critique du scan : SCAN_TOURS comptait des tours CPU,
 *       pas des millisecondes. Sur ESP32 un PID muet pouvait donc etre
 *       declare mort avant meme que la reponse CAN ait eu le temps
 *       d arriver. Chaque emission dispose maintenant d une fenetre
 *       reelle de SCAN_WAIT_MS, sans bloquer HTTP. canSend29 ne bloque
 *       plus 20 ms apres chaque emission. /stat expose TX/RX/EFLG.
 *
 * v5.2: page. Jauge d'occupation flash live, rafraichie a la
 *       seconde depuis /stat : barre verte sous 70 %, orange a
 *       70, rouge a 90, libelle ko / ko, pourcentage, nombre de
 *       lignes, mention PLEINE quand logPlein est leve.
 *       Bouton PURGE arme en deux appuis, fenetre de 4 s, meme
 *       traitement visuel que le reset.
 *
 * v5.1: readBytesUntil remplace par une lecture caractere par
 *       caractere : la methode Stream porte un timeout de
 *       1000 ms par appel, une tempo cachee.
 *
 * v5.0: on journalise ce que le bus dit, pas ce qu'on attend.
 *       adrVu[256] est marque sur l'adresse SOURCE de toute
 *       trame recue pendant un scan, sans aucun rapport avec
 *       l'adresse interrogee au meme instant : une reponse
 *       tardive, ou un calculateur qui parle sans avoir ete
 *       sollicite, entre dans l'inventaire comme les autres.
 *       Les lignes # RAW portent scanSrc[], l'adresse qui a
 *       repondu, pour qu'un repondant inattendu reste visible.
 *       Plus aucun while dans le fichier : toutes les boucles
 *       sont bornees par un compteur. Sans watchdog arme, un
 *       while sur un registre SPI qui renvoie 0xFF ne sort
 *       jamais. Le drain RX passe par canDrain(), 64 passes.
 *
 * v4.9: sequence de scan conforme a la specification.
 *       Pre-header : tour complet des 256 adresses physiques en
 *       asynchrone, les repondantes s'accumulent dans un
 *       tableau. Aucune ecriture pendant le tour : un flush
 *       LittleFS immobilise la boucle et ferait perdre les
 *       reponses en vol. Le tour fini, le tampon RX est vide
 *       par pompage jusqu'a CANINTF nul - pas d'horloge - puis
 *       le tableau part en flash d'un bloc (# ADR xx, puis
 *       # BUS n ADRESSES).
 *       Le journal est ensuite relu (adrRelit) : la liste de
 *       travail vient du fichier, pas de la RAM, et le compte
 *       relu est ecrit (# RELECTURE n ADRESSES).
 *       Balayage : adresse par adresse, 256 PID, ce qui rentre
 *       va dans le tableau. La 256e emise, meme vidage du
 *       tampon RX, puis les 256 lignes partent en flash d'un
 *       bloc, puis l'adresse suivante.
 *       logWrite verifie le retour de println : une ecriture
 *       refusee leve logPlein et l'annonce sur le port serie
 *       au lieu de disparaitre.
 *
 * v4.8: inventaire du bus avant tout balayage, et suppression
 *       de la temporisation de scan.
 *       Etape 0 : 02 01 00 est emis sur les 256 adresses
 *       physiques 18DA<xx>F1. Toute reponse, positive ou 7F,
 *       prouve la presence d'un calculateur a xx ; la liste est
 *       ecrite (# BUS n ADRESSES, puis une ligne # ADR par
 *       cible). Plus aucune adresse n'est supposee connue.
 *       Etape 1 : les 256 PID du mode 01 sont balayees sur
 *       chaque adresse retenue. Le tableau est remis a l'init
 *       avant chaque cible et transcrit en entier apres :
 *       256 lignes # RAW <adr> <pid>, les muettes portant " -".
 *       Aucune horloge : un pas se termine des que la reponse
 *       attendue est entree, ou apres SCAN_TOURS pompes a vide
 *       pour une cible qui ne repond pas. SCAN_MS supprime.
 *       flagLoad accepte PH_ATTENTE : apres coupure demarreur,
 *       la reprise enchaine sur SCAN RUN au lieu de refaire un
 *       SCAN OFF moteur tournant.
 *
 * v4.7: le journal du scan devient exhaustif. scanVide() ne
 *       filtrait plus : une ligne "# RAW xx" par PID, avec ses
 *       octets ou " -" si muette. 256 lignes moteur arrete,
 *       256 lignes moteur tournant, systematiquement, quel que
 *       soit le nombre de repondantes. Drain de queue de 4 pas
 *       (100 ms) avant ecriture : les dernieres reponses ne se
 *       perdent plus.
 *
 * v4.6: deux corrections.
 *       Scan : les balayages emettaient sur l'adresse physique
 *       18DA10F1, muette sur ce calculateur - zero reponse sur
 *       les 256+256 PID alors que l'acquisition fonctionnelle
 *       repondait. Le scan emet desormais sur 18DB33F1.
 *       Telechargement : le bouton rapatriait le fichier sans
 *       jamais l'effacer. Il passe par fetch/blob et n'envoie
 *       PURGE! qu'apres reception effective ; un transfert
 *       interrompu laisse le journal intact.
 *
 * v4.5: ordre de session et codes defaut.
 *       Sequence : SCAN OFF -> attente demarrage -> SCAN RUN ->
 *       arbitrage A/B -> acquisition. L'arbitrage passe apres
 *       les deux scans : il se joue desormais moteur tournant,
 *       ou la reponse groupee est representative de ce que le
 *       calculateur sait faire en marche.
 *       Codes defaut : les trois modes de lecture sont la, 03
 *       memorises, 07 en attente, 0A permanents. Table de
 *       libelles francais en PROGMEM, sous-ensemble diesel de
 *       SAE J2012 / ISO 15031-6 (les familles allumage, sondes
 *       lambda et hybride sont ecartees : impossibles sur ce
 *       moteur). Un code hors table s'affiche seul.
 *       RESET! remplace CLEAR! : releve 03/07/0A et les donnees
 *       gelees, les ecrit dans le journal, efface (mode 04),
 *       puis releve a nouveau. Un code revenu en 03 juste apres
 *       est un defaut actif, pas un residu ; un 0A qui reste
 *       est normal tant que son moniteur n'a pas tourne.
 * ------------------------------------------------------------
 * v4.4: le scan devient asynchrone, comme l'acquisition.
 *       Defaut corrige : scanEcritReponse() comparait la trame
 *       recue a scanPid, deja incremente par le pas precedent.
 *       La comparaison echouait toujours, aucune ligne # RAW
 *       n'etait ecrite et brutN restait a 0.
 *       Le couplage requete/reponse est supprime. L'emission
 *       balaye 00..FF ; la pompe depose chaque reponse dans
 *       scanDat[pid], indexe par la PID que la reponse porte
 *       elle-meme. L'ordre d'arrivee n'a plus d'importance : une
 *       reponse en retard tombe dans sa case a un drain
 *       ulterieur, une PID muette laisse sa case vide. Le
 *       tableau est vide en debut de phase, ecrit d'un bloc a
 *       la fin.
 *       Un pas = une PID : on emet, on vide le tampon tant
 *       qu'il n'est pas vide, des qu'il est vide on passe a la
 *       suivante. Aucune attente, aucune sortie anticipee :
 *       le TX est sequentiel, le RX va dans le tableau, il n'y
 *       a aucun lien entre les deux. Un dernier pas de drain
 *       pur precede l'ecriture, pour la reponse de 0xFF.
 *       SCAN_MS passe de 100 a 25 ms : chaque scan tient en
 *       6,4 s au lieu de 26 s, 13 s pour les deux au lieu de 52.
 * ------------------------------------------------------------
 * v4.3: retour de la lecture des codes defaut et du
 *       dictionnaire SAE, retires a la reecriture.
 *         DTC  / DTCP  modes 03 et 07, memorises et en attente.
 *         PID xx       lit une PID, affiche les octets bruts
 *                      puis la valeur decodee.
 *       Ces trois commandes passent par obdAsk(), une requete
 *       bloquante a 300 ms en adressage physique : elle vide le
 *       bus a son propre compte et perturberait le tick, elle
 *       est donc refusee tant que le journal tourne.
 *       Le mode 04 (effacement des defauts) n'est pas remis :
 *       il efface aussi les donnees gelees, qui sont
 *       precisement l'interet du diagnostic.
 *       Le dictionnaire ne sert qu'a la relecture a l'ecran. Le
 *       journal continue d'ecrire des octets bruts.
 * ------------------------------------------------------------
 * v4.2a: enum Phase remonte avant toute fonction, le generateur
 *       de prototypes de l IDE Arduino le declarait trop tard.
 * ------------------------------------------------------------
 * v4.2: en-tete de session automatique + arbitrage automatique.
 *
 *       Sequence declenchee par LOG ON :
 *         1. ARBITRAGE  3 s   la requete groupee est emise et
 *            l'on compte les PID contenues dans la reponse.
 *            >= 5 PID reconnues -> mode A verrouille, sinon
 *            mode B (six requetes simples). Verdict ecrit dans
 *            le journal, affiche sur la page, memorise dans
 *            /mode.txt : les sessions suivantes demarrent
 *            directement sur le gagnant.
 *         2. SCAN OFF  26 s   balayage des 256 PID du mode 01,
 *            moteur arrete, adressage PHYSIQUE 18DA10F1, a
 *            10 requetes/s. Les bitmaps 00/20/40/60/80 ne sont
 *            pas consultes : ils sont declaratifs et ce
 *            calculateur repond a des PID qu'il omet de
 *            declarer. Toute reponse positive est ecrite telle
 *            quelle : # RAW <pid> <octets>.
 *         3. attente regime > 300 tr/min. Le reboot provoque
 *            par le demarreur est absorbe : l'avancement est
 *            dans /logon.txt, la reprise ne refait pas le scan
 *            deja termine.
 *         4. SCAN RUN  26 s   meme balayage, moteur tournant.
 *            La difference entre les deux listes est
 *            l'information : une PID qui ne repond qu'en
 *            marche est liee a la combustion.
 *         5. acquisition 1 Hz.
 *
 *       Le scan passe en adressage physique et non fonctionnel.
 *       En fonctionnel le BCM ou l'ABS renvoie un 7F avant que
 *       le moteur ne reponde : c'est ce qui avait fait declarer
 *       70, 71 et 77 non supportees a tort. Ce verdict est
 *       annule, elles sont re-balayees comme les 253 autres.
 *       6D (consigne de pression rail, structure Euro 5) est
 *       dans le lot.
 *
 *       Colonnes du CSV : les six PID au decodage certain, plus
 *       deux colonnes de tourniquet pid_brut / val_brut qui
 *       echantillonnent une PID vivante non identifiee par
 *       tick. Une valeur brute isolee ne dit rien, une serie
 *       temporelle mise en regard du regime et de la pedale se
 *       laisse identifier.
 *
 *       Emission bridee aux modes 01 et 22 en lecture. Jamais
 *       2E (writeDataByIdentifier), jamais 31 (routineControl).
 * ------------------------------------------------------------
 * v4.1: reecriture complete, boucle sans attente. canPump()
 *       vide les tampons MCP2515 a chaque tour de loop() :
 *       reponse diag 18DAF1xx -> reassemblage ISO-TP (SF/FF/CF
 *       + Flow Control) -> decodage -> depot dans la cellule.
 *       Toute autre trame est jetee sans etre memorisee. Le
 *       tick ecrit la ligne (cellule non remplie = vide), vide
 *       les cellules, emet la requete suivante, n'attend
 *       jamais. SPI porte a 10 MHz (etait 2 MHz) : en dessous
 *       la lecture d'une trame coute plus que son temps
 *       d'arrivee et la pompe prend du retard sur le bus.
 *       Supprime : per_ms, logDue[], logCand, logStrike[],
 *       logRetry[], logSup[], logAlive[], logDecl[], les deux
 *       passes, le budget de cycle, T_LOG_MS, l'etalement des
 *       echeances, les slots DID, le repli 11 bits.
 * ------------------------------------------------------------
 * v1.0 a v3.17 : acquisition sequentielle bloquante, une
 *       requete puis attente de la reponse. Abandonnee : le
 *       budget de cycle tronquait la fin de la liste des
 *       colonnes a chaque tour.
 * ============================================================ */

#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <StreamString.h>
#include <string.h>
#include <time.h>
#include <FS.h>
#include <LittleFS.h>

#define FW_VER "6.2"

/* ---------- WiFi point d'acces ------------------------------ */
#define AP_SSID   "NEMO-OBD"
#define AP_PASS   "nemo1234"
const IPAddress AP_IP(192,168,1,1);
const IPAddress AP_MASK(255,255,255,0);
WebServer server(80);

/* ---------- Brochage ---------------------------------------- */
#define PIN_CS    D7
#define PIN_INT   D6

/* ---------- Commandes SPI MCP2515 --------------------------- */
#define C_RESET   0xC0
#define C_READ    0x03
#define C_WRITE   0x02
#define C_RTS0    0x81
#define C_BITMOD  0x05

/* ---------- Registres MCP2515 ------------------------------- */
#define R_CANCTRL  0x0F
#define R_CANSTAT  0x0E
#define R_CNF1     0x2A
#define R_CNF2     0x29
#define R_CNF3     0x28
#define R_CANINTE  0x2B
#define R_CANINTF  0x2C
#define R_RXB0CTRL 0x60
#define R_RXB1CTRL 0x70
#define R_TXB0CTRL 0x30
#define R_TXB0SIDH 0x31
#define R_RXB0SIDH 0x61
#define R_RXB1SIDH 0x71
#define R_EFLG     0x2D
#define R_TEC      0x1C
#define R_REC      0x1D

/* ---------- Adressage diagnostic 29 bits --------------------
 * Fonctionnel : tous les calculateurs repondent, dont des 7F
 *               parasites. Sert a l'acquisition, ou ces 7F sont
 *               sans consequence (pompe non bloquante).
 * Physique    : moteur seul. Sert au scan, pour que la liste
 *               obtenue soit bien celle du MJD8F3.
 * ----------------------------------------------------------- */
#define ID29_FUNC 0x18DB33F1UL
#define ID29_PHYS 0x18DA10F1UL

/* ---------- Fichiers ---------------------------------------- */
#define LOG_FICH   "/log.csv"
#define LOG_FLAG   "/logon.txt"    /* etat de session, survit au reboot */
/* v6 : plus d'arbitrage A/B persistant */
#define LOG_MARGE  8192            /* octets libres mini avant arret    */
#define TP_BUF     64

/* ---------- Cadences ---------------------------------------- */
#define LOG_PERIOD_MS 100UL        /* cadence cible : 10 echantillons/s */
#define LOG_FLUSH_BYTES 4096UL
#define LOG_FLUSH_MS    500UL
/* SCAN_MS supprime en v4.8 : le scan n est plus temporise */

#define SCAN_WAIT_MS 50UL          /* attente reelle d une reponse CAN */

/* ---------- Phases de session -------------------------------
 * Declare ici, avant toute fonction : le generateur de
 * prototypes de l'IDE Arduino insere ses declarations juste
 * apres les #include. Un type defini plus bas dans le fichier
 * serait inconnu au moment ou il ecrit le prototype de
 * flagLoad().
 * ----------------------------------------------------------- */
enum Phase { PH_IDLE=0, PH_SCAN_OFF=1, PH_ATTENTE=2, PH_SCAN_RUN=3,
             PH_ACQ=5 };

bool mcpOk = false;
Print *OUT = &Serial;

/* ============================================================
 * Couche SPI bas niveau (reprise v3.17, horloge portee a 10 MHz)
 * ============================================================ */
static inline void csL(){ digitalWrite(PIN_CS, LOW); }
static inline void csH(){ digitalWrite(PIN_CS, HIGH); }

void mcpReset(){
  csL(); SPI.transfer(C_RESET); csH();
  delay(10);
}

uint8_t mcpRead(uint8_t reg){
  csL();
  SPI.transfer(C_READ); SPI.transfer(reg);
  uint8_t v = SPI.transfer(0x00);
  csH();
  return v;
}

void mcpWrite(uint8_t reg, uint8_t val){
  csL();
  SPI.transfer(C_WRITE); SPI.transfer(reg); SPI.transfer(val);
  csH();
}

void mcpWriteN(uint8_t reg, const uint8_t *buf, uint8_t n){
  csL();
  SPI.transfer(C_WRITE); SPI.transfer(reg);
  for(uint8_t i=0;i<n;i++) SPI.transfer(buf[i]);
  csH();
}

void mcpBitMod(uint8_t reg, uint8_t mask, uint8_t val){
  csL();
  SPI.transfer(C_BITMOD); SPI.transfer(reg);
  SPI.transfer(mask); SPI.transfer(val);
  csH();
}

/* 500 kbit/s sur quartz 16 MHz. Filtres materiels grands ouverts
   (RXM=11) : les reponses diag sont en 29 bits, le tri est fait
   en logiciel par canPump().                                    */
bool mcpInit(){
  mcpReset();
  mcpBitMod(R_CANCTRL, 0xE0, 0x80);
  if((mcpRead(R_CANSTAT) & 0xE0) != 0x80) return false;

  mcpWrite(R_CNF1, 0x00);
  mcpWrite(R_CNF2, 0xF0);
  mcpWrite(R_CNF3, 0x86);

  mcpWrite(R_RXB0CTRL, 0x64);   /* RXM=11 + rollover vers RXB1 */
  mcpWrite(R_RXB1CTRL, 0x60);
  mcpWrite(R_CANINTE, 0x00);    /* pas d'IRQ : scrutation      */

  mcpBitMod(R_CANCTRL, 0xE0, 0x00);
  return (mcpRead(R_CANSTAT) & 0xE0) == 0x00;
}

/* ============================================================
 * Emission / reception CAN
 * ============================================================ */
bool canSend29(uint32_t id, const uint8_t *data, uint8_t len){
  /* TXB0 ne doit pas etre ecrase tant que TXREQ est actif.
     On attend tres peu ici : le scan possede sa propre fenetre
     d'attente et le serveur HTTP doit rester reactif. */
  uint32_t t0 = millis();
  while(mcpRead(R_TXB0CTRL) & 0x08){
    if((uint32_t)(millis() - t0) >= 3){
      /* Pas d'ACK ou bus bloque : abandon explicite du TX. */
      mcpBitMod(R_TXB0CTRL, 0x08, 0x00);
      return false;
    }
    yield();
  }

  uint8_t hdr[5];
  hdr[0] = (id >> 21) & 0xFF;
  hdr[1] = (((id >> 18) & 0x07) << 5) | 0x08 | ((id >> 16) & 0x03);
  hdr[2] = (id >> 8) & 0xFF;
  hdr[3] = id & 0xFF;
  hdr[4] = len & 0x0F;

  mcpWriteN(R_TXB0SIDH, hdr, 5);
  mcpWriteN(R_TXB0SIDH + 5, data, len);

  csL(); SPI.transfer(C_RTS0); csH();

  /* RTS lance l'emission. Ne pas attendre ici : l'ESP32 doit
     continuer a servir /stat et /cmd pendant le scan. */
  return true;
}

/* une trame si disponible, 0 sinon. Ne bloque jamais. */
uint8_t canRecvAny(uint32_t *id, bool *ext, uint8_t *data){
  uint8_t intf = mcpRead(R_CANINTF);
  uint8_t base;
  if(intf & 0x01)      base = R_RXB0SIDH;
  else if(intf & 0x02) base = R_RXB1SIDH;
  else return 0;

  uint8_t sidh = mcpRead(base);
  uint8_t sidl = mcpRead(base + 1);
  uint8_t eid8 = mcpRead(base + 2);
  uint8_t eid0 = mcpRead(base + 3);
  uint8_t dlc  = mcpRead(base + 4) & 0x0F;
  if(dlc > 8) dlc = 8;
  for(uint8_t i=0;i<dlc;i++) data[i] = mcpRead(base + 5 + i);

  *ext = sidl & 0x08;
  if(*ext){
    *id = ((uint32_t)sidh << 21) | ((uint32_t)(sidl >> 5) << 18)
        | ((uint32_t)(sidl & 0x03) << 16) | ((uint32_t)eid8 << 8) | eid0;
  }else{
    *id = ((uint32_t)sidh << 3) | (sidl >> 5);
  }
  mcpBitMod(R_CANINTF, (base==R_RXB0SIDH)?0x01:0x02, 0x00);
  return dlc;
}

/* ============================================================
 * Colonnes nommees - PID au decodage certain
 * ============================================================ */
#define NCOL 5

struct Col {
  uint8_t     pid;
  float       step;
  const char *nom;
};

/* Une seule piste de pedale est volontairement retenue : PID 49. */
const Col COL[NCOL] = {
  {0x0C, 0.25f, "regime_trmin"},
  {0x23, 10.0f, "rail_kPa"    },
  {0x0B, 1.0f , "map_kPa"     },
  {0x49, 0.1f , "pedale_pct"  },
  {0x0D, 1.0f , "vitesse_kmh" }
};

float    cellVal[NCOL];
uint32_t cellMs[NCOL];      /* 0 = pas ecrite depuis le dernier tick */
float    cellShow[NCOL];    /* derniere valeur connue, pour l'IHM    */
bool     cellSeen[NCOL];

float    regimeNow = 0;     /* detection du demarrage moteur         */

/* decodage SAE J1979 des cinq PID retenues.
   0C = AB/4 . 23 = AB x10 . 0B = A . 49 et 04 = A x100/255 . 0D = A */
static bool pidDecode(uint8_t pid, const uint8_t *v, uint8_t n, float *out){
  switch(pid){
    case 0x0C: if(n < 2) return false;
               *out = (((uint16_t)v[0] << 8) | v[1]) / 4.0f;     return true;
    case 0x23: if(n < 2) return false;
               *out = (((uint16_t)v[0] << 8) | v[1]) * 10.0f;    return true;
    case 0x0B: if(n < 1) return false; *out = v[0];              return true;
    case 0x49:
    case 0x04: if(n < 1) return false;
               *out = v[0] * 100.0f / 255.0f;                    return true;
    case 0x0D: if(n < 1) return false; *out = v[0];              return true;
  }
  return false;
}

/* ============================================================
 * Tourniquet des PID vivantes non nommees
 * Le scan remplit brutPid[]. Une PID par tick est interrogee et
 * journalisee brute. Une valeur isolee ne dit rien ; une serie
 * temporelle mise en regard du regime et de la pedale se laisse
 * identifier.
 * ============================================================ */
#define BRUT_MAX 48
uint8_t  brutPid[BRUT_MAX];
uint8_t  brutN   = 0;
uint8_t  brutIdx = 0;
uint8_t  brutEnCours = 0xFF;
uint32_t brutVal = 0;
bool     brutOk  = false;

/* Ordonnanceur d'acquisition : une PID par requete, une reponse ou
 * un timeout avant la suivante. */
#define ACQ_WAIT_MS 150UL
uint8_t  acqSlot = 0;
bool     acqPending = false;
uint8_t  acqPidPending = 0xFF;
uint32_t acqDeadline = 0;
uint32_t acqRspSeq = 0;
uint32_t acqRspAtSend = 0;

static bool brutConnue(uint8_t p){
  for(uint8_t i=0;i<NCOL;i++) if(COL[i].pid == p) return true;
  return false;
}

static void brutAjoute(uint8_t p){
  if(brutConnue(p)) return;                       /* deja une colonne  */
  if(p == 0x00 || p == 0x20 || p == 0x40
  || p == 0x60 || p == 0x80 || p == 0xA0) return; /* bitmaps, inutiles */
  for(uint8_t i=0;i<brutN;i++) if(brutPid[i] == p) return;
  if(brutN < BRUT_MAX) brutPid[brutN++] = p;
}

/* ============================================================
 * Depot des valeurs dans les cellules
 * ============================================================ */

static void cellPut(uint8_t pid, const uint8_t *v, uint8_t n){

  if(pid == brutEnCours){                    /* colonne de tourniquet */
    uint32_t x = 0;
    for(uint8_t i=0;i<n && i<4;i++) x = (x << 8) | v[i];
    brutVal = x; brutOk = true;
  }

  float x;
  if(!pidDecode(pid, v, n, &x)) return;
  if(acqPending && pid == acqPidPending) acqRspSeq++;
  if(pid == 0x0C) regimeNow = x;
  for(uint8_t i=0;i<NCOL;i++){
    if(COL[i].pid != pid) continue;
    cellVal[i]  = x;
    cellMs[i]   = millis();
    cellShow[i] = x;
    cellSeen[i] = true;
  }
}

/* longueur normalisee des PID que l'on sait decouper dans une
   reponse chainee. 0 = inconnue, on ne sait pas avancer.        */
static uint8_t pidLen(uint8_t pid){
  switch(pid){
    case 0x0C: case 0x23: return 2;
    case 0x0B: case 0x49: case 0x04: case 0x0D: return 1;
  }
  return 0;
}

/* une reponse mode 01 peut chainer plusieurs PID :
   41 <pid> <donnees> <pid> <donnees> ...
   Si la premiere PID n'est pas decoupable, la reponse est
   forcement mono-PID : tout le reste lui appartient.            */
static void rspSplit(const uint8_t *b, uint16_t n){
  if(n < 2 || b[0] != 0x41) return;
  uint16_t k = 1;
  bool premier = true;
  for(uint16_t g=0; g<64 && k < n; g++){
    uint8_t p = b[k];
    uint8_t l = pidLen(p);
    if(!l){
      if(premier) cellPut(p, b + k + 1, n - k - 1);
      return;
    }
    if(k + 1 + l > n) return;
    cellPut(p, b + k + 1, l);
    k += 1 + l;
    premier = false;
  }
}

/* ============================================================
 * Pompe CAN - appelee a chaque tour de loop(), non bloquante
 * Reponse diag 18DAF1xx : reassemblage ISO-TP.
 * Tout le reste : jete, sans memoire.
 * ============================================================ */
uint8_t  tpBuf[TP_BUF];
uint16_t tpTotal = 0;
uint16_t tpGot   = 0;
uint8_t  tpSN    = 0;

uint32_t nRx = 0, nRsp = 0, nDrop = 0, nNeg = 0, nTx = 0;

/* ---------- Tableau du scan ---------------------------------
 * 256 entrees, une par PID. La pompe y depose ce qui arrive,
 * quand ca arrive : aucun lien entre l'ordre d'emission et
 * l'ordre de reception. Le tableau est vide en debut de phase,
 * ecrit d'un bloc a la fin.
 * ----------------------------------------------------------- */
#define SCAN_DAT 6                /* octets utiles retenus par PID */
uint8_t  scanDat[256][SCAN_DAT];
uint8_t  scanLen[256];            /* 0 = pas de reponse            */
uint8_t  scanSrc[256];            /* adresse source de la reponse  */
uint8_t  adrVu[256];              /* 1 = cette adresse a parle     */
bool     scanActif = false;       /* la pompe remplit le tableau   */
bool     scanNeg   = false;       /* 7F recu depuis la derniere emission */
bool     scanFini  = false;       /* toutes les adresses balayees  */
uint8_t  adrPid    = 0;           /* adresse interrogee (etape 0)  */
bool     adrRep    = false;       /* cette adresse a repondu       */
uint8_t  rspAdr    = 0;           /* source de la derniere trame   */

static void scanRaz(){
  memset(scanLen, 0, sizeof(scanLen));
  memset(scanSrc, 0, sizeof(scanSrc));
  scanNeg = false;
}

static void rspTraite(const uint8_t *b, uint16_t n){
  if(n < 1) return;
  if(scanActif){
    adrVu[rspAdr] = 1;              /* qui a parle, attendu ou pas */
    if(rspAdr == adrPid) adrRep = true;
  }
  if(b[0] == 0x7F){
    nNeg++;
    if(scanActif) scanNeg = true;
    else if(n >= 3 && acqPending && b[2] == acqPidPending)
      acqRspSeq++;
    return;
  }
  if(b[0] != 0x41 || n < 2) return;

  if(scanActif){                  /* depot dans le tableau du scan */
    uint8_t p = b[1];
    uint8_t l = n - 2;
    if(l > SCAN_DAT) l = SCAN_DAT;
    memcpy(scanDat[p], b + 2, l);
    scanSrc[p] = rspAdr;          /* qui a repondu a cette PID     */
    scanLen[p] = l ? l : 1;       /* reponse vide = presente quand meme */
    return;                       /* pas de decodage pendant le scan   */
  }
  rspSplit(b, n);
}

/* Vidage du tampon RX. Boucle bornee : un SPI decroche renvoie
   0xFF en permanence, un while sur CANINTF ne sortirait jamais
   et le watchdog reprendrait la main. 64 passes suffisent, le
   MCP2515 n'a que deux tampons de reception.                  */
static void canDrain(){
  for(uint8_t g=0; g<64; g++){
    if(!(mcpRead(R_CANINTF) & 0x03)) return;
    canPump();
  }
}

static void canFc(uint32_t rspId){
  uint8_t fc[8] = {0x30, 0x00, 0x05, 0, 0, 0, 0, 0};   /* CTS, STmin 5 ms */
  canSend29(0x18DA00F1UL | ((rspId & 0xFF) << 8), fc, 8);
}

void canPump(){
  uint32_t id; bool ext; uint8_t d[8];
  uint8_t n;

  for(uint8_t garde=0; garde<32; garde++){
    n = canRecvAny(&id, &ext, d);
    if(!n) return;
    nRx++;
    if(!(ext && (id & 0xFFFFFF00UL) == 0x18DAF100UL)){ nDrop++; continue; }
    nRsp++;
    rspAdr = (uint8_t)(id & 0xFF);      /* qui parle */

    uint8_t pci = d[0] >> 4;

    if(pci == 0){                                    /* single frame */
      uint8_t len = d[0] & 0x0F;
      if(len > 7) len = 7;
      rspTraite(d + 1, len);
      tpTotal = 0;
      continue;
    }

    if(pci == 1){                                    /* first frame  */
      tpTotal = ((uint16_t)(d[0] & 0x0F) << 8) | d[1];
      if(tpTotal > TP_BUF) tpTotal = TP_BUF;
      tpGot = 0;
      for(uint8_t i=2;i<8 && tpGot<tpTotal;i++) tpBuf[tpGot++] = d[i];
      tpSN = 1;
      canFc(id);
      continue;
    }

    if(pci == 2 && tpTotal){                         /* consecutive  */
      if((d[0] & 0x0F) != (tpSN & 0x0F)){ tpTotal = 0; continue; }
      tpSN++;
      for(uint8_t i=1;i<8 && tpGot<tpTotal;i++) tpBuf[tpGot++] = d[i];
      if(tpGot >= tpTotal){ rspTraite(tpBuf, tpTotal); tpTotal = 0; }
    }
  }
}

/* ============================================================
 * Emission des requetes - aucune attente
 * Modes 01 et 22 en lecture seule. Jamais 2E, jamais 31.
 * ============================================================ */

static void askOne(uint8_t pid, uint32_t addr){
  uint8_t f[8] = {0x02, 0x01, pid, 0,0,0,0,0};
  if(canSend29(addr, f, 8)) nTx++;
}

/* Acquisition : UNE PID par requete.
 * On attend la reponse (ou un timeout court) avant d'envoyer la
 * suivante. Cela rend le debit deterministe et evite de dependre
 * d'une capacite constructeur a accepter une requete groupee.
 *
 * Les 5 PID nommees passent en priorite. Une PID brute est injectee
 * ensuite, puis le cycle recommence. */
static void acqSendNext(){
  uint8_t pid = 0xFF;
  bool isRaw = false;

  if(acqSlot < NCOL){
    pid = COL[acqSlot].pid;
    acqSlot++;
  }else{
    if(brutN){
      if(brutIdx >= brutN) brutIdx = 0;
      pid = brutPid[brutIdx++];
      isRaw = true;
    }
    acqSlot = 0;
  }

  if(pid == 0xFF) return;

  if(isRaw){
    brutEnCours = pid;
    brutOk = false;
  }else{
    brutEnCours = 0xFF;
  }

  uint8_t f[8] = {0x02, 0x01, pid, 0,0,0,0,0};
  if(canSend29(ID29_FUNC, f, 8)){
    nTx++;
    acqPidPending = pid;
    acqPending = true;
    acqRspAtSend = acqRspSeq;
    acqDeadline = millis() + ACQ_WAIT_MS;
  }
}

static void acqTick(){
  if(!acqPending){
    acqSendNext();
    return;
  }

  if(acqRspSeq != acqRspAtSend){
    acqPending = false;
    acqSendNext();
    return;
  }

  if((int32_t)(millis() - acqDeadline) >= 0){
    acqPending = false;
    acqSendNext();
  }
}

/* ============================================================
 * Horodatage - base poussee par le telephone (TIME!)
 * ============================================================ */
uint32_t modEpoch   = 0;
uint32_t modEpochMs = 0;

static void tsNow(uint32_t ms, char *buf, size_t max){
  if(modEpoch){
    time_t t = modEpoch + (ms - modEpochMs) / 1000;
    struct tm tm; gmtime_r(&t, &tm);
    snprintf(buf, max, "%04d-%02d-%02d %02d:%02d:%02d.%03u",
             tm.tm_year+1900, tm.tm_mon+1, tm.tm_mday,
             tm.tm_hour, tm.tm_min, tm.tm_sec,
             (unsigned)((ms - modEpochMs) % 1000));
  }else{
    snprintf(buf, max, "%lu.%03lu", (unsigned long)(ms/1000),
             (unsigned long)(ms%1000));
  }
}

/* ============================================================
 * Journal LittleFS - append only, survit reboot / reflash
 * ============================================================ */
File     logFile;
bool     logFs    = false;
bool     logMode  = false;
bool     logEcho  = false;
bool     logPlein = false;
uint32_t logLines = 0;
uint32_t logNext  = 0;
uint32_t logBufBytes = 0;
uint32_t logLastFlush = 0;
uint16_t markNum  = 0;
char     logLastLine[224] = "";
char     uiStatus[80] = "PRET - appuyez sur START LOG";

Phase    phase   = PH_IDLE;
uint8_t  scanPid = 0;
uint8_t  scanEmis = 0;            /* derniere PID emise, attendue en RX */

static const char *phaseNom(){
  switch(phase){
    case PH_SCAN_OFF: return "scan moteur arrete";
    case PH_ATTENTE:  return "demarrez le moteur";
    case PH_SCAN_RUN: return "scan moteur tournant";
    case PH_ACQ:      return "acquisition";
    default:          return "arret";
  }
}

/* progression du header : survit au reboot du demarreur */
static void flagSave(){
  File f = LittleFS.open(LOG_FLAG, "w");
  if(!f) return;
  f.print((int)phase);
  f.close();
}

static Phase flagLoad(){
  File f = LittleFS.open(LOG_FLAG, "r");
  if(!f) return PH_IDLE;
  int p = f.parseInt();
  f.close();
  if(p < PH_ATTENTE || p > PH_ACQ) return PH_IDLE;
  return (Phase)p;
}

static void logHeaderCsv(char *out, size_t max){
  size_t k = snprintf(out, max, "horodatage,ms");
  for(uint8_t i=0;i<NCOL;i++)
    k += snprintf(out+k, max-k, ",%s", COL[i].nom);
  snprintf(out+k, max-k, ",pid_brut,val_brut");
}

static bool logOpen(){
  if(!logFs) return false;
  bool neuf = !LittleFS.exists(LOG_FICH);
  logFile = LittleFS.open(LOG_FICH, "a");
  if(!logFile) return false;
  if(neuf){
    char h[224]; logHeaderCsv(h, sizeof(h));
    logFile.println(h);
    logFlush();
  }else{
    /* Reconstitue le compteur apres un reboot sans modifier le fichier. */
    File r = LittleFS.open(LOG_FICH, "r");
    if(r){
      logLines = 0;
      char ln[256];
      while(r.available()){
        size_t n=0;
        while(r.available() && n<sizeof(ln)-1){ int c=r.read(); if(c=='\n') break; if(c!='\r') ln[n++]=(char)c; }
        ln[n]=0;
        if(n && ln[0]!='#' && strncmp(ln,"horodatage",10)!=0) logLines++;
      }
      r.close();
    }
  }
  logLastFlush = millis();
  return true;
}

static void logFlush(){
  if(!logFile) return;
  logFile.flush();
  logBufBytes = 0;
  logLastFlush = millis();
}

static void logWrite(const char *s){
  if(logFile){
    size_t w = logFile.println(s);
    if(!w && !logPlein){
      logPlein = true;
      Serial.println("# ECRITURE FLASH REFUSEE");
    }
    if(w) logBufBytes += w;
    if(logBufBytes >= LOG_FLUSH_BYTES || millis() - logLastFlush >= LOG_FLUSH_MS)
      logFlush();
  }
  if(logEcho) Serial.println(s);
}

static void logWriteTs(const char *quoi){
  char ts[32], l[80];
  tsNow(millis(), ts, sizeof(ts));
  snprintf(l, sizeof(l), "# %s %s", quoi, ts);
  logWrite(l);
}

/* ============================================================
 * Acquisition v6 : pas d'arbitrage multi-PID.
 * Le format d'acquisition est fixe et deterministe.
 * ============================================================ */

/* ============================================================
 * Phases 2 et 4 : inventaire du bus, puis 256 PID par adresse
 * ------------------------------------------------------------
 * Etape 0 : les 256 adresses physiques 18DA<xx>F1 recoivent
 *           02 01 00. Toute reponse - positive ou 7F - prouve
 *           la presence d'un calculateur a xx. La liste est
 *           ecrite dans le journal.
 * Etape 1 : pour chaque adresse retenue, les 256 PID du mode 01
 *           sont balayees sur cette adresse. Le tableau est
 *           remis a l'init avant chaque cible et transcrit en
 *           entier apres : 256 lignes, muettes comprises.
 *
 * Aucune temporisation. Un pas se termine des que la reponse
 * attendue est entree, ou apres SCAN_TOURS passages a vide de
 * la pompe pour une cible muette.
 * ============================================================ */
#define SCAN_TOURS 400            /* diagnostic uniquement, plus decisionnel */
#define ADR_MAX    16             /* calculateurs retenus au maximum */

uint8_t  adrTab[ADR_MAX];
uint8_t  adrN    = 0;             /* adresses trouvees               */
uint8_t  adrIdx  = 0;             /* adresse en cours de balayage    */
uint8_t  scanEtape = 0;           /* 0 = adresses, 1 = PID           */
uint16_t scanTours = 0;           /* compteur de pompes, diagnostic      */
uint32_t scanDeadline = 0;          /* fin reelle de la fenetre d attente */
uint32_t scanCible = 0;           /* identifiant CAN de la cible     */

static uint32_t adrId(uint8_t xx){
  return 0x18DA00F1UL | ((uint32_t)xx << 8);
}

/* ---------- etape 0 : inventaire ----------------------------
 * Rien n'est ecrit pendant le balayage : un flush LittleFS
 * immobilise la boucle et fait perdre les reponses en vol. Le
 * marquage se fait dans adrVu[] sur l'adresse SOURCE de la
 * trame recue, jamais sur l'adresse interrogee : une reponse
 * tardive, ou une adresse qui parle sans avoir ete sollicitee,
 * est retenue comme les autres. La flash est ecrite une seule
 * fois, tampon RX vide, a la fin du tour complet.           */
static void adrEcrit(){
  char l[96];
  adrN = 0;
  for(uint16_t x=0;x<256;x++){
    if(!adrVu[x]) continue;
    snprintf(l, sizeof(l), "# ADR %02X  (18DA%02XF1 -> 18DAF1%02X)",
             (unsigned)x, (unsigned)x, (unsigned)x);
    logWrite(l);
    if(adrN < ADR_MAX) adrTab[adrN++] = (uint8_t)x;
  }
  snprintf(l, sizeof(l), "# BUS %u ADRESSES", adrN);
  logWrite(l);
  logFlush();
}

/* ---------- relecture du journal ----------------------------
 * La liste de travail n'est pas celle de la RAM : le fichier
 * est rouvert en lecture et les lignes # ADR du bloc courant
 * sont relues. Le compte relu est ecrit, meme s'il est nul. */
static void adrRelit(){
  adrN = 0;
  if(logFile) logFile.flush();
  File f = LittleFS.open(LOG_FICH, "r");
  if(!f){ logWrite("# RELECTURE IMPOSSIBLE - FICHIER ABSENT"); return; }

  char ln[128];
  for(uint32_t g=0; g<200000 && f.available(); g++){
    size_t k = 0;                     /* lecture directe, sans Stream */
    for(uint16_t c=0; c<sizeof(ln)-1 && f.available(); c++){
      int ch = f.read();
      if(ch < 0 || ch == '\n') break;
      if(ch != '\r') ln[k++] = (char)ch;
    }
    ln[k] = 0;
    if(!strncmp(ln, "# SCAN OFF", 10) || !strncmp(ln, "# SCAN RUN", 10))
      adrN = 0;                       /* nouveau bloc : on repart    */
    else if(!strncmp(ln, "# ADR ", 6)){
      unsigned v = 0;
      if(sscanf(ln + 6, "%2X", &v) == 1 && adrN < ADR_MAX)
        adrTab[adrN++] = (uint8_t)v;
    }
  }
  f.close();

  char l[64];
  snprintf(l, sizeof(l), "# RELECTURE %u ADRESSES", adrN);
  logWrite(l);
}

static void adrPas(){
  if(adrPid == 0xFF){                  /* tour complet termine       */
    canDrain();                        /* tampon RX vide, borne      */
    adrEcrit();                        /* tout ce qui a parle        */
    adrRelit();                        /* liste relue dans le fichier*/
    scanEtape = 1;
    adrIdx    = 0;
    if(!adrN){                         /* bus muet : rien a balayer  */
      logWrite("# AUCUNE ADRESSE - SCAN ABANDONNE");
      scanActif = false;
      scanFini  = true;
      return;
    }
    scanCible = adrId(adrTab[0]);
    scanPid   = 0;
    scanRaz();
    char l[48];
    snprintf(l, sizeof(l), "# SCAN ADR %02X", adrTab[0]);
    logWrite(l);
    scanEmis  = 0;
    scanNeg   = false;
    scanTours = 0;
    askOne(0x00, scanCible);
    scanDeadline = millis() + SCAN_WAIT_MS;
    return;
  }
  adrPid++;
  adrRep    = false;
  scanTours = 0;
  askOne(0x00, adrId(adrPid));         /* 02 01 00 a l'adresse xx    */
  scanDeadline = millis() + SCAN_WAIT_MS;
}

/* ---------- etape 1 : 256 PID de l'adresse courante --------- */
static void scanVide(){
  uint16_t n = 0;
  char l[160];
  for(uint16_t p=0;p<256;p++){
    /* l'adresse portee est celle qui a REPONDU, pas celle qu'on
       a interrogee : un repondant inattendu reste visible      */
    size_t k = snprintf(l, sizeof(l), "# RAW %02X %02X",
                        scanLen[p] ? (unsigned)scanSrc[p]
                                   : (unsigned)adrTab[adrIdx],
                        (unsigned)p);
    if(!scanLen[p]){
      snprintf(l+k, sizeof(l)-k, " -");    /* case restee a l'init   */
      logWrite(l);
      continue;
    }
    for(uint8_t i=0;i<scanLen[p] && k < sizeof(l)-4;i++)
      k += snprintf(l+k, sizeof(l)-k, " %02X", scanDat[p][i]);
    logWrite(l);
    brutAjoute((uint8_t)p);
    n++;
  }
  snprintf(l, sizeof(l), "# ADR %02X REPONDANTES %u / 256",
           adrTab[adrIdx], n);
  logWrite(l);
  logFlush();
}

static void scanPas(){
  if(scanPid == 0xFF){                 /* adresse terminee           */
    canDrain();                        /* tampon RX vide, borne      */
    scanVide();
    if(++adrIdx >= adrN){              /* plus d'adresse             */
      scanActif = false;
      scanFini  = true;
      return;
    }
    scanCible = adrId(adrTab[adrIdx]);
    scanPid   = 0;
    scanRaz();
    char l[48];
    snprintf(l, sizeof(l), "# SCAN ADR %02X", adrTab[adrIdx]);
    logWrite(l);
  }else scanPid++;

  scanEmis  = scanPid;
  scanNeg   = false;
  scanTours = 0;
  askOne(scanPid, scanCible);
  scanDeadline = millis() + SCAN_WAIT_MS;
}

/* ---------- avancement, commande par la reception ------------ */
/* Appelee a chaque tour de loop() tant que la phase est un scan.
   Elle ne consulte aucune horloge : le pas suivant part des que
   la reponse est entree, ou apres SCAN_TOURS pompes a vide.   */
static void scanTour(){
  /* Une reponse CAN n'arrive pas "au prochain tour de loop".
     L'ancien code comptait 400 tours de CPU : sur ESP32 cela
     pouvait durer bien moins d'une milliseconde et le PID etait
     deja considere muet. On attend maintenant une vraie duree
     en millisecondes, tout en laissant loop() repasser par
     server.handleClient(). */
  canPump();

  bool recu = (scanEtape == 0) ? adrRep
                               : (scanLen[scanEmis] != 0 || scanNeg);

  if(recu){
    if(scanEtape == 0) adrPas();
    else               scanPas();
    return;
  }

  if((int32_t)(millis() - scanDeadline) < 0){
    scanTours++;
    return;
  }

  if(scanEtape == 0) adrPas();
  else               scanPas();

  if(!scanFini) return;

  /* fin de la phase de scan */
  if(phase == PH_SCAN_OFF){
    logWrite("# FIN SCAN OFF");
    snprintf(uiStatus, sizeof(uiStatus), "SCAN 1 OK - DEMARREZ LE MOTEUR");
    phase = PH_ATTENTE;
  }else{
    logWrite("# FIN SCAN RUN");
    snprintf(uiStatus, sizeof(uiStatus), "SCAN 2 OK - LOG EN COURS");
    char l[64];
    snprintf(l, sizeof(l), "# PID BRUTES SUIVIES %u", brutN);
    logWrite(l);
    logWrite("# ACQUISITION HAUTE CADENCE");
    phase   = PH_ACQ;
    logNext = millis() + LOG_PERIOD_MS;
    flagSave();
  }
  flagSave();
}

/* mise en place d'une phase de scan */
static void scanDebut(Phase ph, const char *quoi){
  phase      = ph;
  scanEtape  = 0;
  adrN       = 0;
  adrIdx     = 0;
  adrPid     = 0;
  adrRep     = false;
  scanPid    = 0;
  scanEmis   = 0;
  scanFini   = false;
  scanTours  = 0;
  scanActif  = true;
  memset(adrVu, 0, sizeof(adrVu));
  scanRaz();
  logWriteTs(quoi);
  flagSave();
  askOne(0x00, adrId(0x00));           /* premiere adresse du bus */
  scanDeadline = millis() + SCAN_WAIT_MS;
}

/* ============================================================
 * Phase 5 : acquisition, un tick par seconde
 * ============================================================ */
void logTickAcq(){
  char ts[32];
  uint32_t now = millis();
  tsNow(now, ts, sizeof(ts));

  size_t k = snprintf(logLastLine, sizeof(logLastLine), "%s,%lu",
                      ts, (unsigned long)now);
  for(uint8_t i=0;i<NCOL;i++){
    if(cellMs[i]){
      uint8_t dec = (COL[i].step >= 1.0f) ? 0 : 1;
      k += snprintf(logLastLine+k, sizeof(logLastLine)-k, ",%.*f",
                    dec, (double)cellVal[i]);
    }else{
      k += snprintf(logLastLine+k, sizeof(logLastLine)-k, ",");
    }
  }
  if(brutEnCours != 0xFF && brutOk)
    snprintf(logLastLine+k, sizeof(logLastLine)-k, ",%02X,%lu",
             brutEnCours, (unsigned long)brutVal);
  else
    snprintf(logLastLine+k, sizeof(logLastLine)-k, ",,");

  logWrite(logLastLine);
  logLines++;

  for(uint8_t i=0;i<NCOL;i++) cellMs[i] = 0;
  brutOk = false;

  if(LittleFS.totalBytes() - LittleFS.usedBytes() < LOG_MARGE){
    logFlush();
    logWrite("# FLASH PLEINE");
    logFlush();
    logPlein = true;
    logMode  = false;
    phase    = PH_IDLE;
    if(logFile) logFile.close();
    LittleFS.remove(LOG_FLAG);
    return;
  }

  /* L'ordonnanceur CAN tourne independamment du rythme CSV.
     La ligne CSV photographie les dernieres valeurs disponibles. */
}

/* ============================================================
 * Marche / arret de session
 * ============================================================ */
void logStart(bool reprise){
  if(logMode) return;
  if(!logOpen()){ OUT->println("# LOG: flash indisponible"); return; }
  logPlein = false;
  logMode  = true;
  for(uint8_t i=0;i<NCOL;i++){ cellMs[i] = 0; cellSeen[i] = false; }
  brutOk = false; brutEnCours = 0xFF;
  acqSlot = 0;
  acqPending = false;
  acqPidPending = 0xFF;
  acqRspAtSend = acqRspSeq;

  Phase repris = reprise ? flagLoad() : PH_IDLE;
  char l[80];
  snprintf(l, sizeof(l), "# LOG ON v%s", FW_VER);
  logWrite(l);
  logWriteTs("SESSION");

  if(repris >= PH_ATTENTE){
    phase = repris;
    logWrite("# REPRISE apres coupure");
    if(phase == PH_SCAN_RUN) scanDebut(PH_SCAN_RUN, "SCAN RUN");
    if(phase == PH_ACQ){
      logNext = millis() + LOG_PERIOD_MS;
      logWrite("# REPRISE ACQUISITION");
    }
  }else{
    logLines = 0;
    brutN = 0; brutIdx = 0;
    scanDebut(PH_SCAN_OFF, "SCAN OFF");
  }
  flagSave();
  OUT->print("# session lancee, phase "); OUT->println(phaseNom());
}

void logStop(){
  if(!logMode) return;
  logMode = false;
  phase   = PH_IDLE;
  logWrite("# LOG OFF");
  logFlush();
  if(logFile) logFile.close();
  LittleFS.remove(LOG_FLAG);
  for(uint8_t i=0;i<NCOL;i++) cellSeen[i] = false;
  OUT->println("# LOG OFF");
}

void logPurge(){
  if(logMode) logStop();
  LittleFS.remove(LOG_FICH);
  LittleFS.remove(LOG_FLAG);
  logLines = 0;
  brutN = 0; brutIdx = 0;
  OUT->println("# PURGE OK");
}

static void downloadDone(){
  if(logMode) logStop();
  LittleFS.remove(LOG_FICH);
  LittleFS.remove(LOG_FLAG);
  logLines = 0;
  brutN = 0; brutIdx = 0;
  OUT->println("# TELECHARGEMENT TERMINE - RESET");
  delay(100);
  ESP.restart();
}

/* ============================================================
 * Machine d'etat de session - appelee a chaque tour de loop()
 * ============================================================ */
void sessionTick(){
  if(!logMode) return;
  uint32_t now = millis();

  switch(phase){
    case PH_SCAN_OFF:
    case PH_SCAN_RUN:
      scanTour();
      break;

    case PH_ATTENTE: {
      static uint32_t tR = 0;
      if(regimeNow > 300){
        scanDebut(PH_SCAN_RUN, "SCAN RUN");
      }else if((int32_t)(now - tR) >= 0){
        tR = now + 250;
        askOne(0x0C, ID29_FUNC);
      }
      break;
    }

    case PH_ACQ:
      if((int32_t)(now - logNext) >= 0){
        logTickAcq();
        logNext = now + LOG_PERIOD_MS;
      }
      acqTick();
      break;

    default: break;
  }
}

/* ============================================================
 * Requete ponctuelle bloquante
 * Reservee aux commandes interactives (DTC, PID) : elle vide le
 * bus a son propre compte et perturberait le tick. Refusee tant
 * que le journal tourne.
 * Retourne la longueur utile, 0 si pas de reponse.
 * ============================================================ */
static uint16_t obdAsk(const uint8_t *req, uint8_t reqLen,
                       uint8_t *rsp, uint16_t rspMax){
  uint8_t f[8] = {0,0,0,0,0,0,0,0};
  f[0] = reqLen;
  for(uint8_t i=0;i<reqLen && i<7;i++) f[1+i] = req[i];
  if(!canSend29(ID29_PHYS, f, 8)) return 0;
  nTx++;

  uint16_t total = 0, got = 0;
  uint8_t  sn = 1;
  uint32_t id; bool ext; uint8_t d[8];
  uint32_t t0 = millis();

  for(uint32_t g=0; g<500000 && millis() - t0 < 300; g++){
    uint8_t n = canRecvAny(&id, &ext, d);
    if(!n) continue;
    if(!(ext && (id & 0xFFFFFF00UL) == 0x18DAF100UL)) continue;
    uint8_t pci = d[0] >> 4;

    if(pci == 0){
      uint8_t len = d[0] & 0x0F;
      if(len > 7) len = 7;
      if(d[1] == 0x7F){
        /* seul 7F xx 78 (reponse en attente) justifie de patienter */
        if(len >= 3 && d[3] == 0x78){ t0 = millis(); continue; }
        return 0;
      }
      if(len > rspMax) len = rspMax;
      memcpy(rsp, d + 1, len);
      return len;
    }
    if(pci == 1){
      total = ((uint16_t)(d[0] & 0x0F) << 8) | d[1];
      if(total > rspMax) total = rspMax;
      got = 0;
      for(uint8_t i=2;i<8 && got<total;i++) rsp[got++] = d[i];
      canFc(id);
      sn = 1;
      t0 = millis();
    }
    if(pci == 2 && total){
      if((d[0] & 0x0F) != (sn & 0x0F)) return 0;
      sn++;
      for(uint8_t i=1;i<8 && got<total;i++) rsp[got++] = d[i];
      t0 = millis();
      if(got >= total) return total;
    }
  }
  return 0;
}

static bool interdit(){
  if(logMode){
    OUT->println("# occupe : journal en cours, faites LOG OFF");
    return true;
  }
  if(!mcpOk){ OUT->println("# MCP2515 injoignable"); return true; }
  return false;
}

/* ============================================================
 * Libelles des codes defaut - SAE J2012 / ISO 15031-6
 * Sous-ensemble pertinent pour un diesel a rampe commune : les
 * familles allumage, sondes lambda et hybride sont ecartees,
 * elles ne peuvent pas apparaitre sur ce moteur. Un code absent
 * de la table est affiche seul, sans libelle.
 * ============================================================ */
struct Dtc { uint16_t code; const char *txt; };

const Dtc DTCTXT[] PROGMEM = {
  {0x0001,"Regulateur volume carburant - circuit ouvert"},
  {0x0002,"Regulateur volume carburant - plage/performance"},
  {0x0003,"Regulateur volume carburant - circuit trop bas"},
  {0x0004,"Regulateur volume carburant - circuit trop haut"},
  {0x0005,"Electrovanne coupure carburant - circuit ouvert"},
  {0x0006,"Electrovanne coupure carburant - circuit trop bas"},
  {0x0007,"Electrovanne coupure carburant - circuit trop haut"},
  {0x0008,"Calage moteur ligne 1 - performance"},
  {0x0016,"Vilebrequin/arbre a cames capteur A - correlation"},
  {0x0017,"Vilebrequin/arbre a cames capteur B - correlation"},
  {0x0033,"Electrovanne decharge turbo - panne du circuit"},
  {0x0034,"Electrovanne decharge turbo - circuit trop bas"},
  {0x0035,"Electrovanne decharge turbo - circuit trop haut"},
  {0x0039,"Soupape derivation turbo - plage/performance"},
  {0x0045,"Commande pression suralimentation - circuit ouvert"},
  {0x0046,"Commande pression suralimentation - plage/performance"},
  {0x0047,"Commande pression suralimentation - circuit trop bas"},
  {0x0048,"Commande pression suralimentation - circuit trop haut"},
  {0x0049,"Turbine turbo - regime excessif"},
  {0x0068,"Correlation MAP / debitmetre / papillon"},
  {0x0069,"Correlation MAP / pression atmospherique"},
  {0x0070,"Sonde temperature exterieure - panne du circuit"},
  {0x0071,"Sonde temperature exterieure - plage/performance"},
  {0x0087,"Rampe commune / pression systeme trop faible"},
  {0x0088,"Rampe commune / pression systeme trop haute"},
  {0x0089,"Regulateur pression carburant - performance"},
  {0x0090,"Electrovanne dosage carburant (IMV) - circuit ouvert"},
  {0x0091,"Electrovanne dosage carburant (IMV) - c-c masse"},
  {0x0092,"Electrovanne dosage carburant (IMV) - c-c alim"},
  {0x0093,"Fuite circuit carburant - fuite importante"},
  {0x0094,"Fuite circuit carburant - petite fuite"},
  {0x0095,"Sonde temperature air admission 2 - circuit"},
  {0x0100,"Debitmetre d'air - panne du circuit"},
  {0x0101,"Debitmetre d'air - plage/performance"},
  {0x0102,"Debitmetre d'air - valeur trop basse"},
  {0x0103,"Debitmetre d'air - valeur trop haute"},
  {0x0104,"Debitmetre d'air - circuit intermittent"},
  {0x0105,"Capteur MAP/atmospherique - panne du circuit"},
  {0x0106,"Capteur MAP/atmospherique - plage/performance"},
  {0x0107,"Capteur MAP/atmospherique - valeur trop basse"},
  {0x0108,"Capteur MAP/atmospherique - valeur trop haute"},
  {0x0109,"Capteur MAP/atmospherique - circuit intermittent"},
  {0x0110,"Sonde temperature air admission - circuit"},
  {0x0111,"Sonde temperature air admission - plage/performance"},
  {0x0112,"Sonde temperature air admission - valeur trop basse"},
  {0x0113,"Sonde temperature air admission - valeur trop haute"},
  {0x0114,"Sonde temperature air admission - intermittent"},
  {0x0115,"Sonde temperature eau - panne du circuit"},
  {0x0116,"Sonde temperature eau - plage/performance"},
  {0x0117,"Sonde temperature eau - valeur trop basse"},
  {0x0118,"Sonde temperature eau - valeur trop haute"},
  {0x0119,"Sonde temperature eau - circuit intermittent"},
  {0x0120,"Capteur pedale/papillon A - panne du circuit"},
  {0x0121,"Capteur pedale/papillon A - plage/performance"},
  {0x0122,"Capteur pedale/papillon A - valeur trop basse"},
  {0x0123,"Capteur pedale/papillon A - valeur trop haute"},
  {0x0124,"Capteur pedale/papillon A - circuit intermittent"},
  {0x0128,"Thermostat - eau sous la temperature de regulation"},
  {0x0148,"Erreur de debit de carburant"},
  {0x0149,"Erreur de calage d'injection"},
  {0x0168,"Temperature du carburant trop haute"},
  {0x0180,"Sonde temperature carburant A - circuit"},
  {0x0181,"Sonde temperature carburant A - plage/performance"},
  {0x0182,"Sonde temperature carburant A - valeur trop basse"},
  {0x0183,"Sonde temperature carburant A - valeur trop haute"},
  {0x0190,"Capteur pression rampe - panne du circuit"},
  {0x0191,"Capteur pression rampe - plage/performance"},
  {0x0192,"Capteur pression rampe - valeur trop basse"},
  {0x0193,"Capteur pression rampe - valeur trop haute"},
  {0x0194,"Capteur pression rampe - circuit intermittent"},
  {0x0195,"Sonde temperature huile moteur - circuit"},
  {0x0200,"Injecteur - panne du circuit"},
  {0x0201,"Injecteur 1 - panne du circuit"},
  {0x0202,"Injecteur 2 - panne du circuit"},
  {0x0203,"Injecteur 3 - panne du circuit"},
  {0x0204,"Injecteur 4 - panne du circuit"},
  {0x0215,"Electrovanne coupure carburant - panne du circuit"},
  {0x0216,"Commande calage injection - panne du circuit"},
  {0x0217,"Surchauffe du moteur"},
  {0x0219,"Regime excessif"},
  {0x0220,"Capteur pedale/papillon B - panne du circuit"},
  {0x0221,"Capteur pedale/papillon B - plage/performance"},
  {0x0222,"Capteur pedale/papillon B - valeur trop basse"},
  {0x0223,"Capteur pedale/papillon B - valeur trop haute"},
  {0x0224,"Capteur pedale/papillon B - circuit intermittent"},
  {0x0230,"Pompe a carburant circuit primaire - panne"},
  {0x0234,"Suralimentation - limite depassee"},
  {0x0235,"Capteur pression turbo A - panne du circuit"},
  {0x0236,"Capteur pression turbo A - plage/performance"},
  {0x0237,"Capteur pression turbo A - valeur trop basse"},
  {0x0238,"Capteur pression turbo A - valeur trop haute"},
  {0x0243,"Electrovanne decharge turbo A - panne du circuit"},
  {0x0245,"Electrovanne decharge turbo A - circuit trop bas"},
  {0x0246,"Electrovanne decharge turbo A - circuit trop haut"},
  {0x0251,"Pompe injection A rotor/cames - panne du circuit"},
  {0x0252,"Pompe injection A rotor/cames - plage/performance"},
  {0x0261,"Injecteur 1 - circuit trop bas"},
  {0x0262,"Injecteur 1 - circuit trop haut"},
  {0x0263,"Cylindre 1 - defaut d'equilibrage de debit"},
  {0x0264,"Injecteur 2 - circuit trop bas"},
  {0x0265,"Injecteur 2 - circuit trop haut"},
  {0x0266,"Cylindre 2 - defaut d'equilibrage de debit"},
  {0x0267,"Injecteur 3 - circuit trop bas"},
  {0x0268,"Injecteur 3 - circuit trop haut"},
  {0x0269,"Cylindre 3 - defaut d'equilibrage de debit"},
  {0x0270,"Injecteur 4 - circuit trop bas"},
  {0x0271,"Injecteur 4 - circuit trop haut"},
  {0x0272,"Cylindre 4 - defaut d'equilibrage de debit"},
  {0x0297,"Vitesse vehicule excessive"},
  {0x0298,"Temperature huile moteur trop haute"},
  {0x0299,"Turbo - pression de suralimentation faible"},
  {0x0300,"Rates de combustion aleatoires"},
  {0x0301,"Cylindre 1 - rates de combustion"},
  {0x0302,"Cylindre 2 - rates de combustion"},
  {0x0303,"Cylindre 3 - rates de combustion"},
  {0x0304,"Cylindre 4 - rates de combustion"},
  {0x0313,"Rates detectes avec niveau de carburant bas"},
  {0x0315,"Position vilebrequin - variation non apprise"},
  {0x0320,"Capteur vilebrequin/regime - panne du circuit"},
  {0x0321,"Capteur vilebrequin/regime - plage/performance"},
  {0x0322,"Capteur vilebrequin/regime - aucun signal"},
  {0x0323,"Capteur vilebrequin/regime - intermittent"},
  {0x0335,"Capteur vilebrequin - panne du circuit"},
  {0x0336,"Capteur vilebrequin - plage/performance"},
  {0x0337,"Capteur vilebrequin - valeur trop basse"},
  {0x0338,"Capteur vilebrequin - valeur trop haute"},
  {0x0339,"Capteur vilebrequin - circuit intermittent"},
  {0x0340,"Capteur arbre a cames A - panne du circuit"},
  {0x0341,"Capteur arbre a cames A - plage/performance"},
  {0x0342,"Capteur arbre a cames A - valeur trop basse"},
  {0x0343,"Capteur arbre a cames A - valeur trop haute"},
  {0x0344,"Capteur arbre a cames A - circuit intermittent"},
  {0x0380,"Bougies de prechauffage circuit A - panne"},
  {0x0381,"Temoin bougies de prechauffage - circuit"},
  {0x0382,"Bougies de prechauffage circuit B - panne"},
  {0x0400,"Systeme EGR - probleme de debit"},
  {0x0401,"Systeme EGR - debit insuffisant"},
  {0x0402,"Systeme EGR - debit excessif"},
  {0x0403,"Recyclage gaz echappement - panne du circuit"},
  {0x0404,"Systeme EGR - plage/performance"},
  {0x0405,"Capteur position vanne EGR A - valeur trop basse"},
  {0x0406,"Capteur position vanne EGR A - valeur trop haute"},
  {0x0409,"Capteur EGR A - panne du circuit"},
  {0x0470,"Capteur pression gaz echappement - circuit"},
  {0x0471,"Capteur pression gaz echappement - plage/perf"},
  {0x0472,"Capteur pression gaz echappement - trop basse"},
  {0x0473,"Capteur pression gaz echappement - trop haute"},
  {0x0475,"Electrovanne pression echappement - circuit"},
  {0x0480,"Motoventilateur refroidissement 1 - circuit"},
  {0x0481,"Motoventilateur refroidissement 2 - circuit"},
  {0x0486,"Capteur position vanne EGR B - panne du circuit"},
  {0x0489,"Systeme EGR - circuit trop bas"},
  {0x0490,"Systeme EGR - circuit trop haut"},
  {0x0500,"Capteur vitesse vehicule - panne du circuit"},
  {0x0501,"Capteur vitesse vehicule - plage/performance"},
  {0x0502,"Capteur vitesse vehicule - valeur trop basse"},
  {0x0503,"Capteur vitesse vehicule - intermittent/trop haute"},
  {0x0504,"Contacteur de freinage - correlation A/B"},
  {0x0520,"Capteur pression huile - panne du circuit"},
  {0x0524,"Pression d'huile moteur trop basse"},
  {0x0540,"Chauffage air admission A - panne du circuit"},
  {0x0560,"Tension du systeme - panne"},
  {0x0562,"Tension du systeme - basse"},
  {0x0563,"Tension du systeme - haute"},
  {0x0600,"Bus de donnees CAN - panne"},
  {0x0601,"Calculateur moteur - erreur checksum memoire"},
  {0x0602,"Calculateur moteur - erreur de programmation"},
  {0x0603,"Calculateur moteur - erreur KAM"},
  {0x0604,"Calculateur moteur - erreur RAM"},
  {0x0605,"Calculateur moteur - erreur ROM"},
  {0x0606,"Calculateur moteur - panne processeur"},
  {0x0611,"Boitier injecteurs - probleme de performance"},
  {0x0615,"Relais du demarreur - panne du circuit"},
  {0x0627,"Commande pompe a carburant A - circuit ouvert"},
  {0x0628,"Commande pompe a carburant A - circuit trop bas"},
  {0x0629,"Commande pompe a carburant A - circuit trop haut"},
  {0x0641,"Tension reference capteur A - circuit ouvert"},
  {0x0651,"Tension reference capteur B - circuit ouvert"},
  {0x0670,"Boitier bougies de prechauffage - circuit"},
  {0x0671,"Bougie de prechauffage cylindre 1 - circuit"},
  {0x0672,"Bougie de prechauffage cylindre 2 - circuit"},
  {0x0673,"Bougie de prechauffage cylindre 3 - circuit"},
  {0x0674,"Bougie de prechauffage cylindre 4 - circuit"},
  {0x0685,"Relais alimentation calculateur - circuit ouvert"},
  {0x0687,"Relais gestion moteur - court-circuit masse"},
  {0x0688,"Relais gestion moteur - court-circuit alim"},
  {0x0704,"Contacteur pedale embrayage - panne du circuit"},
  {0x1000,"Code constructeur - voir documentation Fiat/PSA"}
};
#define NDTCTXT (sizeof(DTCTXT)/sizeof(DTCTXT[0]))

/* rend le libelle, ou NULL. Ne cherche que dans la famille P0 :
   les codes constructeur P1xxx ne sont pas normalises.         */
static const char *dtcLibelle(uint8_t a, uint8_t b){
  if((a >> 4) != 0x00) return NULL;          /* P0 uniquement */
  uint16_t code = ((uint16_t)(a & 0x0F) << 8) | b;
  for(uint16_t i=0;i<NDTCTXT;i++){
    if(pgm_read_word(&DTCTXT[i].code) == code)
      return (const char *)pgm_read_ptr(&DTCTXT[i].txt);
  }
  return NULL;
}

/* ============================================================
 * Codes defaut - modes 03 (memorises), 07 (en attente),
 * 0A (permanents, non effacables tant que le moniteur associe
 * n'a pas confirme la reparation).
 * ============================================================ */
static void dtcNom(uint8_t a, uint8_t b, char *out){
  const char L[4] = {'P','C','B','U'};
  snprintf(out, 7, "%c%X%X%02X", L[(a >> 6) & 3], (a >> 4) & 3, a & 0x0F, b);
}

uint8_t cmdDtc(uint8_t mode){
  if(interdit()) return 0;
  uint8_t req[1] = { mode };
  uint8_t rsp[TP_BUF];
  uint16_t n = obdAsk(req, 1, rsp, sizeof(rsp));

  const char *quoi = (mode == 0x03) ? "memorises"
                   : (mode == 0x07) ? "en attente" : "permanents";

  if(!n || rsp[0] != (uint8_t)(mode + 0x40)){
    OUT->print("# "); OUT->print(quoi); OUT->println(" : pas de reponse");
    return 0;
  }
  /* rsp : <mode+40> <nb> puis paires d'octets, ou directement
     les paires selon les calculateurs. On saute l'octet de
     comptage quand la longueur restante est impaire.          */
  uint16_t k = 1;
  if(((n - 1) & 1) == 1) k = 2;
  uint8_t cnt = 0;
  char code[8];
  for(uint16_t g=0; g<128 && k + 1 < n; g++){
    if(rsp[k] || rsp[k+1]){
      dtcNom(rsp[k], rsp[k+1], code);
      const char *t = dtcLibelle(rsp[k], rsp[k+1]);
      OUT->print("# "); OUT->print(code);
      if(t){ OUT->print("  "); OUT->print(t); }
      OUT->println();
      cnt++;
    }
    k += 2;
  }
  OUT->print("# "); OUT->print(quoi); OUT->print(" : "); OUT->println(cnt);
  return cnt;
}

/* ============================================================
 * Dictionnaire SAE J1979 - lecture a l'ecran uniquement
 * Le journal, lui, ecrit des octets bruts : ce dictionnaire ne
 * sert qu'a relire une PID a la console sans passer par le CSV.
 * ============================================================ */
static void decodePid01(uint8_t pid, const uint8_t *v, uint8_t n){
  float A = (n > 0) ? v[0] : 0;
  float B = (n > 1) ? v[1] : 0;
  uint16_t AB = ((uint16_t)(n>0?v[0]:0) << 8) | (n>1?v[1]:0);

  switch(pid){
    case 0x04: OUT->print("charge moteur "); OUT->print(A*100/255,1); OUT->println(" %"); return;
    case 0x05: OUT->print("temp eau ");      OUT->print(A-40,0);      OUT->println(" C"); return;
    case 0x06: OUT->print("trim court B1 "); OUT->print(A*100/128-100,1); OUT->println(" %"); return;
    case 0x07: OUT->print("trim long B1 ");  OUT->print(A*100/128-100,1); OUT->println(" %"); return;
    case 0x0A: OUT->print("pression carb "); OUT->print(A*3,0);       OUT->println(" kPa"); return;
    case 0x0B: OUT->print("MAP ");           OUT->print(A,0);         OUT->println(" kPa"); return;
    case 0x0C: OUT->print("regime ");        OUT->print(AB/4.0f,0);   OUT->println(" tr/min"); return;
    case 0x0D: OUT->print("vitesse ");       OUT->print(A,0);         OUT->println(" km/h"); return;
    case 0x0E: OUT->print("avance ");        OUT->print(A/2-64,1);    OUT->println(" deg"); return;
    case 0x0F: OUT->print("temp air adm ");  OUT->print(A-40,0);      OUT->println(" C"); return;
    case 0x10: OUT->print("debit MAF ");     OUT->print(AB/100.0f,2); OUT->println(" g/s"); return;
    case 0x11: OUT->print("papillon ");      OUT->print(A*100/255,1); OUT->println(" %"); return;
    case 0x1F: OUT->print("temps depuis dem "); OUT->print(AB);       OUT->println(" s"); return;
    case 0x21: OUT->print("km avec MIL ");   OUT->print(AB);          OUT->println(" km"); return;
    case 0x22: OUT->print("rail rel vide "); OUT->print(AB*0.079f,1); OUT->println(" kPa"); return;
    case 0x23: OUT->print("rail ");          OUT->print(AB*10.0f,0);  OUT->println(" kPa"); return;
    case 0x2C: OUT->print("EGR commande ");  OUT->print(A*100/255,1); OUT->println(" %"); return;
    case 0x2D: OUT->print("EGR erreur ");    OUT->print(A*100/128-100,1); OUT->println(" %"); return;
    case 0x2E: OUT->print("purge canister ");OUT->print(A*100/255,1); OUT->println(" %"); return;
    case 0x2F: OUT->print("niveau carb ");   OUT->print(A*100/255,1); OUT->println(" %"); return;
    case 0x30: OUT->print("rearmements ");   OUT->println(A,0);       return;
    case 0x31: OUT->print("km depuis effac ");OUT->print(AB);         OUT->println(" km"); return;
    case 0x33: OUT->print("pression atmo "); OUT->print(A,0);         OUT->println(" kPa"); return;
    case 0x3C: OUT->print("temp cata B1S1 ");OUT->print(AB/10.0f-40,1); OUT->println(" C"); return;
    case 0x42: OUT->print("tension calc ");  OUT->print(AB/1000.0f,2);OUT->println(" V"); return;
    case 0x43: OUT->print("charge absolue ");OUT->print(AB*100/255.0f,1); OUT->println(" %"); return;
    case 0x44: OUT->print("lambda cmd ");    OUT->print(AB*2.0f/65536,3); OUT->println(""); return;
    case 0x45: OUT->print("papillon rel ");  OUT->print(A*100/255,1); OUT->println(" %"); return;
    case 0x46: OUT->print("temp ambiante "); OUT->print(A-40,0);      OUT->println(" C"); return;
    case 0x47: OUT->print("papillon abs B ");OUT->print(A*100/255,1); OUT->println(" %"); return;
    case 0x49: OUT->print("pedale D ");      OUT->print(A*100/255,1); OUT->println(" %"); return;
    case 0x4A: OUT->print("pedale E ");      OUT->print(A*100/255,1); OUT->println(" %"); return;
    case 0x4C: OUT->print("papillon cmd ");  OUT->print(A*100/255,1); OUT->println(" %"); return;
    case 0x4D: OUT->print("duree MIL ");     OUT->print(AB);          OUT->println(" min"); return;
    case 0x4E: OUT->print("duree depuis effac "); OUT->print(AB);     OUT->println(" min"); return;
    case 0x5C: OUT->print("temp huile ");    OUT->print(A-40,0);      OUT->println(" C"); return;
    case 0x5E: OUT->print("conso carb ");    OUT->print(AB*0.05f,2);  OUT->println(" L/h"); return;
    case 0x5F: OUT->print("exigences emis ");OUT->println(A,0);       return;
    case 0x62: OUT->print("couple relatif ");OUT->print(A-125,0);     OUT->println(" %"); return;
    case 0x63: OUT->print("couple ref ");    OUT->print(AB);          OUT->println(" Nm"); return;
    case 0x66: OUT->print("MAF capteur (multi-octets)"); OUT->println(); return;
    case 0x69: OUT->print("EGR cmd/reel (multi-octets)"); OUT->println(); return;
    case 0x6D: OUT->print("rail consigne ");
               if(n >= 3){ OUT->print(((uint16_t)v[1]<<8|v[2])*10.0f,0); OUT->print(" kPa"); }
               if(n >= 5){ OUT->print("  reel "); OUT->print(((uint16_t)v[3]<<8|v[4])*10.0f,0); OUT->print(" kPa"); }
               if(n >= 6){ OUT->print("  temp "); OUT->print(v[5]-40); OUT->print(" C"); }
               OUT->println(); return;
    case 0x70: OUT->print("suralimentation (multi-octets)"); OUT->println(); return;
    case 0x71: OUT->print("VGT cmd/reel (multi-octets)"); OUT->println(); return;
    case 0x77: OUT->print("temp air apres echangeur (multi-octets)"); OUT->println(); return;
    case 0x7A: OUT->print("delta P filtre a particules"); OUT->println(); return;
    case 0x7C: OUT->print("temp FAP (multi-octets)"); OUT->println(); return;
  }
  (void)B;
  OUT->println("(pas au dictionnaire)");
}

void cmdPid(const String &arg){
  if(interdit()) return;
  uint8_t pid = (uint8_t)strtoul(arg.c_str(), NULL, 16);
  uint8_t req[2] = {0x01, pid};
  uint8_t rsp[TP_BUF];
  uint16_t n = obdAsk(req, 2, rsp, sizeof(rsp));

  if(!n || rsp[0] != 0x41 || rsp[1] != pid){
    OUT->print("# PID "); if(pid<16) OUT->print('0');
    OUT->print(pid, HEX); OUT->println(" : pas de reponse");
    return;
  }
  OUT->print("# PID "); if(pid<16) OUT->print('0');
  OUT->print(pid, HEX); OUT->print(" RAW");
  for(uint16_t i=2;i<n;i++){
    OUT->print(' ');
    if(rsp[i] < 16) OUT->print('0');
    OUT->print(rsp[i], HEX);
  }
  OUT->println();
  OUT->print("#   ");
  decodePid01(pid, rsp + 2, n - 2);
}

/* ============================================================
 * Mode 02 - donnees gelees (freeze frame)
 * Instantane des parametres moteur fige a l'apparition du
 * defaut. C'est la seule photo de l'incident lui-meme, pas de
 * son contexte : elle dit a quel regime et sous quelle charge
 * la coupure s'est produite, sans avoir a etre en train de
 * journaliser au bon moment.
 * Requete : 02 <pid> <trame>, trame 00 = la seule memorisee sur
 * la plupart des calculateurs.
 * ============================================================ */
void cmdFrz(){
  if(interdit()) return;

  /* PID 02 de la trame gelee : le DTC qui l'a declenchee */
  uint8_t req[3] = {0x02, 0x02, 0x00};
  uint8_t rsp[TP_BUF];
  uint16_t n = obdAsk(req, 3, rsp, sizeof(rsp));
  if(!n || rsp[0] != 0x42){
    OUT->println("# pas de donnees gelees");
    return;
  }
  if(n >= 5 && (rsp[3] || rsp[4])){
    char code[8];
    dtcNom(rsp[3], rsp[4], code);
    OUT->print("# GEL declenche par "); OUT->println(code);
  }else{
    OUT->println("# GEL present, DTC declencheur non renseigne");
  }

  /* les parametres qui nous interessent dans la trame gelee */
  static const uint8_t GEL[] = {0x04, 0x05, 0x0B, 0x0C, 0x0D, 0x0F,
                                0x10, 0x11, 0x23, 0x2C, 0x2F, 0x33,
                                0x42, 0x49, 0x5C, 0x6D};
  for(uint8_t i=0;i<sizeof(GEL);i++){
    uint8_t q[3] = {0x02, GEL[i], 0x00};
    uint16_t m = obdAsk(q, 3, rsp, sizeof(rsp));
    if(!m || rsp[0] != 0x42 || rsp[1] != GEL[i]) continue;
    OUT->print("#   ");
    if(GEL[i] < 16) OUT->print('0');
    OUT->print(GEL[i], HEX);
    OUT->print(' ');
    decodePid01(GEL[i], rsp + 3, m - 3);   /* rsp: 42 <pid> <trame> <data> */
  }
}

/* ============================================================
 * Tentative d'effacement - sequence recommandee
 * L'effacement seul ne prouve rien : un defaut reel revient des
 * que son moniteur re-tourne. La sequence releve donc l'etat
 * avant, efface, puis releve l'etat apres.
 *   1. lecture 03 / 07 / 0A et des donnees gelees, tout est
 *      ecrit dans le journal avant d'etre detruit
 *   2. mode 04
 *   3. relecture 03 / 07 / 0A
 * Un code encore present en 03 juste apres l'effacement est un
 * defaut actif, pas un residu. Un code permanent (0A) ne part
 * pas au mode 04, c'est normal : il faut que le moniteur
 * concerne tourne et confirme.
 * L'effacement est refuse moteur tournant par la plupart des
 * calculateurs : contact mis, moteur arrete.
 * ============================================================ */
void cmdReset(){
  if(interdit()) return;

  OUT->println("# --- avant effacement ---");
  logWriteTs("RESET DTC");
  uint8_t a3 = cmdDtc(0x03);
  uint8_t a7 = cmdDtc(0x07);
  uint8_t aA = cmdDtc(0x0A);
  if(a3) cmdFrz();

  if(!a3 && !a7 && !aA){
    OUT->println("# rien a effacer");
    return;
  }

  uint8_t req[1] = {0x04};
  uint8_t rsp[TP_BUF];
  uint16_t n = obdAsk(req, 1, rsp, sizeof(rsp));
  if(!n || rsp[0] != 0x44){
    OUT->println("# effacement REFUSE (moteur tournant ?)");
    return;
  }
  OUT->println("# efface");
  delay(500);                       /* le calculateur reconstruit sa table */

  OUT->println("# --- apres effacement ---");
  uint8_t b3 = cmdDtc(0x03);
  uint8_t b7 = cmdDtc(0x07);
  uint8_t bA = cmdDtc(0x0A);

  if(b3) OUT->println("# defaut ACTIF : revenu immediatement");
  else if(bA) OUT->println("# permanents restants : moniteur a faire tourner");
  else if(b7) OUT->println("# en attente restants : surveiller");
  else OUT->println("# memoire propre");
}

/* ============================================================
 * Commandes
 * ============================================================ */
void cmdHelp(){
  OUT->println("# NEMO-OBD v" FW_VER);
  OUT->println("# LOG ON / LOG OFF / LOG LAST / STAT?");
  OUT->println("# ECHO ON / ECHO OFF");
  OUT->println("# MARK     marqueur dans le journal");
  OUT->println("# DTC      tous les codes : 03 + 07 + 0A");
  OUT->println("# DTCP     en attente seuls / DTCX permanents seuls");
  OUT->println("# FRZ      donnees gelees a l'apparition du defaut");
  OUT->println("# RESET!   releve, efface, releve a nouveau");
  OUT->println("# PID xx   lit une PID et la decode (journal a l'arret)");
  OUT->println("# PURGE!   efface /log.csv");
  OUT->println("# DOWNLOAD_DONE!   efface et redemarre");
  OUT->println("# TIME! <epoch_s>");
  OUT->println("# BUS?     compteurs de la pompe");
}

void cmdStat(){
  OUT->print("# v"); OUT->print(FW_VER);
  OUT->print(" phase "); OUT->print(phaseNom());
  OUT->print(" lignes "); OUT->print(logLines);
  OUT->print(" brutes "); OUT->print(brutN);
  OUT->print(" flash "); OUT->print(LittleFS.usedBytes());
  OUT->print("/"); OUT->println(LittleFS.totalBytes());
}

void cmdBus(){
  OUT->print("# rx "); OUT->print(nRx);
  OUT->print(" rsp "); OUT->print(nRsp);
  OUT->print(" jetees "); OUT->print(nDrop);
  OUT->print(" neg "); OUT->print(nNeg);
  OUT->print(" tx "); OUT->print(nTx);
  OUT->print(" EFLG 0x"); OUT->print(mcpRead(R_EFLG), HEX);
  OUT->print(" TEC "); OUT->print(mcpRead(R_TEC));
  OUT->print(" REC "); OUT->println(mcpRead(R_REC));
}

void execCmd(String c){
  c.trim();
  String u = c; u.toUpperCase();
  if(!u.length()) return;

  if(u == "LOG ON")   { logStart(false); return; }
  if(u == "LOG OFF")  { logStop();       return; }
  if(u == "LOG LAST") { OUT->println(logLastLine); return; }
  if(u == "STAT?" || u == "LOG STAT"){ cmdStat(); return; }
  if(u == "ECHO ON")  { logEcho = true;  OUT->println("# echo on");  return; }
  if(u == "ECHO OFF") { logEcho = false; OUT->println("# echo off"); return; }
  if(u == "PURGE!")   { logPurge(); return; }
  if(u == "DOWNLOAD_DONE!"){ downloadDone(); return; }
  if(u == "BUS?")     { cmdBus();   return; }
  if(u == "DTC")      { cmdDtc(0x03); cmdDtc(0x07); cmdDtc(0x0A); return; }
  if(u == "DTCP")     { cmdDtc(0x07); return; }
  if(u == "DTCX")     { cmdDtc(0x0A); return; }
  if(u == "FRZ")      { cmdFrz();  return; }
  if(u == "RESET!")   { cmdReset(); return; }
  if(u.startsWith("PID ")){ cmdPid(c.substring(4)); return; }
  if(u == "HELP?")    { cmdHelp();  return; }
  if(u.startsWith("TIME!")){
    modEpoch   = (uint32_t)strtoul(c.c_str() + 5, NULL, 10);
    modEpochMs = millis();
    OUT->print("# horloge "); OUT->println(modEpoch);
    return;
  }
  if(u.startsWith("MARK")){
    char l[48];
    snprintf(l, sizeof(l), "# MARK %u", ++markNum);
    logWrite(l);
    OUT->println(l);
    return;
  }
  OUT->println("# ?");
}

/* ============================================================
 * Page web - HTML/CSS/JS pur, auto-contenue, aucun lien externe
 * ============================================================ */
const char PAGE[] PROGMEM = R"HTML(<!DOCTYPE html><html lang="fr"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>NEMO-OBD</title><style>
*{box-sizing:border-box}body{margin:0;padding:10px;background:#111;color:#ddd;font:15px/1.3 system-ui,sans-serif}
h1{font-size:18px;margin:0 0 7px;color:#7cf}#v{float:right;font-size:11px;color:#777;font-weight:400}
#status{background:#1a2632;border:1px solid #2d4356;border-radius:7px;padding:8px 10px;margin-bottom:8px;font-weight:600;color:#7cf;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.g{display:grid;grid-template-columns:1fr 1fr;gap:7px;margin-bottom:8px}.c{background:#1c1c1c;border:1px solid #333;border-radius:8px;padding:8px}
.c:last-child{grid-column:1/-1}.n{font-size:10px;color:#888;text-transform:uppercase;letter-spacing:.4px}.c .v{font-size:25px;font-weight:650;color:#7cf;margin-top:1px}
.c.old .v{color:#555}.gb{height:6px;background:#252525;border-radius:4px;overflow:hidden;margin-top:5px}.gi{height:100%;width:0;background:#7cf;transition:width .25s}
#fl{background:#1a2632;border:1px solid #2d4356;border-radius:7px;padding:8px 10px;margin-bottom:8px}
#fll{display:flex;justify-content:space-between;font-size:12px;color:#9ab}#fln{color:#7cf;font-weight:600}
#fb{height:10px;background:#222;border-radius:5px;margin-top:5px;overflow:hidden}#fbin{height:100%;width:0;background:#2e9e4f;transition:width .3s}
#fbin.mid{background:#c8912a}#fbin.hot{background:#c03535}.b{display:grid;grid-template-columns:1fr 1fr;gap:7px;margin-bottom:7px}
button{font:14px system-ui;padding:11px 5px;border:0;border-radius:7px;background:#2a3a4a;color:#cde}button:active{background:#3d5568}
.off{background:#5c1e1e;color:#fcc}.on{background:#1e5c2e;color:#cfc}.dgr{background:#3a2020;color:#e99}.dgr.armed{background:#a01e1e;color:#fff;font-weight:700}
#s{font:11px ui-monospace,monospace;color:#8a8;white-space:pre-wrap;background:#161616;border:1px solid #2a2a2a;border-radius:6px;padding:6px;min-height:24px}
</style></head><body>
<h1>NEMO-OBD<span id="v"></span></h1>
<div id="status">PRET - appuyez sur START LOG</div>
<div class="g" id="g"></div>
<div id="fl"><div id="fll"><span>JOURNAL EN FLASH</span><span id="fln">--</span></div><div id="fb"><div id="fbin"></div></div></div>
<div class="b">
<button class="on" onclick="cmd('LOG ON')">START LOG</button>
<button class="off" onclick="cmd('LOG OFF')">ARRET LOG</button>
<button onclick="dl()">TELECHARGER CSV</button>
<button onclick="cmd('DTC')">CHECK DEFAUTS</button>
<button class="dgr" onclick="eff()">RESET DEFAUTS</button>
<button class="dgr" id="pg" onclick="pur()">PURGE</button>
</div>
<div id="s">pret</div>
<script>
var NOM=["compte tours tr/min","vitesse km/h","pedale %","pression rail kPa","MAP / turbo kPa"],MAX=[7000,250,100,200000,300],g=document.getElementById("g");
for(var i=0;i<NOM.length;i++)g.innerHTML+='<div class="c old" id="c'+i+'"><div class="n">'+NOM[i]+'</div><div class="v">--</div><div class="gb"><div class="gi"></div></div></div>';
function cmd(c){fetch("/cmd?c="+encodeURIComponent(c)).then(function(r){return r.text()}).then(function(t){document.getElementById("s").textContent=t;maj()})}
function dl(){var s=document.getElementById("s");s.textContent="telechargement...";
fetch("/log.csv",{cache:"no-store"}).then(function(r){if(!r.ok)throw 0;return r.blob()}).then(function(b){if(!b.size)throw 0;var u=URL.createObjectURL(b),a=document.createElement("a");
a.href=u;a.download="log_nemo.csv";document.body.appendChild(a);a.click();document.body.removeChild(a);setTimeout(function(){URL.revokeObjectURL(u)},60000);cmd("DOWNLOAD_DONE!")})
.catch(function(){s.textContent="# echec telechargement - journal conserve"})}
var pb=null;function pur(){var b=document.getElementById("pg");if(pb){clearTimeout(pb);pb=null;b.textContent="PURGE";b.classList.remove("armed");cmd("PURGE!");return}
b.textContent="EFFACER ?";b.classList.add("armed");pb=setTimeout(function(){pb=null;b.textContent="PURGE";b.classList.remove("armed")},4000)}
var eb=null;function eff(){var b=document.querySelector(".dgr");if(eb){clearTimeout(eb);eb=null;b.textContent="RESET DEFAUTS";b.classList.remove("armed");document.getElementById("s").textContent="sequence en cours...";cmd("RESET!");return}
b.textContent="CONFIRMER ?";b.classList.add("armed");eb=setTimeout(function(){eb=null;b.textContent="RESET DEFAUTS";b.classList.remove("armed")},4000)}
function maj(){fetch("/stat",{cache:"no-store"}).then(function(r){return r.json()}).then(function(j){
document.getElementById("v").textContent="v"+j.fw+"  "+j.n+" lignes";document.getElementById("status").textContent=j.status;
var fp=j.tot?(j.use*100/j.tot):0,fi=document.getElementById("fbin");fi.style.width=fp.toFixed(1)+"%";fi.className=(fp>=90)?"hot":((fp>=70)?"mid":"");
document.getElementById("fln").textContent=(j.use/1024).toFixed(1)+" ko / "+(j.tot/1024).toFixed(0)+" ko ("+fp.toFixed(1)+"%) — "+j.n+" lignes"+(j.plein?" — PLEINE":"");
for(var i=0;i<NOM.length;i++){var c=document.getElementById("c"+i),v=j.val[i],x=c.querySelector(".v"),gi=c.querySelector(".gi");x.textContent=(v===null)?"--":v;c.className="c"+(j.frais[i]?"":" old");gi.style.width=(v===null||!MAX[i])?"0%":Math.min(100,Math.max(0,Math.abs(v)*100/MAX[i]))+"%";}
}).catch(function(){})}
fetch("/cmd?c="+encodeURIComponent("TIME! "+Math.floor(Date.now()/1000)));setInterval(maj,500);maj();
</script></body></html>)HTML";

void handleRoot(){
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html", PAGE);
}

void handleCmd(){
  String c = server.arg("c");
  StreamString ss;
  OUT = &ss;
  execCmd(c);
  OUT = &Serial;
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", (const String&)ss);
}

void handleStat(){
  String j = "{\"fw\":\"" FW_VER "\",\"ph\":";    j += (int)phase;
  j += ",\"phase\":\""; j += phaseNom(); j += "\"";
  j += ",\"status\":\""; j += uiStatus; j += "\"";
  j += "\",\"scan\":";  j += (scanEtape == 0) ? adrPid : scanPid;
  j += ",\"etape\":";   j += scanEtape;
  j += ",\"adrn\":";    j += adrN;
  j += ",\"adri\":";    j += adrIdx;
  j += ",\"brut\":";    j += brutN;
  j += ",\"n\":";       j += logLines;
  j += ",\"use\":";     j += (uint32_t)LittleFS.usedBytes();
  j += ",\"tot\":";     j += (uint32_t)LittleFS.totalBytes();
  j += ",\"log\":";     j += logMode ? "true" : "false";
  j += ",\"plein\":";   j += logPlein ? "true" : "false";
  j += ",\"tx\":";       j += nTx;
  j += ",\"rx\":";       j += nRx;
  j += ",\"rsp\":";      j += nRsp;
  j += ",\"drop\":";     j += nDrop;
  j += ",\"neg\":";      j += nNeg;
  j += ",\"eflg\":";     j += mcpOk ? mcpRead(R_EFLG) : 0;
  j += ",\"txctrl\":";   j += mcpOk ? mcpRead(R_TXB0CTRL) : 0;
  j += ",\"cadence_ms\":"; j += LOG_PERIOD_MS;
  j += ",\"wait\":";     j += (phase == PH_SCAN_OFF || phase == PH_SCAN_RUN)
                              ? (uint32_t)((int32_t)(scanDeadline - millis()) > 0
                                  ? scanDeadline - millis() : 0)
                              : 0;
  j += ",\"val\":[";
  for(uint8_t i=0;i<NCOL;i++){
    if(i) j += ",";
    if(!cellSeen[i]) j += "null";
    else j += String(cellShow[i], (COL[i].step >= 1.0f) ? 0 : 1);
  }
  j += "],\"frais\":[";
  for(uint8_t i=0;i<NCOL;i++){
    if(i) j += ",";
    j += cellMs[i] ? "true" : "false";
  }
  j += "]}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", j);
}

void handleLogCsv(){
  if(logFile) logFile.flush();
  File f = LittleFS.open(LOG_FICH, "r");
  if(!f){ server.send(404, "text/plain", "# pas de journal"); return; }
  server.sendHeader("Content-Disposition", "attachment; filename=log_nemo.csv");
  server.streamFile(f, "text/csv");
  f.close();
}

/* ============================================================
 * Setup / boucle
 * ============================================================ */
void setup(){
  pinMode(PIN_CS, OUTPUT); csH();
  pinMode(PIN_INT, INPUT_PULLUP);
  Serial.begin(115200);
  delay(300);

  Serial.println("# obd_can_bridge v" FW_VER " - Eric Perret F1OCM");
  Serial.println("# Nemo 1.3 HDi - Marelli MJD 8F3.F6 - ISO 15765-4 29 bits");

  SPI.begin();
  /* 10 MHz : maximum MCP2515. En dessous la pompe ne suit pas. */
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));

  mcpOk = mcpInit();
  Serial.println(mcpOk ? "# MCP2515 pret" : "# MCP2515 injoignable - retry auto");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, mcpOk ? LOW : HIGH);

  logFs = LittleFS.begin(true);
  Serial.println("# acquisition v6 : mode fixe / haute cadence");

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_IP, AP_MASK);
  WiFi.softAP(AP_SSID, AP_PASS);
  server.on("/",        handleRoot);
  server.on("/cmd",     handleCmd);
  server.on("/stat",    handleStat);
  server.on("/log.csv", handleLogCsv);
  server.begin();
  Serial.print("# WiFi " AP_SSID " (" AP_PASS ") -> http://");
  Serial.println(WiFi.softAPIP());

  if(logFs && LittleFS.exists(LOG_FLAG)){
    Serial.println("# session interrompue : reprise");
    logStart(true);
  }
  cmdHelp();
}

String line;

void loop(){
  server.handleClient();

  static uint32_t tRetry = 0;
  if(!mcpOk && millis() - tRetry >= 300){
    tRetry = millis();
    mcpOk = mcpInit();
    if(mcpOk){
      digitalWrite(LED_BUILTIN, LOW);
      Serial.println("# MCP2515 accroche");
    }
  }

  canPump();        /* on lit ce qui arrive, a chaque tour */
  sessionTick();    /* on ecrit et on redemande, a l'heure */
  yield();          /* laisse la pile WiFi respirer */

  for(uint16_t g=0; g<256 && Serial.available(); g++){
    char c = Serial.read();
    if(c == '\r') continue;
    if(c != '\n'){ line += c; if(line.length() > 60) line = ""; continue; }
    execCmd(line);
    line = "";
  }
}
