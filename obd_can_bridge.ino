/* ============================================================
 * obd_can_bridge.ino
 * ------------------------------------------------------------
 * Pont Serie USB + WiFi <-> Bus CAN OBD-II
 * Cible   : Seeed XIAO ESP32-S3 emboite sur
 *           Seeed CAN Bus Breakout Board (713-105100001)
 *           MCP2515 @ 16 MHz (confirme visuellement) + SN65HVD230
 * Vehicule: Citroen Nemo 1.3 HDi (Fiat 1.3 Multijet, code moteur Fiat 199A2000)
 *           Code moteur PSA : F13DTE5 - Boite : C510
 *           1ere immat. : 2011-06-16 - VIN : VF7AAFHZ0B8126615
 *           calculateur moteur CAN 500 kbit/s, ISO 15765
 *           Calculateur : Marelli MJD 8F3.F6 (confirme)
 * Auteur  : Eric Perret (F1OCM) - assistance Claude
 * Version : 2.9 - 2026-07-26
 * Licence : usage personnel
 * ------------------------------------------------------------
 * v2.9 : decodage PID 01 (voyant MIL + nb defauts), 1C (norme
 *        OBD), 4A (pedale D).
 * v2.8 : decodage en clair SAE J1979 (30 PIDs mode 01, libelles
 *        FR + formules + unites) applique a PID 01 xx.
 *        Commande SCAN? : lit les bitmaps 00/20/40/60 puis
 *        interroge chaque PID supporte -> liste complete en
 *        clair (regime, temperatures, rail, EGR, pedale...).
 * v2.7 : DIAGNOSTIC 29 BITS (confirme par PROBE? sur vehicule) :
 *        requetes 18DB33F1 (fonct.) / 18DA10F1 (phys.),
 *        reponses 18DAF1xx, flow control 18DA<src>F1 STmin 5ms.
 *        Repli 11 bits 7DF/7E0 conserve ; adressage gagnant
 *        memorise. Filtres materiels ouverts en permanence
 *        (les filtres 7E8 11 bits rejetaient les reponses).
 *        Sniffer decode et marque les IDs 29 bits (suffixe x).
 * v2.6 : commande PROBE? : sonde d'adressage diagnostic.
 *        Baseline du trafic de fond puis mode 01 PID 00 sous
 *        4 adressages (7DF, 7E0, 29b 18DB33F1, 29b 18DA10F1),
 *        filtres ouverts : tout repondant est liste avec son ID.
 *        Emission/reception 29 bits ajoutees au driver.
 * v2.5 : requetes OBD : repli automatique adressage fonctionnel
 *        0x7DF -> physique 0x7E0 (Marelli MJD8 parfois sourd au
 *        fonctionnel). En echec TX : dump EFLG/TEC/REC pour
 *        distinguer "trame non emise" de "ECU muet".
 * v2.4 : SNIF apprentissage de jeu de payloads (4/ID) : les
 *        trames multiplexees (ex 286, 3 variantes cycliques)
 *        s'affichent une fois par variante puis silence total.
 *        Seule une donnee jamais vue ressort. HELP accepte
 *        sans "?".
 * v2.3 : SNIF masque auto des compteurs roulants : un octet qui
 *        change 3 fois de suite est classe "alive counter" et
 *        exclu de la comparaison (ex: 204 octet 2, 84 octet 1).
 *        Signal unique "# ID x octet n = compteur, masque".
 * v2.2 : SNIF filtre delta : une trame n'est affichee que si
 *        ses donnees changent pour cet ID (anti-rafales reseau
 *        endormi / network management). Compteur "(xN identiques)"
 *        emis au changement. Cache 32 IDs, reset a SNIF ON.
 * v2.1 : traduction FR des DTC dans la page web embarquee.
 *        Dictionnaire CODES_FR (JS, cote client, PROGMEM) genere
 *        depuis Codes_OBD.pdf (SAE J2012/ISO 15031-6), plage
 *        P0000-P0999, 986 entrees. Lacunes de la source elle-meme :
 *        P0045, P0397-399, P0450-459 (non definies dans le PDF).
 *        Codes hors plage (P1xxx constructeur, C/B/U) : affiches
 *        bruts, mention "(code non repertorie)".
 * v2.0 : point d'acces WiFi "NEMO-OBD" (mdp nemo1234),
 *        page de pilotage embarquee -> http://192.168.1.1
 *        (Android : rester connecte malgre "pas d'internet").
 *        Bannieres HELP au boot + commande HELP?.
 *        SNIF reste USB seulement (flux continu).
 * v1.8 : retry auto MCP en boucle (300ms) tant qu'injoignable.
 * v1.7 : SPI ralenti 2 MHz, INFO? (identite cartes + test SPI).
 * Lignes de debug prefixees "# " : hors protocole de reponse.
 * ------------------------------------------------------------
 * LED utilisateur (GPIO21, logique inversee LOW=allumee) :
 *   eteinte      -> MCP2515 non detecte
 *   fixe         -> MCP2515 OK, en attente
 *   clignote 60ms-> reponse OBD recue
 * ------------------------------------------------------------
 * AUCUNE LIBRAIRIE EXTERNE. SPI.h / WiFi.h / WebServer.h /
 * StreamString.h = core Arduino ESP32 natif.
 * Driver MCP2515 ecrit en direct (registres + commandes SPI).
 * ISO-TP (ISO 15765-2) : single frame + multi-frame complet.
 * ------------------------------------------------------------
 * PROTOCOLE (serie 115200 ou http://192.168.1.1/cmd?c=...) :
 *   HELP?              -> rappel de toutes les commandes
 *   PING?              -> PONG v1.0
 *   INFO?              -> identite ESP32, etat MCP2515, test SPI
 *   INIT?              -> MCP OK | MCP FAIL
 *   DTC?               -> DTC P0409 P0400 ... | DTC NONE | NO_DATA
 *   PEND?              -> idem, codes en attente (mode 07)
 *   CLEAR!             -> CLEAR OK | CLEAR FAIL
 *   EGR?               -> EGR CMD=42.0% ERR=-3.1% | NO_DATA
 *   PID <mode> <pid>   -> RAW <hex...> | NO_DATA   (ex: PID 01 0C)
 *   SNIF ON | SNIF OFF -> sniffer trames brutes (USB seulement)
 * ============================================================ */

#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <StreamString.h>

/* ---------- WiFi point d'acces ------------------------------ */
#define AP_SSID   "NEMO-OBD"
#define AP_PASS   "nemo1234"          /* WPA2, 8 car. minimum   */
const IPAddress AP_IP(192,168,1,1);
const IPAddress AP_MASK(255,255,255,0);
WebServer server(80);

/* ---------- Brochage ---------------------------------------- */
#define PIN_CS    D7        /* chip select MCP2515              */
#define PIN_INT   D6        /* /INT MCP2515 (non utilise: poll) */

/* ---------- Commandes SPI MCP2515 --------------------------- */
#define C_RESET   0xC0
#define C_READ    0x03
#define C_WRITE   0x02
#define C_RTS0    0x81      /* request-to-send TXB0             */
#define C_STATUS  0xA0
#define C_BITMOD  0x05

/* ---------- Registres MCP2515 ------------------------------- */
#define R_CANCTRL 0x0F
#define R_CANSTAT 0x0E
#define R_CNF1    0x2A
#define R_CNF2    0x29
#define R_CNF3    0x28
#define R_CANINTE 0x2B
#define R_CANINTF 0x2C
#define R_RXB0CTRL 0x60
#define R_RXB1CTRL 0x70
#define R_TXB0CTRL 0x30
#define R_TXB0SIDH 0x31
#define R_RXB0SIDH 0x61
#define R_RXB1SIDH 0x71
#define R_RXM0SIDH 0x20
#define R_RXM1SIDH 0x24
#define R_RXF0SIDH 0x00
#define R_RXF2SIDH 0x08

/* ---------- OBD-II / ISO-TP --------------------------------- */
#define ID_REQ    0x7DF     /* requete fonctionnelle            */
#define ID_FC     0x7E0     /* flow control vers ECU moteur     */
#define ID_RSP_LO 0x7E8     /* plage reponses                   */
#define ID_RSP_HI 0x7EF
#define TP_BUF    64        /* tampon reassemblage ISO-TP       */
#define T_RSP_MS  300       /* timeout reponse ECU              */

bool snifMode = false;
bool mcpOk    = false;

/* ---------- SNIF v2.4 : delta + compteurs + jeu de payloads --
 * 1) Octet a pas constant sur 3 receptions = compteur roulant,
 *    masque (exclu des comparaisons).
 * 2) Chaque ID memorise jusqu'a 4 payloads distincts (trames
 *    multiplexees). Une trame egale a un payload connu (hors
 *    octets masques) = silence. Seule une donnee JAMAIS vue
 *    s'affiche. Apres apprentissage (~1 ligne par variante),
 *    silence total sur les trames reseau non sollicitees.     */
#define SNIF_MAX_IDS   32
#define SNIF_MAX_PAY   4
#define SNIF_CNT_SEUIL 3
struct SnifCache {
  uint32_t id;
  uint8_t  dlc;
  uint8_t  last[8];               /* derniere trame (detection compteur) */
  uint8_t  delta[8];              /* dernier pas observe par octet       */
  uint8_t  chg[8];                /* pas identiques consecutifs          */
  uint8_t  ignore;                /* bitmap octets masques (compteurs)   */
  uint8_t  pay[SNIF_MAX_PAY][8];  /* payloads connus                     */
  uint8_t  nPay;
  uint8_t  wPay;                  /* index d'ecrasement circulaire       */
  uint16_t repeats;
  bool     used;
};
SnifCache snifCache[SNIF_MAX_IDS];

void snifReset(){
  memset(snifCache, 0, sizeof(snifCache));
}

/* true = trame a afficher (donnee jamais vue pour cet ID)      */
bool snifFilter(uint32_t id, uint8_t dlc, const uint8_t *d){
  int slot = -1;
  for(uint8_t i=0;i<SNIF_MAX_IDS;i++){
    if(snifCache[i].used && snifCache[i].id == id){ slot = i; break; }
    if(slot < 0 && !snifCache[i].used) slot = i;
  }
  if(slot < 0) return true;               /* cache plein : tout passe */

  SnifCache &c = snifCache[slot];
  if(!(c.used && c.id == id)){            /* ID nouveau               */
    memset(&c, 0, sizeof(c));
    c.id = id; c.dlc = dlc; c.used = true;
    memcpy(c.last, d, dlc);
    memcpy(c.pay[0], d, dlc); c.nPay = 1; c.wPay = 1;
    return true;
  }

  /* 1) detection compteurs (pas constant vs trame precedente)  */
  for(uint8_t i=0;i<dlc;i++){
    if(d[i] != c.last[i]){
      uint8_t dv = (uint8_t)(d[i] - c.last[i]);
      if(!(c.ignore & (1<<i))){
        if(dv == c.delta[i] && ++c.chg[i] >= SNIF_CNT_SEUIL){
          c.ignore |= (1<<i);
          Serial.print("#  ID ");
          Serial.print(id, HEX);
          Serial.print(" octet ");
          Serial.print(i);
          Serial.println(" = compteur, masque");
        }else if(dv != c.delta[i]) c.chg[i] = 1;
      }
      c.delta[i] = dv;
    }else c.chg[i] = 0;
  }
  memcpy(c.last, d, dlc);

  /* 2) comparaison au jeu de payloads connus (hors masque)     */
  if(dlc == c.dlc){
    for(uint8_t p=0;p<c.nPay;p++){
      bool match = true;
      for(uint8_t i=0;i<dlc;i++){
        if(!(c.ignore & (1<<i)) && d[i] != c.pay[p][i]){ match = false; break; }
      }
      if(match){ c.repeats++; return false; }   /* deja vue : silence */
    }
  }else c.dlc = dlc;

  /* donnee jamais vue : memoriser + afficher                   */
  memcpy(c.pay[c.wPay % SNIF_MAX_PAY], d, dlc);
  c.wPay++;
  if(c.nPay < SNIF_MAX_PAY) c.nPay++;

  if(c.repeats > 0){
    Serial.print("#  ID ");
    Serial.print(id, HEX);
    Serial.print(" (x");
    Serial.print(c.repeats + 1);
    Serial.println(" stables)");
  }
  c.repeats = 0;
  return true;
}

/* sortie courante des commandes : Serial (USB) ou StreamString (web) */
Print *OUT = &Serial;

/* ============================================================
 * Couche SPI bas niveau
 * ============================================================ */
static inline void csL(){ digitalWrite(PIN_CS, LOW); }
static inline void csH(){ digitalWrite(PIN_CS, HIGH); }

void mcpReset(){
  csL(); SPI.transfer(C_RESET); csH();
  delay(10);
}

uint8_t mcpRead(uint8_t reg){
  csL(); SPI.transfer(C_READ); SPI.transfer(reg);
  uint8_t v = SPI.transfer(0); csH();
  return v;
}

void mcpWrite(uint8_t reg, uint8_t val){
  csL(); SPI.transfer(C_WRITE); SPI.transfer(reg); SPI.transfer(val); csH();
}

void mcpWriteN(uint8_t reg, const uint8_t *buf, uint8_t n){
  csL(); SPI.transfer(C_WRITE); SPI.transfer(reg);
  for(uint8_t i=0;i<n;i++) SPI.transfer(buf[i]);
  csH();
}

void mcpBitMod(uint8_t reg, uint8_t mask, uint8_t val){
  csL(); SPI.transfer(C_BITMOD); SPI.transfer(reg);
  SPI.transfer(mask); SPI.transfer(val); csH();
}

uint8_t mcpStatus(){
  csL(); SPI.transfer(C_STATUS);
  uint8_t v = SPI.transfer(0); csH();
  return v;
}

/* ============================================================
 * Init MCP2515 : 16 MHz quartz, 500 kbit/s, filtres 7E8-7EF
 * Bit timing : BRP=0, Sync 1TQ + Prop 1TQ + PS1 7TQ + PS2 7TQ
 *            = 16 TQ -> 16MHz / (2*1*16) = 500 kbit/s
 * ============================================================ */
bool mcpInit(){
  mcpReset();
  /* mode configuration */
  mcpBitMod(R_CANCTRL, 0xE0, 0x80);
  if((mcpRead(R_CANSTAT) & 0xE0) != 0x80) return false;

  mcpWrite(R_CNF1, 0x00);
  mcpWrite(R_CNF2, 0xF0);
  mcpWrite(R_CNF3, 0x86);

  /* filtres : accepter 0x7E8..0x7EF sur les 2 tampons RX
     masque 0x7F8, filtre 0x7E8 (11 bits standards)         */
  uint8_t m[4] = { (uint8_t)(0x7F8>>3), (uint8_t)((0x7F8&7)<<5), 0, 0 };
  uint8_t f[4] = { (uint8_t)(0x7E8>>3), (uint8_t)((0x7E8&7)<<5), 0, 0 };
  mcpWriteN(R_RXM0SIDH, m, 4);
  mcpWriteN(R_RXM1SIDH, m, 4);
  mcpWriteN(R_RXF0SIDH, f, 4);
  mcpWriteN(R_RXF2SIDH, f, 4);

  /* v2.7 : filtres materiels OUVERTS en permanence (RXM=11).
     Les reponses diag sont en 29 bits (18DAF1xx) : les filtres
     11 bits 7E8 d'origine les rejetaient. Le tri se fait en
     logiciel (obdIsRsp / snifFilter). STmin 5ms cote flow
     control evite tout debordement des 2 tampons RX.          */
  mcpWrite(R_RXB0CTRL, 0x64);   /* RXM=11 + rollover           */
  mcpWrite(R_RXB1CTRL, 0x60);
  mcpWrite(R_CANINTE, 0x00);    /* pas d'IRQ, on interroge     */

  /* mode normal */
  mcpBitMod(R_CANCTRL, 0xE0, 0x00);
  return (mcpRead(R_CANSTAT) & 0xE0) == 0x00;
}

/* v2.7 : filtres toujours ouverts, fonction conservee no-op    */
void mcpFilterAll(bool all){
  (void)all;
  mcpWrite(R_RXB0CTRL, 0x64);
  mcpWrite(R_RXB1CTRL, 0x60);
}

/* ============================================================
 * Emission / reception trame CAN 11 bits
 * ============================================================ */
bool canSend(uint16_t id, const uint8_t *data, uint8_t len){
  uint8_t hdr[5];
  hdr[0] = id >> 3;
  hdr[1] = (id & 7) << 5;
  hdr[2] = 0; hdr[3] = 0;
  hdr[4] = len & 0x0F;
  mcpWriteN(R_TXB0SIDH, hdr, 5);
  mcpWriteN(R_TXB0SIDH + 5, data, len);
  csL(); SPI.transfer(C_RTS0); csH();
  uint32_t t0 = millis();
  while(millis() - t0 < 50){
    if(!(mcpRead(R_TXB0CTRL) & 0x08)) return true;  /* TXREQ=0 */
  }
  mcpBitMod(R_TXB0CTRL, 0x08, 0x00);                /* abort   */
  return false;
}

/* retourne longueur (>0) si trame recue, remplit id et data[8] */
uint8_t canRecv(uint16_t *id, uint8_t *data){
  uint8_t intf = mcpRead(R_CANINTF);
  uint8_t base;
  if(intf & 0x01)      base = R_RXB0SIDH;
  else if(intf & 0x02) base = R_RXB1SIDH;
  else return 0;

  uint8_t sidh = mcpRead(base);
  uint8_t sidl = mcpRead(base + 1);
  uint8_t dlc  = mcpRead(base + 4) & 0x0F;
  if(dlc > 8) dlc = 8;
  for(uint8_t i=0;i<dlc;i++) data[i] = mcpRead(base + 5 + i);
  *id = ((uint16_t)sidh << 3) | (sidl >> 5);

  mcpBitMod(R_CANINTF, (base==R_RXB0SIDH)?0x01:0x02, 0x00);
  return dlc;
}

/* --- variante 29 bits (v2.6, sonde PROBE?) ------------------ */
bool canSend29(uint32_t id, const uint8_t *data, uint8_t len){
  uint8_t hdr[5];
  hdr[0] = (id >> 21) & 0xFF;                       /* SID10..3  */
  hdr[1] = (((id >> 18) & 0x07) << 5)               /* SID2..0   */
         | 0x08                                     /* EXIDE     */
         | ((id >> 16) & 0x03);                     /* EID17..16 */
  hdr[2] = (id >> 8) & 0xFF;                        /* EID15..8  */
  hdr[3] = id & 0xFF;                               /* EID7..0   */
  hdr[4] = len & 0x0F;
  mcpWriteN(R_TXB0SIDH, hdr, 5);
  mcpWriteN(R_TXB0SIDH + 5, data, len);
  csL(); SPI.transfer(C_RTS0); csH();
  uint32_t t0 = millis();
  while(millis() - t0 < 50){
    if(!(mcpRead(R_TXB0CTRL) & 0x08)) return true;
  }
  mcpBitMod(R_TXB0CTRL, 0x08, 0x00);
  return false;
}

/* reception brute : id 32 bits + drapeau etendu                */
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

/* blip 60ms sur reponse OBD recue, puis retour a l'etat de base */
void ledBlip(){
  digitalWrite(LED_BUILTIN, HIGH);
  delay(60);
  digitalWrite(LED_BUILTIN, mcpOk ? LOW : HIGH);
}

/* ============================================================
 * ISO-TP : requete OBD + reassemblage reponse (v2.7, 29 bits)
 * Vehicule confirme (PROBE?) : ISO 15765-4 adressage etendu.
 *   requete fonctionnelle : 0x18DB33F1
 *   requete physique ECM  : 0x18DA10F1
 *   reponses              : 0x18DAF1xx (xx = adresse source)
 *   flow control          : 0x18DA<src>F1, STmin 5 ms
 * Repli 11 bits (7DF/7E0) conserve pour d'autres vehicules.
 * L'adressage gagnant est memorise pour les requetes suivantes.
 * req : mode/pid (1-3 octets utiles)
 * rsp : tampon TP_BUF, retourne longueur utile ou 0
 * ============================================================ */
#define ID29_FUNC 0x18DB33F1UL
#define ID29_PHYS 0x18DA10F1UL
uint8_t obdAddrPref = 0;   /* 0=29b func, 1=29b phys, 2=7DF, 3=7E0 */

static bool obdIsRsp(uint32_t id, bool ext){
  if(ext)  return (id & 0xFFFFFF00UL) == 0x18DAF100UL;
  return id >= ID_RSP_LO && id <= ID_RSP_HI;
}

static bool obdTx(uint8_t essai, const uint8_t *f){
  switch(essai){
    case 0: return canSend29(ID29_FUNC, f, 8);
    case 1: return canSend29(ID29_PHYS, f, 8);
    case 2: return canSend(0x7DF, f, 8);
    default:return canSend(0x7E0, f, 8);
  }
}

static void obdFc(uint32_t rspId, bool ext){
  uint8_t fc[8] = {0x30, 0x00, 0x05, 0, 0, 0, 0, 0};  /* STmin 5ms */
  if(ext) canSend29(0x18DA00F1UL | ((rspId & 0xFF) << 8), fc, 8);
  else    canSend(ID_FC, fc, 8);
}

uint16_t obdQuery(const uint8_t *req, uint8_t reqLen,
                  uint8_t *rsp, uint16_t rspMax){
  uint8_t f[8] = {0,0,0,0,0,0,0,0};
  f[0] = reqLen;                          /* PCI single frame  */
  for(uint8_t i=0;i<reqLen;i++) f[1+i] = req[i];

  /* ordre d'essai : l'adressage gagnant d'abord               */
  uint8_t ordre[4];
  ordre[0] = obdAddrPref;
  for(uint8_t k=0, w=1; k<4; k++) if(k != obdAddrPref) ordre[w++] = k;

  for(uint8_t e=0; e<4; e++){
    uint8_t essai = ordre[e];
    if(!obdTx(essai, f)){
      OUT->print("# TX FAIL: trame non emise (EFLG=0x");
      OUT->print(mcpRead(0x2D), HEX);
      OUT->print(" TEC="); OUT->print(mcpRead(0x1C));
      OUT->print(" REC="); OUT->print(mcpRead(0x1D));
      OUT->println(")");
      return 0;
    }

    uint16_t got = 0, total = 0;
    uint32_t id; bool ext;
    uint8_t  d[8], nextSN = 1;
    uint32_t t0 = millis();

    while(millis() - t0 < T_RSP_MS){
      uint8_t n = canRecvAny(&id, &ext, d);
      if(!n) continue;
      if(!obdIsRsp(id, ext)) continue;
      uint8_t pci = d[0] >> 4;

      if(pci == 0){                       /* single frame      */
        uint8_t len = d[0] & 0x0F;
        if(len > rspMax) len = rspMax;
        for(uint8_t i=0;i<len;i++) rsp[i] = d[1+i];
        /* 7F = reponse negative : ignorer, attendre la vraie */
        if(rsp[0] == 0x7F){ t0 = millis(); continue; }
        obdAddrPref = essai;
        ledBlip();
        return len;
      }
      if(pci == 1){                       /* first frame       */
        total = ((uint16_t)(d[0] & 0x0F) << 8) | d[1];
        if(total > rspMax) total = rspMax;
        got = 0;
        for(uint8_t i=2;i<8 && got<total;i++) rsp[got++] = d[i];
        obdFc(id, ext);                   /* flow ctrl : go    */
        nextSN = 1;
        t0 = millis();
      }
      if(pci == 2 && total){              /* consecutive frame */
        if((d[0] & 0x0F) != (nextSN & 0x0F)) return 0; /* perte */
        nextSN++;
        for(uint8_t i=1;i<8 && got<total;i++) rsp[got++] = d[i];
        t0 = millis();
        if(got >= total){ obdAddrPref = essai; ledBlip(); return total; }
      }
    }
  }
  return 0;
}

/* ============================================================
 * Decodage DTC (2 octets -> Pxxxx / Cxxxx / Bxxxx / Uxxxx)
 * ============================================================ */
void printDtc(uint8_t a, uint8_t b){
  const char L[4] = {'P','C','B','U'};
  char buf[7];
  snprintf(buf, sizeof(buf), "%c%01X%01X%01X%01X",
           L[a>>6], (a>>4)&3, a&0x0F, b>>4, b&0x0F);
  OUT->print(buf);
}

/* mode 03 (stockes) ou 07 (en attente)                         */
void cmdDtc(uint8_t mode){
  uint8_t req[1] = { mode };
  uint8_t rsp[TP_BUF];
  uint16_t n = obdQuery(req, 1, rsp, TP_BUF);
  if(!n || rsp[0] != mode + 0x40){ OUT->println("NO_DATA"); return; }
  uint8_t count = rsp[1];
  if(!count){ OUT->println("DTC NONE"); return; }
  OUT->print("DTC");
  for(uint8_t i=0; i<count && (2+2*i+1)<n; i++){
    OUT->print(' ');
    printDtc(rsp[2+2*i], rsp[3+2*i]);
  }
  OUT->println();
}

void cmdClear(){
  uint8_t req[1] = { 0x04 };
  uint8_t rsp[TP_BUF];
  uint16_t n = obdQuery(req, 1, rsp, TP_BUF);
  OUT->println((n && rsp[0]==0x44) ? "CLEAR OK" : "CLEAR FAIL");
}

/* mode 01 : PID 2C consigne EGR, PID 2D erreur EGR             */
void cmdEgr(){
  uint8_t rsp[TP_BUF];
  uint8_t r1[2] = {0x01, 0x2C};
  uint16_t n1 = obdQuery(r1, 2, rsp, TP_BUF);
  float cmd = -1;
  if(n1 >= 3 && rsp[0]==0x41 && rsp[1]==0x2C) cmd = rsp[2]*100.0/255.0;

  uint8_t r2[2] = {0x01, 0x2D};
  uint16_t n2 = obdQuery(r2, 2, rsp, TP_BUF);
  float err = -999;
  if(n2 >= 3 && rsp[0]==0x41 && rsp[1]==0x2D) err = (rsp[2]-128)*100.0/128.0;

  if(cmd < 0 && err < -500){ OUT->println("NO_DATA"); return; }
  OUT->print("EGR CMD=");
  if(cmd >= 0) OUT->print(cmd, 1); else OUT->print("NA");
  OUT->print("% ERR=");
  if(err > -500) OUT->print(err, 1); else OUT->print("NA");
  OUT->println("%");
}

/* requete generique : PID <mode hexa> <pid hexa>               */
/* ------------------------------------------------------------
 * v2.9 : decodage PID 01 (voyant MIL + nb defauts), 1C (norme
 *        OBD), 4A (pedale D).
 * v2.8 : decodage en clair des PIDs mode 01 (SAE J1979)
 * data = octets apres 41 <pid>, n = leur nombre
 * retourne false si PID inconnu du decodeur
 * ------------------------------------------------------------ */
bool decodePid01(uint8_t pid, const uint8_t *v, uint8_t n){
  uint16_t AB = (n>=2) ? ((uint16_t)v[0]<<8 | v[1]) : 0;
  uint8_t  A  = (n>=1) ? v[0] : 0;
  switch(pid){
    case 0x01:
      OUT->print("Voyant MIL = "); OUT->print((A & 0x80) ? "ALLUME" : "eteint");
      OUT->print(", "); OUT->print(A & 0x7F); OUT->println(" defaut(s) confirme(s)");
      break;
    case 0x1C:
      OUT->print("Norme OBD = ");
      OUT->println(A==6?"EOBD (Europe)":A==1?"OBD-II (USA)":A==3?"OBD+OBD-II":"autre");
      break;
    case 0x4A: OUT->print("Pedale accelerateur D = "); OUT->print(A*100.0/255.0,1); OUT->println(" %"); break;
    case 0x04: OUT->print("Charge moteur = ");        OUT->print(A*100.0/255.0,1); OUT->println(" %"); break;
    case 0x05: OUT->print("Temp. eau = ");            OUT->print((int)A-40);      OUT->println(" C"); break;
    case 0x0A: OUT->print("Pression carburant = ");   OUT->print(A*3);            OUT->println(" kPa"); break;
    case 0x0B: OUT->print("Pression admission = ");   OUT->print(A);              OUT->println(" kPa"); break;
    case 0x0C: OUT->print("Regime = ");               OUT->print(AB/4);           OUT->println(" tr/min"); break;
    case 0x0D: OUT->print("Vitesse = ");              OUT->print(A);              OUT->println(" km/h"); break;
    case 0x0E: OUT->print("Avance = ");               OUT->print(A/2.0-64.0,1);   OUT->println(" deg"); break;
    case 0x0F: OUT->print("Temp. air admission = ");  OUT->print((int)A-40);      OUT->println(" C"); break;
    case 0x10: OUT->print("Debit air MAF = ");        OUT->print(AB/100.0,2);     OUT->println(" g/s"); break;
    case 0x11: OUT->print("Papillon = ");             OUT->print(A*100.0/255.0,1);OUT->println(" %"); break;
    case 0x1F: OUT->print("Depuis demarrage = ");     OUT->print(AB);             OUT->println(" s"); break;
    case 0x21: OUT->print("Distance voyant MIL = ");  OUT->print(AB);             OUT->println(" km"); break;
    case 0x23: OUT->print("Pression rail = ");        OUT->print(AB*10);          OUT->println(" kPa"); break;
    case 0x2C: OUT->print("EGR commandee = ");        OUT->print(A*100.0/255.0,1);OUT->println(" %"); break;
    case 0x2D: OUT->print("Erreur EGR = ");           OUT->print((A-128)*100.0/128.0,1); OUT->println(" %"); break;
    case 0x2F: OUT->print("Niveau carburant = ");     OUT->print(A*100.0/255.0,1);OUT->println(" %"); break;
    case 0x30: OUT->print("Cycles depuis CLEAR = ");  OUT->println(A); break;
    case 0x31: OUT->print("Distance depuis CLEAR = ");OUT->print(AB);             OUT->println(" km"); break;
    case 0x33: OUT->print("Pression atmospherique = ");OUT->print(A);             OUT->println(" kPa"); break;
    case 0x3C: OUT->print("Temp. catalyseur = ");     OUT->print(AB/10.0-40.0,1); OUT->println(" C"); break;
    case 0x42: OUT->print("Tension calculateur = ");  OUT->print(AB/1000.0,2);    OUT->println(" V"); break;
    case 0x43: OUT->print("Charge absolue = ");       OUT->print(AB*100.0/255.0,1);OUT->println(" %"); break;
    case 0x45: OUT->print("Papillon relatif = ");     OUT->print(A*100.0/255.0,1);OUT->println(" %"); break;
    case 0x46: OUT->print("Temp. ambiante = ");       OUT->print((int)A-40);      OUT->println(" C"); break;
    case 0x49: OUT->print("Pedale accelerateur = ");  OUT->print(A*100.0/255.0,1);OUT->println(" %"); break;
    case 0x4C: OUT->print("Consigne papillon = ");    OUT->print(A*100.0/255.0,1);OUT->println(" %"); break;
    case 0x51: OUT->print("Carburant = ");            OUT->println(A==4?"diesel":A==1?"essence":"autre"); break;
    case 0x5C: OUT->print("Temp. huile = ");          OUT->print((int)A-40);      OUT->println(" C"); break;
    case 0x5E: OUT->print("Debit carburant = ");      OUT->print(AB/20.0,2);      OUT->println(" L/h"); break;
    default: return false;
  }
  return true;
}

void cmdPid(const String &line){
  int s1 = line.indexOf(' ');
  int s2 = line.indexOf(' ', s1+1);
  if(s1 < 0 || s2 < 0){ OUT->println("ERR SYNTAX"); return; }
  uint8_t req[2];
  req[0] = strtol(line.substring(s1+1, s2).c_str(), NULL, 16);
  req[1] = strtol(line.substring(s2+1).c_str(), NULL, 16);
  uint8_t rsp[TP_BUF];
  uint16_t n = obdQuery(req, 2, rsp, TP_BUF);
  if(!n){ OUT->println("NO_DATA"); return; }
  OUT->print("RAW");
  for(uint16_t i=0;i<n;i++){
    OUT->print(' ');
    if(rsp[i] < 16) OUT->print('0');
    OUT->print(rsp[i], HEX);
  }
  OUT->println();
  /* decodage en clair si mode 01 et PID connu                  */
  if(n >= 3 && rsp[0] == 0x41) decodePid01(rsp[1], rsp+2, n-2);
}

/* ============================================================
 * SCAN? (v2.8) : lit les bitmaps PID 00/20/40/60, interroge
 * chaque PID supporte et affiche tout en clair
 * ============================================================ */
void cmdScan(){
  uint8_t rsp[TP_BUF];
  uint8_t nb = 0;
  for(uint8_t page=0; page<4; page++){
    uint8_t pidBase = page*0x20;
    uint8_t req[2] = {0x01, pidBase};
    uint16_t n = obdQuery(req, 2, rsp, TP_BUF);
    if(n < 6 || rsp[0]!=0x41 || rsp[1]!=pidBase){
      if(page==0){ OUT->println("NO_DATA"); return; }
      break;
    }
    uint32_t bm = ((uint32_t)rsp[2]<<24)|((uint32_t)rsp[3]<<16)
                | ((uint32_t)rsp[4]<<8)|rsp[5];
    for(uint8_t b=0;b<32;b++){
      if(!(bm & (0x80000000UL >> b))) continue;
      uint8_t pid = pidBase + 1 + b;
      if((pid & 0x1F) == 0) continue;    /* bitmap suivant : saute */
      uint8_t rq[2] = {0x01, pid};
      uint16_t m = obdQuery(rq, 2, rsp, TP_BUF);
      if(m < 3 || rsp[0]!=0x41 || rsp[1]!=pid) continue;
      OUT->print("# PID ");
      if(pid < 16) OUT->print('0');
      OUT->print(pid, HEX);
      OUT->print(" : ");
      if(!decodePid01(pid, rsp+2, m-2)){
        OUT->print("brut");
        for(uint16_t i=2;i<m;i++){
          OUT->print(' ');
          if(rsp[i] < 16) OUT->print('0');
          OUT->print(rsp[i], HEX);
        }
        OUT->println();
      }
      nb++;
    }
    /* bitmap suivant supporte ? bit 32 de la page              */
    if(!(bm & 1)) break;
  }
  OUT->print("SCAN OK "); OUT->print(nb); OUT->println(" PID(s)");
}

/* ============================================================
 * PROBE? (v2.6) : sonde d'adressage diagnostic, filtres ouverts
 * 1. baseline : IDs du trafic de fond pendant 1 s
 * 2. envoi mode 01 PID 00 sous 4 adressages successifs :
 *    11b fonctionnel 7DF, 11b physique 7E0,
 *    29b fonctionnel 18DB33F1, 29b physique 18DA10F1
 * 3. toute trame HORS baseline pendant 400 ms = repondant
 * ============================================================ */
void cmdProbe(){
  uint32_t base[40]; uint8_t nBase = 0;
  uint32_t id; bool ext; uint8_t d[8];

  mcpFilterAll(true);
  OUT->println("# PROBE: baseline du trafic de fond (1s)...");
  uint32_t t0 = millis();
  while(millis() - t0 < 1000){
    if(canRecvAny(&id, &ext, d) && nBase < 40){
      bool known = false;
      for(uint8_t i=0;i<nBase;i++) if(base[i]==id){ known=true; break; }
      if(!known) base[nBase++] = id;
    }
  }
  OUT->print("# baseline: "); OUT->print(nBase); OUT->println(" ID(s) de fond");

  const char *nom[4] = {
    "11b fonctionnel 7DF", "11b physique 7E0",
    "29b fonctionnel 18DB33F1", "29b physique 18DA10F1" };
  uint8_t f[8] = {0x02, 0x01, 0x00, 0, 0, 0, 0, 0};   /* 01 00 */
  bool trouve = false;

  for(uint8_t e=0;e<4;e++){
    OUT->print("# essai "); OUT->println(nom[e]);
    bool ok;
    if(e==0)      ok = canSend(0x7DF, f, 8);
    else if(e==1) ok = canSend(0x7E0, f, 8);
    else if(e==2) ok = canSend29(0x18DB33F1UL, f, 8);
    else          ok = canSend29(0x18DA10F1UL, f, 8);
    if(!ok){ OUT->println("#   TX FAIL"); continue; }

    t0 = millis();
    while(millis() - t0 < 400){
      uint8_t n = canRecvAny(&id, &ext, d);
      if(!n) continue;
      bool known = false;
      for(uint8_t i=0;i<nBase;i++) if(base[i]==id){ known=true; break; }
      if(known) continue;
      trouve = true;
      OUT->print("#   REPONDANT ID ");
      OUT->print(id, HEX);
      OUT->print(ext ? " (29b)" : " (11b)");
      for(uint8_t i=0;i<n;i++){
        OUT->print(' ');
        if(d[i] < 16) OUT->print('0');
        OUT->print(d[i], HEX);
      }
      OUT->println();
      if(nBase < 40) base[nBase++] = id;  /* eviter re-listing */
    }
  }
  mcpFilterAll(snifMode);
  if(trouve) OUT->println("PROBE OK");
  else{
    OUT->println("# aucun repondant sur les 4 adressages.");
    OUT->println("# -> verifier broches 6 (CAN-H) et 14 (CAN-L) du connecteur OBD");
    OUT->println("PROBE NONE");
  }
}

/* ============================================================
 * HELP? : rappel de toutes les commandes
 * ============================================================ */
void cmdHelp(){
  OUT->println("# --- Commandes obd_can_bridge v2.9 ---");
  OUT->println("# HELP?            cette aide");
  OUT->println("# PING?            test de vie -> PONG");
  OUT->println("# INFO?            identite ESP32 + MCP2515 + test SPI");
  OUT->println("# INIT?            (re)initialise le MCP2515");
  OUT->println("# DTC?             defauts confirmes (mode 03)");
  OUT->println("# PEND?            defauts en attente (mode 07)");
  OUT->println("# CLEAR!           efface defauts + voyant (mode 04)");
  OUT->println("# EGR?             consigne et erreur EGR (PID 2C/2D)");
  OUT->println("# PID <m> <p>      requete libre hexa + decodage, ex: PID 01 0C");
  OUT->println("# SCAN?            tous les PIDs supportes, en clair");
  OUT->println("# PROBE?           sonde adressage diag (11b/29b)");
  OUT->println("# SNIF ON|OFF      sniffer CAN brut (USB seulement)");
  OUT->println("# WiFi: SSID " AP_SSID " mdp " AP_PASS " -> http://192.168.1.1");
  OUT->println("HELP OK");
}

/* ============================================================
 * INFO? : tout ce que les deux cartes savent dire d'elles-memes
 * ============================================================ */
void cmdInfo(){
  /* ---- carte 1 : XIAO ESP32-S3 (API core, aucune librairie) ---- */
  OUT->println("# --- XIAO ESP32-S3 ---");
  OUT->print("# chip    : "); OUT->print(ESP.getChipModel());
  OUT->print(" rev ");         OUT->print(ESP.getChipRevision());
  OUT->print(", ");            OUT->print(ESP.getChipCores());
  OUT->println(" coeurs");
  OUT->print("# cpu     : "); OUT->print(ESP.getCpuFreqMHz()); OUT->println(" MHz");
  OUT->print("# flash   : "); OUT->print(ESP.getFlashChipSize()/1048576); OUT->println(" Mo");
  OUT->print("# psram   : "); OUT->print(ESP.getPsramSize()/1048576); OUT->println(" Mo");
  OUT->print("# sketch  : "); OUT->print(ESP.getSketchSize()); OUT->println(" octets");
  OUT->print("# heap    : "); OUT->print(ESP.getFreeHeap()); OUT->println(" octets libres");
  uint64_t mac = ESP.getEfuseMac();
  OUT->print("# mac     : ");
  for(int8_t i=0;i<6;i++){
    uint8_t b = (mac >> (8*i)) & 0xFF;
    if(b < 16) OUT->print('0');
    OUT->print(b, HEX);
    if(i < 5) OUT->print(':');
  }
  OUT->println();
  OUT->print("# wifi    : AP " AP_SSID " -> http://");
  OUT->print(WiFi.softAPIP());
  OUT->print(", ");
  OUT->print(WiFi.softAPgetStationNum());
  OUT->println(" client(s)");

  /* ---- carte 2 : breakout CAN / MCP2515 ---- */
  OUT->println("# --- Breakout CAN MCP2515 ---");
  uint8_t stat = mcpRead(R_CANSTAT), ctrl = mcpRead(R_CANCTRL);
  OUT->print("# canstat : 0x"); OUT->print(stat, HEX);
  const char *modes[8] = {"normal","sleep","loopback","listen-only",
                          "configuration","?","?","?"};
  OUT->print(" (mode "); OUT->print(modes[stat >> 5]); OUT->println(")");
  OUT->print("# canctrl : 0x"); OUT->println(ctrl, HEX);
  OUT->print("# status  : 0x"); OUT->println(mcpStatus(), HEX);
  OUT->print("# tec/rec : "); OUT->print(mcpRead(0x1C));  /* TEC : erreurs TX */
  OUT->print(" / ");           OUT->println(mcpRead(0x1D)); /* REC : erreurs RX */

  /* test integrite SPI : CNF1 accepte l'ecriture en mode config.
     Bascule temporaire en config, ecrit 0x55, relit, restaure tout. */
  mcpBitMod(R_CANCTRL, 0xE0, 0x80);           /* -> mode config          */
  if((mcpRead(R_CANSTAT) >> 5) == 4){
    uint8_t old = mcpRead(R_CNF1);
    mcpWrite(R_CNF1, 0x55);
    uint8_t back = mcpRead(R_CNF1);
    mcpWrite(R_CNF1, old);
    OUT->print("# test spi: ecrit 0x55, relu 0x"); OUT->print(back, HEX);
    OUT->println(back == 0x55 ? " -> SPI fiable" : " -> SPI CORROMPU (contacts!)");
  }else{
    OUT->println("# test spi: impossible d'entrer en mode config (puce muette?)");
  }
  mcpBitMod(R_CANCTRL, 0xE0, stat & 0xE0);    /* restaure le mode initial */
  OUT->println(mcpOk ? "INFO OK" : "INFO MCP_FAIL");
}

/* ============================================================
 * Dispatch commun : meme table pour USB et web
 * fromWeb : bloque SNIF (flux continu -> USB seulement)
 * ============================================================ */
void execCmd(const String &cmd, bool fromWeb){
  if(!cmd.length()) return;
  if(cmd == "HELP?" || cmd == "HELP") cmdHelp();
  else if(cmd == "PING?")        OUT->println("PONG v1.0");
  else if(cmd == "INFO?")        cmdInfo();
  else if(cmd == "INIT?"){
    mcpOk = mcpInit();                  /* reessai live            */
    digitalWrite(LED_BUILTIN, mcpOk ? LOW : HIGH);
    OUT->println(mcpOk ? "MCP OK" : "MCP FAIL");
  }
  else if(cmd == "DTC?")         cmdDtc(0x03);
  else if(cmd == "PEND?")        cmdDtc(0x07);
  else if(cmd == "CLEAR!")       cmdClear();
  else if(cmd == "EGR?")         cmdEgr();
  else if(cmd == "PROBE?")       cmdProbe();
  else if(cmd == "SCAN?")        cmdScan();
  else if(cmd.startsWith("PID ")) cmdPid(cmd);
  else if(cmd == "SNIF ON"){
    if(fromWeb){ OUT->println("ERR USB SEULEMENT"); return; }
    snifReset();
    snifMode = true;  mcpFilterAll(true);  OUT->println("SNIF ON");
  }
  else if(cmd == "SNIF OFF"){
    if(fromWeb){ OUT->println("ERR USB SEULEMENT"); return; }
    snifMode = false; mcpFilterAll(false); OUT->println("SNIF OFF");
  }
  else                           OUT->println("ERR UNKNOWN");
}

/* ============================================================
 * Page web embarquee (HTML/CSS/JS pur, aucune ressource externe)
 * ============================================================ */
const char PAGE[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="fr"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>NEMO-OBD</title>
<style>
 body{margin:0;font-family:system-ui,sans-serif;background:#111;color:#ddd}
 header{background:#1b3a5c;padding:10px 14px;font-weight:bold}
 header small{font-weight:normal;color:#9cf;float:right}
 .btns{display:grid;grid-template-columns:1fr 1fr;gap:8px;padding:12px}
 button{padding:14px 6px;font-size:1.05em;border:0;border-radius:8px;
        background:#2a5d8f;color:#fff}
 button:active{background:#3f79b4}
 #bClear{background:#8f2a2a}
 .pid{display:flex;gap:8px;padding:0 12px 12px}
 .pid input{flex:1;padding:10px;font-size:1em;border-radius:8px;border:0;
            background:#222;color:#ddd}
 #log{margin:0 12px 12px;padding:10px;background:#000;color:#8f8;
      font-family:monospace;font-size:.9em;white-space:pre-wrap;
      border-radius:8px;min-height:40vh;overflow-y:auto}
 .hdr{color:#888}
</style></head><body>
<header>NEMO-OBD <small>Nemo 1.3 HDi</small></header>
<div class="btns">
 <button onclick="cmd('PING?')">Ping</button>
 <button onclick="cmd('INIT?')">Init MCP</button>
 <button onclick="cmd('DTC?')">Lire d&eacute;fauts</button>
 <button onclick="cmd('PEND?')">D&eacute;fauts en attente</button>
 <button onclick="cmd('EGR?')">&Eacute;tat EGR</button>
 <button onclick="cmd('INFO?')">Infos cartes</button>
 <button id="bClear" onclick="doClear()">EFFACER d&eacute;fauts</button>
 <button onclick="cmd('HELP?')">Aide</button>
</div>
<div class="pid">
 <input id="free" placeholder="commande libre, ex: PID 01 0C">
 <button onclick="cmd(document.getElementById('free').value)">Envoyer</button>
</div>
<pre id="log">pr&ecirc;t.</pre>
<script>
const log=document.getElementById('log');
const CODES_FR = {"P0000":"Aucune panne détectée","P0001":"Commande de régulateur de volume de carburant - circuit ouvert","P0002":"Commande de régulateur de volume de carburant - plage de mesure/performance du circuit","P0003":"Commande de régulateur de volume de carburant - circuit trop bas","P0004":"Commande de régulateur de volume de carburant - circuit trop haut","P0005":"Électrovanne de coupure carburant - circuit ouvert","P0006":"Électrovanne de coupure carburant - circuit trop bas","P0007":"Électrovanne de coupure carburant - circuit trop haut","P0008":"Calage moteur, ligne 1 - performance du moteur","P0009":"Calage moteur, ligne 2 - performance du moteur","P0010":"Capteur d'arbre à cames d'admission, ligne 1 - panne du circuit","P0011":"Position d'arbre à cames, ligne 1 - calage excessivement avancé/performance du système","P0012":"Position d'arbre à cames, ligne 1 - calage excessivement retardé","P0013":"Capteur d'arbre à cames d'échappement, ligne 1 - panne du circuit","P0014":"Capteur d'arbre à cames d'échappement, ligne 1 - calage excessivement avancé/performance du système","P0015":"Capteur d'arbre à cames d'échappement, ligne 1 - calage excessivement retardé","P0016":"Position du vilebrequin/position d'arbre à cames, ligne 1 capteur A - corrélation","P0017":"Position du vilebrequin/position d'arbre à cames, ligne 1 capteur B - corrélation","P0018":"Position du vilebrequin/position d'arbre à cames, ligne 2 capteur A - corrélation","P0019":"Position du vilebrequin/position d'arbre à cames, ligne 2 capteur B - corrélation","P0020":"Capteur d'arbre à cames d'admission, ligne 2 - panne du circuit","P0021":"Position d'arbre à cames d'admission, ligne 2 - calage excessivement avancé","P0022":"Position d'arbre à cames d'admission, ligne 2 - calage excessivement retardé","P0023":"Capteur d'arbre à cames d'échappement, ligne 2 - panne du circuit","P0024":"Position d'arbre à cames d'échappement, ligne 2 - calage excessivement avancé","P0025":"Position d'arbre à cames d'échappement, ligne 2 - calage excessivement retardé","P0026":"Système électrovanne de commande de soupape d'admission, ligne 1 - plage de mesure/performance","P0027":"Système électrovanne de commande de soupape d'échappement, ligne 1 - plage de mesure/performance","P0028":"Système électrovanne de commande de soupape d'admission, ligne 2 - plage de mesure/performance","P0029":"Système électrovanne de commande de soupape d'échappement, ligne 2 - plage de mesure/performance","P0030":"Sonde Lambda 1, ligne 1, commande de chauffage - panne du circuit","P0031":"Sonde Lambda 1, ligne 1, commande de chauffage - circuit trop bas","P0032":"Sonde Lambda 1, ligne 1, commande de chauffage - circuit trop haut","P0033":"Électrovanne de décharge du turbocompresseur - panne du circuit","P0034":"Électrovanne de décharge du turbocompresseur - circuit trop bas","P0035":"Électrovanne de décharge du turbocompresseur - circuit trop haut","P0036":"Sonde Lambda 2, ligne 1 , commande de chauffage - panne du circuit","P0037":"Sonde Lambda 2, ligne 1 , commande de chauffage - circuit trop bas","P0038":"Sonde Lambda 2, ligne 1 , commande de chauffage - circuit trop haut","P0039":"Soupape de dérivation du compresseur/turbocompresseur, circuit de commande - plage de mesure/performance","P0040":"Signaux sondes Lambda inversées, ligne 1 capteur 1/ligne 2 capteur 1","P0041":"Signaux sondes Lambda inversées, ligne 1 capteur 2/ligne 2 capteur 2","P0042":"Sonde Lambda 3, ligne 1 , commande de chauffage - panne du circuit","P0043":"Sonde Lambda 3, ligne 1 , commande de chauffage - circuit trop bas","P0044":"Sonde Lambda 3, ligne 1 , commande de chauffage - circuit trop haut","P0046":"Électrovanne de commande de la pression de suralimentation - circuit ouvert ; Électrovanne de commande de la pression de suralimentation du compresseur/turbocompresseur - plage de mesure/performance du circuit","P0047":"Électrovanne de commande de la pression de suralimentation du compresseur/turbocompresseur - circuit trop bas","P0048":"Électrovanne de commande de la pression de suralimentation du compresseur/turbocompresseur - circuit trop haut","P0049":"Turbine de compresseur/turbocompresseur - régime excessif","P0050":"Sonde Lambda 1, ligne 2, commande de chauffage - panne du circuit","P0051":"Sonde Lambda 1, ligne 2, commande de chauffage - circuit trop bas","P0052":"Sonde Lambda 1, ligne 2, commande de chauffage - circuit trop haut","P0053":"Sonde Lambda, ligne 1 , capteur 1 - résistance du chauffage","P0054":"Sonde Lambda, ligne 1 , capteur 2 - résistance du chauffage","P0055":"Sonde Lambda, ligne 1 , capteur 3 - résistance du chauffage","P0056":"Sonde Lambda 2, ligne 2, commande de chauffage - panne du circuit","P0057":"Sonde Lambda 2, ligne 2, commande de chauffage - circuit de chauffage trop faible","P0058":"Sonde Lambda 2, ligne 2, commande de chauffage - circuit trop haut","P0059":"Sonde Lambda, ligne 2, capteur 1 - résistance du chauffage","P0060":"Sonde Lambda, ligne 2, capteur 2 - résistance du chauffage","P0061":"Sonde Lambda, ligne 2, capteur 3 -résistance du chauffage","P0062":"Sonde Lambda 3, ligne 2, commande de chauffage - panne du circuit","P0063":"Sonde Lambda 3, ligne 2, commande de chauffage - circuit trop bas","P0064":"Sonde Lambda 3, ligne 2, commande de chauffage - circuit trop haut","P0065":"Injecteur assisté par air comprimé - problème de performance/de limites","P0066":"Injecteur assisté par air comprimé - panne du circuit/circuit trop bas","P0067":"Injecteur assisté par air comprimé - circuit trop haut","P0068":"Corrélation capteur de pression absolue du collecteur d'admission/débitmètre d'air/position du papillon","P0069":"Corrélation capteur de pression absolue du collecteur d'admission/capteur de pression atmosphérique","P0070":"Sonde de température extérieure - panne du circuit","P0071":"Sonde de température extérieure - problème de performance/de limites","P0072":"Sonde de température extérieure - valeur d'entrée trop basse","P0073":"Sonde de température extérieure - valeur d'entrée trop haute","P0074":"Sonde de température extérieure - circuit intermittent","P0075":"Électrovanne de commande de soupape d'admission, ligne 1 - panne du circuit","P0076":"Électrovanne de commande de soupape d'admission, ligne 1 - circuit trop bas","P0077":"Électrovanne de commande de soupape d'admission, ligne 1 - circuit trop haut","P0078":"Électrovanne de commande de soupape d'échappement ligne 1 - panne du circuit","P0079":"Électrovanne de commande de soupape d'échappement ligne 1 - circuit trop bas","P0080":"Électrovanne de commande de soupape d'échappement ligne 1 - circuit trop haut","P0081":"Électrovanne de commande de soupape d'admission, ligne 2 - panne du circuit","P0082":"Électrovanne de commande de soupape d'admission, ligne 2 - circuit trop bas","P0083":"Électrovanne de commande de soupape d'admission, ligne 2 - circuit trop haut","P0084":"Électrovanne de commande de soupape d'échappement ligne 2 - panne du circuit","P0085":"Électrovanne de commande de soupape d'échappement ligne 2 - circuit trop bas","P0086":"Électrovanne de commande de soupape d'échappement ligne 2 - circuit trop haut","P0087":"Rampe de distribution/pression du système trop faible","P0088":"Rampe de distribution/pression du système trop haute","P0089":"Régulateur de pression du carburant - problème de performance","P0090":"Électrovanne de dosage de carburant - circuit ouvert","P0091":"Électrovanne de dosage de carburant - court-circuit sur masse","P0092":"Électrovanne de dosage de carburant - court-circuit sur l'alimentation","P0093":"Fuite dans le système d'alimentation en carburant - fuite importante détectée","P0094":"Fuite dans le système d'alimentation en carburant - petit fuite détectée","P0095":"Sonde de température d'air d'admission 2 - panne du circuit","P0096":"Sonde de température d'air d'admission 2 - plage de mesure/performance du circuit","P0097":"Sonde de température d'air d'admission 2 - signal d'entrée du circuit trop bas","P0098":"Sonde de température d'air d'admission 2 - signal d'entrée du circuit trop haut","P0099":"Sonde de température d'air d'admission 2 - circuit intermittent/instable","P0100":"Débitmètre d'air - panne du circuit","P0101":"Débitmètre d'air - problème de performance/de limites","P0102":"Débitmètre d'air - valeur d'entrée trop basse","P0103":"Débitmètre d'air - valeur d'entrée trop haute","P0104":"Débitmètre d'air - circuit intermittent","P0105":"Capteur de pression absolue du collecteur d'admission/capteur de pression atmosphérique - panne du circuit","P0106":"Capteur de pression absolue du collecteur d'admission/capteur de pression atmosphérique - problème de performance/de limites","P0107":"Capteur de pression absolue du collecteur d'admission/capteur de pression atmosphérique - valeur d'entrée trop basse","P0108":"Capteur de pression absolue du collecteur d'admission/capteur de pression atmosphérique - valeur d'entrée trop haute","P0109":"Capteur de pression absolue du collecteur d'admission/capteur de pression atmosphérique - circuit intermittent","P0110":"Sonde de température d'air d'admission - panne du circuit","P0111":"Sonde de température d'air d'admission - problème de performance/de limites","P0112":"Sonde de température d'air d'admission - valeur d'entrée trop basse","P0113":"Sonde de température d'air d'admission - valeur d'entrée trop haute","P0114":"Sonde de température d'air d'admission - circuit intermittent","P0115":"Sonde de température du liquide de refroidissement - panne du circuit","P0116":"Sonde de température du liquide de refroidissement - problème de performance/de limites","P0117":"Sonde de température du liquide de refroidissement - valeur d'entrée trop basse","P0118":"Sonde de température du liquide de refroidissement - valeur d'entrée trop haute","P0119":"Sonde de température du liquide de refroidissement - circuit intermittent","P0120":"Capteur de position de papillon A/capteur de position de la pédale d'accélérateur A - panne du circuit","P0121":"Capteur de position de papillon A/capteur de position de la pédale d'accélérateur A - problème de performance/de limites","P0122":"Capteur de position de papillon A/capteur de position de la pédale d'accélérateur A - valeur d'entrée trop basse","P0123":"Capteur de position de papillon A/capteur de position de la pédale d'accélérateur A - valeur d'entrée trop haute","P0124":"Capteur de position de papillon A/capteur de position de la pédale d'accélérateur A - circuit intermittent","P0125":"Température du liquide de refroidissement insuffisante pour commande en boucle fermée","P0126":"Température du liquide de refroidissement insuffisante pour un fonctionnement stable","P0127":"Température d'air d'admission trop haute","P0128":"Thermostat du liquide de refroidissement - température du liquide de refroidissement inférieure à la température de régulation du thermostat","P0129":"Pression atmosphérique trop basse","P0130":"Sonde Lambda 1, ligne 1 - panne du circuit","P0131":"Sonde Lambda 1, ligne 1 - signal bas","P0132":"Sonde Lambda 1, ligne 1 - signal haut","P0133":"Sonde Lambda 1, ligne 1 - réponse lente","P0134":"Sonde Lambda 1, ligne 1 - pas d'activité détectée","P0135":"Sonde Lambda 1, ligne 1 - commande de chauffage - panne de circuit","P0136":"Sonde Lambda 2, ligne 1 - panne du circuit","P0137":"Sonde Lambda 2, ligne 1 - signal bas","P0138":"Sonde Lambda 2, ligne 1 - signal haut","P0139":"Sonde Lambda 2, ligne 1 - réponse lente","P0140":"Sonde Lambda 2, ligne 1 - pas d'activité détectée","P0141":"Sonde Lambda 2, ligne 1 - commande de chauffage - panne de circuit","P0142":"Sonde Lambda 3, ligne 1 - panne du circuit","P0143":"Sonde Lambda 3, ligne 1 - signal bas","P0144":"Sonde Lambda 3, ligne 1 - signal haut","P0145":"Sonde Lambda 3, ligne 1 - réponse lente","P0146":"Sonde Lambda 3, ligne 1 - pas d'activité détectée","P0147":"Sonde Lambda 3, ligne 1 - commande de chauffage - panne de circuit","P0148":"Erreur de débit de carburant","P0149":"Erreur de calage d'injection","P0150":"Sonde Lambda 1, ligne 2 - panne du circuit","P0151":"Sonde Lambda 1, ligne 2 - signal bas","P0152":"Sonde Lambda 1, ligne 2 - signal haut","P0153":"Sonde Lambda 1, ligne 2 - réponse lente","P0154":"Sonde Lambda 1, ligne 2 - pas d'activité détectée","P0155":"Sonde Lambda 1, ligne 2 - commande de chauffage - panne de circuit","P0156":"Sonde Lambda 2, ligne 2 - panne du circuit","P0157":"Sonde Lambda 2, ligne 2 - signal bas","P0158":"Sonde Lambda 2, ligne 2 - signal haut","P0159":"Sonde Lambda 2, ligne 2 - réponse lente","P0160":"Sonde Lambda 2, ligne 2 - pas d'activité détectée","P0161":"Sonde Lambda 2, ligne 2 - commande de chauffage - panne de circuit","P0162":"Sonde Lambda 3, ligne 2 - panne du circuit","P0163":"Sonde Lambda 3, ligne 2 - signal bas","P0164":"Sonde Lambda 3, ligne 2 - signal haut","P0165":"Sonde Lambda 3, ligne 2 - réponse lente","P0166":"Sonde Lambda 3, ligne 2 - pas d'activité détectée","P0167":"Sonde Lambda 3, ligne 2 - commande de chauffage - panne de circuit","P0168":"Température du carburant trop haute","P0169":"Erreur de composition du carburant","P0170":"Ajustement du carburant, ligne 1 - panne","P0171":"Mélange trop pauvre, ligne 1","P0172":"Mélange trop riche, ligne 1","P0173":"Ajustement du carburant, ligne 2 - panne","P0174":"Mélange trop pauvre, ligne 2","P0175":"Mélange trop riche, ligne 2","P0176":"Capteur de composition du carburant - panne du circuit","P0177":"Capteur de composition du carburant - problème de performance/de limites","P0178":"Capteur de composition du carburant - valeur d'entrée trop basse","P0179":"Capteur de composition du carburant - valeur d'entrée trop haute","P0180":"Sonde de température du carburant A - panne du circuit","P0181":"Sonde de température du carburant A - problème de performance/de limites","P0182":"Sonde de température du carburant A - valeur d'entrée trop basse","P0183":"Sonde de température du carburant A - valeur d'entrée trop haute","P0184":"Sonde de température du carburant A - circuit intermittent","P0185":"Sonde de température du carburant B - panne du circuit","P0186":"Sonde de température du carburant B - problème de performance/de limites","P0187":"Sonde de température du carburant B - valeur d'entrée trop basse","P0188":"Sonde de température du carburant B - valeur d'entrée trop haute","P0189":"Sonde de température du carburant B - circuit intermittent","P0190":"Capteur de pression de la rampe de distribution - panne du circuit","P0191":"Capteur de pression de la rampe de distribution - problème de performance/de limites","P0192":"Capteur de pression de la rampe de distribution - valeur d'entrée trop basse","P0193":"Capteur de pression de la rampe de distribution - valeur d'entrée trop haute","P0194":"Capteur de pression de la rampe de distribution - circuit intermittent","P0195":"Sonde de température d'huile moteur - panne du circuit","P0196":"Sonde de température d'huile moteur - problème de performance/de limites","P0197":"Sonde de température d'huile moteur - valeur d'entrée trop basse","P0198":"Sonde de température d'huile moteur - valeur d'entrée trop haute","P0199":"Sonde de température d'huile moteur - circuit intermittent","P0200":"Injecteur - panne du circuit","P0201":"Injecteur 1 - panne du circuit","P0202":"Injecteur 2 - panne du circuit","P0203":"Injecteur 3 - panne du circuit","P0204":"Injecteur 4 - panne du circuit","P0205":"Injecteur 5 - panne du circuit","P0206":"Injecteur 6 - panne du circuit","P0207":"Injecteur 7 - panne du circuit","P0208":"Injecteur 8 - panne du circuit","P0209":"Injecteur 9 - panne du circuit","P0210":"Injecteur 10 - panne du circuit","P0211":"Injecteur 11 - panne du circuit","P0212":"Injecteur 12 - panne du circuit","P0213":"Injecteur de départ à froid 1 - panne du circuit","P0214":"Injecteur de départ à froid 2 - panne du circuit","P0215":"Électrovanne de coupure de carburant - panne du circuit","P0216":"Commande de calage d'injection - panne du circuit","P0217":"Surchauffe du moteur","P0218":"Surchauffe de la transmission","P0219":"Régime excessif","P0220":"Capteur de position de papillon B/capteur de position de la pédale d'accélérateur B - panne du circuit","P0221":"Capteur de position de papillon B/capteur de position de la pédale d'accélérateur B - problème de performance/de limites","P0222":"Capteur de position de papillon B/capteur de position de la pédale d'accélérateur B- valeur d'entrée trop basse","P0223":"Capteur de position de papillon B/capteur de position de la pédale d'accélérateur B - valeur d'entrée trop haute","P0224":"Capteur de position de papillon B/capteur de position de la pédale d'accélérateur B - circuit intermittent","P0225":"Capteur de position de papillon C/capteur de position de la pédale d'accélérateur C - panne du circuit","P0226":"Capteur de position de papillon C/capteur de position de la pédale d'accélérateur C - problème de performance/de limites","P0227":"Capteur de position de papillon C/capteur de position de la pédale d'accélérateur C - valeur d'entrée trop basse","P0228":"Capteur de position de papillon C/capteur de position de la pédale d'accélérateur C- valeur d'entrée trop haute","P0229":"Capteur de position de papillon C/capteur de position de la pédale d'accélérateur C- circuit intermittent","P0230":"Circuit primaire de pompe à carburant - panne du circuit","P0231":"Circuit secondaire de pompe à carburant - circuit trop bas","P0232":"Circuit secondaire de pompe à carburant - circuit trop haut","P0233":"Circuit secondaire de pompe à carburant - circuit intermittent","P0234":"Condition de suralimentation du moteur - limite dépassée","P0235":"Capteur de pression absolue du collecteur d'admission A, circuit du turbocompresseur - panne du circuit","P0236":"Capteur de pression absolue du collecteur d'admission A, circuit du turbocompresseur - problème de performance/de limites","P0237":"Capteur de pression absolue du collecteur d'admission A, circuit du turbocompresseur - valeur d'entrée trop basse","P0238":"Capteur de pression absolue du collecteur d'admission A, circuit du turbocompresseur - valeur d'entrée trop haute","P0239":"Capteur de pression absolue du collecteur d'admission B, circuit du turbocompresseur - panne du circuit","P0240":"Capteur de pression absolue du collecteur d'admission B, circuit du turbocompresseur - problème de performance/de limites","P0241":"Capteur de pression absolue du collecteur d'admission B, circuit du turbocompresseur - valeur d'entrée trop basse","P0242":"Capteur de pression absolue du collecteur d'admission B, circuit du turbocompresseur - valeur d'entrée trop haute","P0243":"Électrovanne de décharge du turbocompresseur A - panne du circuit","P0244":"Électrovanne de décharge du turbocompresseur A - problème de performance/de limites","P0245":"Électrovanne de décharge du turbocompresseur A - circuit trop bas","P0246":"Électrovanne de décharge du turbocompresseur A - circuit trop haut","P0247":"Électrovanne de décharge du turbocompresseur B - panne du circuit","P0248":"Électrovanne de décharge du turbocompresseur B - problème de performance/de limites","P0249":"Électrovanne de décharge du turbocompresseur B - circuit trop bas","P0250":"Électrovanne de décharge du turbocompresseur B - circuit trop haut","P0251":"Pompe d'injection A, rotor/cames - panne du circuit","P0252":"Pompe d'injection A, rotor/cames - problème de performance/de limites","P0253":"Pompe d'injection A, rotor/cames - circuit trop bas","P0254":"Pompe d'injection A, rotor/cames - circuit trop haut","P0255":"Pompe d'injection A, rotor/cames - circuit intermittent","P0256":"Pompe d'injection B, rotor/cames - panne du circuit","P0257":"Pompe d'injection B, rotor/cames - problème de performance/de limites","P0258":"Pompe d'injection B, rotor/cames - circuit trop bas","P0259":"Pompe d'injection B, rotor/cames - circuit trop haut","P0260":"Pompe d'injection B, rotor/cames - circuit intermittent","P0261":"Injecteur 1 - circuit trop bas","P0262":"Injecteur 1 - circuit trop haut","P0263":"Cylindre 1 - Quantité de fuel injecté - défaut d'équilibrage","P0264":"Injecteur 2 - circuit trop bas","P0265":"Injecteur 2 - circuit trop haut","P0266":"Cylindre 2 - Quantité de fuel injecté - défaut d'équilibrage","P0267":"Injecteur 3 - circuit trop bas","P0268":"Injecteur 3 - circuit trop haut","P0269":"Cylindre 3 - Quantité de fuel injecté - défaut d'équilibrage","P0270":"Injecteur 4 - circuit trop bas","P0271":"Injecteur 4 - circuit trop haut","P0272":"Cylindre 4 - Quantité de fuel injecté - défaut d'équilibrage","P0273":"Injecteur 5 - circuit trop bas","P0274":"Injecteur 5 - circuit trop haut","P0275":"Cylindre 5 - Quantité de fuel injecté - défaut d'équilibrage","P0276":"Injecteur 6 - circuit trop bas","P0277":"Injecteur 6 - circuit trop haut","P0278":"Cylindre 6 - Quantité de fuel injecté - défaut d'équilibrage","P0279":"Injecteur 7 - circuit trop bas","P0280":"Injecteur 7 - circuit trop haut","P0281":"Cylindre 7 - Quantité de fuel injecté - défaut d'équilibrage","P0282":"Injecteur 8 - circuit trop bas","P0283":"Injecteur 8 - circuit trop haut","P0284":"Cylindre 8 - Quantité de fuel injecté - défaut d'équilibrage","P0285":"Injecteur 9 - circuit trop bas","P0286":"Injecteur 9 - circuit trop haut","P0287":"Cylindre 9 - Quantité de fuel injecté - défaut d'équilibrage","P0288":"Injecteur 10 - circuit trop bas","P0289":"Injecteur 10 - circuit trop haut","P0290":"Cylindre 10 - Quantité de fuel injecté - défaut d'équilibrage","P0291":"Injecteur 11 - circuit trop bas","P0292":"Injecteur 11 - circuit trop haut","P0293":"Cylindre 11 - Quantité de fuel injecté - défaut d'équilibrage","P0294":"Injecteur 12 - circuit trop bas","P0295":"Injecteur 12 - circuit trop haut","P0296":"Cylindre 12 - Quantité de fuel injecté - défaut d'équilibrage","P0297":"Vitesse excessive véhicule","P0298":"Température d'huile moteur trop haute","P0299":"Compresseur/turbocompresseur - pression de suralimentation faible","P0300":"Cylindre(s) multiple(s) - ratés d'allumage aléatoire détectés","P0301":"Cylindre 1 - ratés d'allumage détectés","P0302":"Cylindre 2 - ratés d'allumage détectés","P0303":"Cylindre 3 - ratés d'allumage détectés","P0304":"Cylindre 4 - ratés d'allumage détectés","P0305":"Cylindre 5 - ratés d'allumage détectés","P0306":"Cylindre 6 - ratés d'allumage détectés","P0307":"Cylindre 7 - ratés d'allumage détectés","P0308":"Cylindre 8 - ratés d'allumage détectés","P0309":"Cylindre 9 - ratés d'allumage détectés","P0310":"Cylindre 10 - ratés d'allumage détectés","P0311":"Cylindre 11 - ratés d'allumage détectés","P0312":"Cylindre 12 - ratés d'allumage détectés","P0313":"Ratés d'allumage détectés avec niveau de carburant trop bas","P0314":"Raté d'allumage dans un seul cylindre - cylindre non spécifié","P0315":"Système position du vilebrequin - variation non apprise","P0316":"Ratés d'allumage détectés au démarrage - dans les 1000 premiers tours moteur","P0317":"Composant détecteur de route accidentée absent","P0318":"Signal du détecteur de route accidentée A - panne du circuit","P0319":"Signal du détecteur de route accidentée B - panne du circuit","P0320":"Capteur de vilebrequin/de régime - panne du circuit","P0321":"Capteur de vilebrequin/de régime - problème de performance/de limites","P0322":"Capteur de vilebrequin/de régime - aucun signal","P0323":"Capteur de vilebrequin/de régime - circuit intermittent","P0324":"Erreur de système anti-cliquetis","P0325":"Détecteur de cliquetis 1 (ligne 2 ou capteur unique) - panne du circuit","P0326":"Détecteur de cliquetis 1 (ligne 2 ou capteur unique) - problème de performance/de limites","P0327":"Détecteur de cliquetis 1 (ligne 2 ou capteur unique) - valeur d'entrée trop basse","P0328":"Détecteur de cliquetis 1 (ligne 2 ou capteur unique) - valeur d'entrée trop haute","P0329":"Détecteur de cliquetis 1 (ligne 2 ou capteur unique) - circuit intermittent","P0330":"Détecteur de cliquetis 2, ligne 2 - panne du circuit","P0331":"Détecteur de cliquetis 2, ligne 2 - problème de performance/de limites","P0332":"Détecteur de cliquetis 2, ligne 2 - valeur d'entrée trop basse","P0333":"Détecteur de cliquetis 2, ligne 2 - valeur d'entrée trop haute","P0334":"Détecteur de cliquetis 2, ligne 2 - circuit intermittent","P0335":"Capteur de vilebrequin - panne du circuit","P0336":"Capteur de vilebrequin - problème de performance/de limites","P0337":"Capteur de vilebrequin - valeur d'entrée trop basse","P0338":"Capteur de vilebrequin - valeur d'entrée trop haute","P0339":"Capteur de vilebrequin - circuit intermittent","P0340":"Capteur d'arbre à cames A ligne 1 - panne du circuit","P0341":"Capteur d'arbre à cames A ligne 1 - problème de performance/de limites","P0342":"Capteur d'arbre à cames A ligne 1 - valeur d'entrée trop basse","P0343":"Capteur d'arbre à cames A ligne 1 - valeur d'entrée trop haute","P0344":"Capteur d'arbre à cames A ligne 1 - circuit intermittent","P0345":"Capteur d'arbre à cames A ligne 2 - panne du circuit","P0346":"Capteur d'arbre à cames A ligne 2 - problème de performance/de limites","P0347":"Capteur d'arbre à cames A ligne 2 - valeur d'entrée trop basse","P0348":"Capteur d'arbre à cames A ligne 2 - valeur d'entrée trop haute","P0349":"Capteur d'arbre à cames A ligne 2 - circuit intermittent","P0350":"Bobine d'allumage, primaire/secondaire - panne du circuit","P0351":"Bobine d'allumage A, primaire/secondaire - panne du circuit","P0352":"Bobine d'allumage B, primaire/secondaire - panne du circuit","P0353":"Bobine d'allumage C, primaire/secondaire - panne du circuit","P0354":"Bobine d'allumage D, primaire/secondaire - panne du circuit","P0355":"Bobine d'allumage E, primaire/secondaire - panne du circuit","P0356":"Bobine d'allumage F, primaire/secondaire - panne du circuit","P0357":"Bobine d'allumage G, primaire/secondaire - panne du circuit","P0358":"Bobine d'allumage H, primaire/secondaire - panne du circuit","P0359":"Bobine d'allumage I, primaire/secondaire - panne du circuit","P0360":"Bobine d'allumage J, primaire/secondaire - panne du circuit","P0361":"Bobine d'allumage K, primaire/secondaire - panne du circuit","P0362":"Bobine d'allumage L, primaire/secondaire - panne du circuit","P0363":"Ratés d'allumage détectés - alimentation en carburant désactivée","P0364":"Bobine d'allumage L, primaire/secondaire - panne du circuit","P0365":"Capteur d'arbre à cames B, ligne 1 - panne du circuit","P0366":"Capteur d'arbre à cames B, ligne 1 - plage de mesure/performance du circuit","P0367":"Capteur d'arbre à cames B, ligne 1 - signal d'entrée du circuit trop bas","P0368":"Capteur d'arbre à cames B, ligne 1 - signal d'entrée du circuit trop haut","P0369":"Capteur d'arbre à cames B, ligne 1 - circuit intermittent","P0370":"Référence de calage, signal haute résolution A - panne","P0371":"Référence de calage, signal haute résolution A - trop de signaux","P0372":"Référence de calage, signal haute résolution A - trop peu de signaux","P0373":"Référence de calage, signal haute résolution A - signaux irréguliers intermittents","P0374":"Référence de calage, signal haute résolution A - pas de signaux","P0375":"Référence de calage, signal haute résolution B - panne","P0376":"Référence de calage, signal haute résolution B - trop de signaux","P0377":"Référence de calage, signal haute résolution B - trop peu de signaux","P0378":"Référence de calage, signal haute résolution B - signaux irréguliers intermittents","P0379":"Référence de calage, signal haute résolution B - pas de signaux","P0380":"Bougies de préchauffage, circuit A - panne","P0381":"Lampe témoin bougies de préchauffage - panne du circuit","P0382":"Bougies de préchauffage, circuit B - panne","P0383":"Lampe témoin bougies de préchauffage","P0384":"Bougies de préchauffage, circuit B","P0385":"Capteur de vilebrequin B - panne du circuit","P0386":"Capteur de vilebrequin B - problème de performance/de limites","P0387":"Capteur de vilebrequin B - valeur d'entrée trop basse","P0388":"Capteur de vilebrequin B - valeur d'entrée trop haute","P0389":"Capteur de vilebrequin B - circuit intermittent","P0390":"Capteur d'arbre à cames B, ligne 2 - panne du circuit","P0391":"Capteur d'arbre à cames B, ligne 2 - plage de mesure/performance du circuit","P0392":"Capteur d'arbre à cames B, ligne 2 - signal d'entrée du circuit trop bas","P0393":"Capteur d'arbre à cames B, ligne 2 - signal d'entrée du circuit trop haut","P0394":"Capteur d'arbre à cames B, ligne 2 - circuit intermittent","P0395":"Capteur d'arbre à cames B, ligne 2 - signal d'entrée du circuit trop haut","P0396":"Capteur d'arbre à cames B, ligne 2 - circuit intermittent","P0400":"Système EGR - problème de débit","P0401":"Système EGR - débit insuffisant détecté","P0402":"Système EGR - débit excessif détecté","P0403":"Recyclage des gaz d'échappement - panne du circuit","P0404":"Système EGR - problème de performance/de limites","P0405":"Capteur de position de la valve EGR A - valeur d'entrée trop basse","P0406":"Capteur de position de la valve EGR A - valeur d'entrée trop haute","P0407":"Capteur de position de la valve EGR B - valeur d'entrée trop basse","P0408":"Capteur de position de la valve EGR B - valeur d'entrée trop haute","P0409":"Capteur EGR A - panne du circuit","P0410":"Système d'injection d'air secondaire - panne","P0411":"Système d'injection d'air secondaire - débit incorrect détecté","P0412":"Électrovanne d'injection d'air secondaire A - panne du circuit","P0413":"Électrovanne d'injection d'air secondaire A - circuit ouvert","P0414":"Électrovanne d'injection d'air secondaire A - courtcircuit","P0415":"Électrovanne d'injection d'air secondaire B - panne du circuit","P0416":"Électrovanne d'injection d'air secondaire B - circuit ouvert","P0417":"Electrovanne d'injection d'air secondaire B - courtcircuit","P0418":"Relais de la pompe d'injection d'air secondaire A - panne du circuit","P0419":"Relais de la pompe d'injection d'air secondaire B - panne du circuit","P0420":"Circuit de catalyseur, ligne 1 - rendement inférieur au seuil","P0421":"Pré-catalyseur, ligne 1 - rendement inférieur au seuil","P0422":"Catalyseur principal, ligne 1 - rendement inférieur au seuil","P0423":"Catalyseur chauffé, ligne 1 - rendement inférieur au seuil","P0424":"Catalyseur chauffé, ligne 1 - température inférieur au seuil","P0425":"Sonde de température du catalyseur 1, ligne 1","P0426":"Sonde de température du catalyseur 1, ligne 1 - plage de mesure/performance","P0427":"Sonde de température du catalyseur 1, ligne 1 - valeur d'entrée trop basse","P0428":"Sonde de température du catalyseur 1, ligne 1 - valeur d'entrée trop haute","P0429":"Chauffage catalyseur, ligne 1 - panne du circuit de commande","P0430":"Circuit de catalyseur, ligne 2 - rendement inférieur au seuil","P0431":"Pré-catalyseur, ligne 2 - rendement inférieur au seuil","P0432":"Catalyseur principal, ligne 2 - rendement inférieur au seuil","P0433":"Catalyseur chauffé, ligne 2 - rendement inférieur au seuil","P0434":"Catalyseur chauffé, ligne 2 - température inférieur au seuil","P0435":"Sonde de température du catalyseur 1, ligne 2","P0436":"Sonde de température du catalyseur 1, ligne 2 - plage de mesure/performance","P0437":"Sonde de température du catalyseur 1, ligne 2 - valeur d'entrée trop basse","P0438":"Sonde de température du catalyseur 1, ligne 2 - valeur d'entrée trop haute","P0439":"Chauffage catalyseur, ligne 2 - panne du circuit","P0440":"Système de purge canister - panne","P0441":"Système de purge canister - débit incorrect détecté","P0442":"Système de purge canister - petite fuite détectée","P0443":"Électrovanne de purge canister - panne du circuit","P0444":"Électrovanne de purge canister - circuit ouvert","P0445":"Électrovanne de purge canister - courtcircuit","P0446":"Système de purge canister, commande de ventilation - panne du circuit","P0447":"Système de purge canister, commande de ventilation - circuit ouvert","P0448":"Système de purge canister, commande de ventilation - court-circuit","P0449":"Système de purge canister, soupape de ventilation - panne du circuit","P0460":"Capteur de pression de purge canister - panne du circuit ; Sonde de niveau du réservoir de carburant - panne du circuit","P0461":"Capteur de pression de purge canister - problème de performance/de limites ; Sonde de niveau du réservoir de carburant - problème de performance/de limites","P0462":"Capteur de pression de purge canister - valeur d'entrée trop basse ; Sonde de niveau du réservoir de carburant - valeur d'entrée trop basse","P0463":"Capteur de pression de purge canister - valeur d'entrée trop haute ; Sonde de niveau du réservoir de carburant - valeur d'entrée trop haute","P0464":"Capteur de pression de purge canister - circuit intermittent ; Sonde de niveau du réservoir de carburant - circuit intermittent","P0465":"Système de purge canister - fuite importante détectée ; Capteur de flux de purge canister - panne du circuit","P0466":"Système de purge canister - très petite fuite détectée ; Capteur de flux de purge canister - problème de performance/de limites","P0467":"Système de purge canister - fuite détectée ; Capteur de flux de purge canister - valeur d'entrée trop basse","P0468":"Système de purge canister, électrovanne de purge canister - circuit trop bas ; Capteur de flux de purge canister - valeur d'entrée trop haute","P0469":"Système de purge canister, électrovanne de purge canister - circuit trop haut ; Capteur de flux de purge canister - circuit intermittent","P0470":"Capteur de pression des gaz d'échappement - panne du circuit","P0471":"Capteur de pression des gaz d'échappement - problème de performance/de limites","P0472":"Capteur de pression des gaz d'échappement - valeur d'entrée trop basse","P0473":"Capteur de pression des gaz d'échappement - valeur d'entrée trop haute","P0474":"Capteur de pression des gaz d'échappement - circuit intermittent","P0475":"Électrovanne de commande de pression des gaz d'échappement-panne du circuit","P0476":"Électrovanne de commande de pression des gaz d'échappement-problème de performance/de limites","P0477":"Électrovanne de commande de pression des gaz d'échappement - valeur d'entrée trop basse","P0478":"Électrovanne de commande de pression des gaz d'échappement - valeur d'entrée trop haute","P0479":"Électrovanne de commande de pression des gaz d'échappement- circuit intermittent","P0480":"Motoventilateur de refroidissement 1 - panne du circuit","P0481":"Motoventilateur de refroidissement 2 - panne du circuit","P0482":"Motoventilateur de refroidissement 3 - panne du circuit","P0483":"Motoventilateur de refroidissement, contrôle de rationalité - panne","P0484":"Motoventilateur de refroidissement - surcharge du circuit","P0485":"Motoventilateur de refroidissement, alimentation/masse - panne du circuit","P0486":"Capteur de position de la valve EGR B - panne du circuit","P0487":"Système EGR, commande de position du papillon - panne du circuit","P0488":"Système EGR, commande de position du papillon - plage de mesure/performance","P0489":"Système EGR - circuit trop bas","P0490":"Système EGR - circuit trop haut","P0491":"Système d'injection d'air secondaire, ligne 1 - panne","P0492":"Système d'injection d'air secondaire, ligne 2 - panne","P0493":"Vitesse motoventilateur de refroidissement moteur - excessive","P0494":"Vitesse motoventilateur de refroidissement moteur - basse","P0495":"Vitesse motoventilateur de refroidissement moteur - haute","P0496":"Système de purge canister - flux de purge élevé","P0497":"Système de purge canister - flux de purge faible","P0498":"Système de purge canister, commande de ventilation - circuit trop bas","P0499":"Système de purge canister, commande de ventilation - circuit trop haut","P0500":"Capteur de vitesse du véhicule - panne du circuit","P0501":"Capteur de vitesse du véhicule - problème de performance/de limites","P0502":"Capteur de vitesse du véhicule - valeur d'entrée trop basse","P0503":"Capteur de vitesse du véhicule - valeur d'entrée intermittente/irrégulière/trop haute","P0504":"Contacteur de freinage - corrélation A/B","P0505":"Commande du ralenti - panne","P0506":"Commande du ralenti - régime plus lent que prévu","P0507":"Commande du ralenti - régime plus rapide que prévu","P0508":"Commande d'air au ralenti - circuit trop bas","P0509":"Commande d'air au ralenti - circuit trop haut","P0510":"Contacteur de position fermée de papillon - panne du circuit","P0511":"Commande d'air au ralenti - panne du circuit","P0512":"Circuit de commande du démarreur - panne","P0513":"Clé de l'antidémarrage incorrecte","P0514":"Sonde de température batterie - plage de mesure/performance du circuit","P0515":"Sonde de température batterie - panne du circuit","P0516":"Sonde de température batterie - circuit trop bas","P0517":"Sonde de température batterie - circuit trop haut","P0518":"Commande d'air au ralenti - circuit intermittent","P0519":"Commande d'air au ralenti - performance circuit","P0520":"Capteur de pression/pressostat d'huile moteur - panne du circuit","P0521":"Capteur de pression/pressostat d'huile moteur - problème de performance/de limites","P0522":"Capteur de pression/pressostat d'huile moteur - basse tension","P0523":"Capteur de pression/pressostat d'huile moteur - haute tension","P0524":"Pression d'huile moteur trop basse","P0525":"Régulateur de vitesse, commande d'actuateur - plage de mesure/performance du circuit","P0526":"Capteur de vitesse du motoventilateur de refroidissement moteur- panne du circuit","P0527":"Capteur de vitesse du motoventilateur de refroidissement moteur - plage de mesure/performance du circuit","P0528":"Capteur de vitesse du motoventilateur de refroidissement moteur - aucun signal","P0529":"Capteur de vitesse du motoventilateur de refroidissement moteur - circuit intermittent","P0530":"Capteur de pression du réfrigérant de la climatisation - panne du circuit","P0531":"Capteur de pression du réfrigérant de la climatisation - problème de performance/de limites","P0532":"Capteur de pression du réfrigérant de la climatisation - valeur d'entrée trop basse","P0533":"Capteur de pression du réfrigérant de la climatisation - valeur d'entrée trop haute","P0534":"Perte de charge du réfrigérant de la climatisation","P0535":"Sonde de température de l'évaporateur climatisation - panne du circuit","P0536":"Sonde de température de l'évaporateur climatisation - plage de mesure/performance du circuit","P0537":"Sonde de température de l'évaporateur climatisation - circuit trop bas","P0538":"Sonde de température de l'évaporateur climatisation - circuit trop haut","P0539":"Sonde de température de l'évaporateur climatisation - circuit intermittent","P0540":"Chauffage d'air d'admission A - panne du circuit","P0541":"Chauffage d'air d'admission A - circuit trop bas","P0542":"Chauffage d'air d'admission A - circuit trop haut","P0543":"Chauffage d'air d'admission A - circuit ouvert","P0544":"Sonde de température EGR, ligne 1 - panne du circuit","P0545":"Sonde de température EGR, ligne 1 - valeur d'entrée trop basse","P0546":"Sonde de température EGR, ligne 1 - valeur d'entrée trop haute","P0547":"Sonde de température des gaz d'échappement, ligne 2 capteur 1 - panne du circuit","P0548":"Sonde de température des gaz d'échappement, ligne 2 capteur 1 - circuit trop bas","P0549":"Sonde de température des gaz d'échappement, ligne 2 capteur 1 - circuit trop haut","P0550":"Capteur/pressostat de direction assistée - panne du circuit","P0551":"Capteur/pressostat de direction assistée - problème de performance/de limites","P0552":"Capteur/pressostat de direction assistée - valeur d'entrée trop basse","P0553":"Capteur/pressostat de direction assistée - valeur d'entrée trop haute","P0554":"Capteur/pressostat de direction assistée - circuit intermittent","P0555":"Capteur de pression du servofrein - panne du circuit","P0556":"Capteur de pression du servofrein - plage de mesure/performance du circuit","P0557":"Capteur de pression du servofrein - signal d'entrée du circuit trop bas","P0558":"Capteur de pression du servofrein - signal d'entrée du circuit trop haut","P0559":"Capteur de pression du servofrein - circuit intermittent","P0560":"Tension du système - panne","P0561":"Tension du système - instable","P0562":"Tension du système - basse","P0563":"Tension du système - haute","P0564":"Régulateur de vitesse, signal d'entrée A du contacteur multifonction - panne du circuit","P0565":"Commutateur principal du régulateur de vitesse, signal de marche (ON) - panne","P0566":"Commutateur principal du régulateur de vitesse, signal d'arrêt (OFF) - panne","P0567":"Sélecteur de reprise du régulateur de vitesse - panne","P0568":"Commutateur principal du régulateur de vitesse, signal SET - panne","P0569":"Sélecteur du régulateur de vitesse, signal COAST - panne","P0570":"Régulateur de vitesse, signal du capteur de position de la pédale - panne","P0571":"Contacteur de régulateur de vitesse/de freinage A - panne du circuit","P0572":"Contacteur de régulateur de vitesse/de freinage A - circuit trop bas","P0573":"Contacteur de régulateur de vitesse/de freinage A - circuit trop haut","P0574":"Régulateur de vitesse - vitesse trop haute","P0575":"Régulateur de vitesse - panne du circuit d'entrée","P0576":"Régulateur de vitesse - circuit d'entrée trop faible","P0577":"Régulateur de vitesse - circuit d'entrée trop fort","P0578":"Régulateur de vitesse, signal d'entrée A du contacteur multifonction - circuit bloqué","P0579":"Régulateur de vitesse, signal d'entrée A du contacteur multifonction - plage de mesure/performance du circuit","P0580":"Régulateur de vitesse, signal d'entrée A du contacteur multifonction - circuit trop bas","P0581":"Régulateur de vitesse, signal d'entrée A du contacteur multifonction - circuit trop haut","P0582":"Régulateur de vitesse, commande par dépression - circuit ouvert","P0583":"Régulateur de vitesse, commande par dépression - circuit trop bas","P0584":"Régulateur de vitesse, commande par dépression - circuit trop haut","P0585":"Régulateur de vitesse, signal d'entrée A/B du contacteur multifonction - corrélation","P0586":"Régulateur de vitesse, commande de ventilation - circuit ouvert","P0587":"Régulateur de vitesse, commande de ventilation - circuit trop bas","P0588":"Régulateur de vitesse, commande de ventilation - circuit trop haut","P0589":"Régulateur de vitesse, signal d'entrée B du contacteur multifonction - panne du circuit","P0590":"Régulateur de vitesse, signal d'entrée B du contacteur multifonction - circuit bloqué","P0591":"Régulateur de vitesse, signal d'entrée B du contacteur multifonction - plage de mesure/performance du circuit","P0592":"Régulateur de vitesse, signal d'entrée B du contacteur multifonction - circuit trop bas","P0593":"Régulateur de vitesse, signal d'entrée B du contacteur multifonction - circuit trop haut","P0594":"Régulateur de vitesse, commande d'actuateur - circuit ouvert","P0595":"Régulateur de vitesse, commande d'actuateur - circuit trop bas","P0596":"Régulateur de vitesse, commande d'actuateur - circuit trop haut","P0597":"Dispositif de commande du chauffage à thermostat - circuit ouvert","P0598":"Dispositif de commande du chauffage à thermostat - circuit trop bas","P0599":"Dispositif de commande du chauffage à thermostat - circuit trop haut","P0600":"Bus de données CAN - panne","P0601":"Calculateur de gestion moteur - erreur du total de contrôle de mémoire","P0602":"Calculateur de gestion moteur - erreur de programmation","P0603":"Calculateur de gestion moteur - erreur KAM","P0604":"Calculateur de gestion moteur - erreur RAM","P0605":"Calculateur de gestion moteur - erreur ROM","P0606":"Calculateur de gestion moteur/calculateur combiné moteur-transmission","P0607":"Calculateur/boîtier électronique - problème de performance - panne de processeur","P0608":"Calculateur de gestion moteur, signal de sortie du capteur de vitesse du véhicule A - panne","P0609":"Calculateur de gestion moteur, signal de sortie du capteur de vitesse du véhicule B - panne","P0610":"Calculateur/boîtier électronique - erreur options véhicule","P0611":"Boîtier électronique d'injecteur de carburant - problème de performance","P0612":"Boîtier électronique d'injecteur carburant - circuit du relais de commande","P0613":"Calculateur de transmission - erreur de processeur","P0614":"Calculateur de gestion moteur/calculateur de transmission - désaccord","P0615":"Relais du démarreur - panne du circuit de gestion moteur","P0616":"Relais du démarreur - circuit trop bas de gestion moteur","P0617":"Relais du démarreur - circuit trop haut calculateur de gestion moteur","P0618":"Boîtier électronique carburant alternatif - erreur KAM","P0619":"Boîtier électronique carburant alternatif - erreur RAM/ROM","P0620":"Alternateur, commande - panne du circuit","P0621":"Alternateur - lampe témoin de charge L - panne du circuit","P0622":"Alternateur - lampe témoin de charge F - panne du circuit","P0623":"Lampe témoin de charge - panne du circuit","P0624":"Lampe témoin trappe du réservoir de carburant - panne du circuit","P0625":"Borne champ du générateur - circuit trop bas","P0626":"Borne champ du générateur - circuit trop haut","P0627":"Commande de pompe à carburant A - circuit ouvert","P0628":"Commande de pompe à carburant A - circuit trop bas","P0629":"Commande de pompe à carburant A - circuit trop haut","P0630":"VIN non programmé ou désaccord d'identification - calculateur de gestion moteur/calculateur transmission/gestion moteur","P0631":"VIN non programmé ou désaccord d'identification - calculateur de transmission","P0632":"Compteur kilométrique non programmé - calculateur de gestion moteur/calculateur transmission/gestion moteur","P0633":"Clé de l'antidémarrage non programmée - calculateur de gestion moteur/calculateur transmission/gestion moteur","P0634":"Calculateur transmission/gestion moteur/calculateur de gestion moteur/calculateur de transmission - température interne trop haute","P0635":"Commande direction assistée - panne du circuit","P0636":"Commande direction assistée - circuit trop bas","P0637":"Commande direction assistée - circuit trop haut","P0638":"Commande d'actuateur de papillon, ligne 1 - problème de performance/de limites","P0639":"Commande d'actuateur de papillon, ligne 2 - plage mesure/performance","P0640":"Commande de chauffage d'air d'admission - panne du circuit","P0641":"Tension de référence du capteur A - circuit ouvert","P0642":"Calculateur de gestion moteur, anti-c1iquetis - défectueux","P0643":"Tension de référence du capteur A - circuit trop haut","P0644":"Affichage conducteur, communication de séries de données - panne du circuit","P0645":"Relais d'embrayage du compresseur de climatisation","P0646":"Relais embrayage du compresseur de climatisation - circuit trop bas","P0647":"Relais embrayage du compresseur de climatisation - circuit trop haut","P0648":"Lampe témoin antidémarrage - panne du circuit","P0649":"Lampe témoin régulateur de vitesse - circuit moteur","P0650":"Lampe témoin d'affichage des défauts - panne du circuit","P0651":"Tension de référence du capteur B - circuit ouvert","P0652":"Tension de référence du capteur B - circuit trop bas","P0653":"Tension de référence du capteur B - circuit trop haut","P0654":"Régime moteur, sortie - panne du circuit","P0655":"Signal de sortie du témoin de surchauffe du moteur - panne du circuit","P0656":"Signal de sortie de niveau de carburant - panne du circuit","P0657":"Tension d'alimentation d'actuateur - circuit ouvert","P0658":"Tension d'alimentation d'actuateur - circuit trop bas","P0659":"Tension d'alimentation d'actuateur - circuit trop haut","P0660":"Électrovanne d'admission d'air dans le collecteur, ligne 1 - circuit ouvert","P0661":"Électrovanne d'admission d'air dans le collecteur, ligne 1 - circuit trop bas","P0662":"Électrovanne d'admission d'air dans le collecteur, ligne 1 - circuit trop haut","P0663":"Électrovanne d'admission d'air dans le collecteur, ligne 2 - circuit ouvert","P0664":"Électrovanne d'admission d'air dans le collecteur, ligne 2 - circuit trop bas","P0665":"Électrovanne d'admission d'air dans le collecteur, ligne 2 - circuit trop haut","P0666":"Sonde de température interne calculateur combiné moteur-transmission/ calculateur de gestion moteur/calculateur de transmission - panne du circuit","P0667":"Sonde de température interne calculateur combiné moteur-transmission/ calculateur de gestion moteur/calculateur de transmission - plage de mesure/performance","P0668":"Sonde de température interne calculateur combiné moteur-transmission/ calculateur de gestion moteur/calculateur de transmission - circuit trop bas","P0669":"Sonde de température interne calculateur combiné moteur-transmission/calculateur de gestion moteur/calculateur de transmission - circuit trop haut","P0670":"Boîtier électronique des bougies de préchauffage - panne du circuit","P0671":"Bougies de préchauffage, cylindre 1 - panne du circuit","P0672":"Bougies de préchauffage, cylindre 2 - panne du circuit","P0673":"Bougies de préchauffage, cylindre 3 - panne du circuit","P0674":"Bougies de préchauffage, cylindre 4 - panne du circuit","P0675":"Bougies de préchauffage, cylindre 5 - panne du circuit","P0676":"Bougies de préchauffage, cylindre 6 - panne du circuit","P0677":"Bougies de préchauffage, cylindre 7 - panne du circuit","P0678":"Bougies de préchauffage, cylindre 8 - panne du circuit","P0679":"Bougies de préchauffage, cylindre 9 - panne du circuit","P0680":"Bougies de préchauffage, cylindre 10 - panne du circuit","P0681":"Bougies de préchauffage, cylindre 11 - panne du circuit","P0682":"Bougies de préchauffage, cylindre 12 - panne du circuit","P0683":"Communication boîtier électronique des bougies de préchauffage/ calculateur de gestion moteur/calculateur combiné moteur-transmission - panne","P0684":"Communication boîtier électronique des bougies de préchauffage/calculateur de gestion moteur/calculateur combiné moteur-transmission - plage de mesure/performance","P0685":"Relais d'alimentation électrique calculateur de gestion moteur/calculateur combiné transmission-gestion moteur - circuit ouvert","P0686":"Relais d'alimentation électrique calculateur de gestion moteur/calculateur combiné transmission-gestion moteur - circuit trop bas","P0687":"Relais de gestion du moteur - court-circuit sur la masse","P0688":"Relais de gestion du moteur - court-circuit sur l'alimentation","P0689":"Relais d'alimentation électrique calculateur de gestion moteur/calculateur combiné transmission-gestion moteur - circuit de sondage trop faible","P0690":"Relais d'alimentation électrique calculateur de gestion moteur/calculateur combiné transmission-gestion moteur - circuit de sondage trop fort","P0691":"Motoventilateur de refroidissement 1 - circuit trop bas","P0692":"Motoventilateur de refroidissement 1 - circuit trop haut","P0693":"Motoventilateur de refroidissement 2 - circuit trop bas","P0694":"Motoventilateur de refroidissement 2 - circuit trop haut","P0695":"Motoventilateur de refroidissement 3 - circuit trop bas","P0696":"Motoventilateur de refroidissement 3 - circuit trop haut","P0697":"Tension de référence du capteur C - circuit ouvert","P0698":"Tension de référence du capteur C - circuit trop bas","P0699":"Tension de référence du capteur C - circuit trop haut","P0700":"Commande de la transmission - panne","P0701":"Commande de la transmission - problème de performance/de limites","P0702":"Commande de la transmission - électrique","P0703":"Convertisseur de couple/contacteur de freinage B - panne du circuit","P0704":"Contacteur de position de la pédale d'embrayage - panne du circuit","P0705":"Contacteur/capteur de position du levier de vitesse, signal d'entrée PIRMIDL - panne du circuit","P0706":"Contacteur/capteur de position du levier de vitesse - problème de performance/de limites","P0707":"Contacteur/capteur de position du levier de vitesse - valeur d'entrée trop basse","P0708":"Contacteur/capteur de position du levier de vitesse - valeur d'entrée trop haute","P0709":"Contacteur/capteur de position du levier de vitesse - circuit intermittent","P0710":"Sonde de température d'huile de transmission - panne du circuit","P0711":"Sonde de température d'huile de transmission - problème de performance/de limites","P0712":"Sonde de température d'huile de transmission - valeur d'entrée trop basse","P0713":"Sonde de température d'huile de transmission - valeur d'entrée trop haute","P0714":"Sonde de température d'huile de transmission - circuit intermittent","P0715":"Capteur de vitesse d'arbre de turbine - panne du circuit","P0716":"Capteur de vitesse d'arbre de turbine - problème de performance/de limites","P0717":"Capteur de vitesse d'arbre de turbine - aucun signal","P0718":"Capteur de vitesse d'arbre de turbine - circuit intermittent","P0719":"Convertisseur de couple/contacteur de freinage B - circuit trop bas","P0720":"Capteur de vitesse du véhicule - panne du circuit","P0721":"Capteur de vitesse du véhicule - problème de performance/de limites","P0722":"Capteur de vitesse du véhicule - aucun signal","P0723":"Capteur de vitesse du véhicule - circuit intermittent","P0724":"Convertisseur de couple/contacteur de freinage B - circuit trop haut","P0725":"Entrée du régime moteur - panne du circuit","P0726":"Entrée du régime moteur - problème de performance/de limites","P0727":"Entrée du régime moteur - aucun signal","P0728":"Entrée du régime moteur - circuit intermittent","P0729":"Vitesse 6 - rapport incorrect","P0730":"Rapport de démultiplication de la vitesse incorrect","P0731":"Vitesse 1 - ratio incorrect","P0732":"Vitesse 2 - ratio incorrect","P0733":"Vitesse 3 - ratio incorrect","P0734":"Vitesse 4 - ratio incorrect","P0735":"Vitesse 5 - ratio incorrect","P0736":"Marche arrière - rapport incorrect","P0737":"Régime moteur calculateur de transmission - circuit de sortie","P0738":"Régime moteur calculateur de transmission - circuit de sortie faible","P0739":"Référence de temps à haute résolution signal B - aucun pulse","P0740":"Electrovanne du convertisseur de couple - panne du circuit","P0741":"Electrovanne du convertisseur de couple - performance ou blocage","P0742":"Electrovanne du convertisseur de couple - blocage","P0743":"Electrovanne du convertisseur de couple - électrique","P0744":"Electrovanne du convertisseur de couple - circuit intermittent","P0745":"Electrovanne du convertisseur de couple - panne du circuit","P0746":"Electrovanne du convertisseur de couple - performance ou blocage","P0747":"Electrovanne du convertisseur de couple - blocage","P0748":"Electrovanne du convertisseur de couple - électrique","P0749":"Electrovanne du convertisseur de couple - circuit intermittent","P0750":"Electrovanne de changement de vitesse A - panne du circuit","P0751":"Electrovanne de changement de vitesse A - performance ou blocage","P0752":"Electrovanne de changement de vitesse A - blocage","P0753":"Electrovanne de changement de vitesse A - électrique","P0754":"Electrovanne de changement de vitesse A - circuit intermittent","P0755":"Electrovanne de changement de vitesse B - panne du circuit","P0756":"Electrovanne de changement de vitesse B - performance ou blocage","P0757":"Electrovanne de changement de vitesse B - blocage","P0758":"Electrovanne de changement de vitesse B - électrique","P0759":"Electrovanne de changement de vitesse B - circuit intermittent","P0760":"Electrovanne de changement de vitesse C - panne du circuit","P0761":"Electrovanne de changement de vitesse C - performance ou blocage","P0762":"Electrovanne de changement de vitesse C - blocage","P0763":"Electrovanne de changement de vitesse C - électrique","P0764":"Electrovanne de changement de vitesse C - circuit intermittent","P0765":"Electrovanne de changement de vitesse D - panne du circuit","P0766":"Electrovanne de changement de vitesse D - performance ou blocage","P0767":"Electrovanne de changement de vitesse D - blocage","P0768":"Electrovanne de changement de vitesse D - électrique","P0769":"Electrovanne de changement de vitesse D - circuit intermittent","P0770":"Electrovanne de changement de vitesse E - panne du circuit","P0771":"Electrovanne de changement de vitesse E - performance ou blocage","P0772":"Electrovanne de changement de vitesse E - blocage","P0773":"Electrovanne de changement de vitesse E - électrique","P0774":"Electrovanne de changement de vitesse E - circuit intermittent","P0775":"Electrovanne de commande de pression B - panne","P0776":"Electrovanne de commande de pression B - performance ou blocage","P0777":"Electrovanne de commande de pression B - blocage","P0778":"Electrovanne de commande de pression B - problème électrique","P0779":"Electrovanne de commande de pression B - circuit intermittent","P0780":"Sélection de la vitesse - problème de changement de vitesse","P0781":"Sélection de la vitesse, 1-2 - problème de changement de vitesse","P0782":"Sélection de la vitesse, 2-3 - problème de changement de vitesse","P0783":"Sélection de la vitesse, 3-4 - problème de changement de vitesse","P0784":"Sélection de la vitesse, 4-5 - problème de changement de vitesse","P0785":"Electrovanne de changement de vitesse A - panne du circuit","P0786":"Electrovanne de changement de vitesse A - problème de performance/de limites","P0787":"Electrovanne de changement de vitesse A - basse","P0788":"Electrovanne de changement de vitesse A - haute","P0789":"Electrovanne de changement de vitesse A - intermittent","P0790":"Commutateur sélection de mode de transmission - panne du circuit","P0791":"Capteur de vitesse de l'arbre intermédiaire de transmission - panne du cicuit","P0792":"Capteur de vitesse de l'arbre intermédiaire de transmission - problème de mesure/performance","P0793":"Capteur de vitesse de l'arbre intermédiaire de transmission - aucun signal","P0794":"Capteur de vitesse de l'arbre intermédiaire de transmission - panne intermittente du circuit","P0795":"Electrovanne de pression d'huile de transmission C - panne du circuit","P0796":"Electrovanne de pression d'huile de transmission C - performance ou blocage","P0797":"Electrovanne de pression d'huile de transmission C - blocage","P0798":"Electrovanne de pression d'huile de transmission C - problème électrique","P0799":"Electrovanne de pression d'huile de transmission C - panne intermittente du circuit","P0800":"Système de commande de la boîte de transfert, commande de lampe témoin d'affichage des défauts - panne","P0801":"Circuit de blocage de marche arrière - panne","P0802":"Commande de la transmission, commande de lampe témoin d'affichage des défauts - circuit ouvert","P0803":"Electrovanne de montée des vitesse 1-4 (saut de vitesses) - panne du circuit","P0804":"Lampe témoin de montée des vitesses 1-4 (saut de vitesses) - panne de circuit","P0805":"Capteur de la position de la pédale d'embrayage - panne du circuit","P0806":"Capteur de la position de la pédale d'embrayage - problème de mesure/performance","P0807":"Capteur de la position de la pédale d'embrayage - valeur d'entrée trop basse","P0808":"Capteur de la position de la pédale d'embrayage - valeur d'entrée trop élevée","P0809":"Capteur de la position de la pédale d'embrayage - panne intermittente du circuit","P0810":"Erreur de commande de position de la pédale d'embrayage","P0811":"Patinage excessif de l'embrayage","P0812":"Marche arrière - panne du circuit d'entrée","P0813":"Marche arrière - panne du circuit de sortie","P0814":"Affichage de position du levier de vitesses - panne du circuit","P0815":"Contacteur de montée des vitesses - panne du circuit","P0816":"Contacteur de descente des vitesses - panne du circuit","P0817":"Circuit de mise hors service du démarreur - panne","P0818":"Interrupteur de fil d'attaque - panne du circuit","P0819":"Corrélation entre le contacteur montée/descente des vitesses et la position du levier de vitesses","P0820":"Capteur de position X-Y du levier de vitesses - panne du circuit","P0821":"Capteur de position X du levier de vitesses - panne du circuit","P0822":"Capteur de position Y du levier de vitesses - panne du circuit","P0823":"Capteur de position X du levier de vitesses - circuit intermittent","P0824":"Capteur de position Y du levier de vitesses - circuit intermittent","P0825":"Commutateur à pression et tirage du levier de vitesses - panne du circuit","P0826":"Contacteur montée/descente des vitesses - circuit d'entrée","P0827":"Contacteur montée/descente des vitesses - circuit d'entrée trop faible","P0828":"Contacteur montée/descente des vitesses - circuit d'entrée trop fort","P0829":"Montée des vitesses 5-6","P0830":"Contacteur de position de la pédale d'embrayage A - panne du circuit","P0831":"Contacteur de position de la pédale d'embrayage A - valeur d'entrée trop basse","P0832":"Contacteur de position de la pédale d'embrayage A - valeur d'entrée trop haute","P0833":"Contacteur de position de la pédale d'embrayage B - panne du circuit","P0834":"Contacteur de position de la pédale d'embrayage B - valeur d'entrée trop basse","P0835":"Contacteur de position de la pédale d'embrayage B - valeur d'entrée trop haute","P0836":"Contacteur 4 roues motrices - panne du circuit","P0837":"Contacteur 4 roues motrices - problème de mesure/performance","P0838":"Contacteur 4 roues motrices - valeur d'entrée trop basse","P0839":"Contacteur 4 roues motrices - valeur d'entrée trop élevée","P0840":"Pressostat/capteur de pression d'huile de transmission A - panne du circuit","P0841":"Pressostat/capteur de pression d'huile de transmission A - problème de mesure/performance","P0842":"Pressostat/capteur de pression d'huile de transmission A - valeur d'entrée trop basse","P0843":"Pressostat/capteur de pression d'huile de transmission A - valeur d'entrée trop haute","P0844":"Pressostat/capteur de pression d'huile de transmission A - panne intermittente du circuit","P0845":"Pressostat/capteur de pression d'huile de transmission B - panne du circuit","P0846":"Pressostat/capteur de pression d'huile de transmission B - problème de mesure/performance","P0847":"Pressostat/capteur de pression d'huile de transmission B - valeur d'entrée trop basse","P0848":"Pressostat/capteur de pression d'huile de transmission B - valeur d'entrée trop haute","P0849":"Pressostat/capteur de pression d'huile de transmission B - panne intermittente du circuit","P0850":"Contacteur de position Neutre/parking - panne du circuit d'entrée","P0851":"Contacteur de position Neutre/Parking - circuit d'entrée trop faible","P0852":"Contacteur de position Neutre/Parking - circuit d'entrée trop fort","P0853":"Contacteur de position Drive - panne du circuit d'entrée","P0854":"Contacteur de position Drive - circuit d'entrée trop faible","P0855":"Contacteur de position Drive - circuit d'entrée trop fort","P0856":"Signal d'entrée anti-patinage - panne","P0857":"Signal d'entrée anti-patinage - prolème de mesure/performance","P0858":"Signal d'entrée anti-patinage - faible","P0859":"Signal d'entrée anti-patinage - haut","P0860":"Circuit de communication du boîtier électronique de changement de vitesses - panne","P0861":"Circuit de communication du boîtier électronique de changement de vitesses - valeur d'entrée trop basse","P0862":"Circuit de communication du boîtier électronique de changement de vitesses - valeur d'entrée trop haute","P0863":"Circuit de communication du calculateur de transmission - panne","P0864":"Circuit de communication du calculateur de transmission - problème de mesure/performance","P0865":"Circuit de communication du calculateur de transmission - valeur d'entrée trop basse","P0866":"Circuit de communication du calculateur de transmission - valeur d'entrée trop haute","P0867":"Capteur de pression d'huile de transmission","P0868":"Capteur de pression d'huile de transmission - faible","P0869":"Capteur de pression d'huile de transmission - haut","P0870":"Capteur de pression d'huile de transmission C - panne du circuit","P0871":"Capteur de pression d'huile de transmission C - plage de mesure/performance","P0872":"Capteur de pression d'huile de transmission C - circuit trop bas","P0873":"Capteur de pression d'huile de transmission C - circuit trop haut","P0874":"Capteur de pression d'huile de transmission C - panne intermittente du circuit","P0875":"Capteur de pression d'huile de transmission D - panne du circuit","P0876":"Capteur de pression d'huile de transmission D - plage de mesure/performance","P0877":"Capteur de pression d'huile de transmission D - circuit trop bas","P0878":"Capteur de pression d'huile de transmission D - circuit trop haut","P0879":"Capteur de pression d'huile de transmission D - panne intermittente du circuit","P0880":"Calculateur de transmission - panne du signal d'entrée de l'amimentation électrique","P0881":"Calculateur de transmission - plage de mesure/performance du signal d'entrée de l'amimentation électrique","P0882":"Calculateur de transmission - signal d'entrée de l'amimentation électrique trop faible","P0883":"Calculateur de transmission - signal d'entrée de l'amimentation électrique trop fort","P0884":"Calculateur de transmission - panne intermittente du signal d'entrée de l'amimentation électrique","P0885":"Relais d'alimentation électrique du calculateur de transmission - circuit de commande ouvert","P0886":"Relais d'alimentation électrique du calculateur de transmission - circuit de commande trop faible","P0887":"Relais d'alimentation électrique du calculateur de transmission - circuit de commande trop fort","P0888":"Relais d'alimentation électrique du calculateur de transmission - panne du circuit de sondage","P0889":"Relais d'alimentation électrique du calculateur de transmission - plage de mesure/performance du circuit de sondage","P0890":"Relais d'alimentation électrique du calculateur de transmission - circuit de sondage trop faible","P0891":"Relais d'alimentation électrique du calculateur de transmission - circuit de sondage trop fort","P0892":"Relais d'alimentation électrique du calculateur de transmission - panne intermittente du circuit de sondage","P0893":"Plusieurs vitesses engagées","P0894":"Patinage d'un composant de la transmission","P0895":"Durée de changement de vitesse trop courte","P0896":"Durée de changement de vitesse trop longue","P0897":"Huile de transmission dégradée","P0898":"Commande de transmission - commande de lampe témoin d'affichage des défauts - circuit trop bas","P0899":"Commande de transmission - commande de lampe témoin d'affichage des défauts - circuit trop haut","P0900":"Capteur d'embrayage - circuit ouvert","P0901":"Capteur d'embrayage - plage de msure/performance du circuit","P0902":"Capteur d'embrayage - circuit trop bas","P0903":"Capteur d'embrayage - circuit trop haut","P0904":"Circuit de sélection de coulisse de transmission - panne","P0905":"Circuit de sélection de coulisse de transmission - plage de mesure/performance","P0906":"Circuit de sélection de coulisse de transmission - basse","P0907":"Circuit de sélection de coulisse de transmission - haute","P0908":"Circuit de sélection de coulisse de transmission - panne intermittente du circuit","P0909":"Erreur de commande de sélection de coulisse de transmission","P0910":"Capteur de sélection de coulisse de transmission - circuit ouvert","P0911":"Capteur de sélection de coulisse de transmission - plage de mesure/performance","P0912":"Capteur de sélection de coulisse de transmission - circuit trop bas","P0913":"Capteur de sélection de coulisse de transmission - circuit trop haut","P0914":"Circuit de position de changement de vitesses - panne","P0915":"Circuit de position de changement de vitesses - plage de mesure/performance","P0916":"Circuit de position de changement de vitesses - basse","P0917":"Circuit de position de changement de vitesses - haute","P0918":"Circuit de position de changement de vitesses - panne intermittente","P0919":"Commande de position de changement de vitesses - erreur","P0920":"Capteur de changement de vitesses vers l'avant - circuit ouvert","P0921":"Capteur de changement de vitesses vers l'avant - plage de mesure/performance","P0922":"Capteur de changement de vitesses vers l'avant - circuit trop bas","P0923":"Capteur de changement de vitesses vers l'avant - circuit trop haut","P0924":"Capteur de changement de vitesses vers l'arrière - circuit ouvert","P0925":"Capteur de changement de vitesses vers l'arrière - plage de mesure/performance","P0926":"Capteur de changement de vitesses vers l'arrière - circuit trop bas","P0927":"Capteur de changement de vitesses vers l'arrière - circuit trop haut","P0928":"Electrovanne de blocage de changement de vitesse - circuit ouvert","P0929":"Electrovanne de blocage de changement de vitesse - plage de mesure/performance du circuit","P0930":"Electrovanne de blocage de changement de vitesse - circuit trop bas","P0931":"Electrovanne de blocage de changement de vitesse - circuit trop haut","P0932":"Capteur de pression hydraulique - panne du circuit","P0933":"Capteur de pression hydraulique - plage de mesure/performance du circuit","P0934":"Capteur de pression hydraulique - signal d'entrée du circuit trop bas","P0935":"Capteur de pression hydraulique - signal d'entrée du circuit trop haut","P0936":"Capteur de pression hydraulique - circuit intermittent","P0937":"Sonde de température du fluide hydraulique - panne du circuit","P0938":"Sonde de température du fluide hydraulique - plage de mesure/performance","P0939":"Sonde de température du fluide hydraulique - signal d'entrée du circuit trop bas","P0940":"Sonde de température du fluide hydraulique - signal d'entrée du circuit trop haut","P0941":"Sonde de température du fluide hydraulique - circuit intermittent","P0942":"Module de pression hydraulique","P0943":"Module de pression hydraulique - durée de cycle trop courte","P0944":"Module de pression hydraulique - perte de pression","P0945":"Relais de la pompe hydraulique - circuit ouvert","P0946":"Relais de la pompe hydraulique - plage de mesure/performance du circuit","P0947":"Relais de la pompe hydraulique - circuit trop bas","P0948":"Relais de la pompe hydraulique - circuit trop haut","P0949":"Boite de vitesse à passage automatique - apprentissage adaptatif non effectué","P0950":"Circuit de commande boite de vitesses à passage automatique","P0951":"Circuit de commande boite de vitesses à passage automatique - plage de mesure/performance","P0952":"Circuit de commande boite de vitesses à passage automatique - basse","P0953":"Circuit de commande boite de vitesses à passage automatique - haute","P0954":"Circuit de commande boite de vitesses à passage automatique - panne intermittente du circuit","P0955":"Circuit de programme boite de vitesses à passage automatique - panne","P0956":"Circuit de programme boite de vitesses à passage automatique - plage de mesure/performance","P0957":"Circuit de programme boite de vitesses à passage automatique - basse","P0958":"Circuit de programme boite de vitesses à passage automatique - haute","P0959":"Circuit de programme boite de vitesses à passage automatique - panne intermittente du circuit","P0960":"Electrovanne de régulation de pression A - circuit de commande ouvert","P0961":"Electrovanne de régulation de pression A - plage de mesure/performance du circuit de commande","P0962":"Electrovanne de régulation de pression A - circuit de commande trop faible","P0963":"Electrovanne de régulation de pression A - circuit de commande trop fort","P0964":"Electrovanne de régulation de pression B - circuit de commande ouvert","P0965":"Electrovanne de régulation de pression B - plage de mesure/performance du circuit de commande","P0966":"Electrovanne de régulation de pression B - circuit de commande trop faible","P0967":"Electrovanne de régulation de pression B - circuit de commande trop fort","P0968":"Electrovanne de régulation de pression C - circuit de commande ouvert","P0969":"Electrovanne de régulation de pression C - plage de mesure/performance du circuit de commande","P0970":"Electrovanne de régulation de pression C - circuit de commande trop faible","P0971":"Electrovanne de régulation de pression C - circuit de commande trop fort","P0972":"Electrovanne de changement de vitesse A - plage de mesure/performance du circuit de commande","P0973":"Electrovanne de changement de vitesse A - circuit de commande trop faible","P0974":"Electrovanne de changement de vitesse A - circuit de commande trop fort","P0975":"Electrovanne de changement de vitesse B - plage de mesure/performance du circuit de commande","P0976":"Electrovanne de changement de vitesse B - circuit de commande trop faible","P0977":"Electrovanne de changement de vitesse B - circuit de commande trop fort","P0978":"Electrovanne de changement de vitesse C - plage de mesure/performance du circuit de commande","P0979":"Electrovanne de changement de vitesse C - circuit de commande trop faible","P0980":"Electrovanne de changement de vitesse C - circuit de commande trop fort","P0981":"Electrovanne de changement de vitesse D - plage de mesure/performance du circuit de commande","P0982":"Electrovanne de changement de vitesse D - circuit de commande trop faible","P0983":"Electrovanne de changement de vitesse D - circuit de commande trop fort","P0984":"Electrovanne de changement de vitesse E - plage de mesure/performance du circuit de commande","P0985":"Electrovanne de changement de vitesse E - circuit de commande trop faible","P0986":"Electrovanne de changement de vitesse E - circuit de commande trop fort","P0987":"Capteur de pression d'huile de transmission E - panne du circuit","P0988":"Capteur de pression d'huile de transmission E - plage de mesure/performance du circuit","P0989":"Capteur de pression d'huile de transmission E - circuit trop bas","P0990":"Capteur de pression d'huile de transmission E - circuit trop haut","P0991":"Capteur de pression d'huile de transmission E - circuit intermittent","P0992":"Capteur de pression d'huile de transmission F - panne du circuit","P0993":"Capteur de pression d'huile de transmission F - plage de mesure/performance du circuit","P0994":"Capteur de pression d'huile de transmission F - circuit trop bas","P0995":"Capteur de pression d'huile de transmission F - circuit trop haut","P0996":"Capteur de pression d'huile de transmission F - circuit intermittent","P0997":"Electrovanne de changement de vitesse F - plage de mesure/performance du circuit de commande","P0998":"Electrovanne de changement de vitesse F - circuit de commande trop faible","P0999":"Electrovanne de changement de vitesse F - circuit de commande trop fort"};
function traduireDTC(txt){
 const trouves = txt.match(/P0[0-9A-F]{3}/gi);
 if(!trouves) return txt;
 const vus = new Set(); const lignes = [txt];
 trouves.forEach(c => {
  const code = c.toUpperCase();
  if(vus.has(code)) return;
  vus.add(code);
  const desc = CODES_FR[code];
  lignes.push(code + ' : ' + (desc || '(code non repertorie)'));
 });
 return lignes.join('\n');
}
function add(t,c){log.textContent+='\n'+t;log.scrollTop=log.scrollHeight;}
async function cmd(c){
 if(!c)return;
 add('> '+c);
 try{
  const r=await fetch('/cmd?c='+encodeURIComponent(c));
  add(traduireDTC(await r.text()));
 }catch(e){add('!! liaison perdue');}
}
function doClear(){
 if(confirm('Effacer tous les defauts et le voyant ?\nIrreversible : note les codes avant.'))
  cmd('CLEAR!');
}
</script></body></html>)HTML";

void handleRoot(){ server.send_P(200, "text/html", PAGE); }

void handleCmd(){
  String c = server.arg("c");
  c.trim();
  StreamString ss;
  OUT = &ss;                 /* redirige la sortie vers la reponse http */
  execCmd(c, true);
  OUT = &Serial;
  server.send(200, "text/plain", (const String&)ss);
}

/* ============================================================
 * Setup / boucle
 * ============================================================ */
void setup(){
  pinMode(PIN_CS, OUTPUT); csH();
  pinMode(PIN_INT, INPUT_PULLUP);
  Serial.begin(115200);
  delay(300);   /* laisse le temps a l'USB CDC/moniteur de s'attacher */

  Serial.println("# obd_can_bridge v2.0 - Eric Perret F1OCM");
  Serial.println("# Nemo 1.3 HDi - Marelli MJD 8F3.F6");

  SPI.begin();
  /* 2 MHz : large marge (MCP2515 max 10 MHz) */
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));

  mcpOk = mcpInit();
  if(mcpOk) Serial.println("# MCP2515 pret");
  else      Serial.println("# MCP2515 injoignable - retry auto...");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, mcpOk ? LOW : HIGH);  /* fixe si OK */

  /* ---- WiFi point d'acces + serveur web ---- */
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_IP, AP_MASK);       /* DHCP integre */
  WiFi.softAP(AP_SSID, AP_PASS);
  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
  server.begin();
  Serial.print("# WiFi AP "); Serial.print(AP_SSID);
  Serial.print(" (mdp "); Serial.print(AP_PASS);
  Serial.print(") -> http://"); Serial.println(WiFi.softAPIP());

  cmdHelp();    /* rappel des commandes au boot */
}

String line;

void loop(){
  server.handleClient();

  /* retry auto tant que le MCP2515 n'a pas accroche :
     tentative toutes les 300ms, un point par essai */
  static uint32_t tRetry = 0;
  static uint8_t  nDots  = 0;
  if(!mcpOk && millis() - tRetry >= 300){
    tRetry = millis();
    mcpOk = mcpInit();
    if(mcpOk){
      digitalWrite(LED_BUILTIN, LOW);
      if(nDots){ Serial.println(); nDots = 0; }
      Serial.println("# MCP2515 accroche ! LED fixe, tape HELP? pour les commandes");
    }else{
      if(!nDots) Serial.print("# ");
      Serial.print('.');
      if(++nDots >= 40){ Serial.println(); nDots = 0; }
    }
  }

  /* mode sniffer : deverser toute trame recue (USB)            */
  if(snifMode){
    uint32_t id; bool ext; uint8_t d[8];
    uint8_t n = canRecvAny(&id, &ext, d);
    if(n && snifFilter(id, n, d)){
      Serial.print("RX ");
      Serial.print(id, HEX);
      if(ext) Serial.print("x");          /* marqueur 29 bits  */
      Serial.print(' ');
      Serial.print(n);
      for(uint8_t i=0;i<n;i++){
        Serial.print(' ');
        if(d[i] < 16) Serial.print('0');
        Serial.print(d[i], HEX);
      }
      Serial.println();
    }
  }

  /* lecture commande serie                                     */
  while(Serial.available()){
    char c = Serial.read();
    if(c == '\r') continue;
    if(c != '\n'){ line += c; if(line.length() > 40) line = ""; continue; }
    execCmd(line, false);
    line = "";
  }
}
