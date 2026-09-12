#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <StreamString.h>
#include <string.h>
#include <time.h>
#include <FS.h>
#include <LittleFS.h>

#define FW_VER "8.4"

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

/* ---------- Adresse ECU connue (decouverte en Phase 1) ------ */
#define ECU_ADDR    0x10
#define BROADCAST_ID 0x18DB33F1UL                                          // fonctionnel, prouve sur ce vehicule
#define RESP_ID     (0x18DA0000UL | (0xF1UL << 8) | (uint32_t)ECU_ADDR)   // 0x18DAF110
#define PID_TIMEOUT_MS 80  // N_Bs=75ms (ISO 15765-4 tableau 6) + marge

// PID de test isole (connu/attendu supporte) - ajuster ici si besoin (ex: 0x4C)
#define TEST_PID 0x0C

/* ---------- Log continu : PID surveilles, ordre = synchronisation ---- */
// Pedale et vitesse en premier et adjacents (paire critique de detection),
// regime juste apres (verification embrayage/rapport), le reste ensuite.
const uint8_t MONITOR_PIDS[] = {0x49, 0x0D, 0x0C, 0x04, 0x10, 0x0B, 0x23, 0x2C, 0x4A};
#define NB_MONITOR (sizeof(MONITOR_PIDS)/sizeof(MONITOR_PIDS[0]))
#define IDX_PEDALE 0   // position de 0x49 dans MONITOR_PIDS
#define IDX_VITESSE 1  // position de 0x0D dans MONITOR_PIDS
#define SEUIL_PEDALE_PCT 75.0f
#define DUREE_RAFALE_MS 15000UL
#define DUREE_MANCHE_NORMALE_MS 1000UL

/* ---------- Table de decodage Mode 01 (formules SAE J1979, Wikipedia) */
// offset = decalage en octets apres SID+PID (donc data[3+offset]...)
// nbytes = 1, 2 ou 4 ; signed_ = interpretation complement a 2
struct PidDef { uint8_t pid; uint8_t offset; uint8_t nbytes; bool signed_; float mul; float add; const char* unit; const char* name; };
const PidDef PIDTABLE[] = {
  {0x04,0,1,false,100.0f/255.0f,0,   "%",     "Charge moteur"},
  {0x05,0,1,false,1.0f,-40,          "C",     "Temp liquide refroidissement"},
  {0x06,0,1,false,100.0f/128.0f,-100,"%",     "STFT Banc1"},
  {0x07,0,1,false,100.0f/128.0f,-100,"%",     "LTFT Banc1"},
  {0x08,0,1,false,100.0f/128.0f,-100,"%",     "STFT Banc2"},
  {0x09,0,1,false,100.0f/128.0f,-100,"%",     "LTFT Banc2"},
  {0x0A,0,1,false,3.0f,0,            "kPa",   "Pression carburant"},
  {0x0B,0,1,false,1.0f,0,            "kPa",   "Pression admission"},
  {0x0C,0,2,false,0.25f,0,           "rpm",   "Regime moteur"},
  {0x0D,0,1,false,1.0f,0,            "km/h",  "Vitesse vehicule"},
  {0x0E,0,1,false,0.5f,-64,          "deg",   "Avance allumage"},
  {0x0F,0,1,false,1.0f,-40,          "C",     "Temp air admission"},
  {0x10,0,2,false,0.01f,0,           "g/s",   "Debit MAF"},
  {0x11,0,1,false,100.0f/255.0f,0,   "%",     "Position papillon"},
  {0x14,0,1,false,1.0f/200.0f,0,     "V",     "O2 capteur1 tension"},
  {0x14,1,1,false,100.0f/128.0f,-100,"%",     "O2 capteur1 STFT"},
  {0x15,0,1,false,1.0f/200.0f,0,     "V",     "O2 capteur2 tension"},
  {0x15,1,1,false,100.0f/128.0f,-100,"%",     "O2 capteur2 STFT"},
  {0x16,0,1,false,1.0f/200.0f,0,     "V",     "O2 capteur3 tension"},
  {0x16,1,1,false,100.0f/128.0f,-100,"%",     "O2 capteur3 STFT"},
  {0x17,0,1,false,1.0f/200.0f,0,     "V",     "O2 capteur4 tension"},
  {0x17,1,1,false,100.0f/128.0f,-100,"%",     "O2 capteur4 STFT"},
  {0x18,0,1,false,1.0f/200.0f,0,     "V",     "O2 capteur5 tension"},
  {0x18,1,1,false,100.0f/128.0f,-100,"%",     "O2 capteur5 STFT"},
  {0x19,0,1,false,1.0f/200.0f,0,     "V",     "O2 capteur6 tension"},
  {0x19,1,1,false,100.0f/128.0f,-100,"%",     "O2 capteur6 STFT"},
  {0x1A,0,1,false,1.0f/200.0f,0,     "V",     "O2 capteur7 tension"},
  {0x1A,1,1,false,100.0f/128.0f,-100,"%",     "O2 capteur7 STFT"},
  {0x1B,0,1,false,1.0f/200.0f,0,     "V",     "O2 capteur8 tension"},
  {0x1B,1,1,false,100.0f/128.0f,-100,"%",     "O2 capteur8 STFT"},
  {0x1F,0,2,false,1.0f,0,            "s",     "Temps depuis demarrage"},
  {0x21,0,2,false,1.0f,0,            "km",    "Distance MIL allume"},
  {0x22,0,2,false,0.079f,0,          "kPa",   "Pression rampe (rel)"},
  {0x23,0,2,false,10.0f,0,           "kPa",   "Pression rampe (abs, diesel)"},
  {0x24,0,2,false,1.0f/32768.0f,0,   "ratio", "Lambda O2-1"},
  {0x24,2,2,false,1.0f/8192.0f,0,    "V",     "O2-1 tension large bande"},
  {0x25,0,2,false,1.0f/32768.0f,0,   "ratio", "Lambda O2-2"},
  {0x25,2,2,false,1.0f/8192.0f,0,    "V",     "O2-2 tension large bande"},
  {0x26,0,2,false,1.0f/32768.0f,0,   "ratio", "Lambda O2-3"},
  {0x26,2,2,false,1.0f/8192.0f,0,    "V",     "O2-3 tension large bande"},
  {0x27,0,2,false,1.0f/32768.0f,0,   "ratio", "Lambda O2-4"},
  {0x27,2,2,false,1.0f/8192.0f,0,    "V",     "O2-4 tension large bande"},
  {0x28,0,2,false,1.0f/32768.0f,0,   "ratio", "Lambda O2-5"},
  {0x28,2,2,false,1.0f/8192.0f,0,    "V",     "O2-5 tension large bande"},
  {0x29,0,2,false,1.0f/32768.0f,0,   "ratio", "Lambda O2-6"},
  {0x29,2,2,false,1.0f/8192.0f,0,    "V",     "O2-6 tension large bande"},
  {0x2A,0,2,false,1.0f/32768.0f,0,   "ratio", "Lambda O2-7"},
  {0x2A,2,2,false,1.0f/8192.0f,0,    "V",     "O2-7 tension large bande"},
  {0x2B,0,2,false,1.0f/32768.0f,0,   "ratio", "Lambda O2-8"},
  {0x2B,2,2,false,1.0f/8192.0f,0,    "V",     "O2-8 tension large bande"},
  {0x2C,0,1,false,100.0f/255.0f,0,   "%",     "EGR commande"},
  {0x2D,0,1,false,100.0f/128.0f,-100,"%",     "Erreur EGR"},
  {0x2E,0,1,false,100.0f/255.0f,0,   "%",     "Purge evap commandee"},
  {0x2F,0,1,false,100.0f/255.0f,0,   "%",     "Niveau reservoir"},
  {0x31,0,2,false,1.0f,0,            "km",    "Distance depuis effacement codes"},
  {0x32,0,2,true, 0.25f,0,           "Pa",    "Pression vapeur evap"},
  {0x33,0,1,false,1.0f,0,            "kPa",   "Pression barometrique"},
  {0x34,0,2,false,1.0f/32768.0f,0,   "ratio", "Lambda O2-1 (courant)"},
  {0x34,2,2,true, 1.0f/256.0f,-128,  "mA",    "O2-1 courant large bande"},
  {0x35,0,2,false,1.0f/32768.0f,0,   "ratio", "Lambda O2-2 (courant)"},
  {0x35,2,2,true, 1.0f/256.0f,-128,  "mA",    "O2-2 courant large bande"},
  {0x36,0,2,false,1.0f/32768.0f,0,   "ratio", "Lambda O2-3 (courant)"},
  {0x36,2,2,true, 1.0f/256.0f,-128,  "mA",    "O2-3 courant large bande"},
  {0x37,0,2,false,1.0f/32768.0f,0,   "ratio", "Lambda O2-4 (courant)"},
  {0x37,2,2,true, 1.0f/256.0f,-128,  "mA",    "O2-4 courant large bande"},
  {0x38,0,2,false,1.0f/32768.0f,0,   "ratio", "Lambda O2-5 (courant)"},
  {0x38,2,2,true, 1.0f/256.0f,-128,  "mA",    "O2-5 courant large bande"},
  {0x39,0,2,false,1.0f/32768.0f,0,   "ratio", "Lambda O2-6 (courant)"},
  {0x39,2,2,true, 1.0f/256.0f,-128,  "mA",    "O2-6 courant large bande"},
  {0x3A,0,2,false,1.0f/32768.0f,0,   "ratio", "Lambda O2-7 (courant)"},
  {0x3A,2,2,true, 1.0f/256.0f,-128,  "mA",    "O2-7 courant large bande"},
  {0x3B,0,2,false,1.0f/32768.0f,0,   "ratio", "Lambda O2-8 (courant)"},
  {0x3B,2,2,true, 1.0f/256.0f,-128,  "mA",    "O2-8 courant large bande"},
  {0x3C,0,2,false,0.1f,-40,          "C",     "Temp catalyseur B1S1"},
  {0x3D,0,2,false,0.1f,-40,          "C",     "Temp catalyseur B2S1"},
  {0x3E,0,2,false,0.1f,-40,          "C",     "Temp catalyseur B1S2"},
  {0x3F,0,2,false,0.1f,-40,          "C",     "Temp catalyseur B2S2"},
  {0x42,0,2,false,0.001f,0,          "V",     "Tension calculateur"},
  {0x43,0,2,false,100.0f/255.0f,0,   "%",     "Charge absolue"},
  {0x44,0,2,false,2.0f/65536.0f,0,   "ratio", "Ratio air/carburant commande"},
  {0x45,0,1,false,100.0f/255.0f,0,   "%",     "Position papillon relative"},
  {0x46,0,1,false,1.0f,-40,          "C",     "Temp air ambiant"},
  {0x47,0,1,false,100.0f/255.0f,0,   "%",     "Position papillon B"},
  {0x48,0,1,false,100.0f/255.0f,0,   "%",     "Position papillon C"},
  {0x49,0,1,false,100.0f/255.0f,0,   "%",     "Position pedale D"},
  {0x4A,0,1,false,100.0f/255.0f,0,   "%",     "Position pedale E"},
  {0x4B,0,1,false,100.0f/255.0f,0,   "%",     "Position pedale F"},
  {0x4C,0,1,false,100.0f/255.0f,0,   "%",     "Actionneur papillon commande"},
  {0x4D,0,2,false,1.0f,0,            "min",   "Temps MIL allume"},
  {0x4E,0,2,false,1.0f,0,            "min",   "Temps depuis effacement codes"},
  {0x52,0,1,false,100.0f/255.0f,0,   "%",     "Ethanol %"},
  {0x53,0,2,false,1.0f/200.0f,0,     "kPa",   "Pression evap absolue"},
  {0x54,0,2,true, 1.0f,0,            "Pa",    "Pression evap (large)"},
  {0x55,0,1,false,100.0f/128.0f,-100,"%",     "STFT O2 secondaire B1"},
  {0x55,1,1,false,100.0f/128.0f,-100,"%",     "STFT O2 secondaire B3"},
  {0x56,0,1,false,100.0f/128.0f,-100,"%",     "LTFT O2 secondaire B1"},
  {0x56,1,1,false,100.0f/128.0f,-100,"%",     "LTFT O2 secondaire B3"},
  {0x57,0,1,false,100.0f/128.0f,-100,"%",     "STFT O2 secondaire B2"},
  {0x57,1,1,false,100.0f/128.0f,-100,"%",     "STFT O2 secondaire B4"},
  {0x58,0,1,false,100.0f/128.0f,-100,"%",     "LTFT O2 secondaire B2"},
  {0x58,1,1,false,100.0f/128.0f,-100,"%",     "LTFT O2 secondaire B4"},
  {0x59,0,2,false,10.0f,0,           "kPa",   "Pression rampe carburant abs"},
  {0x5A,0,1,false,100.0f/255.0f,0,   "%",     "Position pedale relative"},
  {0x5B,0,1,false,100.0f/255.0f,0,   "%",     "Charge batterie hybride restante"},
  {0x5C,0,1,false,1.0f,-40,          "C",     "Temp huile moteur"},
  {0x5D,0,2,false,1.0f/128.0f,-210,  "deg",   "Calage injection carburant"},
  {0x5E,0,2,false,0.05f,0,           "L/h",   "Debit carburant moteur"},
  {0x63,0,2,false,1.0f,0,            "Nm",    "Couple moteur reference"},
  {0x64,0,1,false,1.0f,-125,         "%",     "Couple % au ralenti"},
  {0x64,1,1,false,1.0f,-125,         "%",     "Couple % point1"},
  {0x64,2,1,false,1.0f,-125,         "%",     "Couple % point2"},
  {0x64,3,1,false,1.0f,-125,         "%",     "Couple % point3"},
  {0x64,4,1,false,1.0f,-125,         "%",     "Couple % point4"},
  {0x66,1,2,false,1.0f/32.0f,0,      "g/s",   "Debit MAF capteur A"},
  {0x66,3,2,false,1.0f/32.0f,0,      "g/s",   "Debit MAF capteur B"},
  {0x67,1,1,false,1.0f,-40,          "C",     "Temp liquide refroid. capteur1"},
  {0x67,2,1,false,1.0f,-40,          "C",     "Temp liquide refroid. capteur2"},
  {0x68,1,1,false,1.0f,-40,          "C",     "Temp air admission B1S1"},
  {0x68,2,1,false,1.0f,-40,          "C",     "Temp air admission B1S2"},
  {0x68,3,1,false,1.0f,-40,          "C",     "Temp air admission B1S3"},
  {0x68,4,1,false,1.0f,-40,          "C",     "Temp air admission B2S1"},
  {0x68,5,1,false,1.0f,-40,          "C",     "Temp air admission B2S2"},
  {0x68,6,1,false,1.0f,-40,          "C",     "Temp air admission B2S3"},
  {0x69,1,1,false,100.0f/255.0f,0,   "%",     "EGR A commande"},
  {0x69,2,1,false,100.0f/255.0f,0,   "%",     "EGR A reelle"},
  {0x69,3,1,false,100.0f/128.0f,-100,"%",     "EGR A erreur"},
  {0x69,4,1,false,100.0f/255.0f,0,   "%",     "EGR B commande"},
  {0x69,5,1,false,100.0f/255.0f,0,   "%",     "EGR B reelle"},
  {0x69,6,1,false,100.0f/128.0f,-100,"%",     "EGR B erreur"},
  {0x6C,1,1,false,100.0f/255.0f,0,   "%",     "Actionneur papillon A commande"},
  {0x6C,2,1,false,100.0f/255.0f,0,   "%",     "Position papillon A relative"},
  {0x6C,3,1,false,100.0f/255.0f,0,   "%",     "Actionneur papillon B commande"},
  {0x6C,4,1,false,100.0f/255.0f,0,   "%",     "Position papillon B relative"},
  {0x6D,1,2,false,10.0f,0,           "kPa",   "Rampe A commandee"},
  {0x6D,3,2,false,10.0f,0,           "kPa",   "Rampe A pression"},
  {0x6D,5,1,false,1.0f,-40,          "C",     "Rampe A temperature"},
  {0x6D,6,2,false,10.0f,0,           "kPa",   "Rampe B commandee"},
  {0x6D,8,2,false,10.0f,0,           "kPa",   "Rampe B pression"},
  {0x6D,10,1,false,1.0f,-40,         "C",     "Rampe B temperature"},
  {0x70,1,2,false,1.0f/32.0f,0,      "kPa",   "Suralimentation A commandee"},
  {0x70,3,2,false,1.0f/32.0f,0,      "kPa",   "Suralimentation A mesuree"},
  {0x70,5,2,false,1.0f/32.0f,0,      "kPa",   "Suralimentation B commandee"},
  {0x70,7,2,false,1.0f/32.0f,0,      "kPa",   "Suralimentation B mesuree"},
  {0x72,1,1,false,100.0f/255.0f,0,   "%",     "Wastegate A commandee"},
  {0x72,2,1,false,100.0f/255.0f,0,   "%",     "Wastegate A position"},
  {0x72,3,1,false,100.0f/255.0f,0,   "%",     "Wastegate B commandee"},
  {0x72,4,1,false,100.0f/255.0f,0,   "%",     "Wastegate B position"},
  {0x7F,1,4,false,1.0f,0,            "s",     "Temps fonctionnement total moteur"},
  {0x7F,5,4,false,1.0f,0,            "s",     "Temps ralenti total"},
  {0x7F,9,4,false,1.0f,0,            "s",     "Temps PTO actif total"},
  {0x84,0,1,false,1.0f,-40,          "C",     "Temp surface collecteur"},
  {0x8D,0,1,false,100.0f/255.0f,0,   "%",     "Position papillon G"},
  {0x8E,0,1,false,1.0f,-125,         "%",     "Couple friction moteur"},
  {0x9A,2,2,false,1.0f/64.0f,0,      "V",     "Tension batterie hybride"},
  {0x9A,4,2,true, 0.1f,0,            "A",     "Courant batterie hybride"},
  {0x9D,0,2,false,0.02f,0,           "g/s",   "Debit carburant moteur (alt)"},
  {0x9D,2,2,false,0.02f,0,           "g/s",   "Debit carburant vehicule"},
  {0x9E,0,2,false,0.2f,0,            "kg/h",  "Debit gaz echappement"},
  {0xA2,0,2,false,1.0f/32.0f,0,      "mg/coup","Debit carburant cylindre"},
  {0xA6,0,4,false,0.1f,0,            "km",    "Kilometrage (odometre)"},
  {0xAA,0,1,false,1.0f,0,            "km/h",  "Vitesse max limitee"},
  {0xB2,0,1,false,100.0f/255.0f,0,   "%",     "Etat sante batterie"},
  {0xD2,1,1,false,100.0f/255.0f,0,   "%",     "Etat energie certifiee batterie"},
  {0xD2,2,1,false,100.0f/255.0f,0,   "%",     "Etat autonomie certifiee batterie"},
  {0xD3,0,4,false,0.1f,0,            "km",    "Kilometrage moteur"}
};
#define NPID (sizeof(PIDTABLE)/sizeof(PIDTABLE[0]))

String decodePid(uint8_t pid, uint8_t* data, uint8_t len) {
  String out = "";
  bool found = false;
  for (uint16_t i = 0; i < NPID; i++) {
    if (PIDTABLE[i].pid != pid) continue;
    uint8_t off = 3 + PIDTABLE[i].offset;
    if ((uint16_t)(off + PIDTABLE[i].nbytes - 1) >= len) continue;

    long raw;
    if (PIDTABLE[i].nbytes == 1) {
      raw = PIDTABLE[i].signed_ ? (long)(int8_t)data[off] : (long)data[off];
    } else if (PIDTABLE[i].nbytes == 2) {
      uint16_t u = ((uint16_t)data[off] << 8) | data[off + 1];
      raw = PIDTABLE[i].signed_ ? (long)(int16_t)u : (long)u;
    } else {
      uint32_t u = ((uint32_t)data[off] << 24) | ((uint32_t)data[off + 1] << 16) |
                   ((uint32_t)data[off + 2] << 8) | data[off + 3];
      raw = (long)u;
    }

    float val = PIDTABLE[i].mul * raw + PIDTABLE[i].add;
    if (found) out += "; ";
    out += String(val, 2) + " " + PIDTABLE[i].unit + " (" + PIDTABLE[i].name + ")";
    found = true;
  }
  return found ? out : "Brut";
}

// Valeur numerique brute (premiere correspondance) - utilisee pour les
// jauges et la detection perte de puissance, distincte de decodePid()
// qui renvoie du texte formate.
bool getNumeric(uint8_t pid, uint8_t* data, uint8_t len, float &out) {
  for (uint16_t i = 0; i < NPID; i++) {
    if (PIDTABLE[i].pid != pid) continue;
    uint8_t off = 3 + PIDTABLE[i].offset;
    if ((uint16_t)(off + PIDTABLE[i].nbytes - 1) >= len) continue;

    long raw;
    if (PIDTABLE[i].nbytes == 1) {
      raw = PIDTABLE[i].signed_ ? (long)(int8_t)data[off] : (long)data[off];
    } else if (PIDTABLE[i].nbytes == 2) {
      uint16_t u = ((uint16_t)data[off] << 8) | data[off + 1];
      raw = PIDTABLE[i].signed_ ? (long)(int16_t)u : (long)u;
    } else {
      uint32_t u = ((uint32_t)data[off] << 24) | ((uint32_t)data[off + 1] << 16) |
                   ((uint32_t)data[off + 2] << 8) | data[off + 3];
      raw = (long)u;
    }
    out = PIDTABLE[i].mul * raw + PIDTABLE[i].add;
    return true;
  }
  return false;
}

/* ---------- Fonction dediee : test d'un seul PID connu ------- */
/* ---------- Etat du log continu ------------------------------ */
enum LogMode { LOG_IDLE, LOG_NORMAL, LOG_RAFALE };
LogMode  logMode = LOG_IDLE;
uint32_t logT0 = 0;            // debut du log (horodatage relatif)
uint32_t rafaleFinMs = 0;
uint32_t mancheDebutMs = 0;
uint8_t  pidIndex = 0;

float   pedalePrec = -1, vitessePrec = -1;
bool    derniereOk[NB_MONITOR];
String  derniereVal[NB_MONITOR];   // texte decode, pour les jauges
float   derniereNum[NB_MONITOR];   // valeur numerique, pour les jauges
File    logFile;
String  logBuffer;             // tampon RAM, ecrit sur flash par lots (pas a chaque ligne)
uint32_t derniereEcritureMs = 0;
#define FLUSH_INTERVAL_MS 5000UL   // ecriture flash au plus toutes les 5 s
#define RAFALE_PAUSE_MS 100UL      // pause entre manches en rafale (calmer le debit)

bool testerUnPid(uint8_t pid, uint32_t &respIdOut, uint8_t* dataOut, uint8_t &lenOut, uint8_t &bufOut) {
  uint32_t d_id; uint8_t d_data[8]; uint8_t d_len; uint8_t d_buf;
  uint16_t drainCount = 0;
  while (mcp_recv(d_id, d_data, d_len, d_buf)) {   // vider les buffers avant l'essai
    drainCount++;
    if (drainCount > 100) {
      Serial.println("[DBG] !! vidage buffers > 100 iterations, sortie forcee (bus flottant/bruit ?)");
      break;
    }
    yield();
  }

  const uint8_t req_data[8] = {0x02, 0x01, pid, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC};
  mcp_send_ext(BROADCAST_ID, req_data, 8);

  unsigned long t_start = millis();
  bool got = false;
  while (millis() - t_start < PID_TIMEOUT_MS) {
    uint32_t rx_id; uint8_t rx_data[8]; uint8_t rx_len; uint8_t rx_buf;
    if (!got && mcp_recv(rx_id, rx_data, rx_len, rx_buf)) {
      if (rx_id == RESP_ID) {
        respIdOut = rx_id;
        lenOut = rx_len;
        memcpy(dataOut, rx_data, rx_len);
        bufOut = rx_buf;   // RXB0 (0) ou RXB1 (1) - pour verifier l'hypothese d'alternance
        got = true;
      }
    }
    server.handleClient();   // sert le web pendant l'attente, sinon STOP/download restent bloques ~720ms/manche
    yield();
  }
  // Cycle de duree fixe (PID_TIMEOUT_MS) qu'il y ait reponse rapide ou non,
  // pour laisser un delai de repos constant avant la prochaine requete
  // (hypothese : requete suivante trop rapprochee = ignoree par l'ECU).
  return got;
}

/* ---------- HTML Principal ---------------------------------- */
const char* HTML_INDEX = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>NEMO OBD - FW 8.4</title>
  <style>
    body { font-family: Arial, sans-serif; text-align: center; background: #000; color: #fff; padding: 20px; }
    h1, h2 { color: #fff; }
    button { padding: 15px 40px; font-size: 24px; font-weight: bold; cursor: pointer; border-radius: 8px; border: none; background: #007bff; color: white; transition: 0.3s; }
    button:disabled { background: #444; color: #888; cursor: not-allowed; }
    table { margin: 30px auto; border-collapse: collapse; width: 90%; max-width: 800px; background: #111; box-shadow: 0 0 10px rgba(255,255,255,0.1); }
    th, td { border: 1px solid #333; padding: 12px; color: #fff; }
    th { background: #222; }
    #download { display: none; margin-top: 20px; font-size: 18px; text-decoration: none; padding: 10px 20px; background: #28a745; color: white; border-radius: 5px; }
    .jauge-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 24px; width: 95%; max-width: 760px; margin: 0 auto; }
    .jauge { text-align: center; }
    .jauge svg { display: block; margin: 0 auto; transform: rotate(-90deg); }
    .jauge .val { font-weight: 500; margin-top: 6px; color: #fff; font-size: 17px; }
    .jauge .lbl { font-weight: 500; font-size: 17px; color: #fff; }
    #logStatus { margin: 10px 0; font-weight: bold; color: #fff; }
    #logStatus.rafale { color: #ff4444; }
    #downloadLog { display: none; margin-top: 10px; text-decoration: none; padding: 10px 20px; background: #28a745; color: white; border-radius: 5px; }
  </style>
</head>
<body>
  <h2>Log continu</h2>
  <button id="btnLog" onclick="toggleLog()">LOG</button>
  <div id="logStatus">Arrete</div>
  <a id="downloadLog" href="/download_log" download="drivelog.csv">Telecharger log CSV</a>
  <button id="btnPurge" onclick="purgerLog()" style="background:#c0392b;">PURGER LOG</button>
  <div id="fsBarWrap" style="width:95%;max-width:760px;margin:12px auto;background:#333;border-radius:6px;overflow:hidden;height:22px;">
    <div id="fsBar" style="height:100%;width:0%;background:#2ecc71;transition:width 0.3s;"></div>
  </div>
  <div id="fsText" style="margin-bottom:10px;color:#ccc;">-- </div>
  <div id="jauges" class="jauge-grid"></div>

  <hr style="border-color:#333; margin:30px 0;">

  <h1>Scan PID - ECU 0x10 (Nemo C-CAN) - FW 8.4</h1>
  <button id="btnCall" onclick="startCall()">CALL</button>
  <br><br>
  <button id="btnTest" onclick="testPid()">TEST PID 0x0C</button>
  <div id="testResult" style="margin-top:10px;font-weight:bold;"></div>
  <br>
  <a id="download" href="/download" download="result.csv">Telecharger CSV</a>
  <table>
    <thead>
      <tr><th>PID</th><th>Reponse</th><th>Buf</th><th>Donnees (Hex)</th><th>Decodage</th></tr>
    </thead>
    <tbody id="results">
    </tbody>
  </table>

  <script>
    function startCall() {
      const btn = document.getElementById('btnCall');
      const tbody = document.getElementById('results');
      const dl = document.getElementById('download');

      btn.disabled = true;
      tbody.innerHTML = '';
      dl.style.display = 'none';

      const evtSource = new EventSource('/do_call');

      evtSource.onmessage = function(e) {
        const res = JSON.parse(e.data);
        const tr = document.createElement('tr');
        tr.innerHTML = `<td>${res.pid}</td><td>${res.rep}</td><td>${res.buf}</td><td>${res.data}</td><td>${res.info}</td>`;
        tbody.appendChild(tr);
      };

      evtSource.addEventListener('done', function(e) {
        evtSource.close();
        btn.disabled = false;
        dl.style.display = 'inline-block';
      });

      evtSource.onerror = function(e) {
        evtSource.close();
        btn.disabled = false;
      };
    }
    function testPid() {
      const div = document.getElementById('testResult');
      div.textContent = '...';
      fetch('/test_pid').then(r => r.json()).then(res => {
        div.textContent = res.ok
          ? ('PID ' + res.pid + ' -> ' + res.resp + ' | ' + res.data + ' | ' + res.info)
          : ('PID ' + res.pid + ' -> aucune reponse');
      });
    }

    // ---- Jauges / log continu ----
    // idx = position dans le tableau pids[] renvoye par /log_status, qui suit
    // l'ordre de LECTURE CAN (MONITOR_PIDS, fige pour la synchronisation).
    // L'ordre ci-dessous est l'ordre d'AFFICHAGE, independant du precedent.
    var JAUGES = [
      {idx:0, label:'Pedale D',   max:100,    unit:'%'},
      {idx:8, label:'Pedale E',   max:100,    unit:'%'},
      {idx:1, label:'Vitesse',    max:200,    unit:'km/h'},
      {idx:2, label:'Regime',     max:5000,   unit:'rpm'},
      {idx:3, label:'Charge',     max:100,    unit:'%'},
      {idx:4, label:'MAF',        max:50,     unit:'g/s'},
      {idx:5, label:'P admission',max:250,    unit:'kPa'},
      {idx:6, label:'P rampe',    max:200000, unit:'kPa'},
      {idx:7, label:'EGR',        max:100,    unit:'%'}
    ];
    var RAYON = 45, CIRC = 2 * Math.PI * RAYON;

    function initJauges() {
      var div = document.getElementById('jauges');
      JAUGES.forEach(function(j, i) {
        var box = document.createElement('div');
        box.className = 'jauge';
        box.innerHTML =
          '<svg width="140" height="140" viewBox="0 0 100 100">' +
            '<circle cx="50" cy="50" r="' + RAYON + '" fill="none" stroke="#333" stroke-width="10"/>' +
            '<circle id="arc' + i + '" cx="50" cy="50" r="' + RAYON + '" fill="none" stroke="#2ecc71" ' +
              'stroke-width="10" stroke-dasharray="' + CIRC + '" stroke-dashoffset="' + CIRC + '" stroke-linecap="round"/>' +
          '</svg>' +
          '<div class="val" id="val' + i + '">--</div>' +
          '<div class="lbl">' + j.label + '</div>';
        div.appendChild(box);
      });
    }

    var logTimer = null, logging = false;
    function toggleLog() {
      var btn = document.getElementById('btnLog');
      if (!logging) {
        fetch('/log_start').then(function() {
          logging = true; btn.textContent = 'STOP LOG';
          document.getElementById('downloadLog').style.display = 'none';
          logTimer = setInterval(majJauges, 500);
        });
      } else {
        fetch('/log_stop').then(function() {
          logging = false; btn.textContent = 'LOG';
          clearInterval(logTimer);
          var st = document.getElementById('logStatus');
          st.textContent = 'Arrete'; st.className = '';
          document.getElementById('downloadLog').style.display = 'inline-block';
        });
      }
    }

    function purgerLog() {
      fetch('/log_purge').then(function(r) { return r.json(); }).then(function(d) {
        document.getElementById('fsText').textContent = d.ok ? 'Log purge.' : ('Purge impossible : ' + d.raison);
        majJauges();
      });
    }

    function majJauges() {
      fetch('/log_status').then(function(r) { return r.json(); }).then(function(d) {
        var st = document.getElementById('logStatus');
        if (d.mode === 'rafale') {
          st.textContent = 'RAFALE - ' + Math.max(0, Math.round(d.rafale_restant_ms / 1000)) + 's restantes';
          st.className = 'rafale';
        } else {
          st.textContent = 'Normal (1 Hz)';
          st.className = '';
        }
        var bar = document.getElementById('fsBar');
        bar.style.width = d.fs_pct + '%';
        bar.style.background = d.fs_pct > 85 ? '#e74c3c' : '#2ecc71';
        document.getElementById('fsText').textContent =
          d.fs_pct.toFixed(1) + '% stockage utilise (' + (d.fs_used / 1024).toFixed(0) + ' Ko / ' + (d.fs_total / 1024).toFixed(0) + ' Ko)';
        JAUGES.forEach(function(j, slot) {
          var p = d.pids[j.idx];
          if (!p) return;
          var frac = p.ok ? Math.max(0, Math.min(1, p.num / j.max)) : 0;
          var arc = document.getElementById('arc' + slot);
          if (arc) arc.setAttribute('stroke-dashoffset', CIRC * (1 - frac));
          var val = document.getElementById('val' + slot);
          if (val) val.textContent = p.ok ? (p.num.toFixed(1) + ' ' + j.unit) : '--';
        });
      });
    }

    initJauges();
    majJauges();
  </script>
</body>
</html>
)rawliteral";


/* ---------- Fonctions SPI Brutes MCP2515 -------------------- */

void mcp_write(uint8_t reg, uint8_t value) {
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(C_WRITE);
  SPI.transfer(reg);
  SPI.transfer(value);
  digitalWrite(PIN_CS, HIGH);
}

uint8_t mcp_read(uint8_t reg) {
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(C_READ);
  SPI.transfer(reg);
  uint8_t res = SPI.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  return res;
}

void mcp_bitmod(uint8_t reg, uint8_t mask, uint8_t value) {
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(C_BITMOD);
  SPI.transfer(reg);
  SPI.transfer(mask);
  SPI.transfer(value);
  digitalWrite(PIN_CS, HIGH);
}

void mcp_reset() {
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(C_RESET);
  digitalWrite(PIN_CS, HIGH);
  delay(10);
}

// Ecrit un ID 29 bits sur 4 registres consecutifs (SIDH,SIDL,EID8,EID0).
// Utilise pour les trames TX ET pour les registres de masque/filtre RX,
// qui partagent le meme format d'encodage cote MCP2515.
void mcp_write_id_regs(uint8_t baseAddr, uint32_t id, bool setExide) {
  uint8_t sidh = (id >> 21) & 0xFF;
  uint8_t sidl = (((id >> 18) & 0x07) << 5) | (setExide ? 0x08 : 0x00) | ((id >> 16) & 0x03);
  uint8_t eid8 = (id >> 8) & 0xFF;
  uint8_t eid0 = id & 0xFF;
  mcp_write(baseAddr, sidh);
  mcp_write(baseAddr + 1, sidl);
  mcp_write(baseAddr + 2, eid8);
  mcp_write(baseAddr + 3, eid0);
}

void mcp_init() {
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_INT, INPUT_PULLUP);

  SPI.begin();

  mcp_reset();

  // Mode Configuration
  mcp_bitmod(R_CANCTRL, 0xE0, 0x80);
  delay(1);

  // Vitesse : 500 kbps @ 16 MHz
  mcp_write(R_CNF1, 0x00);
  mcp_write(R_CNF2, 0xF0);
  mcp_write(R_CNF3, 0x86);

  // Filtre materiel : n'accepter que les reponses OBD physiques 0x18DAF1xx
  // (masque = ignore uniquement l'octet adresse ECU). Tout le reste du trafic
  // bus (chatter moteur, autres ECU) est elimine par le silicium, avant meme
  // de remonter en SPI ou d'occuper l'un des 2 buffers de reception.
  mcp_write_id_regs(0x20, 0x1FFFFF00UL, true);  // RXM0 (masque RXB0)
  mcp_write_id_regs(0x00, 0x18DAF100UL, true);  // RXF0 (RXB0)
  mcp_write_id_regs(0x04, 0x18DAF100UL, true);  // RXF1 (RXB0)
  mcp_write_id_regs(0x24, 0x1FFFFF00UL, true);  // RXM1 (masque RXB1)
  mcp_write_id_regs(0x08, 0x18DAF100UL, true);  // RXF2 (RXB1)

  // RXB0 : filtres actifs (RXM=00), rollover vers RXB1 si plein (BUKT=1)
  // RXB1 : filtres actifs (RXM=00)
  mcp_write(R_RXB0CTRL, 0x04);
  mcp_write(R_RXB1CTRL, 0x00);

  // Vider les interruptions
  mcp_write(R_CANINTF, 0x00);

  // Passage en Mode Normal
  mcp_bitmod(R_CANCTRL, 0xE0, 0x00);
  delay(1);
}

void mcp_send_ext(uint32_t id, const uint8_t* data, uint8_t len) {
  // Attendre que le buffer TXB0 soit libre, avec timeout. Sans accuse de
  // reception (ACK) d'un autre noeud CAN, le controleur retransmet la trame
  // precedente indefiniment en interne et TXREQ ne redescend jamais tout
  // seul : au-dela de 10ms on force l'abandon plutot que d'attendre a l'infini.
  uint32_t tAttente = millis();
  while (mcp_read(R_TXB0CTRL) & 0x08) {
    if (millis() - tAttente > 10) {
      mcp_bitmod(R_TXB0CTRL, 0x08, 0x00);   // force TXREQ=0 : abandon de l'envoi precedent
      Serial.println("[DBG] !! TXB0 bloque (pas d'ACK recu) - envoi precedent abandonne de force");
      break;
    }
    yield();
  }

  // Calcul du format d'ID Etendu 29 bits pour les registres du MCP2515
  uint8_t sidh = (id >> 21) & 0xFF;
  uint8_t sidl = (((id >> 18) & 0x07) << 5) | 0x08 | ((id >> 16) & 0x03);
  uint8_t eid8 = (id >> 8) & 0xFF;
  uint8_t eid0 = id & 0xFF;

  mcp_write(R_TXB0SIDH, sidh);
  mcp_write(R_TXB0SIDH + 1, sidl);
  mcp_write(R_TXB0SIDH + 2, eid8);
  mcp_write(R_TXB0SIDH + 3, eid0);
  mcp_write(R_TXB0SIDH + 4, len);

  for (int i = 0; i < len; i++) {
    mcp_write(R_TXB0SIDH + 5 + i, data[i]);
  }

  // Declenchement de l'envoi
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(C_RTS0);
  digitalWrite(PIN_CS, HIGH);
}

bool mcp_recv(uint32_t &id, uint8_t* data, uint8_t &len, uint8_t &bufOut) {
  uint8_t intf = mcp_read(R_CANINTF);
  uint8_t base_addr = 0;

  if (intf & 0x01) {
    base_addr = R_RXB0SIDH;
    bufOut = 0;
  } else if (intf & 0x02) {
    base_addr = R_RXB1SIDH;
    bufOut = 1;
  } else {
    return false; // Pas de donnees
  }

  uint8_t sidh = mcp_read(base_addr);
  uint8_t sidl = mcp_read(base_addr + 1);
  uint8_t eid8 = mcp_read(base_addr + 2);
  uint8_t eid0 = mcp_read(base_addr + 3);
  len = mcp_read(base_addr + 4) & 0x0F;

  for (int i = 0; i < len; i++) {
    data[i] = mcp_read(base_addr + 5 + i);
  }

  // Acquittement de l'interruption
  if (base_addr == R_RXB0SIDH) mcp_bitmod(R_CANINTF, 0x01, 0x00);
  else mcp_bitmod(R_CANINTF, 0x02, 0x00);

  bool is_ext = (sidl & 0x08) != 0;
  if (is_ext) {
    id = ((uint32_t)sidh << 21) | ((uint32_t)(sidl >> 5) << 18) | ((uint32_t)(sidl & 0x03) << 16) | ((uint32_t)eid8 << 8) | eid0;
    return true;
  }
  return false;
}


/* ---------- Setup & Loop ------------------------------------ */

void setup() {
  Serial.begin(115200);
  Serial.printf("obd_can_bridge FW %s\n", FW_VER);
  LittleFS.begin(true);

  mcp_init();

  WiFi.softAP(AP_SSID, AP_PASS);
  delay(100);
  WiFi.softAPConfig(AP_IP, AP_IP, AP_MASK);

  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", HTML_INDEX);
  });

  server.on("/download", HTTP_GET, []() {
    if (LittleFS.exists("/result.csv")) {
      File f = LittleFS.open("/result.csv", "r");
      server.streamFile(f, "text/csv");
      f.close();
    } else {
      server.send(404, "text/plain", "Fichier introuvable.");
    }
  });

  server.on("/test_pid", HTTP_GET, []() {
    uint32_t respId; uint8_t data[8]; uint8_t len; uint8_t buf;
    bool ok = testerUnPid(TEST_PID, respId, data, len, buf);

    String pid_str = String((uint8_t)TEST_PID, HEX); pid_str.toUpperCase();
    String json;
    if (ok) {
      String resp_str = String(respId, HEX); resp_str.toUpperCase();
      String data_str = "";
      for (int i = 0; i < len; i++) {
        if (data[i] < 16) data_str += "0";
        data_str += String(data[i], HEX) + " ";
      }
      data_str.toUpperCase(); data_str.trim();
      String info = (len >= 3 && data[1] == 0x41) ? decodePid(TEST_PID, data, len) :
                    (len >= 4 && data[1] == 0x7F) ? ("NEG NRC " + String(data[3], HEX)) : "Brut";
      json = "{\"ok\":true,\"pid\":\"0x" + pid_str + "\",\"resp\":\"0x" + resp_str +
             "\",\"data\":\"" + data_str + "\",\"info\":\"" + info + "\",\"buf\":" + String(buf) + "}";
    } else {
      json = "{\"ok\":false,\"pid\":\"0x" + pid_str + "\"}";
    }
    server.send(200, "application/json", json);
  });

  server.on("/do_call", HTTP_GET, []() {
    // Entetes Server-Sent Events (SSE)
    server.sendContent("HTTP/1.1 200 OK\r\n");
    server.sendContent("Content-Type: text/event-stream\r\n");
    server.sendContent("Cache-Control: no-cache\r\n");
    server.sendContent("Connection: keep-alive\r\n\r\n");

    WiFiClient client = server.client();
    String csv_content = "PID,Reponse,Buffer,Donnees,Decodage\n";

    auto envoyerLigne = [&](uint8_t pid, bool got, uint8_t* data, uint8_t len, uint8_t buf) {
      String pid_str = String(pid, HEX); pid_str.toUpperCase();
      String data_str = "-", info = "-";
      String buf_str = got ? String(buf) : "-";
      if (got) {
        data_str = "";
        for (int i = 0; i < len; i++) {
          if (data[i] < 16) data_str += "0";
          data_str += String(data[i], HEX) + " ";
        }
        data_str.toUpperCase(); data_str.trim();
        if (len >= 4 && data[1] == 0x7F) info = "NEG NRC " + String(data[3], HEX);
        else if (len >= 3 && data[1] == 0x41) info = decodePid(pid, data, len);
        else info = "Brut";
      }
      csv_content += "0x" + pid_str + "," + (got ? "OUI" : "NON") + "," + buf_str + "," + data_str + "," + info + "\n";
      String json = "{\"pid\":\"0x" + pid_str + "\",\"rep\":\"" + (got ? "OUI" : "NON") +
                    "\",\"buf\":\"" + buf_str + "\",\"data\":\"" + data_str + "\",\"info\":\"" + info + "\"}";
      client.print("data: " + json + "\n\n");
      client.flush();
    };

    // Etape 1 : suivre la chaine d'index 0x00 -> 0x20 -> 0x40 -> ...
    // Chaque reponse est un bitmap 32 bits : bits 31..1 = PID index+1..index+31
    // supportes ou non, bit 0 = le groupe suivant existe (et double comme
    // indicateur "PID index+32 supporte").
    uint8_t pidsSupportes[256]; uint16_t nbSupportes = 0;
    uint16_t indexPid = 0x00;

    while (client.connected()) {
      uint32_t respId; uint8_t data[8]; uint8_t len; uint8_t buf;
      bool got = testerUnPid((uint8_t)indexPid, respId, data, len, buf);
      envoyerLigne((uint8_t)indexPid, got, data, len, buf);

      if (!got || len < 7 || data[1] != 0x41) break;   // chaine cassee, on s'arrete

      uint32_t bitmap = ((uint32_t)data[3] << 24) | ((uint32_t)data[4] << 16) |
                         ((uint32_t)data[5] << 8) | data[6];
      bool continueChaine = bitmap & 0x01;

      for (uint8_t i = 0; i < 31; i++) {
        if (bitmap & (0x80000000UL >> i)) {
          if (nbSupportes < 256) pidsSupportes[nbSupportes++] = (uint8_t)(indexPid + 1 + i);
        }
      }

      if (!continueChaine) break;
      indexPid += 0x20;
      if (indexPid > 0xFF) break;
    }

    // Etape 2 : n'interroger que les PID reellement annonces supportes
    for (uint16_t i = 0; i < nbSupportes && client.connected(); i++) {
      uint32_t respId; uint8_t data[8]; uint8_t len; uint8_t buf;
      bool got = testerUnPid(pidsSupportes[i], respId, data, len, buf);
      envoyerLigne(pidsSupportes[i], got, data, len, buf);
      yield();
    }

    // Ecriture sur LittleFS
    File f = LittleFS.open("/result.csv", "w");
    if (f) {
      f.print(csv_content);
      f.close();
    }

    // Fin du stream
    client.print("event: done\ndata: end\n\n");
    client.flush();
  });

  server.on("/log_start", HTTP_GET, []() {
    Serial.println("[DBG] /log_start recu");
    if (logMode == LOG_IDLE) {
      logFile = LittleFS.open("/drivelog.csv", "w");
      if (logFile) logFile.println("t_ms,pid_hex,reponse,valeur_decodee,brut_hex");
      logT0 = millis();
      mancheDebutMs = millis();
      pidIndex = 0;
      pedalePrec = -1; vitessePrec = -1;
      for (uint8_t i = 0; i < NB_MONITOR; i++) { derniereOk[i] = false; derniereVal[i] = "-"; derniereNum[i] = 0; }
      logMode = LOG_NORMAL;
      Serial.printf("[DBG] logMode=NORMAL, heap=%u\n", ESP.getFreeHeap());
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/log_stop", HTTP_GET, []() {
    Serial.println("[DBG] /log_stop recu");   // si cette ligne n'apparait jamais -> la requete n'arrive meme pas au serveur
    logMode = LOG_IDLE;   // la fermeture du fichier est differee au debut de loop()
                          // (ce handler peut s'executer en pleine lecture PID)
    server.send(200, "text/plain", "OK");
    Serial.println("[DBG] /log_stop reponse envoyee");
  });

  server.on("/log_status", HTTP_GET, []() {
    size_t fsUsed = LittleFS.usedBytes();
    size_t fsTotal = LittleFS.totalBytes();
    float fsPct = fsTotal ? (100.0f * fsUsed / fsTotal) : 0;

    String json = "{\"mode\":\"" + String(logMode == LOG_IDLE ? "idle" : (logMode == LOG_RAFALE ? "rafale" : "normal")) +
                  "\",\"rafale_restant_ms\":" + String(logMode == LOG_RAFALE ? (long)(rafaleFinMs - millis()) : 0) +
                  ",\"fs_used\":" + String((unsigned long)fsUsed) + ",\"fs_total\":" + String((unsigned long)fsTotal) +
                  ",\"fs_pct\":" + String(fsPct, 1) + ",\"pids\":[";
    for (uint8_t i = 0; i < NB_MONITOR; i++) {
      if (i) json += ",";
      char pidhex[3]; sprintf(pidhex, "%02X", MONITOR_PIDS[i]);
      json += "{\"pid\":\"0x" + String(pidhex) + "\",\"ok\":" + String(derniereOk[i] ? "true" : "false") +
              ",\"num\":" + String(derniereNum[i], 2) + ",\"txt\":\"" + derniereVal[i] + "\"}";
    }
    json += "]}";
    server.send(200, "application/json", json);
  });

  server.on("/log_purge", HTTP_GET, []() {
    if (logMode != LOG_IDLE) {
      server.send(200, "application/json", "{\"ok\":false,\"raison\":\"log en cours\"}");
      return;
    }
    bool removed = !LittleFS.exists("/drivelog.csv") || LittleFS.remove("/drivelog.csv");
    server.send(200, "application/json", removed ? "{\"ok\":true}" : "{\"ok\":false,\"raison\":\"echec suppression\"}");
  });

  server.on("/download_log", HTTP_GET, []() {
    if (!LittleFS.exists("/drivelog.csv")) { server.send(404, "text/plain", "Pas de log"); return; }
    File f = LittleFS.open("/drivelog.csv", "r");
    server.sendHeader("Content-Disposition", "attachment; filename=nemo_drivelog.csv");
    server.streamFile(f, "text/csv");
    f.close();
  });

  server.begin();
}

void loop() {
  server.handleClient();

  if (logMode == LOG_IDLE) {
    if (logFile) {
      Serial.println("[DBG] LOG_IDLE detecte dans loop() -> fermeture fichier");
      if (logBuffer.length()) { logFile.print(logBuffer); logBuffer = ""; }
      logFile.close();   // fermeture differee (voir /log_stop)
      Serial.println("[DBG] fichier ferme");
    }
    return;
  }

  uint8_t pid = MONITOR_PIDS[pidIndex];
  Serial.printf("[DBG] round pid=0x%02X idx=%u mode=%d heap=%u t=%lu\n",
                pid, pidIndex, (int)logMode, ESP.getFreeHeap(), millis());

  uint32_t respId; uint8_t data[8]; uint8_t len; uint8_t buf;
  uint32_t tAvant = millis();
  bool got = testerUnPid(pid, respId, data, len, buf);
  uint32_t dureeAppel = millis() - tAvant;
  if (dureeAppel > PID_TIMEOUT_MS + 20) {
    Serial.printf("[DBG] !! testerUnPid a dure %lums (attendu ~%dms)\n", dureeAppel, PID_TIMEOUT_MS);
  }

  uint32_t t = millis() - logT0;
  String valTxt = "-"; String rawHex = "-"; float num = 0; bool numOk = false;

  if (got) {
    rawHex = "";
    for (int i = 0; i < len; i++) {
      char b[3]; sprintf(b, "%02X", data[i]);
      rawHex += b; if (i < len - 1) rawHex += " ";
    }
    if (len >= 3 && data[1] == 0x41) {
      valTxt = decodePid(pid, data, len);
      numOk = getNumeric(pid, data, len, num);
    } else if (len >= 4 && data[1] == 0x7F) {
      valTxt = "NEG NRC " + String(data[3], HEX);
    } else {
      valTxt = "Brut";
    }
  }

  // Maj des dernieres valeurs (jauges)
  derniereOk[pidIndex] = got;
  derniereVal[pidIndex] = valTxt;
  if (numOk) derniereNum[pidIndex] = num;

  // Ligne accumulee en RAM (pas d'ecriture flash ici, voir plus bas)
  char pidhex[3]; sprintf(pidhex, "%02X", pid);
  logBuffer += String(t) + ",0x" + pidhex + "," + (got ? "OUI" : "NON") + "," + valTxt + "," + rawHex + "\n";

  // Detection perte de puissance : pedale (index 0) puis vitesse (index 1),
  // adjacentes dans MONITOR_PIDS pour un ecart temporel minimal entre les deux.
  if (pidIndex == IDX_VITESSE && numOk && derniereOk[IDX_PEDALE]) {
    float pedaleNow = derniereNum[IDX_PEDALE];
    float vitesseNow = num;
    if (pedalePrec >= 0 && vitessePrec >= 0 && logMode != LOG_RAFALE) {
      bool motif = (pedaleNow > pedalePrec && vitesseNow < vitessePrec) || (pedaleNow > SEUIL_PEDALE_PCT);
      if (motif) {
        Serial.println("[DBG] RAFALE declenchee");
        logMode = LOG_RAFALE;
        rafaleFinMs = millis() + DUREE_RAFALE_MS;
      }
    }
    pedalePrec = pedaleNow;
    vitessePrec = vitesseNow;
  }

  pidIndex++;
  if (pidIndex >= NB_MONITOR) {
    pidIndex = 0;
    if (logMode == LOG_RAFALE && millis() > rafaleFinMs) {
      Serial.println("[DBG] fin RAFALE -> NORMAL");
      logMode = LOG_NORMAL;
    }
    Serial.printf("[DBG] fin manche mode=%d bufferLen=%u heap=%u\n", (int)logMode, logBuffer.length(), ESP.getFreeHeap());

    if (logMode == LOG_NORMAL) {
      // Cadence ~1 Hz : on complete la manche jusqu'a la seconde pleine
      uint32_t ecoule = millis() - mancheDebutMs;
      if (ecoule < DUREE_MANCHE_NORMALE_MS) delay(DUREE_MANCHE_NORMALE_MS - ecoule);
      mancheDebutMs = millis();

      // Ecriture flash uniquement ici (jamais en rafale), espacee dans le temps :
      // une ecriture flash peut bloquer le WiFi le temps de l'operation, donc on
      // la limite en frequence plutot que d'ecrire a chaque manche.
      if (logFile && millis() - derniereEcritureMs >= FLUSH_INTERVAL_MS && logBuffer.length()) {
        Serial.println("[DBG] ecriture flash en cours...");
        uint32_t tFlush = millis();
        logFile.print(logBuffer);
        logFile.flush();
        logBuffer = "";
        derniereEcritureMs = millis();
        Serial.printf("[DBG] ecriture flash OK (%lums)\n", millis() - tFlush);
      }
    } else {
      // Rafale : pause courte entre manches (stabilite avant vitesse), aucune
      // ecriture flash tant que la rafale dure - le tampon RAM absorbe tout.
      delay(RAFALE_PAUSE_MS);
    }
  }
}
