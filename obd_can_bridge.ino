#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <StreamString.h>
#include <string.h>
#include <time.h>
#include <FS.h>
#include <LittleFS.h>

#define FW_VER "8.17"

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
#define IDX_REGIME 2   // position de 0x0C dans MONITOR_PIDS
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

/* ---------- Niveau 2 : PID 0x1C (norme OBD, table plate) ------ */
struct NormeObd { uint8_t val; const char* nom; };
const NormeObd NORMES_OBD[] = {
  {1,"OBD II (Californie ARB)"},{2,"OBD (EPA federal)"},{3,"OBD et OBD II"},{4,"OBD I"},
  {5,"Non conforme OBD"},{6,"EOBD"},{7,"EOBD et OBD II"},{8,"EOBD et OBD"},
  {9,"EOBD, OBD, OBD II"},{10,"JOBD"},{11,"JOBD et OBD II"},{12,"JOBD et EOBD"},
  {13,"JOBD, EOBD, OBD II"},{14,"Poids lourds (Euro IV) B1"},{15,"Poids lourds (Euro V) B2"},
  {16,"Poids lourds (Euro EEC) C"},{17,"Diagnostic constructeur (EMD)"},{18,"Diagnostic constructeur ameliore (EMD+)"},
  {19,"HD OBD partiel"},{20,"HD OBD"},{21,"WWH OBD"},{23,"HD EOBD-I sans NOx"},
  {24,"HD EOBD-I avec NOx"},{25,"HD EOBD-II sans NOx"},{26,"HD EOBD-II avec NOx"},
  {27,"Poids lourds ZEV"},{28,"Bresil OBD Phase 1"},{29,"Bresil OBD Phase 2/2+"},
  {30,"OBD coreen"},{31,"Inde BS4 OBD I"},{32,"Inde BS4 OBD II"},{33,"Euro VI"},
  {34,"OBD, OBD II et HD OBD"},{35,"Bresil OBD Phase 3"},{36,"Moto Euro OBD-I"},
  {37,"Moto Euro OBD-II"},{38,"Moto Chine OBD-I"},{39,"Moto Taiwan OBD-I"},{40,"Moto Japon OBD-I"},
  {41,"Chine niveau national Stage 6"},{42,"Bresil OBD Phase 7"},{43,"Chine poids lourds VI"},
  {44,"Inde BS6 OBD I"},{45,"Inde BS6 OBD II"},{46,"Inde BSVI poids lourds"},{47,"Bresil OBD Phase 8"},
  {48,"Japon poids lourds OBD-II"},{49,"Coree poids lourds OBD-II"},{50,"Chine tout-terrain IV OBD"},
  {51,"ZEV leger ACC-II"},{52,"Moto Japon OBD-II"},{53,"Moto Californie CARB OBD"},
  {54,"Moto federal EPA OBD"},{55,"Moto 50-Etats CARB+EPA OBD"},{56,"Poids lourds ZEV CARB ZEP"},
  {57,"ZEV leger CARB ACC-II + EPA Tier4"},{58,"ZEV leger EPA Tier4"},{59,"EPA poids lourds"}
};
#define NNORMES (sizeof(NORMES_OBD)/sizeof(NORMES_OBD[0]))

String decodeNorme1C(uint8_t v) {
  for (uint16_t i = 0; i < NNORMES; i++) if (NORMES_OBD[i].val == v) return String(NORMES_OBD[i].nom);
  return "Valeur inconnue (" + String(v) + ")";
}

/* ---------- Niveau 2 : PID 0x01/0x41 (statut moniteurs, bits) --
 * Octet A : A7=MIL, A6-A0=nb codes (uniquement PID 01, A=0 sur PID 41)
 * Octet B : B0=rates supporte, B1=carburant supporte, B2=composants supporte,
 *           B3=type allumage (0=essence,1=diesel), B4/B5/B6=NON termine (rates/carburant/composants)
 * Octet C : bit=1 -> moniteur supporte (C0..C7 ci-dessous)
 * Octet D : memes positions que C, bit=1 -> NON termine ce cycle
 * (verifie sur donnees reelles du vehicule ; le sens "1=non termine" est
 * contre-intuitif mais confirme par plusieurs sources croisees) */
struct MoniteurNC { uint8_t bit; const char* nom; };
const MoniteurNC MONITEURS_NC[] = {
  {0,"Catalyseur"},{1,"Catalyseur chauffe"},{2,"Systeme evaporatif"},{3,"Air secondaire"},
  {4,"Climatisation (refrigerant)"},{5,"Sonde O2"},{6,"Rechauffage sonde O2"},{7,"EGR"}
};
const char* MONITEURS_CONT[] = {"Rates d'allumage", "Systeme carburant", "Composants divers"};

String decodeStatutMoniteurs(uint8_t* data, bool cyclActuel) {
  uint8_t A = data[3], B = data[4], C = data[5], D = data[6];
  String out = "";
  if (!cyclActuel) {
    out += (A & 0x80) ? "MIL:ON, " : "MIL:off, ";
    out += String(A & 0x7F) + " code(s), ";
  }
  out += (B & 0x08) ? "diesel" : "essence";
  out += " | ";
  bool premier = true;
  const char* ok = cyclActuel ? "termine" : "pret";
  const char* non = cyclActuel ? "en cours" : "pas pret";
  for (uint8_t i = 0; i < 3; i++) {
    if (B & (1 << i)) {
      if (!premier) out += ", ";
      out += String(MONITEURS_CONT[i]) + ":" + ((B & (1 << (i + 4))) ? non : ok);
      premier = false;
    }
  }
  for (uint8_t i = 0; i < 8; i++) {
    if (C & (1 << MONITEURS_NC[i].bit)) {
      if (!premier) out += ", ";
      out += String(MONITEURS_NC[i].nom) + ":" + ((D & (1 << MONITEURS_NC[i].bit)) ? non : ok);
      premier = false;
    }
  }
  if (premier) out += "aucun moniteur supporte";
  return out;
}

/* ---------- Estimation du rapport engage (source RTA, pneu exact
 * 185/65 R15 monte sur ce Nemo). La tolerance ci-dessous absorbe la
 * marge normale (usure, gonflage) sans qu'il soit besoin de deviner
 * une taille de pneu differente. ------------------------------------- */
const float VITESSE_A_1000RPM[6] = {0, 8.16f, 14.25f, 22.09f, 31.00f, 41.59f};  // index 1..5
#define RAPPORT_TOLERANCE 0.15f   // 15% d'ecart max, sinon embrayage/point mort/roue folle

uint8_t estimerRapport(float rpm, float vitesse) {
  if (rpm < 500 || vitesse < 3) return 0;   // moteur au ralenti ou vehicule a l'arret
  uint8_t meilleur = 0;
  float meilleurEcart = 999;
  for (uint8_t g = 1; g <= 5; g++) {
    float attendue = (rpm / 1000.0f) * VITESSE_A_1000RPM[g];
    float ecart = attendue > vitesse ? (attendue - vitesse) / attendue : (vitesse - attendue) / attendue;
    if (ecart < meilleurEcart) { meilleurEcart = ecart; meilleur = g; }
  }
  return (meilleurEcart <= RAPPORT_TOLERANCE) ? meilleur : 0;
}

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
enum LogMode { LOG_IDLE, LOG_NORMAL, LOG_RAFALE, LOG_CONDUITE };
LogMode  logMode = LOG_IDLE;
uint32_t logT0 = 0;            // debut du log (horodatage relatif)
uint32_t rafaleFinMs = 0;
uint32_t mancheDebutMs = 0;
uint8_t  pidIndex = 0;

float   pedalePrec = -1, vitessePrec = -1;
uint8_t rapportEstime = 0;   // 0 = indetermine, 1-5 = rapport (fige pendant LOG_CONDUITE)
bool    derniereOk[NB_MONITOR];
String  derniereVal[NB_MONITOR];   // texte decode, pour les jauges
float   derniereNum[NB_MONITOR];   // valeur numerique, pour les jauges
File    logFile;

/* ---------- Mode Conduite : RPM en boucle, rien d'autre --------------- */
float   rpmConduite = 0;
bool    rpmConduiteOk = false;
float   vitesseConduite = 0;
bool    vitesseConduiteOk = false;
uint8_t conduitePhase = 0;   // 0=RPM, 1=vitesse, alterne a chaque appel

/* ---------- Niveau 2 : etat -------------------------------------------- */
String   normeObd = "-";           // 0x1C, interroge une seule fois au demarrage
String   statutMoniteursDepuis = "-";  // 0x01, toutes les 60 s
String   statutMoniteursCycle = "-";   // 0x41, toutes les 60 s
uint32_t derniereMajNiveau2 = 0;
#define NIVEAU2_INTERVAL_MS 60000UL

void listerFlash() {
  Serial.println("[DBG] --- Contenu LittleFS ---");
  File root = LittleFS.open("/");
  File f = root.openNextFile();
  size_t total = 0;
  uint16_t nb = 0;
  while (f) {
    Serial.printf("[DBG]   %-30s %8u octets\n", f.name(), (unsigned)f.size());
    total += f.size();
    nb++;
    f = root.openNextFile();
  }
  root.close();
  Serial.printf("[DBG] --- %u fichier(s), %u octets au total (usedBytes=%u, totalBytes=%u) ---\n",
                nb, (unsigned)total, (unsigned)LittleFS.usedBytes(), (unsigned)LittleFS.totalBytes());
}

void verifierNiveau2() {
  if (millis() - derniereMajNiveau2 < NIVEAU2_INTERVAL_MS && derniereMajNiveau2 != 0) return;
  uint32_t respId; uint8_t data[8]; uint8_t len; uint8_t buf;
  if (testerUnPid(0x01, respId, data, len, buf) && len >= 7 && data[1] == 0x41) {
    statutMoniteursDepuis = decodeStatutMoniteurs(data, false);
  }
  if (testerUnPid(0x41, respId, data, len, buf) && len >= 7 && data[1] == 0x41) {
    statutMoniteursCycle = decodeStatutMoniteurs(data, true);
  }
  derniereMajNiveau2 = millis();
}
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

/* ---------- Requete "mode seul", sans octet PID (03/04/07/0A) - meme patron que testerUnPid */
bool testerMode(uint8_t sid, uint32_t &respIdOut, uint8_t* dataOut, uint8_t &lenOut, uint8_t &bufOut) {
  uint32_t d_id; uint8_t d_data[8]; uint8_t d_len; uint8_t d_buf;
  uint16_t drainCount = 0;
  while (mcp_recv(d_id, d_data, d_len, d_buf)) {
    drainCount++;
    if (drainCount > 100) {
      Serial.println("[DBG] !! vidage buffers > 100 iterations (mode)");
      break;
    }
    yield();
  }

  const uint8_t req_data[8] = {0x01, sid, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC};
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
        bufOut = rx_buf;
        got = true;
      }
    }
    server.handleClient();
    yield();
  }
  return got;
}

/* ---------- Decodage DTC (P/C/B/U + 4 chiffres) --------------- */
/* ---------- Table DTC (986 codes, traduction francaise) ------
 * Source : PDF Codes_OBD (EOBD-Facile), verifie et corrige :
 * conflit P0460-0469 tranche par controle visuel sur le PDF (page 13
 * retenue), decalages de mise en page P0665-0668 et similaires corriges,
 * P0397/P0398/P0399 exclus (non definis dans la source). */
struct DtcDef { const char* code; const char* texte; };
const DtcDef TABLE_DTC[] = {
  {"P0000","Aucune panne détectée"},
  {"P0001","Commande de régulateur de volume de carburant - circuit ouvert"},
  {"P0002","Commande de régulateur de volume de carburant - plage de mesure/performance du circuit"},
  {"P0003","Commande de régulateur de volume de carburant - circuit trop bas"},
  {"P0004","Commande de régulateur de volume de carburant - circuit trop haut"},
  {"P0005","Électrovanne de coupure carburant - circuit ouvert"},
  {"P0006","Électrovanne de coupure carburant - circuit trop bas"},
  {"P0007","Électrovanne de coupure carburant - circuit trop haut"},
  {"P0008","Calage moteur, ligne 1 - performance du moteur"},
  {"P0009","Calage moteur, ligne 2 - performance du moteur"},
  {"P0010","Capteur d'arbre à cames d'admission, ligne 1 - panne du circuit"},
  {"P0011","Position d'arbre à cames, ligne 1 - calage excessivement avancé/performance du système"},
  {"P0012","Position d'arbre à cames, ligne 1 - calage excessivement retardé"},
  {"P0013","Capteur d'arbre à cames d'échappement, ligne 1 - panne du circuit"},
  {"P0014","Capteur d'arbre à cames d'échappement, ligne 1 - calage excessivement avancé/performance du système"},
  {"P0015","Capteur d'arbre à cames d'échappement, ligne 1 - calage excessivement retardé"},
  {"P0016","Position du vilebrequin/position d'arbre à cames, ligne 1 capteur A - corrélation"},
  {"P0017","Position du vilebrequin/position d'arbre à cames, ligne 1 capteur B - corrélation"},
  {"P0018","Position du vilebrequin/position d'arbre à cames, ligne 2 capteur A - corrélation"},
  {"P0019","Position du vilebrequin/position d'arbre à cames, ligne 2 capteur B - corrélation"},
  {"P0020","Capteur d'arbre à cames d'admission, ligne 2 - panne du circuit"},
  {"P0021","Position d'arbre à cames d'admission, ligne 2 - calage excessivement avancé"},
  {"P0022","Position d'arbre à cames d'admission, ligne 2 - calage excessivement retardé"},
  {"P0023","Capteur d'arbre à cames d'échappement, ligne 2 - panne du circuit"},
  {"P0024","Position d'arbre à cames d'échappement, ligne 2 - calage excessivement avancé"},
  {"P0025","Position d'arbre à cames d'échappement, ligne 2 - calage excessivement retardé"},
  {"P0026","Système électrovanne de commande de soupape d'admission, ligne 1 - plage de mesure/performance"},
  {"P0027","Système électrovanne de commande de soupape d'échappement, ligne 1 - plage de mesure/performance"},
  {"P0028","Système électrovanne de commande de soupape d'admission, ligne 2 - plage de mesure/performance"},
  {"P0029","Système électrovanne de commande de soupape d'échappement, ligne 2 - plage de mesure/performance"},
  {"P0030","Sonde Lambda 1, ligne 1, commande de chauffage - panne du circuit"},
  {"P0031","Sonde Lambda 1, ligne 1, commande de chauffage - circuit trop bas"},
  {"P0032","Sonde Lambda 1, ligne 1, commande de chauffage - circuit trop haut"},
  {"P0033","Électrovanne de décharge du turbocompresseur - panne du circuit"},
  {"P0034","Électrovanne de décharge du turbocompresseur - circuit trop bas"},
  {"P0035","Électrovanne de décharge du turbocompresseur - circuit trop haut"},
  {"P0036","Sonde Lambda 2, ligne 1 , commande de chauffage - panne du circuit"},
  {"P0037","Sonde Lambda 2, ligne 1 , commande de chauffage - circuit trop bas"},
  {"P0038","Sonde Lambda 2, ligne 1 , commande de chauffage - circuit trop haut"},
  {"P0039","Soupape de dérivation du compresseur/turbocompresseur, circuit de commande - plage de mesure/performance"},
  {"P0040","Signaux sondes Lambda inversées, ligne 1 capteur 1/ligne 2 capteur 1"},
  {"P0041","Signaux sondes Lambda inversées, ligne 1 capteur 2/ligne 2 capteur 2"},
  {"P0042","Sonde Lambda 3, ligne 1 , commande de chauffage - panne du circuit"},
  {"P0043","Sonde Lambda 3, ligne 1 , commande de chauffage - circuit trop bas"},
  {"P0044","Sonde Lambda 3, ligne 1 , commande de chauffage - circuit trop haut"},
  {"P0046","Électrovanne de commande de la pression de suralimentation du compresseur/turbocompresseur - plage de mesure/performance du circuit"},
  {"P0047","Électrovanne de commande de la pression de suralimentation du compresseur/turbocompresseur - circuit trop bas"},
  {"P0048","Électrovanne de commande de la pression de suralimentation du compresseur/turbocompresseur - circuit trop haut"},
  {"P0049","Turbine de compresseur/turbocompresseur - régime excessif"},
  {"P0050","Sonde Lambda 1, ligne 2, commande de chauffage - panne du circuit"},
  {"P0051","Sonde Lambda 1, ligne 2, commande de chauffage - circuit trop bas"},
  {"P0052","Sonde Lambda 1, ligne 2, commande de chauffage - circuit trop haut"},
  {"P0053","Sonde Lambda, ligne 1 , capteur 1 - résistance du chauffage"},
  {"P0054","Sonde Lambda, ligne 1 , capteur 2 - résistance du chauffage"},
  {"P0055","Sonde Lambda, ligne 1 , capteur 3 - résistance du chauffage"},
  {"P0056","Sonde Lambda 2, ligne 2, commande de chauffage - panne du circuit"},
  {"P0057","Sonde Lambda 2, ligne 2, commande de chauffage - circuit de chauffage trop faible"},
  {"P0058","Sonde Lambda 2, ligne 2, commande de chauffage - circuit trop haut"},
  {"P0059","Sonde Lambda, ligne 2, capteur 1 - résistance du chauffage"},
  {"P0060","Sonde Lambda, ligne 2, capteur 2 - résistance du chauffage"},
  {"P0061","Sonde Lambda, ligne 2, capteur 3 -résistance du chauffage"},
  {"P0062","Sonde Lambda 3, ligne 2, commande de chauffage - panne du circuit"},
  {"P0063","Sonde Lambda 3, ligne 2, commande de chauffage - circuit trop bas"},
  {"P0064","Sonde Lambda 3, ligne 2, commande de chauffage - circuit trop haut"},
  {"P0065","Injecteur assisté par air comprimé - problème de performance/de limites"},
  {"P0066","Injecteur assisté par air comprimé - panne du circuit/circuit trop bas"},
  {"P0067","Injecteur assisté par air comprimé - circuit trop haut"},
  {"P0068","Corrélation capteur de pression absolue du collecteur d'admission/débitmètre d'air/position du papillon"},
  {"P0069","Corrélation capteur de pression absolue du collecteur d'admission/capteur de pression atmosphérique"},
  {"P0070","Sonde de température extérieure - panne du circuit"},
  {"P0071","Sonde de température extérieure - problème de performance/de limites"},
  {"P0072","Sonde de température extérieure - valeur d'entrée trop basse"},
  {"P0073","Sonde de température extérieure - valeur d'entrée trop haute"},
  {"P0074","Sonde de température extérieure - circuit intermittent"},
  {"P0075","Électrovanne de commande de soupape d'admission, ligne 1 - panne du circuit"},
  {"P0076","Électrovanne de commande de soupape d'admission, ligne 1 - circuit trop bas"},
  {"P0077","Électrovanne de commande de soupape d'admission, ligne 1 - circuit trop haut"},
  {"P0078","Électrovanne de commande de soupape d'échappement ligne 1 - panne du circuit"},
  {"P0079","Électrovanne de commande de soupape d'échappement ligne 1 - circuit trop bas"},
  {"P0080","Électrovanne de commande de soupape d'échappement ligne 1 - circuit trop haut"},
  {"P0081","Électrovanne de commande de soupape d'admission, ligne 2 - panne du circuit"},
  {"P0082","Électrovanne de commande de soupape d'admission, ligne 2 - circuit trop bas"},
  {"P0083","Électrovanne de commande de soupape d'admission, ligne 2 - circuit trop haut"},
  {"P0084","Électrovanne de commande de soupape d'échappement ligne 2 - panne du circuit"},
  {"P0085","Électrovanne de commande de soupape d'échappement ligne 2 - circuit trop bas"},
  {"P0086","Électrovanne de commande de soupape d'échappement ligne 2 - circuit trop haut"},
  {"P0087","Rampe de distribution/pression du système trop faible"},
  {"P0088","Rampe de distribution/pression du système trop haute"},
  {"P0089","Régulateur de pression du carburant - problème de performance"},
  {"P0090","Électrovanne de dosage de carburant - circuit ouvert"},
  {"P0091","Électrovanne de dosage de carburant - court-circuit sur masse"},
  {"P0092","Électrovanne de dosage de carburant - court-circuit sur l'alimentation"},
  {"P0093","Fuite dans le système d'alimentation en carburant - fuite importante détectée"},
  {"P0094","Fuite dans le système d'alimentation en carburant - petit fuite détectée"},
  {"P0095","Sonde de température d'air d'admission 2 - panne du circuit"},
  {"P0096","Sonde de température d'air d'admission 2 - plage de mesure/performance du circuit"},
  {"P0097","Sonde de température d'air d'admission 2 - signal d'entrée du circuit trop bas"},
  {"P0098","Sonde de température d'air d'admission 2 - signal d'entrée du circuit trop haut"},
  {"P0099","Sonde de température d'air d'admission 2 - circuit intermittent/instable"},
  {"P0100","Débitmètre d'air - panne du circuit"},
  {"P0101","Débitmètre d'air - problème de performance/de limites"},
  {"P0102","Débitmètre d'air - valeur d'entrée trop basse"},
  {"P0103","Débitmètre d'air - valeur d'entrée trop haute"},
  {"P0104","Débitmètre d'air - circuit intermittent"},
  {"P0105","Capteur de pression absolue du collecteur d'admission/capteur de pression atmosphérique - panne du circuit"},
  {"P0106","Capteur de pression absolue du collecteur d'admission/capteur de pression atmosphérique - problème de performance/de limites"},
  {"P0107","Capteur de pression absolue du collecteur d'admission/capteur de pression atmosphérique - valeur d'entrée trop basse"},
  {"P0108","Capteur de pression absolue du collecteur d'admission/capteur de pression atmosphérique - valeur d'entrée trop haute"},
  {"P0109","Capteur de pression absolue du collecteur d'admission/capteur de pression atmosphérique - circuit intermittent"},
  {"P0110","Sonde de température d'air d'admission - panne du circuit"},
  {"P0111","Sonde de température d'air d'admission - problème de performance/de limites"},
  {"P0112","Sonde de température d'air d'admission - valeur d'entrée trop basse"},
  {"P0113","Sonde de température d'air d'admission - valeur d'entrée trop haute"},
  {"P0114","Sonde de température d'air d'admission - circuit intermittent"},
  {"P0115","Sonde de température du liquide de refroidissement - panne du circuit"},
  {"P0116","Sonde de température du liquide de refroidissement - problème de performance/de limites"},
  {"P0117","Sonde de température du liquide de refroidissement - valeur d'entrée trop basse"},
  {"P0118","Sonde de température du liquide de refroidissement - valeur d'entrée trop haute"},
  {"P0119","Sonde de température du liquide de refroidissement - circuit intermittent"},
  {"P0120","Capteur de position de papillon A/capteur de position de la pédale d'accélérateur A - panne du circuit"},
  {"P0121","Capteur de position de papillon A/capteur de position de la pédale d'accélérateur A - problème de performance/de limites"},
  {"P0122","Capteur de position de papillon A/capteur de position de la pédale d'accélérateur A - valeur d'entrée trop basse"},
  {"P0123","Capteur de position de papillon A/capteur de position de la pédale d'accélérateur A - valeur d'entrée trop haute"},
  {"P0124","Capteur de position de papillon A/capteur de position de la pédale d'accélérateur A - circuit intermittent"},
  {"P0125","Température du liquide de refroidissement insuffisante pour commande en boucle fermée"},
  {"P0126","Température du liquide de refroidissement insuffisante pour un fonctionnement stable"},
  {"P0127","Température d'air d'admission trop haute"},
  {"P0128","Thermostat du liquide de refroidissement - température du liquide de refroidissement inférieure à la température de régulation du thermostat"},
  {"P0129","Pression atmosphérique trop basse"},
  {"P0130","Sonde Lambda 1, ligne 1 - panne du circuit"},
  {"P0131","Sonde Lambda 1, ligne 1 - signal bas"},
  {"P0132","Sonde Lambda 1, ligne 1 - signal haut"},
  {"P0133","Sonde Lambda 1, ligne 1 - réponse lente"},
  {"P0134","Sonde Lambda 1, ligne 1 - pas d'activité détectée"},
  {"P0135","Sonde Lambda 1, ligne 1 - commande de chauffage - panne de circuit"},
  {"P0136","Sonde Lambda 2, ligne 1 - panne du circuit"},
  {"P0137","Sonde Lambda 2, ligne 1 - signal bas"},
  {"P0138","Sonde Lambda 2, ligne 1 - signal haut"},
  {"P0139","Sonde Lambda 2, ligne 1 - réponse lente"},
  {"P0140","Sonde Lambda 2, ligne 1 - pas d'activité détectée"},
  {"P0141","Sonde Lambda 2, ligne 1 - commande de chauffage - panne de circuit"},
  {"P0142","Sonde Lambda 3, ligne 1 - panne du circuit"},
  {"P0143","Sonde Lambda 3, ligne 1 - signal bas"},
  {"P0144","Sonde Lambda 3, ligne 1 - signal haut"},
  {"P0145","Sonde Lambda 3, ligne 1 - réponse lente"},
  {"P0146","Sonde Lambda 3, ligne 1 - pas d'activité détectée"},
  {"P0147","Sonde Lambda 3, ligne 1 - commande de chauffage - panne de circuit"},
  {"P0148","Erreur de débit de carburant"},
  {"P0149","Erreur de calage d'injection"},
  {"P0150","Sonde Lambda 1, ligne 2 - panne du circuit"},
  {"P0151","Sonde Lambda 1, ligne 2 - signal bas"},
  {"P0152","Sonde Lambda 1, ligne 2 - signal haut"},
  {"P0153","Sonde Lambda 1, ligne 2 - réponse lente"},
  {"P0154","Sonde Lambda 1, ligne 2 - pas d'activité détectée"},
  {"P0155","Sonde Lambda 1, ligne 2 - commande de chauffage - panne de circuit"},
  {"P0156","Sonde Lambda 2, ligne 2 - panne du circuit"},
  {"P0157","Sonde Lambda 2, ligne 2 - signal bas"},
  {"P0158","Sonde Lambda 2, ligne 2 - signal haut"},
  {"P0159","Sonde Lambda 2, ligne 2 - réponse lente"},
  {"P0160","Sonde Lambda 2, ligne 2 - pas d'activité détectée"},
  {"P0161","Sonde Lambda 2, ligne 2 - commande de chauffage - panne de circuit"},
  {"P0162","Sonde Lambda 3, ligne 2 - panne du circuit"},
  {"P0163","Sonde Lambda 3, ligne 2 - signal bas"},
  {"P0164","Sonde Lambda 3, ligne 2 - signal haut"},
  {"P0165","Sonde Lambda 3, ligne 2 - réponse lente"},
  {"P0166","Sonde Lambda 3, ligne 2 - pas d'activité détectée"},
  {"P0167","Sonde Lambda 3, ligne 2 - commande de chauffage - panne de circuit"},
  {"P0168","Température du carburant trop haute"},
  {"P0169","Erreur de composition du carburant"},
  {"P0170","Ajustement du carburant, ligne 1 - panne"},
  {"P0171","Mélange trop pauvre, ligne 1"},
  {"P0172","Mélange trop riche, ligne 1"},
  {"P0173","Ajustement du carburant, ligne 2 - panne"},
  {"P0174","Mélange trop pauvre, ligne 2"},
  {"P0175","Mélange trop riche, ligne 2"},
  {"P0176","Capteur de composition du carburant - panne du circuit"},
  {"P0177","Capteur de composition du carburant - problème de performance/de limites"},
  {"P0178","Capteur de composition du carburant - valeur d'entrée trop basse"},
  {"P0179","Capteur de composition du carburant - valeur d'entrée trop haute"},
  {"P0180","Sonde de température du carburant A - panne du circuit"},
  {"P0181","Sonde de température du carburant A - problème de performance/de limites"},
  {"P0182","Sonde de température du carburant A - valeur d'entrée trop basse"},
  {"P0183","Sonde de température du carburant A - valeur d'entrée trop haute"},
  {"P0184","Sonde de température du carburant A - circuit intermittent"},
  {"P0185","Sonde de température du carburant B - panne du circuit"},
  {"P0186","Sonde de température du carburant B - problème de performance/de limites"},
  {"P0187","Sonde de température du carburant B - valeur d'entrée trop basse"},
  {"P0188","Sonde de température du carburant B - valeur d'entrée trop haute"},
  {"P0189","Sonde de température du carburant B - circuit intermittent"},
  {"P0190","Capteur de pression de la rampe de distribution - panne du circuit"},
  {"P0191","Capteur de pression de la rampe de distribution - problème de performance/de limites"},
  {"P0192","Capteur de pression de la rampe de distribution - valeur d'entrée trop basse"},
  {"P0193","Capteur de pression de la rampe de distribution - valeur d'entrée trop haute"},
  {"P0194","Capteur de pression de la rampe de distribution - circuit intermittent"},
  {"P0195","Sonde de température d'huile moteur - panne du circuit"},
  {"P0196","Sonde de température d'huile moteur - problème de performance/de limites"},
  {"P0197","Sonde de température d'huile moteur - valeur d'entrée trop basse"},
  {"P0198","Sonde de température d'huile moteur - valeur d'entrée trop haute"},
  {"P0199","Sonde de température d'huile moteur - circuit intermittent"},
  {"P0200","Injecteur - panne du circuit"},
  {"P0201","Injecteur 1 - panne du circuit"},
  {"P0202","Injecteur 2 - panne du circuit"},
  {"P0203","Injecteur 3 - panne du circuit"},
  {"P0204","Injecteur 4 - panne du circuit"},
  {"P0205","Injecteur 5 - panne du circuit"},
  {"P0206","Injecteur 6 - panne du circuit"},
  {"P0207","Injecteur 7 - panne du circuit"},
  {"P0208","Injecteur 8 - panne du circuit"},
  {"P0209","Injecteur 9 - panne du circuit"},
  {"P0210","Injecteur 10 - panne du circuit"},
  {"P0211","Injecteur 11 - panne du circuit"},
  {"P0212","Injecteur 12 - panne du circuit"},
  {"P0213","Injecteur de départ à froid 1 - panne du circuit"},
  {"P0214","Injecteur de départ à froid 2 - panne du circuit"},
  {"P0215","Électrovanne de coupure de carburant - panne du circuit"},
  {"P0216","Commande de calage d'injection - panne du circuit"},
  {"P0217","Surchauffe du moteur"},
  {"P0218","Surchauffe de la transmission"},
  {"P0219","Régime excessif"},
  {"P0220","Capteur de position de papillon B/capteur de position de la pédale d'accélérateur B - panne du circuit"},
  {"P0221","Capteur de position de papillon B/capteur de position de la pédale d'accélérateur B - problème de performance/de limites"},
  {"P0222","Capteur de position de papillon B/capteur de position de la pédale d'accélérateur B- valeur d'entrée trop basse"},
  {"P0223","Capteur de position de papillon B/capteur de position de la pédale d'accélérateur B - valeur d'entrée trop haute"},
  {"P0224","Capteur de position de papillon B/capteur de position de la pédale d'accélérateur B - circuit intermittent"},
  {"P0225","Capteur de position de papillon C/capteur de position de la pédale d'accélérateur C - panne du circuit"},
  {"P0226","Capteur de position de papillon C/capteur de position de la pédale d'accélérateur C - problème de performance/de limites"},
  {"P0227","Capteur de position de papillon C/capteur de position de la pédale d'accélérateur C - valeur d'entrée trop basse"},
  {"P0228","Capteur de position de papillon C/capteur de position de la pédale d'accélérateur C- valeur d'entrée trop haute"},
  {"P0229","Capteur de position de papillon C/capteur de position de la pédale d'accélérateur C- circuit intermittent"},
  {"P0230","Circuit primaire de pompe à carburant - panne du circuit"},
  {"P0231","Circuit secondaire de pompe à carburant - circuit trop bas"},
  {"P0232","Circuit secondaire de pompe à carburant - circuit trop haut"},
  {"P0233","Circuit secondaire de pompe à carburant - circuit intermittent"},
  {"P0234","Condition de suralimentation du moteur - limite dépassée"},
  {"P0235","Capteur de pression absolue du collecteur d'admission A, circuit du turbocompresseur - panne du circuit"},
  {"P0236","Capteur de pression absolue du collecteur d'admission A, circuit du turbocompresseur - problème de performance/de limites"},
  {"P0237","Capteur de pression absolue du collecteur d'admission A, circuit du turbocompresseur - valeur d'entrée trop basse"},
  {"P0238","Capteur de pression absolue du collecteur d'admission A, circuit du turbocompresseur - valeur d'entrée trop haute"},
  {"P0239","Capteur de pression absolue du collecteur d'admission B, circuit du turbocompresseur - panne du circuit"},
  {"P0240","Capteur de pression absolue du collecteur d'admission B, circuit du turbocompresseur - problème de performance/de limites"},
  {"P0241","Capteur de pression absolue du collecteur d'admission B, circuit du turbocompresseur - valeur d'entrée trop basse"},
  {"P0242","Capteur de pression absolue du collecteur d'admission B, circuit du turbocompresseur - valeur d'entrée trop haute"},
  {"P0243","Électrovanne de décharge du turbocompresseur A - panne du circuit"},
  {"P0244","Électrovanne de décharge du turbocompresseur A - problème de performance/de limites"},
  {"P0245","Électrovanne de décharge du turbocompresseur A - circuit trop bas"},
  {"P0246","Électrovanne de décharge du turbocompresseur A - circuit trop haut"},
  {"P0247","Électrovanne de décharge du turbocompresseur B - panne du circuit"},
  {"P0248","Électrovanne de décharge du turbocompresseur B - problème de performance/de limites"},
  {"P0249","Électrovanne de décharge du turbocompresseur B - circuit trop bas"},
  {"P0250","Électrovanne de décharge du turbocompresseur B - circuit trop haut"},
  {"P0251","Pompe d'injection A, rotor/cames - panne du circuit"},
  {"P0252","Pompe d'injection A, rotor/cames - problème de performance/de limites"},
  {"P0253","Pompe d'injection A, rotor/cames - circuit trop bas"},
  {"P0254","Pompe d'injection A, rotor/cames - circuit trop haut"},
  {"P0255","Pompe d'injection A, rotor/cames - circuit intermittent"},
  {"P0256","Pompe d'injection B, rotor/cames - panne du circuit"},
  {"P0257","Pompe d'injection B, rotor/cames - problème de performance/de limites"},
  {"P0258","Pompe d'injection B, rotor/cames - circuit trop bas"},
  {"P0259","Pompe d'injection B, rotor/cames - circuit trop haut"},
  {"P0260","Pompe d'injection B, rotor/cames - circuit intermittent"},
  {"P0261","Injecteur 1 - circuit trop bas"},
  {"P0262","Injecteur 1 - circuit trop haut"},
  {"P0263","Cylindre 1 - Quantité de fuel injecté - défaut d'équilibrage"},
  {"P0264","Injecteur 2 - circuit trop bas"},
  {"P0265","Injecteur 2 - circuit trop haut"},
  {"P0266","Cylindre 2 - Quantité de fuel injecté - défaut d'équilibrage"},
  {"P0267","Injecteur 3 - circuit trop bas"},
  {"P0268","Injecteur 3 - circuit trop haut"},
  {"P0269","Cylindre 3 - Quantité de fuel injecté - défaut d'équilibrage"},
  {"P0270","Injecteur 4 - circuit trop bas"},
  {"P0271","Injecteur 4 - circuit trop haut"},
  {"P0272","Cylindre 4 - Quantité de fuel injecté - défaut d'équilibrage"},
  {"P0273","Injecteur 5 - circuit trop bas"},
  {"P0274","Injecteur 5 - circuit trop haut"},
  {"P0275","Cylindre 5 - Quantité de fuel injecté - défaut d'équilibrage"},
  {"P0276","Injecteur 6 - circuit trop bas"},
  {"P0277","Injecteur 6 - circuit trop haut"},
  {"P0278","Cylindre 6 - Quantité de fuel injecté - défaut d'équilibrage"},
  {"P0279","Injecteur 7 - circuit trop bas"},
  {"P0280","Injecteur 7 - circuit trop haut"},
  {"P0281","Cylindre 7 - Quantité de fuel injecté - défaut d'équilibrage"},
  {"P0282","Injecteur 8 - circuit trop bas"},
  {"P0283","Injecteur 8 - circuit trop haut"},
  {"P0284","Cylindre 8 - Quantité de fuel injecté - défaut d'équilibrage"},
  {"P0285","Injecteur 9 - circuit trop bas"},
  {"P0286","Injecteur 9 - circuit trop haut"},
  {"P0287","Cylindre 9 - Quantité de fuel injecté - défaut d'équilibrage"},
  {"P0288","Injecteur 10 - circuit trop bas"},
  {"P0289","Injecteur 10 - circuit trop haut"},
  {"P0290","Cylindre 10 - Quantité de fuel injecté - défaut d'équilibrage"},
  {"P0291","Injecteur 11 - circuit trop bas"},
  {"P0292","Injecteur 11 - circuit trop haut"},
  {"P0293","Cylindre 11 - Quantité de fuel injecté - défaut d'équilibrage"},
  {"P0294","Injecteur 12 - circuit trop bas"},
  {"P0295","Injecteur 12 - circuit trop haut"},
  {"P0296","Cylindre 12 - Quantité de fuel injecté - défaut d'équilibrage"},
  {"P0297","Vitesse excessive véhicule"},
  {"P0298","Température d'huile moteur trop haute"},
  {"P0299","Compresseur/turbocompresseur - pression de suralimentation faible"},
  {"P0300","Cylindre(s) multiple(s) - ratés d'allumage aléatoire détectés"},
  {"P0301","Cylindre 1 - ratés d'allumage détectés"},
  {"P0302","Cylindre 2 - ratés d'allumage détectés"},
  {"P0303","Cylindre 3 - ratés d'allumage détectés"},
  {"P0304","Cylindre 4 - ratés d'allumage détectés"},
  {"P0305","Cylindre 5 - ratés d'allumage détectés"},
  {"P0306","Cylindre 6 - ratés d'allumage détectés"},
  {"P0307","Cylindre 7 - ratés d'allumage détectés"},
  {"P0308","Cylindre 8 - ratés d'allumage détectés"},
  {"P0309","Cylindre 9 - ratés d'allumage détectés"},
  {"P0310","Cylindre 10 - ratés d'allumage détectés"},
  {"P0311","Cylindre 11 - ratés d'allumage détectés"},
  {"P0312","Cylindre 12 - ratés d'allumage détectés"},
  {"P0313","Ratés d'allumage détectés avec niveau de carburant trop bas"},
  {"P0314","Raté d'allumage dans un seul cylindre - cylindre non spécifié"},
  {"P0315","Système position du vilebrequin - variation non apprise"},
  {"P0316","Ratés d'allumage détectés au démarrage - dans les 1000 premiers tours moteur"},
  {"P0317","Composant détecteur de route accidentée absent"},
  {"P0318","Signal du détecteur de route accidentée A - panne du circuit"},
  {"P0319","Signal du détecteur de route accidentée B - panne du circuit"},
  {"P0320","Capteur de vilebrequin/de régime - panne du circuit"},
  {"P0321","Capteur de vilebrequin/de régime - problème de performance/de limites"},
  {"P0322","Capteur de vilebrequin/de régime - aucun signal"},
  {"P0323","Capteur de vilebrequin/de régime - circuit intermittent"},
  {"P0324","Erreur de système anti-cliquetis"},
  {"P0325","Détecteur de cliquetis 1 (ligne 2 ou capteur unique) - panne du circuit"},
  {"P0326","Détecteur de cliquetis 1 (ligne 2 ou capteur unique) - problème de performance/de limites"},
  {"P0327","Détecteur de cliquetis 1 (ligne 2 ou capteur unique) - valeur d'entrée trop basse"},
  {"P0328","Détecteur de cliquetis 1 (ligne 2 ou capteur unique) - valeur d'entrée trop haute"},
  {"P0329","Détecteur de cliquetis 1 (ligne 2 ou capteur unique) - circuit intermittent"},
  {"P0330","Détecteur de cliquetis 2, ligne 2 - panne du circuit"},
  {"P0331","Détecteur de cliquetis 2, ligne 2 - problème de performance/de limites"},
  {"P0332","Détecteur de cliquetis 2, ligne 2 - valeur d'entrée trop basse"},
  {"P0333","Détecteur de cliquetis 2, ligne 2 - valeur d'entrée trop haute"},
  {"P0334","Détecteur de cliquetis 2, ligne 2 - circuit intermittent"},
  {"P0335","Capteur de vilebrequin - panne du circuit"},
  {"P0336","Capteur de vilebrequin - problème de performance/de limites"},
  {"P0337","Capteur de vilebrequin - valeur d'entrée trop basse"},
  {"P0338","Capteur de vilebrequin - valeur d'entrée trop haute"},
  {"P0339","Capteur de vilebrequin - circuit intermittent"},
  {"P0340","Capteur d'arbre à cames A ligne 1 - panne du circuit"},
  {"P0341","Capteur d'arbre à cames A ligne 1 - problème de performance/de limites"},
  {"P0342","Capteur d'arbre à cames A ligne 1 - valeur d'entrée trop basse"},
  {"P0343","Capteur d'arbre à cames A ligne 1 - valeur d'entrée trop haute"},
  {"P0344","Capteur d'arbre à cames A ligne 1 - circuit intermittent"},
  {"P0345","Capteur d'arbre à cames A ligne 2 - panne du circuit"},
  {"P0346","Capteur d'arbre à cames A ligne 2 - problème de performance/de limites"},
  {"P0347","Capteur d'arbre à cames A ligne 2 - valeur d'entrée trop basse"},
  {"P0348","Capteur d'arbre à cames A ligne 2 - valeur d'entrée trop haute"},
  {"P0349","Capteur d'arbre à cames A ligne 2 - circuit intermittent"},
  {"P0350","Bobine d'allumage, primaire/secondaire - panne du circuit"},
  {"P0351","Bobine d'allumage A, primaire/secondaire - panne du circuit"},
  {"P0352","Bobine d'allumage B, primaire/secondaire - panne du circuit"},
  {"P0353","Bobine d'allumage C, primaire/secondaire - panne du circuit"},
  {"P0354","Bobine d'allumage D, primaire/secondaire - panne du circuit"},
  {"P0355","Bobine d'allumage E, primaire/secondaire - panne du circuit"},
  {"P0356","Bobine d'allumage F, primaire/secondaire - panne du circuit"},
  {"P0357","Bobine d'allumage G, primaire/secondaire - panne du circuit"},
  {"P0358","Bobine d'allumage H, primaire/secondaire - panne du circuit"},
  {"P0359","Bobine d'allumage I, primaire/secondaire - panne du circuit"},
  {"P0360","Bobine d'allumage J, primaire/secondaire - panne du circuit"},
  {"P0361","Bobine d'allumage K, primaire/secondaire - panne du circuit"},
  {"P0362","Bobine d'allumage L, primaire/secondaire - panne du circuit"},
  {"P0363","Ratés d'allumage détectés - alimentation en carburant désactivée"},
  {"P0364","Bobine d'allumage L, primaire/secondaire - panne du circuit"},
  {"P0365","Capteur d'arbre à cames B, ligne 1 - panne du circuit"},
  {"P0366","Capteur d'arbre à cames B, ligne 1 - plage de mesure/performance du circuit"},
  {"P0367","Capteur d'arbre à cames B, ligne 1 - signal d'entrée du circuit trop bas"},
  {"P0368","Capteur d'arbre à cames B, ligne 1 - signal d'entrée du circuit trop haut"},
  {"P0369","Capteur d'arbre à cames B, ligne 1 - circuit intermittent"},
  {"P0370","Référence de calage, signal haute résolution A - panne"},
  {"P0371","Référence de calage, signal haute résolution A - trop de signaux"},
  {"P0372","Référence de calage, signal haute résolution A - trop peu de signaux"},
  {"P0373","Référence de calage, signal haute résolution A - signaux irréguliers intermittents"},
  {"P0374","Référence de calage, signal haute résolution A - pas de signaux"},
  {"P0375","Référence de calage, signal haute résolution B - panne"},
  {"P0376","Référence de calage, signal haute résolution B - trop de signaux"},
  {"P0377","Référence de calage, signal haute résolution B - trop peu de signaux"},
  {"P0378","Référence de calage, signal haute résolution B - signaux irréguliers intermittents"},
  {"P0379","Référence de calage, signal haute résolution B - pas de signaux"},
  {"P0380","Bougies de préchauffage, circuit A - panne"},
  {"P0381","Lampe témoin bougies de préchauffage - panne du circuit"},
  {"P0382","Bougies de préchauffage, circuit B - panne"},
  {"P0383","Lampe témoin bougies de préchauffage"},
  {"P0384","Bougies de préchauffage, circuit B"},
  {"P0385","Capteur de vilebrequin B - panne du circuit"},
  {"P0386","Capteur de vilebrequin B - problème de performance/de limites"},
  {"P0387","Capteur de vilebrequin B - valeur d'entrée trop basse"},
  {"P0388","Capteur de vilebrequin B - valeur d'entrée trop haute"},
  {"P0389","Capteur de vilebrequin B - circuit intermittent"},
  {"P0390","Capteur d'arbre à cames B, ligne 2 - panne du circuit"},
  {"P0391","Capteur d'arbre à cames B, ligne 2 - plage de mesure/performance du circuit"},
  {"P0392","Capteur d'arbre à cames B, ligne 2 - signal d'entrée du circuit trop bas"},
  {"P0393","Capteur d'arbre à cames B, ligne 2 - signal d'entrée du circuit trop haut"},
  {"P0394","Capteur d'arbre à cames B, ligne 2 - circuit intermittent"},
  {"P0395","Capteur d'arbre à cames B, ligne 2 - signal d'entrée du circuit trop haut"},
  {"P0396","Capteur d'arbre à cames B, ligne 2 - circuit intermittent"},
  {"P0400","Système EGR - problème de débit"},
  {"P0401","Système EGR - débit insuffisant détecté"},
  {"P0402","Système EGR - débit excessif détecté"},
  {"P0403","Recyclage des gaz d'échappement - panne du circuit"},
  {"P0404","Système EGR - problème de performance/de limites"},
  {"P0405","Capteur de position de la valve EGR A - valeur d'entrée trop basse"},
  {"P0406","Capteur de position de la valve EGR A - valeur d'entrée trop haute"},
  {"P0407","Capteur de position de la valve EGR B - valeur d'entrée trop basse"},
  {"P0408","Capteur de position de la valve EGR B - valeur d'entrée trop haute"},
  {"P0409","Capteur EGR A - panne du circuit"},
  {"P0410","Système d'injection d'air secondaire - panne"},
  {"P0411","Système d'injection d'air secondaire - débit incorrect détecté"},
  {"P0412","Électrovanne d'injection d'air secondaire A - panne du circuit"},
  {"P0413","Électrovanne d'injection d'air secondaire A - circuit ouvert"},
  {"P0414","Électrovanne d'injection d'air secondaire A - courtcircuit"},
  {"P0415","Électrovanne d'injection d'air secondaire B - panne du circuit"},
  {"P0416","Électrovanne d'injection d'air secondaire B - circuit ouvert"},
  {"P0417","Electrovanne d'injection d'air secondaire B - courtcircuit"},
  {"P0418","Relais de la pompe d'injection d'air secondaire A - panne du circuit"},
  {"P0419","Relais de la pompe d'injection d'air secondaire B - panne du circuit"},
  {"P0420","Circuit de catalyseur, ligne 1 - rendement inférieur au seuil"},
  {"P0421","Pré-catalyseur, ligne 1 - rendement inférieur au seuil"},
  {"P0422","Catalyseur principal, ligne 1 - rendement inférieur au seuil"},
  {"P0423","Catalyseur chauffé, ligne 1 - rendement inférieur au seuil"},
  {"P0424","Catalyseur chauffé, ligne 1 - température inférieur au seuil"},
  {"P0425","Sonde de température du catalyseur 1, ligne 1"},
  {"P0426","Sonde de température du catalyseur 1, ligne 1 - plage de mesure/performance"},
  {"P0427","Sonde de température du catalyseur 1, ligne 1 - valeur d'entrée trop basse"},
  {"P0428","Sonde de température du catalyseur 1, ligne 1 - valeur d'entrée trop haute"},
  {"P0429","Chauffage catalyseur, ligne 1 - panne du circuit de commande"},
  {"P0430","Circuit de catalyseur, ligne 2 - rendement inférieur au seuil"},
  {"P0431","Pré-catalyseur, ligne 2 - rendement inférieur au seuil"},
  {"P0432","Catalyseur principal, ligne 2 - rendement inférieur au seuil"},
  {"P0433","Catalyseur chauffé, ligne 2 - rendement inférieur au seuil"},
  {"P0434","Catalyseur chauffé, ligne 2 - température inférieur au seuil"},
  {"P0435","Sonde de température du catalyseur 1, ligne 2"},
  {"P0436","Sonde de température du catalyseur 1, ligne 2 - plage de mesure/performance"},
  {"P0437","Sonde de température du catalyseur 1, ligne 2 - valeur d'entrée trop basse"},
  {"P0438","Sonde de température du catalyseur 1, ligne 2 - valeur d'entrée trop haute"},
  {"P0439","Chauffage catalyseur, ligne 2 - panne du circuit"},
  {"P0440","Système de purge canister - panne"},
  {"P0441","Système de purge canister - débit incorrect détecté"},
  {"P0442","Système de purge canister - petite fuite détectée"},
  {"P0443","Électrovanne de purge canister - panne du circuit"},
  {"P0444","Électrovanne de purge canister - circuit ouvert"},
  {"P0445","Électrovanne de purge canister - courtcircuit"},
  {"P0446","Système de purge canister, commande de ventilation - panne du circuit"},
  {"P0447","Système de purge canister, commande de ventilation - circuit ouvert"},
  {"P0448","Système de purge canister, commande de ventilation - court-circuit"},
  {"P0449","Système de purge canister, soupape de ventilation - panne du circuit"},
  {"P0460","Sonde de niveau du réservoir de carburant - panne du circuit"},
  {"P0461","Sonde de niveau du réservoir de carburant - problème de performance/de limites"},
  {"P0462","Sonde de niveau du réservoir de carburant - valeur d'entrée trop basse"},
  {"P0463","Sonde de niveau du réservoir de carburant - valeur d'entrée trop haute"},
  {"P0464","Sonde de niveau du réservoir de carburant - circuit intermittent"},
  {"P0465","Capteur de flux de purge canister - panne du circuit"},
  {"P0466","Capteur de flux de purge canister - problème de performance/de limites"},
  {"P0467","Capteur de flux de purge canister - valeur d'entrée trop basse"},
  {"P0468","Capteur de flux de purge canister - valeur d'entrée trop haute"},
  {"P0469","Capteur de flux de purge canister - circuit intermittent"},
  {"P0470","Capteur de pression des gaz d'échappement - panne du circuit"},
  {"P0471","Capteur de pression des gaz d'échappement - problème de performance/de limites"},
  {"P0472","Capteur de pression des gaz d'échappement - valeur d'entrée trop basse"},
  {"P0473","Capteur de pression des gaz d'échappement - valeur d'entrée trop haute"},
  {"P0474","Capteur de pression des gaz d'échappement - circuit intermittent"},
  {"P0475","Électrovanne de commande de pression des gaz d'échappement-panne du circuit"},
  {"P0476","Électrovanne de commande de pression des gaz d'échappement-problème de performance/de limites"},
  {"P0477","Électrovanne de commande de pression des gaz d'échappement - valeur d'entrée trop basse"},
  {"P0478","Électrovanne de commande de pression des gaz d'échappement - valeur d'entrée trop haute"},
  {"P0479","Électrovanne de commande de pression des gaz d'échappement- circuit intermittent"},
  {"P0480","Motoventilateur de refroidissement 1 - panne du circuit"},
  {"P0481","Motoventilateur de refroidissement 2 - panne du circuit"},
  {"P0482","Motoventilateur de refroidissement 3 - panne du circuit"},
  {"P0483","Motoventilateur de refroidissement, contrôle de rationalité - panne"},
  {"P0484","Motoventilateur de refroidissement - surcharge du circuit"},
  {"P0485","Motoventilateur de refroidissement, alimentation/masse - panne du circuit"},
  {"P0486","Capteur de position de la valve EGR B - panne du circuit"},
  {"P0487","Système EGR, commande de position du papillon - panne du circuit"},
  {"P0488","Système EGR, commande de position du papillon - plage de mesure/performance"},
  {"P0489","Système EGR - circuit trop bas"},
  {"P0490","Système EGR - circuit trop haut"},
  {"P0491","Système d'injection d'air secondaire, ligne 1 - panne"},
  {"P0492","Système d'injection d'air secondaire, ligne 2 - panne"},
  {"P0493","Vitesse motoventilateur de refroidissement moteur - excessive"},
  {"P0494","Vitesse motoventilateur de refroidissement moteur - basse"},
  {"P0495","Vitesse motoventilateur de refroidissement moteur - haute"},
  {"P0496","Système de purge canister - flux de purge élevé"},
  {"P0497","Système de purge canister - flux de purge faible"},
  {"P0498","Système de purge canister, commande de ventilation - circuit trop bas"},
  {"P0499","Système de purge canister, commande de ventilation - circuit trop haut"},
  {"P0500","Capteur de vitesse du véhicule - panne du circuit"},
  {"P0501","Capteur de vitesse du véhicule - problème de performance/de limites"},
  {"P0502","Capteur de vitesse du véhicule - valeur d'entrée trop basse"},
  {"P0503","Capteur de vitesse du véhicule - valeur d'entrée intermittente/irrégulière/trop haute"},
  {"P0504","Contacteur de freinage - corrélation A/B"},
  {"P0505","Commande du ralenti - panne"},
  {"P0506","Commande du ralenti - régime plus lent que prévu"},
  {"P0507","Commande du ralenti - régime plus rapide que prévu"},
  {"P0508","Commande d'air au ralenti - circuit trop bas"},
  {"P0509","Commande d'air au ralenti - circuit trop haut"},
  {"P0510","Contacteur de position fermée de papillon - panne du circuit"},
  {"P0511","Commande d'air au ralenti - panne du circuit"},
  {"P0512","Circuit de commande du démarreur - panne"},
  {"P0513","Clé de l'antidémarrage incorrecte"},
  {"P0514","Sonde de température batterie - plage de mesure/performance du circuit"},
  {"P0515","Sonde de température batterie - panne du circuit"},
  {"P0516","Sonde de température batterie - circuit trop bas"},
  {"P0517","Sonde de température batterie - circuit trop haut"},
  {"P0518","Commande d'air au ralenti - circuit intermittent"},
  {"P0519","Commande d'air au ralenti - performance circuit"},
  {"P0520","Capteur de pression/pressostat d'huile moteur - panne du circuit"},
  {"P0521","Capteur de pression/pressostat d'huile moteur - problème de performance/de limites"},
  {"P0522","Capteur de pression/pressostat d'huile moteur - basse tension"},
  {"P0523","Capteur de pression/pressostat d'huile moteur - haute tension"},
  {"P0524","Pression d'huile moteur trop basse"},
  {"P0525","Régulateur de vitesse, commande d'actuateur - plage de mesure/performance du circuit"},
  {"P0526","Capteur de vitesse du motoventilateur de refroidissement moteur- panne du circuit"},
  {"P0527","Capteur de vitesse du motoventilateur de refroidissement moteur - plage de mesure/performance du circuit"},
  {"P0528","Capteur de vitesse du motoventilateur de refroidissement moteur - aucun signal"},
  {"P0529","Capteur de vitesse du motoventilateur de refroidissement moteur - circuit intermittent"},
  {"P0530","Capteur de pression du réfrigérant de la climatisation - panne du circuit"},
  {"P0531","Capteur de pression du réfrigérant de la climatisation - problème de performance/de limites"},
  {"P0532","Capteur de pression du réfrigérant de la climatisation - valeur d'entrée trop basse"},
  {"P0533","Capteur de pression du réfrigérant de la climatisation - valeur d'entrée trop haute"},
  {"P0534","Perte de charge du réfrigérant de la climatisation"},
  {"P0535","Sonde de température de l'évaporateur climatisation - panne du circuit"},
  {"P0536","Sonde de température de l'évaporateur climatisation - plage de mesure/performance du circuit"},
  {"P0537","Sonde de température de l'évaporateur climatisation - circuit trop bas"},
  {"P0538","Sonde de température de l'évaporateur climatisation - circuit trop haut"},
  {"P0539","Sonde de température de l'évaporateur climatisation - circuit intermittent"},
  {"P0540","Chauffage d'air d'admission A - panne du circuit"},
  {"P0541","Chauffage d'air d'admission A - circuit trop bas"},
  {"P0542","Chauffage d'air d'admission A - circuit trop haut"},
  {"P0543","Chauffage d'air d'admission A - circuit ouvert"},
  {"P0544","Sonde de température EGR, ligne 1 - panne du circuit"},
  {"P0545","Sonde de température EGR, ligne 1 - valeur d'entrée trop basse"},
  {"P0546","Sonde de température EGR, ligne 1 - valeur d'entrée trop haute"},
  {"P0547","Sonde de température des gaz d'échappement, ligne 2 capteur 1 - panne du circuit"},
  {"P0548","Sonde de température des gaz d'échappement, ligne 2 capteur 1 - circuit trop bas"},
  {"P0549","Sonde de température des gaz d'échappement, ligne 2 capteur 1 - circuit trop haut"},
  {"P0550","Capteur/pressostat de direction assistée - panne du circuit"},
  {"P0551","Capteur/pressostat de direction assistée - problème de performance/de limites"},
  {"P0552","Capteur/pressostat de direction assistée - valeur d'entrée trop basse"},
  {"P0553","Capteur/pressostat de direction assistée - valeur d'entrée trop haute"},
  {"P0554","Capteur/pressostat de direction assistée - circuit intermittent"},
  {"P0555","Capteur de pression du servofrein - panne du circuit"},
  {"P0556","Capteur de pression du servofrein - plage de mesure/performance du circuit"},
  {"P0557","Capteur de pression du servofrein - signal d'entrée du circuit trop bas"},
  {"P0558","Capteur de pression du servofrein - signal d'entrée du circuit trop haut"},
  {"P0559","Capteur de pression du servofrein - circuit intermittent"},
  {"P0560","Tension du système - panne"},
  {"P0561","Tension du système - instable"},
  {"P0562","Tension du système - basse"},
  {"P0563","Tension du système - haute"},
  {"P0564","Régulateur de vitesse, signal d'entrée A du contacteur multifonction - panne du circuit"},
  {"P0565","Commutateur principal du régulateur de vitesse, signal de marche (ON) - panne"},
  {"P0566","Commutateur principal du régulateur de vitesse, signal d'arrêt (OFF) - panne"},
  {"P0567","Sélecteur de reprise du régulateur de vitesse - panne"},
  {"P0568","Commutateur principal du régulateur de vitesse, signal SET - panne"},
  {"P0569","Sélecteur du régulateur de vitesse, signal COAST - panne"},
  {"P0570","Régulateur de vitesse, signal du capteur de position de la pédale - panne"},
  {"P0571","Contacteur de régulateur de vitesse/de freinage A - panne du circuit"},
  {"P0572","Contacteur de régulateur de vitesse/de freinage A - circuit trop bas"},
  {"P0573","Contacteur de régulateur de vitesse/de freinage A - circuit trop haut"},
  {"P0574","Régulateur de vitesse - vitesse trop haute"},
  {"P0575","Régulateur de vitesse - panne du circuit d'entrée"},
  {"P0576","Régulateur de vitesse - circuit d'entrée trop faible"},
  {"P0577","Régulateur de vitesse - circuit d'entrée trop fort"},
  {"P0578","Régulateur de vitesse, signal d'entrée A du contacteur multifonction - circuit bloqué"},
  {"P0579","Régulateur de vitesse, signal d'entrée A du contacteur multifonction - plage de mesure/performance du circuit"},
  {"P0580","Régulateur de vitesse, signal d'entrée A du contacteur multifonction - circuit trop bas"},
  {"P0581","Régulateur de vitesse, signal d'entrée A du contacteur multifonction - circuit trop haut"},
  {"P0582","Régulateur de vitesse, commande par dépression - circuit ouvert"},
  {"P0583","Régulateur de vitesse, commande par dépression - circuit trop bas"},
  {"P0584","Régulateur de vitesse, commande par dépression - circuit trop haut"},
  {"P0585","Régulateur de vitesse, signal d'entrée A/B du contacteur multifonction - corrélation"},
  {"P0586","Régulateur de vitesse, commande de ventilation - circuit ouvert"},
  {"P0587","Régulateur de vitesse, commande de ventilation - circuit trop bas"},
  {"P0588","Régulateur de vitesse, commande de ventilation - circuit trop haut"},
  {"P0589","Régulateur de vitesse, signal d'entrée B du contacteur multifonction - panne du circuit"},
  {"P0590","Régulateur de vitesse, signal d'entrée B du contacteur multifonction - circuit bloqué"},
  {"P0591","Régulateur de vitesse, signal d'entrée B du contacteur multifonction - plage de mesure/performance du circuit"},
  {"P0592","Régulateur de vitesse, signal d'entrée B du contacteur multifonction - circuit trop bas"},
  {"P0593","Régulateur de vitesse, signal d'entrée B du contacteur multifonction - circuit trop haut"},
  {"P0594","Régulateur de vitesse, commande d'actuateur - circuit ouvert"},
  {"P0595","Régulateur de vitesse, commande d'actuateur - circuit trop bas"},
  {"P0596","Régulateur de vitesse, commande d'actuateur - circuit trop haut"},
  {"P0597","Dispositif de commande du chauffage à thermostat - circuit ouvert"},
  {"P0598","Dispositif de commande du chauffage à thermostat - circuit trop bas"},
  {"P0599","Dispositif de commande du chauffage à thermostat - circuit trop haut"},
  {"P0600","Bus de données CAN - panne"},
  {"P0601","Calculateur de gestion moteur - erreur du total de contrôle de mémoire"},
  {"P0602","Calculateur de gestion moteur - erreur de programmation"},
  {"P0603","Calculateur de gestion moteur - erreur KAM"},
  {"P0604","Calculateur de gestion moteur - erreur RAM"},
  {"P0605","Calculateur de gestion moteur - erreur ROM"},
  {"P0606","Calculateur de gestion moteur/calculateur combiné moteur-transmission"},
  {"P0607","Calculateur/boîtier électronique - problème de performance - panne de processeur"},
  {"P0608","Calculateur de gestion moteur, signal de sortie du capteur de vitesse du véhicule A - panne"},
  {"P0609","Calculateur de gestion moteur, signal de sortie du capteur de vitesse du véhicule B - panne"},
  {"P0610","Calculateur/boîtier électronique - erreur options véhicule"},
  {"P0611","Boîtier électronique d'injecteur de carburant - problème de performance"},
  {"P0612","Boîtier électronique d'injecteur carburant - circuit du relais de commande"},
  {"P0613","Calculateur de transmission - erreur de processeur"},
  {"P0614","Calculateur de gestion moteur/calculateur de transmission - désaccord"},
  {"P0615","Relais du démarreur - panne du circuit de gestion moteur"},
  {"P0616","Relais du démarreur - circuit trop bas de gestion moteur"},
  {"P0617","Relais du démarreur - circuit trop haut calculateur de gestion moteur"},
  {"P0618","Boîtier électronique carburant alternatif - erreur KAM"},
  {"P0619","Boîtier électronique carburant alternatif - erreur RAM/ROM"},
  {"P0620","Alternateur, commande - panne du circuit"},
  {"P0621","Alternateur - lampe témoin de charge L - panne du circuit"},
  {"P0622","Alternateur - lampe témoin de charge F - panne du circuit"},
  {"P0623","Lampe témoin de charge - panne du circuit"},
  {"P0624","Lampe témoin trappe du réservoir de carburant - panne du circuit"},
  {"P0625","Borne champ du générateur - circuit trop bas"},
  {"P0626","Borne champ du générateur - circuit trop haut"},
  {"P0627","Commande de pompe à carburant A - circuit ouvert"},
  {"P0628","Commande de pompe à carburant A - circuit trop bas"},
  {"P0629","Commande de pompe à carburant A - circuit trop haut"},
  {"P0630","VIN non programmé ou désaccord d'identification - calculateur de gestion moteur/calculateur transmission/gestion moteur"},
  {"P0631","VIN non programmé ou désaccord d'identification - calculateur de transmission"},
  {"P0632","Compteur kilométrique non programmé - calculateur de gestion moteur/calculateur transmission/gestion moteur"},
  {"P0633","Clé de l'antidémarrage non programmée - calculateur de gestion moteur/calculateur transmission/gestion moteur"},
  {"P0634","Calculateur transmission/gestion moteur/calculateur de gestion moteur/calculateur de transmission - température interne trop haute"},
  {"P0635","Commande direction assistée - panne du circuit"},
  {"P0636","Commande direction assistée - circuit trop bas"},
  {"P0637","Commande direction assistée - circuit trop haut"},
  {"P0638","Commande d'actuateur de papillon, ligne 1 - problème de performance/de limites"},
  {"P0639","Commande d'actuateur de papillon, ligne 2 - plage mesure/performance"},
  {"P0640","Commande de chauffage d'air d'admission - panne du circuit"},
  {"P0641","Tension de référence du capteur A - circuit ouvert"},
  {"P0642","Calculateur de gestion moteur, anti-c1iquetis - défectueux"},
  {"P0643","Tension de référence du capteur A - circuit trop haut"},
  {"P0644","Affichage conducteur, communication de séries de données - panne du circuit"},
  {"P0645","Relais d'embrayage du compresseur de climatisation"},
  {"P0646","Relais embrayage du compresseur de climatisation - circuit trop bas"},
  {"P0647","Relais embrayage du compresseur de climatisation - circuit trop haut"},
  {"P0648","Lampe témoin antidémarrage - panne du circuit"},
  {"P0649","Lampe témoin régulateur de vitesse - circuit moteur"},
  {"P0650","Lampe témoin d'affichage des défauts - panne du circuit"},
  {"P0651","Tension de référence du capteur B - circuit ouvert"},
  {"P0652","Tension de référence du capteur B - circuit trop bas"},
  {"P0653","Tension de référence du capteur B - circuit trop haut"},
  {"P0654","Régime moteur, sortie - panne du circuit"},
  {"P0655","Signal de sortie du témoin de surchauffe du moteur - panne du circuit"},
  {"P0656","Signal de sortie de niveau de carburant - panne du circuit"},
  {"P0657","Tension d'alimentation d'actuateur - circuit ouvert"},
  {"P0658","Tension d'alimentation d'actuateur - circuit trop bas"},
  {"P0659","Tension d'alimentation d'actuateur - circuit trop haut"},
  {"P0660","Électrovanne d'admission d'air dans le collecteur, ligne 1 - circuit ouvert"},
  {"P0661","Électrovanne d'admission d'air dans le collecteur, ligne 1 - circuit trop bas"},
  {"P0662","Électrovanne d'admission d'air dans le collecteur, ligne 1 - circuit trop haut"},
  {"P0663","Électrovanne d'admission d'air dans le collecteur, ligne 2 - circuit ouvert"},
  {"P0664","Électrovanne d'admission d'air dans le collecteur, ligne 2 - circuit trop bas"},
  {"P0665","Électrovanne d'admission d'air dans le collecteur, ligne 2 - circuit trop haut"},
  {"P0666","Sonde de température interne calculateur combiné moteur-transmission/ calculateur de gestion moteur/calculateur de transmission - panne du circuit"},
  {"P0667","Sonde de température interne calculateur combiné moteur-transmission/ calculateur de gestion moteur/calculateur de transmission - plage de mesure/performance"},
  {"P0668","Sonde de température interne calculateur combiné moteur-transmission/ calculateur de gestion moteur/calculateur de transmission - circuit trop bas"},
  {"P0669","Sonde de température interne calculateur combiné moteur-transmission/calculateur de gestion moteur/calculateur de transmission - circuit trop haut"},
  {"P0670","Boîtier électronique des bougies de préchauffage - panne du circuit"},
  {"P0671","Bougies de préchauffage, cylindre 1 - panne du circuit"},
  {"P0672","Bougies de préchauffage, cylindre 2 - panne du circuit"},
  {"P0673","Bougies de préchauffage, cylindre 3 - panne du circuit"},
  {"P0674","Bougies de préchauffage, cylindre 4 - panne du circuit"},
  {"P0675","Bougies de préchauffage, cylindre 5 - panne du circuit"},
  {"P0676","Bougies de préchauffage, cylindre 6 - panne du circuit"},
  {"P0677","Bougies de préchauffage, cylindre 7 - panne du circuit"},
  {"P0678","Bougies de préchauffage, cylindre 8 - panne du circuit"},
  {"P0679","Bougies de préchauffage, cylindre 9 - panne du circuit"},
  {"P0680","Bougies de préchauffage, cylindre 10 - panne du circuit"},
  {"P0681","Bougies de préchauffage, cylindre 11 - panne du circuit"},
  {"P0682","Bougies de préchauffage, cylindre 12 - panne du circuit"},
  {"P0683","Communication boîtier électronique des bougies de préchauffage/ calculateur de gestion moteur/calculateur combiné moteur-transmission - panne"},
  {"P0684","Communication boîtier électronique des bougies de préchauffage/calculateur de gestion moteur/calculateur combiné moteur-transmission - plage de mesure/performance"},
  {"P0685","Relais d'alimentation électrique calculateur de gestion moteur/calculateur combiné transmission-gestion moteur - circuit ouvert"},
  {"P0686","Relais d'alimentation électrique calculateur de gestion moteur/calculateur combiné transmission-gestion moteur - circuit trop bas"},
  {"P0687","Relais de gestion du moteur - court-circuit sur la masse"},
  {"P0688","Relais de gestion du moteur - court-circuit sur l'alimentation"},
  {"P0689","Relais d'alimentation électrique calculateur de gestion moteur/calculateur combiné transmission-gestion moteur - circuit de sondage trop faible"},
  {"P0690","Relais d'alimentation électrique calculateur de gestion moteur/calculateur combiné transmission-gestion moteur - circuit de sondage trop fort"},
  {"P0691","Motoventilateur de refroidissement 1 - circuit trop bas"},
  {"P0692","Motoventilateur de refroidissement 1 - circuit trop haut"},
  {"P0693","Motoventilateur de refroidissement 2 - circuit trop bas"},
  {"P0694","Motoventilateur de refroidissement 2 - circuit trop haut"},
  {"P0695","Motoventilateur de refroidissement 3 - circuit trop bas"},
  {"P0696","Motoventilateur de refroidissement 3 - circuit trop haut"},
  {"P0697","Tension de référence du capteur C - circuit ouvert"},
  {"P0698","Tension de référence du capteur C - circuit trop bas"},
  {"P0699","Tension de référence du capteur C - circuit trop haut"},
  {"P0700","Commande de la transmission - panne"},
  {"P0701","Commande de la transmission - problème de performance/de limites"},
  {"P0702","Commande de la transmission - électrique"},
  {"P0703","Convertisseur de couple/contacteur de freinage B - panne du circuit"},
  {"P0704","Contacteur de position de la pédale d'embrayage - panne du circuit"},
  {"P0705","Contacteur/capteur de position du levier de vitesse, signal d'entrée PIRMIDL - panne du circuit"},
  {"P0706","Contacteur/capteur de position du levier de vitesse - problème de performance/de limites"},
  {"P0707","Contacteur/capteur de position du levier de vitesse - valeur d'entrée trop basse"},
  {"P0708","Contacteur/capteur de position du levier de vitesse - valeur d'entrée trop haute"},
  {"P0709","Contacteur/capteur de position du levier de vitesse - circuit intermittent"},
  {"P0710","Sonde de température d'huile de transmission - panne du circuit"},
  {"P0711","Sonde de température d'huile de transmission - problème de performance/de limites"},
  {"P0712","Sonde de température d'huile de transmission - valeur d'entrée trop basse"},
  {"P0713","Sonde de température d'huile de transmission - valeur d'entrée trop haute"},
  {"P0714","Sonde de température d'huile de transmission - circuit intermittent"},
  {"P0715","Capteur de vitesse d'arbre de turbine - panne du circuit"},
  {"P0716","Capteur de vitesse d'arbre de turbine - problème de performance/de limites"},
  {"P0717","Capteur de vitesse d'arbre de turbine - aucun signal"},
  {"P0718","Capteur de vitesse d'arbre de turbine - circuit intermittent"},
  {"P0719","Convertisseur de couple/contacteur de freinage B - circuit trop bas"},
  {"P0720","Capteur de vitesse du véhicule - panne du circuit"},
  {"P0721","Capteur de vitesse du véhicule - problème de performance/de limites"},
  {"P0722","Capteur de vitesse du véhicule - aucun signal"},
  {"P0723","Capteur de vitesse du véhicule - circuit intermittent"},
  {"P0724","Convertisseur de couple/contacteur de freinage B - circuit trop haut"},
  {"P0725","Entrée du régime moteur - panne du circuit"},
  {"P0726","Entrée du régime moteur - problème de performance/de limites"},
  {"P0727","Entrée du régime moteur - aucun signal"},
  {"P0728","Entrée du régime moteur - circuit intermittent"},
  {"P0729","Vitesse 6 - rapport incorrect"},
  {"P0730","Rapport de démultiplication de la vitesse incorrect"},
  {"P0731","Vitesse 1 - ratio incorrect"},
  {"P0732","Vitesse 2 - ratio incorrect"},
  {"P0733","Vitesse 3 - ratio incorrect"},
  {"P0734","Vitesse 4 - ratio incorrect"},
  {"P0735","Vitesse 5 - ratio incorrect"},
  {"P0736","Marche arrière - rapport incorrect"},
  {"P0737","Régime moteur calculateur de transmission - circuit de sortie"},
  {"P0738","Régime moteur calculateur de transmission - circuit de sortie faible"},
  {"P0739","Référence de temps à haute résolution signal B - aucun pulse"},
  {"P0740","Electrovanne du convertisseur de couple - panne du circuit"},
  {"P0741","Electrovanne du convertisseur de couple - performance ou blocage"},
  {"P0742","Electrovanne du convertisseur de couple - blocage"},
  {"P0743","Electrovanne du convertisseur de couple - électrique"},
  {"P0744","Electrovanne du convertisseur de couple - circuit intermittent"},
  {"P0745","Electrovanne du convertisseur de couple - panne du circuit"},
  {"P0746","Electrovanne du convertisseur de couple - performance ou blocage"},
  {"P0747","Electrovanne du convertisseur de couple - blocage"},
  {"P0748","Electrovanne du convertisseur de couple - électrique"},
  {"P0749","Electrovanne du convertisseur de couple - circuit intermittent"},
  {"P0750","Electrovanne de changement de vitesse A - panne du circuit"},
  {"P0751","Electrovanne de changement de vitesse A - performance ou blocage"},
  {"P0752","Electrovanne de changement de vitesse A - blocage"},
  {"P0753","Electrovanne de changement de vitesse A - électrique"},
  {"P0754","Electrovanne de changement de vitesse A - circuit intermittent"},
  {"P0755","Electrovanne de changement de vitesse B - panne du circuit"},
  {"P0756","Electrovanne de changement de vitesse B - performance ou blocage"},
  {"P0757","Electrovanne de changement de vitesse B - blocage"},
  {"P0758","Electrovanne de changement de vitesse B - électrique"},
  {"P0759","Electrovanne de changement de vitesse B - circuit intermittent"},
  {"P0760","Electrovanne de changement de vitesse C - panne du circuit"},
  {"P0761","Electrovanne de changement de vitesse C - performance ou blocage"},
  {"P0762","Electrovanne de changement de vitesse C - blocage"},
  {"P0763","Electrovanne de changement de vitesse C - électrique"},
  {"P0764","Electrovanne de changement de vitesse C - circuit intermittent"},
  {"P0765","Electrovanne de changement de vitesse D - panne du circuit"},
  {"P0766","Electrovanne de changement de vitesse D - performance ou blocage"},
  {"P0767","Electrovanne de changement de vitesse D - blocage"},
  {"P0768","Electrovanne de changement de vitesse D - électrique"},
  {"P0769","Electrovanne de changement de vitesse D - circuit intermittent"},
  {"P0770","Electrovanne de changement de vitesse E - panne du circuit"},
  {"P0771","Electrovanne de changement de vitesse E - performance ou blocage"},
  {"P0772","Electrovanne de changement de vitesse E - blocage"},
  {"P0773","Electrovanne de changement de vitesse E - électrique"},
  {"P0774","Electrovanne de changement de vitesse E - circuit intermittent"},
  {"P0775","Electrovanne de commande de pression B - panne"},
  {"P0776","Electrovanne de commande de pression B - performance ou blocage"},
  {"P0777","Electrovanne de commande de pression B - blocage"},
  {"P0778","Electrovanne de commande de pression B - problème électrique"},
  {"P0779","Electrovanne de commande de pression B - circuit intermittent"},
  {"P0780","Sélection de la vitesse - problème de changement de vitesse"},
  {"P0781","Sélection de la vitesse, 1-2 - problème de changement de vitesse"},
  {"P0782","Sélection de la vitesse, 2-3 - problème de changement de vitesse"},
  {"P0783","Sélection de la vitesse, 3-4 - problème de changement de vitesse"},
  {"P0784","Sélection de la vitesse, 4-5 - problème de changement de vitesse"},
  {"P0785","Electrovanne de changement de vitesse A - panne du circuit"},
  {"P0786","Electrovanne de changement de vitesse A - problème de performance/de limites"},
  {"P0787","Electrovanne de changement de vitesse A - basse"},
  {"P0788","Electrovanne de changement de vitesse A - haute"},
  {"P0789","Electrovanne de changement de vitesse A - intermittent"},
  {"P0790","Commutateur sélection de mode de transmission - panne du circuit"},
  {"P0791","Capteur de vitesse de l'arbre intermédiaire de transmission - panne du cicuit"},
  {"P0792","Capteur de vitesse de l'arbre intermédiaire de transmission - problème de mesure/performance"},
  {"P0793","Capteur de vitesse de l'arbre intermédiaire de transmission - aucun signal"},
  {"P0794","Capteur de vitesse de l'arbre intermédiaire de transmission - panne intermittente du circuit"},
  {"P0795","Electrovanne de pression d'huile de transmission C - panne du circuit"},
  {"P0796","Electrovanne de pression d'huile de transmission C - performance ou blocage"},
  {"P0797","Electrovanne de pression d'huile de transmission C - blocage"},
  {"P0798","Electrovanne de pression d'huile de transmission C - problème électrique"},
  {"P0799","Electrovanne de pression d'huile de transmission C - panne intermittente du circuit"},
  {"P0800","Système de commande de la boîte de transfert, commande de lampe témoin d'affichage des défauts - panne"},
  {"P0801","Circuit de blocage de marche arrière - panne"},
  {"P0802","Commande de la transmission, commande de lampe témoin d'affichage des défauts - circuit ouvert"},
  {"P0803","Electrovanne de montée des vitesse 1-4 (saut de vitesses) - panne du circuit"},
  {"P0804","Lampe témoin de montée des vitesses 1-4 (saut de vitesses) - panne de circuit"},
  {"P0805","Capteur de la position de la pédale d'embrayage - panne du circuit"},
  {"P0806","Capteur de la position de la pédale d'embrayage - problème de mesure/performance"},
  {"P0807","Capteur de la position de la pédale d'embrayage - valeur d'entrée trop basse"},
  {"P0808","Capteur de la position de la pédale d'embrayage - valeur d'entrée trop élevée"},
  {"P0809","Capteur de la position de la pédale d'embrayage - panne intermittente du circuit"},
  {"P0810","Erreur de commande de position de la pédale d'embrayage"},
  {"P0811","Patinage excessif de l'embrayage"},
  {"P0812","Marche arrière - panne du circuit d'entrée"},
  {"P0813","Marche arrière - panne du circuit de sortie"},
  {"P0814","Affichage de position du levier de vitesses - panne du circuit"},
  {"P0815","Contacteur de montée des vitesses - panne du circuit"},
  {"P0816","Contacteur de descente des vitesses - panne du circuit"},
  {"P0817","Circuit de mise hors service du démarreur - panne"},
  {"P0818","Interrupteur de fil d'attaque - panne du circuit"},
  {"P0819","Corrélation entre le contacteur montée/descente des vitesses et la position du levier de vitesses"},
  {"P0820","Capteur de position X-Y du levier de vitesses - panne du circuit"},
  {"P0821","Capteur de position X du levier de vitesses - panne du circuit"},
  {"P0822","Capteur de position Y du levier de vitesses - panne du circuit"},
  {"P0823","Capteur de position X du levier de vitesses - circuit intermittent"},
  {"P0824","Capteur de position Y du levier de vitesses - circuit intermittent"},
  {"P0825","Commutateur à pression et tirage du levier de vitesses - panne du circuit"},
  {"P0826","Contacteur montée/descente des vitesses - circuit d'entrée"},
  {"P0827","Contacteur montée/descente des vitesses - circuit d'entrée trop faible"},
  {"P0828","Contacteur montée/descente des vitesses - circuit d'entrée trop fort"},
  {"P0829","Montée des vitesses 5-6"},
  {"P0830","Contacteur de position de la pédale d'embrayage A - panne du circuit"},
  {"P0831","Contacteur de position de la pédale d'embrayage A - valeur d'entrée trop basse"},
  {"P0832","Contacteur de position de la pédale d'embrayage A - valeur d'entrée trop haute"},
  {"P0833","Contacteur de position de la pédale d'embrayage B - panne du circuit"},
  {"P0834","Contacteur de position de la pédale d'embrayage B - valeur d'entrée trop basse"},
  {"P0835","Contacteur de position de la pédale d'embrayage B - valeur d'entrée trop haute"},
  {"P0836","Contacteur 4 roues motrices - panne du circuit"},
  {"P0837","Contacteur 4 roues motrices - problème de mesure/performance"},
  {"P0838","Contacteur 4 roues motrices - valeur d'entrée trop basse"},
  {"P0839","Contacteur 4 roues motrices - valeur d'entrée trop élevée"},
  {"P0840","Pressostat/capteur de pression d'huile de transmission A - panne du circuit"},
  {"P0841","Pressostat/capteur de pression d'huile de transmission A - problème de mesure/performance"},
  {"P0842","Pressostat/capteur de pression d'huile de transmission A - valeur d'entrée trop basse"},
  {"P0843","Pressostat/capteur de pression d'huile de transmission A - valeur d'entrée trop haute"},
  {"P0844","Pressostat/capteur de pression d'huile de transmission A - panne intermittente du circuit"},
  {"P0845","Pressostat/capteur de pression d'huile de transmission B - panne du circuit"},
  {"P0846","Pressostat/capteur de pression d'huile de transmission B - problème de mesure/performance"},
  {"P0847","Pressostat/capteur de pression d'huile de transmission B - valeur d'entrée trop basse"},
  {"P0848","Pressostat/capteur de pression d'huile de transmission B - valeur d'entrée trop haute"},
  {"P0849","Pressostat/capteur de pression d'huile de transmission B - panne intermittente du circuit"},
  {"P0850","Contacteur de position Neutre/parking - panne du circuit d'entrée"},
  {"P0851","Contacteur de position Neutre/Parking - circuit d'entrée trop faible"},
  {"P0852","Contacteur de position Neutre/Parking - circuit d'entrée trop fort"},
  {"P0853","Contacteur de position Drive - panne du circuit d'entrée"},
  {"P0854","Contacteur de position Drive - circuit d'entrée trop faible"},
  {"P0855","Contacteur de position Drive - circuit d'entrée trop fort"},
  {"P0856","Signal d'entrée anti-patinage - panne"},
  {"P0857","Signal d'entrée anti-patinage - prolème de mesure/performance"},
  {"P0858","Signal d'entrée anti-patinage - faible"},
  {"P0859","Signal d'entrée anti-patinage - haut"},
  {"P0860","Circuit de communication du boîtier électronique de changement de vitesses - panne"},
  {"P0861","Circuit de communication du boîtier électronique de changement de vitesses - valeur d'entrée trop basse"},
  {"P0862","Circuit de communication du boîtier électronique de changement de vitesses - valeur d'entrée trop haute"},
  {"P0863","Circuit de communication du calculateur de transmission - panne"},
  {"P0864","Circuit de communication du calculateur de transmission - problème de mesure/performance"},
  {"P0865","Circuit de communication du calculateur de transmission - valeur d'entrée trop basse"},
  {"P0866","Circuit de communication du calculateur de transmission - valeur d'entrée trop haute"},
  {"P0867","Capteur de pression d'huile de transmission"},
  {"P0868","Capteur de pression d'huile de transmission - faible"},
  {"P0869","Capteur de pression d'huile de transmission - haut"},
  {"P0870","Capteur de pression d'huile de transmission C - panne du circuit"},
  {"P0871","Capteur de pression d'huile de transmission C - plage de mesure/performance"},
  {"P0872","Capteur de pression d'huile de transmission C - circuit trop bas"},
  {"P0873","Capteur de pression d'huile de transmission C - circuit trop haut"},
  {"P0874","Capteur de pression d'huile de transmission C - panne intermittente du circuit"},
  {"P0875","Capteur de pression d'huile de transmission D - panne du circuit"},
  {"P0876","Capteur de pression d'huile de transmission D - plage de mesure/performance"},
  {"P0877","Capteur de pression d'huile de transmission D - circuit trop bas"},
  {"P0878","Capteur de pression d'huile de transmission D - circuit trop haut"},
  {"P0879","Capteur de pression d'huile de transmission D - panne intermittente du circuit"},
  {"P0880","Calculateur de transmission - panne du signal d'entrée de l'amimentation électrique"},
  {"P0881","Calculateur de transmission - plage de mesure/performance du signal d'entrée de l'amimentation électrique"},
  {"P0882","Calculateur de transmission - signal d'entrée de l'amimentation électrique trop faible"},
  {"P0883","Calculateur de transmission - signal d'entrée de l'amimentation électrique trop fort"},
  {"P0884","Calculateur de transmission - panne intermittente du signal d'entrée de l'amimentation électrique"},
  {"P0885","Relais d'alimentation électrique du calculateur de transmission - circuit de commande ouvert"},
  {"P0886","Relais d'alimentation électrique du calculateur de transmission - circuit de commande trop faible"},
  {"P0887","Relais d'alimentation électrique du calculateur de transmission - circuit de commande trop fort"},
  {"P0888","Relais d'alimentation électrique du calculateur de transmission - panne du circuit de sondage"},
  {"P0889","Relais d'alimentation électrique du calculateur de transmission - plage de mesure/performance du circuit de sondage"},
  {"P0890","Relais d'alimentation électrique du calculateur de transmission - circuit de sondage trop faible"},
  {"P0891","Relais d'alimentation électrique du calculateur de transmission - circuit de sondage trop fort"},
  {"P0892","Relais d'alimentation électrique du calculateur de transmission - panne intermittente du circuit de sondage"},
  {"P0893","Plusieurs vitesses engagées"},
  {"P0894","Patinage d'un composant de la transmission"},
  {"P0895","Durée de changement de vitesse trop courte"},
  {"P0896","Durée de changement de vitesse trop longue"},
  {"P0897","Huile de transmission dégradée"},
  {"P0898","Commande de transmission - commande de lampe témoin d'affichage des défauts - circuit trop bas"},
  {"P0899","Commande de transmission - commande de lampe témoin d'affichage des défauts - circuit trop haut"},
  {"P0900","Capteur d'embrayage - circuit ouvert"},
  {"P0901","Capteur d'embrayage - plage de msure/performance du circuit"},
  {"P0902","Capteur d'embrayage - circuit trop bas"},
  {"P0903","Capteur d'embrayage - circuit trop haut"},
  {"P0904","Circuit de sélection de coulisse de transmission - panne"},
  {"P0905","Circuit de sélection de coulisse de transmission - plage de mesure/performance"},
  {"P0906","Circuit de sélection de coulisse de transmission - basse"},
  {"P0907","Circuit de sélection de coulisse de transmission - haute"},
  {"P0908","Circuit de sélection de coulisse de transmission - panne intermittente du circuit"},
  {"P0909","Erreur de commande de sélection de coulisse de transmission"},
  {"P0910","Capteur de sélection de coulisse de transmission - circuit ouvert"},
  {"P0911","Capteur de sélection de coulisse de transmission - plage de mesure/performance"},
  {"P0912","Capteur de sélection de coulisse de transmission - circuit trop bas"},
  {"P0913","Capteur de sélection de coulisse de transmission - circuit trop haut"},
  {"P0914","Circuit de position de changement de vitesses - panne"},
  {"P0915","Circuit de position de changement de vitesses - plage de mesure/performance"},
  {"P0916","Circuit de position de changement de vitesses - basse"},
  {"P0917","Circuit de position de changement de vitesses - haute"},
  {"P0918","Circuit de position de changement de vitesses - panne intermittente"},
  {"P0919","Commande de position de changement de vitesses - erreur"},
  {"P0920","Capteur de changement de vitesses vers l'avant - circuit ouvert"},
  {"P0921","Capteur de changement de vitesses vers l'avant - plage de mesure/performance"},
  {"P0922","Capteur de changement de vitesses vers l'avant - circuit trop bas"},
  {"P0923","Capteur de changement de vitesses vers l'avant - circuit trop haut"},
  {"P0924","Capteur de changement de vitesses vers l'arrière - circuit ouvert"},
  {"P0925","Capteur de changement de vitesses vers l'arrière - plage de mesure/performance"},
  {"P0926","Capteur de changement de vitesses vers l'arrière - circuit trop bas"},
  {"P0927","Capteur de changement de vitesses vers l'arrière - circuit trop haut"},
  {"P0928","Electrovanne de blocage de changement de vitesse - circuit ouvert"},
  {"P0929","Electrovanne de blocage de changement de vitesse - plage de mesure/performance du circuit"},
  {"P0930","Electrovanne de blocage de changement de vitesse - circuit trop bas"},
  {"P0931","Electrovanne de blocage de changement de vitesse - circuit trop haut"},
  {"P0932","Capteur de pression hydraulique - panne du circuit"},
  {"P0933","Capteur de pression hydraulique - plage de mesure/performance du circuit"},
  {"P0934","Capteur de pression hydraulique - signal d'entrée du circuit trop bas"},
  {"P0935","Capteur de pression hydraulique - signal d'entrée du circuit trop haut"},
  {"P0936","Capteur de pression hydraulique - circuit intermittent"},
  {"P0937","Sonde de température du fluide hydraulique - panne du circuit"},
  {"P0938","Sonde de température du fluide hydraulique - plage de mesure/performance"},
  {"P0939","Sonde de température du fluide hydraulique - signal d'entrée du circuit trop bas"},
  {"P0940","Sonde de température du fluide hydraulique - signal d'entrée du circuit trop haut"},
  {"P0941","Sonde de température du fluide hydraulique - circuit intermittent"},
  {"P0942","Module de pression hydraulique"},
  {"P0943","Module de pression hydraulique - durée de cycle trop courte"},
  {"P0944","Module de pression hydraulique - perte de pression"},
  {"P0945","Relais de la pompe hydraulique - circuit ouvert"},
  {"P0946","Relais de la pompe hydraulique - plage de mesure/performance du circuit"},
  {"P0947","Relais de la pompe hydraulique - circuit trop bas"},
  {"P0948","Relais de la pompe hydraulique - circuit trop haut"},
  {"P0949","Boite de vitesse à passage automatique - apprentissage adaptatif non effectué"},
  {"P0950","Circuit de commande boite de vitesses à passage automatique"},
  {"P0951","Circuit de commande boite de vitesses à passage automatique - plage de mesure/performance"},
  {"P0952","Circuit de commande boite de vitesses à passage automatique - basse"},
  {"P0953","Circuit de commande boite de vitesses à passage automatique - haute"},
  {"P0954","Circuit de commande boite de vitesses à passage automatique - panne intermittente du circuit"},
  {"P0955","Circuit de programme boite de vitesses à passage automatique - panne"},
  {"P0956","Circuit de programme boite de vitesses à passage automatique - plage de mesure/performance"},
  {"P0957","Circuit de programme boite de vitesses à passage automatique - basse"},
  {"P0958","Circuit de programme boite de vitesses à passage automatique - haute"},
  {"P0959","Circuit de programme boite de vitesses à passage automatique - panne intermittente du circuit"},
  {"P0960","Electrovanne de régulation de pression A - circuit de commande ouvert"},
  {"P0961","Electrovanne de régulation de pression A - plage de mesure/performance du circuit de commande"},
  {"P0962","Electrovanne de régulation de pression A - circuit de commande trop faible"},
  {"P0963","Electrovanne de régulation de pression A - circuit de commande trop fort"},
  {"P0964","Electrovanne de régulation de pression B - circuit de commande ouvert"},
  {"P0965","Electrovanne de régulation de pression B - plage de mesure/performance du circuit de commande"},
  {"P0966","Electrovanne de régulation de pression B - circuit de commande trop faible"},
  {"P0967","Electrovanne de régulation de pression B - circuit de commande trop fort"},
  {"P0968","Electrovanne de régulation de pression C - circuit de commande ouvert"},
  {"P0969","Electrovanne de régulation de pression C - plage de mesure/performance du circuit de commande"},
  {"P0970","Electrovanne de régulation de pression C - circuit de commande trop faible"},
  {"P0971","Electrovanne de régulation de pression C - circuit de commande trop fort"},
  {"P0972","Electrovanne de changement de vitesse A - plage de mesure/performance du circuit de commande"},
  {"P0973","Electrovanne de changement de vitesse A - circuit de commande trop faible"},
  {"P0974","Electrovanne de changement de vitesse A - circuit de commande trop fort"},
  {"P0975","Electrovanne de changement de vitesse B - plage de mesure/performance du circuit de commande"},
  {"P0976","Electrovanne de changement de vitesse B - circuit de commande trop faible"},
  {"P0977","Electrovanne de changement de vitesse B - circuit de commande trop fort"},
  {"P0978","Electrovanne de changement de vitesse C - plage de mesure/performance du circuit de commande"},
  {"P0979","Electrovanne de changement de vitesse C - circuit de commande trop faible"},
  {"P0980","Electrovanne de changement de vitesse C - circuit de commande trop fort"},
  {"P0981","Electrovanne de changement de vitesse D - plage de mesure/performance du circuit de commande"},
  {"P0982","Electrovanne de changement de vitesse D - circuit de commande trop faible"},
  {"P0983","Electrovanne de changement de vitesse D - circuit de commande trop fort"},
  {"P0984","Electrovanne de changement de vitesse E - plage de mesure/performance du circuit de commande"},
  {"P0985","Electrovanne de changement de vitesse E - circuit de commande trop faible"},
  {"P0986","Electrovanne de changement de vitesse E - circuit de commande trop fort"},
  {"P0987","Capteur de pression d'huile de transmission E - panne du circuit"},
  {"P0988","Capteur de pression d'huile de transmission E - plage de mesure/performance du circuit"},
  {"P0989","Capteur de pression d'huile de transmission E - circuit trop bas"},
  {"P0990","Capteur de pression d'huile de transmission E - circuit trop haut"},
  {"P0991","Capteur de pression d'huile de transmission E - circuit intermittent"},
  {"P0992","Capteur de pression d'huile de transmission F - panne du circuit"},
  {"P0993","Capteur de pression d'huile de transmission F - plage de mesure/performance du circuit"},
  {"P0994","Capteur de pression d'huile de transmission F - circuit trop bas"},
  {"P0995","Capteur de pression d'huile de transmission F - circuit trop haut"},
  {"P0996","Capteur de pression d'huile de transmission F - circuit intermittent"},
  {"P0997","Electrovanne de changement de vitesse F - plage de mesure/performance du circuit de commande"},
  {"P0998","Electrovanne de changement de vitesse F - circuit de commande trop faible"},
  {"P0999","Electrovanne de changement de vitesse F - circuit de commande trop fort"},
};
#define NDTC (sizeof(TABLE_DTC)/sizeof(TABLE_DTC[0]))

String texteDTC(const String& code) {
  for (uint16_t i = 0; i < NDTC; i++) if (code == TABLE_DTC[i].code) return String(TABLE_DTC[i].texte);
  return "Description non disponible (code hors table, 986 entrees)";
}

String decodeDTC(uint8_t b1, uint8_t b2) {
  if (b1 == 0 && b2 == 0) return "";   // bourrage / absence de code
  const char lettres[] = {'P', 'C', 'B', 'U'};
  char lettre = lettres[(b1 >> 6) & 0x03];
  uint8_t d1 = (b1 >> 4) & 0x03;
  char buf[6];
  snprintf(buf, sizeof(buf), "%c%d%X%02X", lettre, d1, b1 & 0x0F, b2);
  return String(buf);
}

// Un seul frame simple = SID + 6 octets max = 3 DTC max (au-dela : multi-trame ISO-TP, non gere)
String listerDTC(uint8_t* data, uint8_t len) {
  uint8_t pciLen = data[0] & 0x0F;   // SID + octets DTC utiles, independant du bourrage a 8
  if (pciLen <= 1) return "Aucun";
  uint8_t nbCodes = (pciLen - 1) / 2;
  String out = "";
  for (uint8_t i = 0; i < nbCodes; i++) {
    String code = decodeDTC(data[2 + i * 2], data[3 + i * 2]);
    if (code.length()) {
      if (out.length()) out += "; ";
      out += code + " (" + texteDTC(code) + ")";
    }
  }
  return out.length() ? out : "Aucun";
}

/* ---------- HTML Accueil ------------------------------------- */
const char* HTML_ACCUEIL = R"rawliteral(
<!DOCTYPE html>
<html lang="fr"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>NEMO OBD</title>
<style>
body { font-family: Arial, sans-serif; text-align: center; background: #000; color: #fff; padding: 50px 20px; }
h1 { font-size: 1.3em; }
p { color: #ccc; }
a.bouton { display: block; margin: 24px auto; padding: 26px; max-width: 320px; font-size: 22px;
  font-weight: bold; border-radius: 12px; text-decoration: none; color: #fff; }
.diag { background: #007bff; }
.conduite { background: #2ecc71; }
</style></head><body>
<h1>NEMO OBD</h1>
<p>Citroen Nemo 1.3 HDi - Marelli MJD 8F3.F6</p>
<a class="bouton diag" href="/diag">DIAGNOSTIC</a>
<a class="bouton conduite" href="/conduite">CONDUITE</a>
</body></html>
)rawliteral";

/* ---------- HTML Conduite (jauge RPM 270 deg plein ecran) ----- */
const char* HTML_CONDUITE = R"rawliteral(
<!DOCTYPE html>
<html lang="fr"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>NEMO Conduite</title>
<style>
body { font-family: Arial, sans-serif; text-align: center; background: #000; color: #fff; margin: 0; padding: 16px; }
h1 { font-size: 1.1em; margin: 4px 0 12px; }
button { padding: 10px 18px; font-size: 15px; font-weight: bold; border: none; border-radius: 8px; color: #fff; margin: 3px; }
#gaugeWrap { width: 92vw; max-width: 480px; margin: 10px auto; }
#gaugeWrap svg { width: 100%; height: auto; }
#rpmVal { font-size: 15vw; font-weight: bold; margin-top: -12vw; }
#rpmUnit { font-size: 1em; color: #aaa; margin-bottom: 10px; }
#retour { display: inline-block; margin-top: 14px; padding: 12px 24px; background: #555; color: #fff;
  text-decoration: none; border-radius: 8px; }
</style></head><body>
<h1>Mode conduite</h1>
<div>
  <button id="btnEcoC" onclick="choisirModeC('eco')" style="background:#2ecc71;">ECO</button>
  <button id="btnNormalC" onclick="choisirModeC('normal')" style="background:#555;">NORMAL</button>
  <button id="btnPerfC" onclick="choisirModeC('perf')" style="background:#555;">PERFORMANCE</button>
</div>
<div id="gaugeWrap"></div>
<div id="rpmVal">--</div>
<div id="rpmUnit">tr/min</div>
<div id="rapportZone" style="display:flex;justify-content:center;align-items:center;gap:20px;margin-top:10px;">
  <div>
    <div style="font-size:12px;color:#888;">RAPPORT</div>
    <div id="rapportActuel" style="font-size:20vw;font-weight:bold;line-height:1;">-</div>
  </div>
  <div id="rapportSuivant" style="font-size:16vw;font-weight:bold;line-height:1;">-</div>
</div>
<a id="retour" href="#" onclick="retourAccueil(); return false;">Retour</a>

<script>
var RAYON = 45, CIRC = 2 * Math.PI * RAYON, ARC_DEG = 270, ARC_LEN = CIRC * (ARC_DEG / 360);
var RPM_MAX = 5000;
var ANGLE0 = 150;   // meme convention que les jauges diag : depart a 8h
var MODES = { eco: {bas:1400,haut:2200}, normal: {bas:1800,haut:2600}, perf: {bas:2700,haut:3900} };
var modeActuel = 'eco';
var dernierRapport = 0;

function zoneArc(startFrac, lenFrac, couleur) {
  if (lenFrac <= 0) return '';
  var startAngle = ANGLE0 + startFrac * ARC_DEG;
  var len = ARC_LEN * lenFrac;
  return '<circle cx="50" cy="50" r="' + RAYON + '" fill="none" stroke="' + couleur + '" stroke-width="10" ' +
    'stroke-dasharray="' + len + ' ' + (CIRC - len) + '" transform="rotate(' + startAngle + ' 50 50)"/>';
}

function dessinerGauge() {
  var m = MODES[modeActuel];
  var bas = (dernierRapport === 1) ? 0 : m.bas;   // pas de retrograder en 1ere
  var haut = m.haut;
  var svg = '<svg viewBox="0 0 100 100">' +
    '<circle cx="50" cy="50" r="' + RAYON + '" fill="none" stroke="#222" stroke-width="10" ' +
      'stroke-dasharray="' + ARC_LEN + ' ' + (CIRC - ARC_LEN) + '" transform="rotate(' + ANGLE0 + ' 50 50)"/>' +
    zoneArc(0, bas / RPM_MAX, '#00e5ff') +
    zoneArc(bas / RPM_MAX, (haut - bas) / RPM_MAX, '#2ecc71') +
    zoneArc(haut / RPM_MAX, (RPM_MAX - haut) / RPM_MAX, '#e74c3c') +
    '<line id="aiguille" x1="50" y1="50" x2="88" y2="50" stroke="#fff" stroke-width="3" stroke-linecap="round"/>' +
    '</svg>';
  document.getElementById('gaugeWrap').innerHTML = svg;
}

function majAiguille(rpm) {
  var frac = Math.max(0, Math.min(1, rpm / RPM_MAX));
  var angle = ANGLE0 + frac * ARC_DEG;
  var a = document.getElementById('aiguille');
  if (a) a.setAttribute('transform', 'rotate(' + angle + ' 50 50)');
}

function choisirModeC(m) {
  modeActuel = m;
  document.getElementById('btnEcoC').style.background = (m === 'eco') ? '#2ecc71' : '#555';
  document.getElementById('btnNormalC').style.background = (m === 'normal') ? '#2ecc71' : '#555';
  document.getElementById('btnPerfC').style.background = (m === 'perf') ? '#2ecc71' : '#555';
  dessinerGauge();
}

var audioCtx = null;
function ctxAudio() {
  if (!audioCtx) audioCtx = new (window.AudioContext || window.webkitAudioContext)();
  if (audioCtx.state === 'suspended') audioCtx.resume();
  return audioCtx;
}
function biper(freq, dureeMs) {
  var ctx = ctxAudio();
  var osc = ctx.createOscillator(), gain = ctx.createGain();
  osc.frequency.value = freq;
  gain.gain.setValueAtTime(0.15, ctx.currentTime);
  osc.connect(gain); gain.connect(ctx.destination);
  osc.start(); osc.stop(ctx.currentTime + dureeMs / 1000);
}
var oscContinuNode = null;
function toneContinueDemarrer(freq) {
  if (oscContinuNode) return;
  var ctx = ctxAudio();
  var osc = ctx.createOscillator(), gain = ctx.createGain();
  osc.frequency.value = freq;
  gain.gain.setValueAtTime(0.12, ctx.currentTime);
  osc.connect(gain); gain.connect(ctx.destination);
  osc.start();
  oscContinuNode = osc;
}
function toneContinueArreter() {
  if (oscContinuNode) { oscContinuNode.stop(); oscContinuNode = null; }
}

var dernierRpm = 0;
function boucleAudioC() {
  var delaiSuivant = 300;
  if (modeActuel !== 'perf') {
    toneContinueArreter();
  } else {
    var m = MODES.perf, rpm = dernierRpm;
    if (rpm >= m.haut) {
      toneContinueDemarrer(1200);
    } else if (rpm >= m.haut - 500) {
      toneContinueArreter();
      biper(1200, 70);
      delaiSuivant = 550 - ((rpm - (m.haut - 500)) / 500) * 450;
    } else if (rpm > 0 && rpm <= m.bas) {
      toneContinueDemarrer(300);
    } else if (rpm <= m.bas + 500) {
      toneContinueArreter();
      biper(300, 70);
      delaiSuivant = 550 - (((m.bas + 500) - rpm) / 500) * 450;
    } else {
      toneContinueArreter();
    }
  }
  setTimeout(boucleAudioC, delaiSuivant);
}

var pollTimer = null;
function majRapportAffiche() {
  var actuelEl = document.getElementById('rapportActuel');
  var suivantEl = document.getElementById('rapportSuivant');
  if (dernierRapport === 0) {
    actuelEl.textContent = '-';
    suivantEl.textContent = '';
    return;
  }
  actuelEl.textContent = dernierRapport;
  var m = MODES[modeActuel];
  var basApplicable = (dernierRapport > 1);   // pas de retrograder en 1ere
  if (dernierRpm > m.haut && dernierRapport < 5) {
    suivantEl.textContent = '\u2191 ' + (dernierRapport + 1);
    suivantEl.style.color = '#e74c3c';
  } else if (basApplicable && dernierRpm < m.bas) {
    suivantEl.textContent = '\u2193 ' + (dernierRapport - 1);
    suivantEl.style.color = '#00e5ff';
  } else {
    suivantEl.textContent = '\u2713';
    suivantEl.style.color = '#2ecc71';
  }
}

function pollConduite() {
  fetch('/conduite_status').then(function(r) { return r.json(); }).then(function(d) {
    if (d.ok) {
      dernierRpm = d.rpm;
      document.getElementById('rpmVal').textContent = Math.round(d.rpm);
      majAiguille(d.rpm);
    }
    if (d.rapport !== dernierRapport) { dernierRapport = d.rapport; dessinerGauge(); }
    majRapportAffiche();
  });
}

function retourAccueil() {
  fetch('/conduite_stop').then(function() { window.location.href = '/'; });
}
window.addEventListener('beforeunload', function() {
  navigator.sendBeacon && navigator.sendBeacon('/conduite_stop');
});

dessinerGauge();
fetch('/conduite_start').then(function() {
  pollTimer = setInterval(pollConduite, 200);
  boucleAudioC();
});
</script>
</body></html>
)rawliteral";

/* ---------- HTML Principal ---------------------------------- */
const char* HTML_INDEX = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>NEMO OBD - FW 8.17</title>
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
    .jauge svg { display: block; margin: 0 auto; transform: rotate(150deg); }
    .jauge .val { font-weight: 500; margin-top: 6px; color: #fff; font-size: 17px; }
    .jauge .lbl { font-weight: 500; font-size: 17px; color: #fff; }
    #logStatus { margin: 10px 0; font-weight: bold; color: #fff; }
    #logStatus.rafale { color: #ff4444; }
    #downloadLog { display: none; margin-top: 10px; text-decoration: none; padding: 10px 20px; background: #28a745; color: white; border-radius: 5px; }
  </style>
</head>
<body>
  <a href="/" style="color:#888;font-size:13px;">&larr; Accueil</a>
  <h2>Log continu</h2>
  <button id="btnLog" onclick="toggleLog()">LOG</button>
  <div id="logStatus">Arrete</div>
  <a id="downloadLog" href="/download_log" download="drivelog.csv">Telecharger log CSV</a>
  <button id="btnPurge" onclick="purgerLog()" style="background:#c0392b;">PURGER (log + scan)</button>
  <div id="fsBarWrap" style="width:95%;max-width:760px;margin:12px auto;background:#333;border-radius:6px;overflow:hidden;height:22px;">
    <div id="fsBar" style="height:100%;width:0%;background:#2ecc71;transition:width 0.3s;"></div>
  </div>
  <div id="fsText" style="margin-bottom:10px;color:#ccc;">-- </div>
  <div id="jauges" class="jauge-grid"></div>

  <div style="width:95%;max-width:760px;margin:16px auto 0;text-align:left;font-size:14px;color:#ccc;line-height:1.6;">
    <div>Norme OBD : <span id="n1c" style="color:#fff;">--</span></div>
    <div>Moniteurs depuis effacement : <span id="n01" style="color:#fff;">--</span></div>
    <div>Moniteurs cycle actuel : <span id="n41" style="color:#fff;">--</span></div>
    <div>Rapport estime (pneu 185/65 R15) : <span id="rapport" style="color:#fff;font-weight:bold;">--</span></div>
  </div>

  <h2>Defauts</h2>
  <button id="btnCheckDtc" onclick="checkDtc()">CHECK DEFAUTS</button>
  <button id="btnClearDtc" onclick="clearDtc()" style="background:#c0392b;">CLEAR DEFAUTS</button>
  <div style="width:95%;max-width:760px;margin:12px auto 0;text-align:left;font-size:14px;color:#ccc;line-height:1.6;">
    <div>Memorises (Mode 03) : <span id="dtc03" style="color:#fff;">--</span></div>
    <div>En attente (Mode 07) : <span id="dtc07" style="color:#fff;">--</span></div>
    <div>Permanents (Mode 0A) : <span id="dtc0a" style="color:#fff;">--</span></div>
  </div>
  <div id="clearResult" style="margin-top:8px;font-weight:bold;"></div>

  <hr style="border-color:#333; margin:30px 0;">

  <h1>Scan PID - ECU 0x10 (Nemo C-CAN) - FW 8.17</h1>
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
    var RAYON = 45, CIRC = 2 * Math.PI * RAYON, ARC_DEG = 270, ARC_LEN = CIRC * (ARC_DEG / 360);

    function initJauges() {
      var div = document.getElementById('jauges');
      JAUGES.forEach(function(j, i) {
        var box = document.createElement('div');
        box.className = 'jauge';
        box.innerHTML =
          '<svg width="140" height="140" viewBox="0 0 100 100">' +
            '<circle cx="50" cy="50" r="' + RAYON + '" fill="none" stroke="#333" stroke-width="10" ' +
              'stroke-dasharray="' + ARC_LEN + ' ' + (CIRC - ARC_LEN) + '" stroke-linecap="round"/>' +
            '<circle id="arc' + i + '" cx="50" cy="50" r="' + RAYON + '" fill="none" stroke="#2ecc71" ' +
              'stroke-width="10" stroke-dasharray="0 ' + CIRC + '" stroke-linecap="round"/>' +
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
          if (arc) arc.setAttribute('stroke-dasharray', (ARC_LEN * frac) + ' ' + (CIRC - ARC_LEN * frac));
          var val = document.getElementById('val' + slot);
          if (val) val.textContent = p.ok ? (p.num.toFixed(1) + ' ' + j.unit) : '--';
        });
      });
    }

    function checkDtc() {
      document.getElementById('dtc03').textContent = '...';
      document.getElementById('dtc07').textContent = '...';
      document.getElementById('dtc0a').textContent = '...';
      fetch('/check_dtc').then(function(r) { return r.json(); }).then(function(d) {
        document.getElementById('dtc03').textContent = d.memorises;
        document.getElementById('dtc07').textContent = d.attente;
        document.getElementById('dtc0a').textContent = d.permanents;
      });
    }

    function clearDtc() {
      if (!confirm('Effacer les defauts memorises reinitialise aussi les moniteurs de conformite (readiness) - un cycle de conduite complet sera necessaire avant un controle technique. Confirmer ?')) return;
      var div = document.getElementById('clearResult');
      div.textContent = '...';
      fetch('/clear_dtc').then(function(r) { return r.json(); }).then(function(d) {
        div.textContent = d.ok ? 'Defauts effaces.' : ('Echec : ' + d.raison);
        if (d.ok) checkDtc();
      });
    }

    function majNiveau2() {
      fetch('/log_status').then(function(r) { return r.json(); }).then(function(d) {
        document.getElementById('n1c').textContent = d.norme1c;
        document.getElementById('n01').textContent = d.statut01;
        document.getElementById('n41').textContent = d.statut41;
        document.getElementById('rapport').textContent = d.rapport > 0 ? d.rapport : '--';
      });
    }

    initJauges();
    majJauges();
    majNiveau2();
    setInterval(majNiveau2, 5000);
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
  listerFlash();

  mcp_init();

  WiFi.softAP(AP_SSID, AP_PASS);
  delay(100);
  WiFi.softAPConfig(AP_IP, AP_IP, AP_MASK);

  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", HTML_ACCUEIL);
  });

  server.on("/diag", HTTP_GET, []() {
    server.send(200, "text/html", HTML_INDEX);
  });

  server.on("/conduite", HTTP_GET, []() {
    server.send(200, "text/html", HTML_CONDUITE);
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
    json += "],\"norme1c\":\"" + normeObd + "\",\"statut01\":\"" + statutMoniteursDepuis +
            "\",\"statut41\":\"" + statutMoniteursCycle + "\",\"rapport\":" + String(rapportEstime) + "}";
    server.send(200, "application/json", json);
  });

  server.on("/conduite_start", HTTP_GET, []() {
    if (logMode != LOG_IDLE) {
      server.send(200, "application/json", "{\"ok\":false,\"raison\":\"log de conduite en cours\"}");
      return;
    }
    rpmConduiteOk = false;
    vitesseConduiteOk = false;
    conduitePhase = 0;
    logMode = LOG_CONDUITE;
    server.send(200, "application/json", "{\"ok\":true}");
  });

  server.on("/conduite_stop", HTTP_GET, []() {
    if (logMode == LOG_CONDUITE) logMode = LOG_IDLE;
    server.send(200, "application/json", "{\"ok\":true}");
  });

  server.on("/conduite_status", HTTP_GET, []() {
    String json = "{\"rpm\":" + String(rpmConduiteOk ? rpmConduite : 0, 1) +
                  ",\"ok\":" + String(rpmConduiteOk ? "true" : "false") +
                  ",\"rapport\":" + String(rapportEstime) + "}";
    server.send(200, "application/json", json);
  });

  server.on("/log_purge", HTTP_GET, []() {
    if (logMode != LOG_IDLE) {
      server.send(200, "application/json", "{\"ok\":false,\"raison\":\"log en cours\"}");
      return;
    }
    bool r1 = !LittleFS.exists("/drivelog.csv") || LittleFS.remove("/drivelog.csv");
    bool r2 = !LittleFS.exists("/result.csv") || LittleFS.remove("/result.csv");
    // Residus d'anciennes versions abandonnees (jamais crees par ce firmware) :
    if (LittleFS.exists("/mode.txt")) LittleFS.remove("/mode.txt");
    if (LittleFS.exists("/trace.csv")) LittleFS.remove("/trace.csv");
    listerFlash();
    server.send(200, "application/json", (r1 && r2) ? "{\"ok\":true}" : "{\"ok\":false,\"raison\":\"echec suppression\"}");
  });

  server.on("/download_log", HTTP_GET, []() {
    if (!LittleFS.exists("/drivelog.csv")) { server.send(404, "text/plain", "Pas de log"); return; }
    File f = LittleFS.open("/drivelog.csv", "r");
    server.sendHeader("Content-Disposition", "attachment; filename=nemo_drivelog.csv");
    server.streamFile(f, "text/csv");
    f.close();
  });

  server.on("/check_dtc", HTTP_GET, []() {
    uint32_t respId; uint8_t data[8]; uint8_t len; uint8_t buf;
    String m03 = "pas de reponse", m07 = "pas de reponse", m0a = "pas de reponse";

    if (testerMode(0x03, respId, data, len, buf)) {
      m03 = (data[1] == 0x43) ? listerDTC(data, len) :
            (data[1] == 0x7F) ? ("NEG NRC " + String(data[3], HEX)) : "Brut";
    }
    if (testerMode(0x07, respId, data, len, buf)) {
      m07 = (data[1] == 0x47) ? listerDTC(data, len) :
            (data[1] == 0x7F) ? ("NEG NRC " + String(data[3], HEX)) : "Brut";
    }
    if (testerMode(0x0A, respId, data, len, buf)) {
      m0a = (data[1] == 0x4A) ? listerDTC(data, len) :
            (data[1] == 0x7F) ? ("NEG NRC " + String(data[3], HEX)) : "Brut";
    }

    String json = "{\"memorises\":\"" + m03 + "\",\"attente\":\"" + m07 + "\",\"permanents\":\"" + m0a + "\"}";
    server.send(200, "application/json", json);
  });

  server.on("/clear_dtc", HTTP_GET, []() {
    uint32_t respId; uint8_t data[8]; uint8_t len; uint8_t buf;
    String json;
    if (testerMode(0x04, respId, data, len, buf)) {
      if (data[1] == 0x44) json = "{\"ok\":true}";
      else if (data[1] == 0x7F) json = "{\"ok\":false,\"raison\":\"NEG NRC " + String(data[3], HEX) + "\"}";
      else json = "{\"ok\":false,\"raison\":\"reponse inattendue\"}";
    } else {
      json = "{\"ok\":false,\"raison\":\"pas de reponse\"}";
    }
    server.send(200, "application/json", json);
  });

  server.begin();

  // Norme OBD (0x1C) : statique, interrogee une seule fois au demarrage
  uint32_t respId; uint8_t data[8]; uint8_t len; uint8_t buf;
  if (testerUnPid(0x1C, respId, data, len, buf) && len >= 4 && data[1] == 0x41) {
    normeObd = decodeNorme1C(data[3]);
  }
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
    verifierNiveau2();   // pas de log en cours : profite du temps libre
    return;
  }

  if (logMode == LOG_CONDUITE) {
    uint32_t respId; uint8_t data[8]; uint8_t len; uint8_t buf;
    uint8_t pid = (conduitePhase == 0) ? 0x0C : 0x0D;
    bool ok = testerUnPid(pid, respId, data, len, buf);
    if (ok && len >= 3 && data[1] == 0x41) {
      float v;
      if (getNumeric(pid, data, len, v)) {
        if (pid == 0x0C) { rpmConduite = v; rpmConduiteOk = true; }
        else { vitesseConduite = v; vitesseConduiteOk = true; }
      }
    } else {
      if (pid == 0x0C) rpmConduiteOk = false; else vitesseConduiteOk = false;
    }
    if (rpmConduiteOk && vitesseConduiteOk) {
      rapportEstime = estimerRapport(rpmConduite, vitesseConduite);
    }
    conduitePhase = 1 - conduitePhase;
    return;   // toujours rien d'autre : pas de pedale, pas de log fichier
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
    if (derniereOk[IDX_REGIME] && derniereOk[IDX_VITESSE]) {
      rapportEstime = estimerRapport(derniereNum[IDX_REGIME], derniereNum[IDX_VITESSE]);
    }
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
      verifierNiveau2();   // jamais en rafale : ne pas perturber la cadence rapide
    } else {
      // Rafale : pause courte entre manches (stabilite avant vitesse), aucune
      // ecriture flash tant que la rafale dure - le tampon RAM absorbe tout.
      delay(RAFALE_PAUSE_MS);
    }
  }
}
