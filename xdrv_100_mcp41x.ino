/*
  xdrv_100_mcp41x.ino — Driver custom MCP41xxx (ex. MCP41010) pour Tasmota

  Fonctionnalités :
    - Bus SPI matériel (rôles: SPI CLK/MOSI/MISO/CS) ou SoftSPI (rôles: SSPI CLK/MOSI/MISO/CS).
    - Priorité : override MCP41CS > SSPI CS > SPI CS.
    - Commandes :
        * MCP41 <0..255>  : écrit le wiper. Sans argument => renvoie la dernière valeur & info bus.
        * MCP41GET        : alias pour lire la dernière valeur.
        * MCP41CS <gpio>  : override CS à chaud ; -1 => retour à la config SPI/SSPI.
    - Persistance : dernière valeur rejouée au boot via Rule (Rule3 par défaut).

  A ajuster :
    - MCP41_CMD_WRITE_WIPER : vérifie l’opcode d’écriture de TON MCP41xxx (datasheet).
    - MCP41_SPI_CLOCK/MODE et MCP41_SSPI_DELAY_US si besoin.

  Prérequis :
    - Build avec USE_SPI
    - Place ce fichier dans tasmota/tasmota_xdrv_driver/
*/

#ifdef USE_MCP41X

#ifndef USE_SPI
  #define USE_SPI
#endif


#include "tasmota.h"
#ifdef USE_SPI
  #include <SPI.h>
#endif

// ------------------------------ Configuration ------------------------------

#define XDRV_100                 100
#define LOG_TAG                  "MCP41"

// Opcode d’écriture (à VALIDER pour ton modèle exact)
#ifndef MCP41_CMD_WRITE_WIPER
  #define MCP41_CMD_WRITE_WIPER  0x11
#endif

// Matériel SPI
#ifndef MCP41_SPI_CLOCK
  #define MCP41_SPI_CLOCK        1000000     // 1 MHz
#endif
#ifndef MCP41_SPI_MODE
  #define MCP41_SPI_MODE         SPI_MODE0
#endif

// SoftSPI (bit-bang)
#ifndef MCP41_SSPI_DELAY_US
  #define MCP41_SSPI_DELAY_US    1           // ~500 kHz théorique ; augmente si besoin
#endif

// Slot de Rule utilisée pour rejouer au boot
#ifndef MCP41_RULE_SLOT
  #define MCP41_RULE_SLOT        3           // Rule1..Rule3
#endif

// ------------------------------ État du driver ------------------------------

static int8_t   mcp41_cs_gpio        = -1;    // CS effectif (override ou config)
static int8_t   mcp41_cs_override    = -1;    // -1 => pas d’override
static bool     mcp41_spi_inited     = false;
static uint8_t  mcp41_last_value     = 0;

static bool     mcp41_use_sspi       = false; // true=SoftSPI, false=SPI matériel
// Broches SoftSPI
static int8_t   mcp41_sspi_clk       = -1;
static int8_t   mcp41_sspi_mosi      = -1;
static int8_t   mcp41_sspi_miso      = -1;    // non utilisé par MCP41 mais on le prend si dispo

// Broches SPI matériel (optionnelles)
static int8_t   mcp41_spi_clk        = -1;
static int8_t   mcp41_spi_mosi       = -1;
static int8_t   mcp41_spi_miso       = -1;

// ------------------------------ Helpers GPIO/Bus ----------------------------

inline void MCP41_Select(bool en) {
  if (mcp41_cs_gpio < 0) return;
  digitalWrite(mcp41_cs_gpio, en ? LOW : HIGH);
}

static void MCP41_DetectPinsFromConfig() {
  // Reset
  mcp41_use_sspi  = false;
  mcp41_sspi_clk  = mcp41_sspi_mosi = mcp41_sspi_miso = -1;
  mcp41_spi_clk   = mcp41_spi_mosi  = mcp41_spi_miso  = -1;

  // Override > SSPI CS > SPI CS
  if (mcp41_cs_override >= 0) {
    mcp41_cs_gpio = mcp41_cs_override;
  } else if (PinUsed(GPIO_SSPI_CS)) {
    mcp41_cs_gpio = (int8_t)Pin(GPIO_SSPI_CS);
    mcp41_use_sspi = true;
  } else if (PinUsed(GPIO_SPI_CS)) {
    mcp41_cs_gpio = (int8_t)Pin(GPIO_SPI_CS);
    mcp41_use_sspi = false;
  } else {
    mcp41_cs_gpio = -1;
  }

  // Lire les autres broches selon le bus retenu
  if (mcp41_use_sspi) {
    if (PinUsed(GPIO_SSPI_SCLK)) mcp41_sspi_clk  = (int8_t)Pin(GPIO_SSPI_SCLK);
    if (PinUsed(GPIO_SSPI_MOSI)) mcp41_sspi_mosi = (int8_t)Pin(GPIO_SSPI_MOSI);
    if (PinUsed(GPIO_SSPI_MISO)) mcp41_sspi_miso = (int8_t)Pin(GPIO_SSPI_MISO);
  } else {
    if (PinUsed(GPIO_SPI_SCLK))  mcp41_spi_clk   = (int8_t)Pin(GPIO_SPI_SCLK);
    if (PinUsed(GPIO_SPI_MOSI))  mcp41_spi_mosi  = (int8_t)Pin(GPIO_SPI_MOSI);
    if (PinUsed(GPIO_SPI_MISO))  mcp41_spi_miso  = (int8_t)Pin(GPIO_SPI_MISO);
  }
}

static void MCP41_ResolveBusAndPins() {
  int8_t prev_cs = mcp41_cs_gpio;
  bool   prev_ss = mcp41_use_sspi;

  MCP41_DetectPinsFromConfig();

  if (prev_cs != mcp41_cs_gpio || prev_ss != mcp41_use_sspi) {
    mcp41_spi_inited = false;                    // re-init requis
    if (mcp41_cs_gpio >= 0) {
      pinMode(mcp41_cs_gpio, OUTPUT);
      digitalWrite(mcp41_cs_gpio, HIGH);         // CS inactif
      AddLog(LOG_LEVEL_INFO, PSTR(LOG_TAG ": CS=GPIO%d (%s)"),
             mcp41_cs_gpio, mcp41_use_sspi ? "SSPI" : "SPI");
    } else {
      AddLog(LOG_LEVEL_INFO, PSTR(LOG_TAG ": CS non défini (SPI/SSPI)"));
    }
  }
}

// ------------------------------ Init SPI / SoftSPI --------------------------

static void MCP41_InitSpiIfNeeded() {
  MCP41_ResolveBusAndPins();
  if (mcp41_spi_inited || mcp41_cs_gpio < 0) return;

  if (mcp41_use_sspi) {
    // SoftSPI : nécessite au minimum CLK + MOSI
    if (mcp41_sspi_clk < 0 || mcp41_sspi_mosi < 0) {
      AddLog(LOG_LEVEL_ERROR, PSTR(LOG_TAG ": SSPI pins incomplets (CLK/MOSI)"));
      return;
    }
    pinMode(mcp41_sspi_clk, OUTPUT);
    pinMode(mcp41_sspi_mosi, OUTPUT);
    digitalWrite(mcp41_sspi_clk, LOW);   // MODE0: CPOL=0
    // MISO non utilisé par MCP41; s'il existe, on peut le mettre en INPUT
    if (mcp41_sspi_miso >= 0) pinMode(mcp41_sspi_miso, INPUT);
  } else {
    // SPI matériel : si broches renseignées, les utiliser, sinon SPI.begin() défaut
    if (mcp41_spi_clk >= 0 || mcp41_spi_miso >= 0 || mcp41_spi_mosi >= 0) {
      // Certaines cartes autorisent SPI.begin(SCK,MISO,MOSI,CS)
      SPI.begin((mcp41_spi_clk >= 0) ? mcp41_spi_clk : SCK,
                (mcp41_spi_miso >= 0) ? mcp41_spi_miso : MISO,
                (mcp41_spi_mosi >= 0) ? mcp41_spi_mosi : MOSI,
                mcp41_cs_gpio);
    } else {
      SPI.begin();
    }
  }

  mcp41_spi_inited = true;
  AddLog(LOG_LEVEL_INFO, PSTR(LOG_TAG ": Bus %s initialisé"),
         mcp41_use_sspi ? "SSPI" : "SPI");
}

// ------------------------------ SoftSPI (bit-bang) --------------------------

static inline void SSPI_Delay() {
  if (MCP41_SSPI_DELAY_US > 0) delayMicroseconds(MCP41_SSPI_DELAY_US);
}

static void SSPI_TransferByte(uint8_t data) {
  // MODE0, MSB first
  for (int i = 7; i >= 0; --i) {
    digitalWrite(mcp41_sspi_mosi, (data & (1 << i)) ? HIGH : LOW);
    SSPI_Delay();
    digitalWrite(mcp41_sspi_clk, HIGH);
    SSPI_Delay();
    digitalWrite(mcp41_sspi_clk, LOW);
  }
}

// ------------------------------ Écriture MCP41 ------------------------------

static bool MCP41_Write(uint8_t value) {
  if (mcp41_cs_gpio < 0) { AddLog(LOG_LEVEL_ERROR, PSTR(LOG_TAG ": CS indisponible")); return false; }
  if (!mcp41_spi_inited) MCP41_InitSpiIfNeeded();
  if (!mcp41_spi_inited) return false;

  if (mcp41_use_sspi) {
    MCP41_Select(true);
    SSPI_TransferByte((uint8_t)MCP41_CMD_WRITE_WIPER); // opcode
    SSPI_TransferByte(value);                          // data
    MCP41_Select(false);
  } else {
    SPI.beginTransaction(SPISettings(MCP41_SPI_CLOCK, MSBFIRST, MCP41_SPI_MODE));
    MCP41_Select(true);
    SPI.transfer((uint8_t)MCP41_CMD_WRITE_WIPER);
    SPI.transfer(value);
    MCP41_Select(false);
    SPI.endTransaction();
  }

  mcp41_last_value = value;
  return true;
}

// ------------------------------ Persistance (Rule) --------------------------

static void MCP41_UpdateBootRule(uint8_t value) {
  // RuleX on System#Boot do MCP41 <value> endon ; RuleX 1
  char cmnd[96];
  snprintf_P(cmnd, sizeof(cmnd), PSTR("Rule%d on System#Boot do MCP41 %u endon"),
             MCP41_RULE_SLOT, (unsigned)value);
  ExecuteCommand(cmnd, SRC_IGNORE);

  snprintf_P(cmnd, sizeof(cmnd), PSTR("Rule%d 1"), MCP41_RULE_SLOT);
  ExecuteCommand(cmnd, SRC_IGNORE);

  AddLog(LOG_LEVEL_INFO, PSTR(LOG_TAG ": Boot rule set (Rule%d -> MCP41 %u)"),
         MCP41_RULE_SLOT, (unsigned)value);
}

// ------------------------------ Commandes console ---------------------------

static void Cmnd_MCP41() {
  if (XdrvMailbox.data_len == 0) {
    Response_P(PSTR("{\"MCP41\":{\"last\":%u,\"cs\":%d,\"bus\":\"%s\"}}"),
               (unsigned)mcp41_last_value,
               (int)mcp41_cs_gpio,
               mcp41_use_sspi ? "SSPI" : (mcp41_cs_gpio >= 0 ? "SPI" : "unset"));
    return;
  }

  uint32_t val = XdrvMailbox.payload;
  if (val > 255) val = 255;

  MCP41_InitSpiIfNeeded();
  if (mcp41_cs_gpio < 0) {
    Response_P(PSTR("{\"MCP41\":\"CS missing (set SPI/SSPI CS or use MCP41CS)\"}"));
    return;
  }

  bool ok = MCP41_Write((uint8_t)val);
  if (ok) {
    MCP41_UpdateBootRule((uint8_t)val);
    Response_P(PSTR("{\"MCP41\":%u}"), (unsigned)val);
  } else {
    Response_P(PSTR("{\"MCP41\":\"error\"}"));
  }
}

static void Cmnd_MCP41GET() {
  Response_P(PSTR("{\"MCP41\":{\"last\":%u}}"), (unsigned)mcp41_last_value);
}

static void Cmnd_MCP41CS() {
  if (XdrvMailbox.data_len == 0) {
    Response_P(PSTR("{\"MCP41CS\":{\"override\":%d,\"effective\":%d,\"bus\":\"%s\"}}"),
               (int)mcp41_cs_override, (int)mcp41_cs_gpio, mcp41_use_sspi ? "SSPI" : "SPI");
    return;
  }

  int32_t gpio = XdrvMailbox.payload;   // -1 pour désactiver override
  if (gpio < -1 || gpio > 39) {
    Response_P(PSTR("{\"MCP41CS\":\"invalid\"}"));
    return;
  }

  mcp41_cs_override = (int8_t)gpio;
  mcp41_spi_inited  = false;            // re-init forcé
  MCP41_InitSpiIfNeeded();

  Response_P(PSTR("{\"MCP41CS\":{\"override\":%d,\"effective\":%d,\"bus\":\"%s\"}}"),
             (int)mcp41_cs_override, (int)mcp41_cs_gpio, mcp41_use_sspi ? "SSPI" : "SPI");
}

// Table & dispatch
static const char kMCP41Commands[] PROGMEM = "MCP41|MCP41GET|MCP41CS";
void (* const MCP41Command[])(void) PROGMEM = { Cmnd_MCP41, Cmnd_MCP41GET, Cmnd_MCP41CS };

static bool MCP41_CommandDispatcher() {
  return CommandMatcher(kMCP41Commands, MCP41Command);
}

// ------------------------------ Entry point ---------------------------------

bool Xdrv100(uint8_t function) {
  bool handled = false;

  switch (function) {
    case FUNC_PRE_INIT:
      MCP41_ResolveBusAndPins();
      break;

    case FUNC_INIT:
      MCP41_InitSpiIfNeeded();
      break;

    case FUNC_COMMAND:
      handled = MCP41_CommandDispatcher();
      break;

    default:
      break;
  }
  return handled;
}

#endif  // USE_MCP41X
