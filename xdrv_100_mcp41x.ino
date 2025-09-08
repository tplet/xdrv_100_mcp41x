/*
  xdrv_100_mcp41x.ino — Driver custom MCP41xxx (ex. MCP41010) pour Tasmota

  Fonctionnalités :
    - Bus SPI matériel (rôles: SPI CLK/MOSI/MISO/CS) ou SoftSPI (rôles: SSPI CLK/MOSI/MISO/CS).
    - Priorité : override MCP41CS > SSPI CS > SPI CS.
    - Commandes :
        * MCP41 <0.0..100.0>       : écrit le wiper en pourcentage. Sans argument => renvoie la dernière valeur & info bus.
        * MCP41GET                 : alias pour lire la dernière valeur, en pourcentage.
        * MCP41CS <gpio>           : override CS à chaud ; -1 => retour à la config SPI/SSPI.
        * MCP41ADD <-100.0..100.0> : increase value (but result will cannot below 0.0 and up to 100.0)
    - Persistance : dernière valeur rejouée au boot via Rule (Rule3 par défaut).

  Prérequis :
    - Build avec USE_MCP41X
    - Placer ce fichier dans tasmota/tasmota_xdrv_driver/
*/

#ifdef USE_MCP41X

#ifndef USE_SPI
  #define USE_SPI
#endif


#ifdef USE_SPI
  #include <SPI.h>
#endif

// ------------------------------ Configuration ------------------------------

#define XDRV_100                 100
#define LOG_TAG                  "MCP41"

// Opcode d’écriture (à VALIDER pour ton modèle exact)
#ifndef MCP41_CMD_WRITE_WIPER
  #define MCP41_CMD_WRITE_WIPER  0x11 // Valeur binaire: B00010001
#endif

// Matériel SPI
#ifndef MCP41_SPI_CLOCK
  #define MCP41_SPI_CLOCK        10000000     // 10 MHz
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

// Valeur maximale du wiper
#ifndef MCP41_SPI_MAX_VALUE
  #define MCP41_SPI_MAX_VALUE        255
#endif
// Valeur minimale du wiper
#ifndef MCP41_SPI_MIN_VALUE
  #define MCP41_SPI_MIN_VALUE        0
#endif


// ------------------------------ État du driver ------------------------------

static int8_t   mcp41_cs_gpio        = -1;    // CS effectif (override ou config)
static int8_t   mcp41_cs_override    = -1;    // -1 => pas d’override
static bool     mcp41_spi_inited     = false;
static float    mcp41_last_value     = 0.0f; // en pourcentage
static uint8_t  mcp41_raw_min        = MCP41_SPI_MIN_VALUE; // en valeur brute
static uint8_t  mcp41_raw_max        = MCP41_SPI_MAX_VALUE; // en valeur brute

static bool     mcp41_use_sspi       = false; // true=SoftSPI, false=SPI matériel
// Broches SoftSPI
static int8_t   mcp41_sspi_clk       = -1;
static int8_t   mcp41_sspi_mosi      = -1;
static int8_t   mcp41_sspi_miso      = -1;    // non utilisé par MCP41 mais on le prend si dispo

// Broches SPI matériel (optionnelles)
static int8_t   mcp41_spi_clk        = -1;
static int8_t   mcp41_spi_mosi       = -1;
static int8_t   mcp41_spi_miso       = -1;
static uint8_t  mcp41_spi_bus        = 0;

// ------------------------------ Helpers GPIO/Bus ----------------------------

/**
 * @brief Sélectionne le CS
 * @param en true=CS actif (état bas: LOW), false=CS inactif (état haut: HIGH)
 */
inline void MCP41_Select(bool en) {
  if (mcp41_cs_gpio < 0) return;
  digitalWrite(mcp41_cs_gpio, en ? LOW : HIGH);
}

/**
 * @brief Détecte les broches SPI matériel (bus0, bus1) et SoftSPI
 */
static void MCP41_DetectPinsFromConfig() {
  // Reset
  mcp41_use_sspi  = false;
  mcp41_sspi_clk  = mcp41_sspi_mosi = mcp41_sspi_miso = -1;
  mcp41_spi_clk   = mcp41_spi_mosi  = mcp41_spi_miso  = -1;
  mcp41_spi_bus   = 0;

  // Override > SSPI CS > SPI CS (bus1) > SPI CS (bus2) > SPI CS (default)
  // Hard define (override)
  if (mcp41_cs_override >= 0) {
    mcp41_cs_gpio = mcp41_cs_override;
  }
  // SSPI
  else if (PinUsed(GPIO_SSPI_CS)) {
    mcp41_cs_gpio = (int8_t)Pin(GPIO_SSPI_CS);
    mcp41_use_sspi = true;
  }
  // SPI (bus default = 0)
  else if (PinUsed(GPIO_SPI_CS)) {
    // GPIO_SPI_CS détecte automatiquement vos broches personnalisées (bus0)
    mcp41_cs_gpio = (int8_t)Pin(GPIO_SPI_CS);
    mcp41_use_sspi = false;
    mcp41_spi_bus = 0;
  }
  // SPI (bus1)
  else if (PinUsed(GPIO_SPI_CS, 1)) {
    // Fallback pour d'autres configurations
    mcp41_cs_gpio = (int8_t)Pin(GPIO_SPI_CS, 1);
    mcp41_use_sspi = false;
    mcp41_spi_bus = 1;
  }
  // Error: no CS pin found
  else {
    mcp41_cs_gpio = -1;
    AddLog(LOG_LEVEL_ERROR, PSTR(LOG_TAG ": No CS pin found!"));
  }

  // Lire les autres broches selon le bus retenu
  // SSPI
  if (mcp41_use_sspi) {
    if (PinUsed(GPIO_SSPI_SCLK)) mcp41_sspi_clk  = (int8_t)Pin(GPIO_SSPI_SCLK);
    if (PinUsed(GPIO_SSPI_MOSI)) mcp41_sspi_mosi = (int8_t)Pin(GPIO_SSPI_MOSI);
    if (PinUsed(GPIO_SSPI_MISO)) mcp41_sspi_miso = (int8_t)Pin(GPIO_SSPI_MISO);
    
    AddLog(LOG_LEVEL_DEBUG, PSTR(LOG_TAG ": SSPI pins detected - CLK: GPIO%d, MOSI: GPIO%d, CS: GPIO%d"),
           mcp41_sspi_clk, mcp41_sspi_mosi, mcp41_cs_gpio);
  }
  // SPI
  else if (mcp41_cs_gpio >= 0) {
    // Lire les broches SPI selon le bus détecté
    if (PinUsed(GPIO_SPI_CLK, mcp41_spi_bus))  mcp41_spi_clk   = (int8_t)Pin(GPIO_SPI_CLK, mcp41_spi_bus);
    if (PinUsed(GPIO_SPI_MOSI, mcp41_spi_bus))  mcp41_spi_mosi  = (int8_t)Pin(GPIO_SPI_MOSI, mcp41_spi_bus);
    if (PinUsed(GPIO_SPI_MISO, mcp41_spi_bus))  mcp41_spi_miso  = (int8_t)Pin(GPIO_SPI_MISO, mcp41_spi_bus);
    AddLog(LOG_LEVEL_DEBUG, PSTR(LOG_TAG ": SPI bus%d pins detected - CLK: GPIO%d, MOSI: GPIO%d, CS: GPIO%d"),
        mcp41_spi_bus, mcp41_spi_clk, mcp41_spi_mosi, mcp41_cs_gpio);
  }
}

/**
 * @brief Initialise la broche CS
 */
static void MCP41_ResolveBusAndPins() {
  int8_t prev_cs = mcp41_cs_gpio;
  bool   prev_ss = mcp41_use_sspi;

  MCP41_DetectPinsFromConfig();

  if (prev_cs != mcp41_cs_gpio || prev_ss != mcp41_use_sspi) {
    mcp41_spi_inited = false;                    // re-init requis
    if (mcp41_cs_gpio >= 0) {
      pinMode(mcp41_cs_gpio, OUTPUT);
      digitalWrite(mcp41_cs_gpio, HIGH);         // CS inactif
      AddLog(LOG_LEVEL_INFO, PSTR(LOG_TAG ": CS=GPIO%d (%s)"), mcp41_cs_gpio, mcp41_use_sspi ? "SSPI" : "SPI");
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
    #ifdef ESP32
    // SPI matériel : si broches renseignées, les utiliser, sinon SPI.begin() défaut
    if (mcp41_spi_clk >= 0 || mcp41_spi_miso >= 0 || mcp41_spi_mosi >= 0) {
      // Certaines cartes autorisent SPI.begin(SCK,MISO,MOSI,CS)
      SPI.begin((mcp41_spi_clk >= 0) ? mcp41_spi_clk : SCK,
                (mcp41_spi_miso >= 0) ? mcp41_spi_miso : MISO,
                (mcp41_spi_mosi >= 0) ? mcp41_spi_mosi : MOSI,
                -1);
    } else {
      SPI.begin();
    }
    #endif
    #ifdef ESP8266
    SPI.begin();
    #endif
  }

  mcp41_spi_inited = true;
  AddLog(LOG_LEVEL_INFO, PSTR(LOG_TAG ": Bus %s initialisé"), mcp41_use_sspi ? "SSPI" : "SPI");
}

// ------------------------------ SoftSPI (bit-bang) --------------------------

static inline void SSPI_Delay() {
  if (MCP41_SSPI_DELAY_US > 0) delayMicroseconds(MCP41_SSPI_DELAY_US);
}

/**
 * @brief Transmet un byte via SoftSPI
 */
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

/**
 * @brief Convertit une valeur en pourcentage
 * @param value Valeur à convertir
 * @return Valeur en pourcentage
 */
float MCP41_ConvertToPercent(uint8_t value) {
  if (value < mcp41_raw_min) {
    return 0.0f;
  } else if (value > mcp41_raw_max) {
    return 100.0f;
  }
  return (float)(value - mcp41_raw_min) / (float)(mcp41_raw_max - mcp41_raw_min) * 100.0f;
}

/**
 * @brief Convertit une valeur en pourcentage
 * @param percent Valeur à convertir
 * @return Valeur en pourcentage
 */
uint8_t MCP41_ConvertToValue(float percent) {
  if (percent < 0.0f) {
    return mcp41_raw_min;
  } else if (percent > 100.0f) {
    return mcp41_raw_max;
  }
  return (uint8_t)(percent * (float)(mcp41_raw_max - mcp41_raw_min) / 100.0f) + mcp41_raw_min;
}

// ------------------------------ Écriture MCP41 ------------------------------

/**
 * @brief Écrit une valeur dans le MCP41
 * @param value Valeur à écrire, en pourcentage
 * @return true si l'écriture a réussi, false sinon
 */
static bool MCP41_Write(float percent) {
  if (mcp41_cs_gpio < 0) { AddLog(LOG_LEVEL_ERROR, PSTR(LOG_TAG ": CS indisponible")); return false; }
  if (!mcp41_spi_inited) MCP41_InitSpiIfNeeded();
  if (!mcp41_spi_inited) return false;

  uint8_t value = MCP41_ConvertToValue(percent);

  // Debug: afficher les données à transmettre
  AddLog(LOG_LEVEL_DEBUG, PSTR(LOG_TAG ": Writing value %d (0x%02X, percent %f) via %s"), value, value, percent, mcp41_use_sspi ? "SSPI" : "SPI");
  AddLog(LOG_LEVEL_DEBUG, PSTR(LOG_TAG ": Opcode: 0x%02X, Data: 0x%02X"), MCP41_CMD_WRITE_WIPER, value);

  if (mcp41_use_sspi) {
    AddLog(LOG_LEVEL_DEBUG, PSTR(LOG_TAG ": SSPI - Setting CS LOW"));
    MCP41_Select(true);
    AddLog(LOG_LEVEL_DEBUG, PSTR(LOG_TAG ": SSPI - Transferring opcode 0x%02X"), MCP41_CMD_WRITE_WIPER);
    SSPI_TransferByte((uint8_t)MCP41_CMD_WRITE_WIPER); // opcode
    AddLog(LOG_LEVEL_DEBUG, PSTR(LOG_TAG ": SSPI - Transferring data 0x%02X"), value);
    SSPI_TransferByte(value);                          // data
    AddLog(LOG_LEVEL_DEBUG, PSTR(LOG_TAG ": SSPI - Setting CS HIGH"));
    MCP41_Select(false);
  } else {
    SPI.beginTransaction(SPISettings(MCP41_SPI_CLOCK, MSBFIRST, MCP41_SPI_MODE));
    AddLog(LOG_LEVEL_DEBUG, PSTR(LOG_TAG ": SPI - Setting CS LOW"));
    MCP41_Select(true);
    AddLog(LOG_LEVEL_DEBUG, PSTR(LOG_TAG ": SPI - Transferring opcode 0x%02X"), MCP41_CMD_WRITE_WIPER);
    SPI.transfer((uint8_t)MCP41_CMD_WRITE_WIPER);     // opcode
    AddLog(LOG_LEVEL_DEBUG, PSTR(LOG_TAG ": SPI - Transferring data 0x%02X"), value);
    SPI.transfer(value);                               // data
    AddLog(LOG_LEVEL_DEBUG, PSTR(LOG_TAG ": SPI - Setting CS HIGH"));
    MCP41_Select(false);
    SPI.endTransaction();
  }

  mcp41_last_value = percent;
  AddLog(LOG_LEVEL_DEBUG, PSTR(LOG_TAG ": Write completed, last_value = %f"), mcp41_last_value);
  return true;
}

/**
 * Try to set value
 */
static bool MCP41_TrySetValue(float val) {
  if (val > 100.0f) val = 100.0f;
  if (val < 0.0f) val = 0.0f;

  MCP41_InitSpiIfNeeded();
  if (mcp41_cs_gpio < 0) {
    Response_P(PSTR("{\"MCP41\":\"CS missing (set SPI/SSPI CS or use MCP41CS)\"}"));
    return false;
  }

  bool ok = MCP41_Write(val);
  if (ok) {
    MCP41_UpdateBootRule(val);
  }

  return ok;
}

// ------------------------------ Persistance (Rule) --------------------------

static void MCP41_UpdateBootRule(float value) {
  // RuleX on System#Boot do MCP41 <value> endon ; RuleX 1
  char cmnd[96];
  snprintf_P(cmnd, sizeof(cmnd), PSTR("Rule%d on System#Boot do MCP41 %f endon"), MCP41_RULE_SLOT, value);
  ExecuteCommand(cmnd, SRC_IGNORE);

  snprintf_P(cmnd, sizeof(cmnd), PSTR("Rule%d 1"), MCP41_RULE_SLOT);
  ExecuteCommand(cmnd, SRC_IGNORE);

  AddLog(LOG_LEVEL_INFO, PSTR(LOG_TAG ": Boot rule set (Rule%d -> MCP41 %f)"), MCP41_RULE_SLOT, value);
}

// ------------------------------ Commandes console ---------------------------

static void Cmnd_MCP41() {
  if (XdrvMailbox.data_len == 0) {
    Response_P(PSTR("{\"MCP41\":{\"last\":%f,\"cs\":%d,\"bus\":\"%s\"}}"),
               mcp41_last_value,
               (int)mcp41_cs_gpio,
               mcp41_use_sspi ? "SSPI" : (mcp41_cs_gpio >= 0 ? "SPI" : "unset"));
    return;
  }

  bool success = MCP41_TrySetValue(CharToFloat(XdrvMailbox.data));
  if (success) {
    Response_P(PSTR("{\"MCP41\":%f}"), mcp41_last_value);
  } else {
    Response_P(PSTR("{\"MCP41\":\"error\"}"));
  }
}

static void Cmnd_MCP41GET() {
  Response_P(PSTR("{\"MCP41\":%f}"), mcp41_last_value);
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

static void Cmnd_MCP41ADD() {
  if (XdrvMailbox.data_len == 0) {
    Response_P(PSTR("{\"MCP41ADD\":\"Increment parameter missing! (-100.0 to 100.0)\"}"));
    return;
  }

  float previous_value = mcp41_last_value;
  float add = CharToFloat(XdrvMailbox.data);
  bool success = MCP41_TrySetValue(mcp41_last_value + add);
  if (success) {
    Response_P(PSTR("{\"MCP41ADD\":{\"previous\":%f, \"add\":%f, \"value\":%f}}"), previous_value, add, mcp41_last_value);
  } else {
    Response_P(PSTR("{\"MCP41ADD\":\"error\"}"));
  }
}

#ifdef USE_WEBSERVER

// --- Ligne d’état dans la page principale -----------------
static void MCP41_WebSensor() {
  // Affiche: MCP41 Wiper | 128 (SPI, CS=5)
  WSContentSend_P(PSTR("{s}MCP41 Wiper{m}%f (%s, CS=%d){e}"),
                  mcp41_last_value,
                  mcp41_use_sspi ? "SSPI" : (mcp41_cs_gpio >= 0 ? "SPI" : "unset"),
                  (int)mcp41_cs_gpio);
}

// --- Boutons rapides (0 / 50 / 100 %) ---------------------
static void MCP41_WebButtons() {
  // Liens WebUI -> commandes (espace encodé %20 ; % doublé pour printf)
  WSContentSend_P(PSTR(
    "<div style='text-align: center;'>"
      "<button name='mcp41_0' style='background: var(--c_btn);' onclick=\"fetch('cm?cmnd=MCP41%%200');\">0%%</button> "
      "<button name='mcp41_50' style='background: var(--c_btn);' onclick=\"fetch('cm?cmnd=MCP41%%2050');\">50%%</button> "
      "<button name='mcp41_100' style='background: var(--c_btn);' onclick=\"fetch('cm?cmnd=MCP41%%20100');\">100%%</button> "
    "</div>"
  ));
}

#endif  // USE_WEBSERVER


// Table & dispatch
static const char kMCP41Commands[] PROGMEM = "|GET|CS";

enum MCP41_Commands {                                // commands useable in console or rules
  CMND_MCP41,                                        // MCP41 <0.0..100.0> - écrire le wiper
  CMND_MCP41GET,                                     // MCP41GET - lire la dernière valeur
  CMND_MCP41CS,                                      // MCP41CS <gpio> - override CS
  CMND_MCP41ADD                                      // MCP41ADD <-100.0..100.0> - increase value (but result will cannot below 0.0 and up to 100.0)
};

void (* const MCP41Command[])(void) PROGMEM = { Cmnd_MCP41, Cmnd_MCP41GET, Cmnd_MCP41CS, Cmnd_MCP41ADD };

static bool MCP41_CommandDispatcher() {
  char command[CMDSZ];
  bool serviced = true;
  uint8_t disp_len = strlen("MCP41");

  // Debug: afficher que la fonction est appelée
  AddLog(LOG_LEVEL_DEBUG, PSTR(LOG_TAG ": CommandDispatcher called with topic '%s', index %d, data '%s'"), 
         XdrvMailbox.topic, XdrvMailbox.index, XdrvMailbox.data);

  // Cas 1: Topic commence par "MCP41" (ex: MCP41GET, MCP41CS)
  if (!strncasecmp_P(XdrvMailbox.topic, PSTR("MCP41"), disp_len))
  {
    int command_code = GetCommandCode(command, sizeof(command), XdrvMailbox.topic + disp_len, kMCP41Commands);

    AddLog(LOG_LEVEL_DEBUG, PSTR(LOG_TAG ": GetCommandCode returned %d for command '%s'"), command_code, command);

    switch (command_code)
    {
      case CMND_MCP41:  // MCP41 (avec valeur)
        AddLog(LOG_LEVEL_DEBUG, PSTR(LOG_TAG ": Command MCP41 matched"));
        Cmnd_MCP41();
        break;

      case CMND_MCP41GET:  // MCP41GET
        AddLog(LOG_LEVEL_DEBUG, PSTR(LOG_TAG ": Command MCP41GET matched"));
        Cmnd_MCP41GET();
        break;

      case CMND_MCP41CS:  // MCP41CS
        AddLog(LOG_LEVEL_DEBUG, PSTR(LOG_TAG ": Command MCP41CS matched"));
        Cmnd_MCP41CS();
        break;
      
      case CMND_MCP41ADD: // MCP41ADD
        AddLog(LOG_LEVEL_DEBUG, PSTR(LOG_TAG ": Command MCP41ADD matched"));
        Cmnd_MCP41ADD();
        break;

      default:
        // Unknown command
        AddLog(LOG_LEVEL_DEBUG, PSTR(LOG_TAG ": Unknown command code %d"), command_code);
        serviced = false;
        break;
    }
  }
  // Cas 2: Topic = "MCP" et index = 41 (ex: MCP41 128)
  else if (!strcasecmp_P(XdrvMailbox.topic, PSTR("MCP")) && XdrvMailbox.index == 41)
  {
    AddLog(LOG_LEVEL_DEBUG, PSTR(LOG_TAG ": Command MCP41 matched via MCP+index41"));
    Cmnd_MCP41();
  }
  else {
    AddLog(LOG_LEVEL_DEBUG, PSTR(LOG_TAG ": Topic prefix 'MCP41' not found in '%s' and not MCP+41"), XdrvMailbox.topic);
    return false;
  }
  
  return serviced;
}

// ------------------------------ Entry point ---------------------------------

bool Xdrv100(uint32_t function) {
  bool handled = false;

  switch (function) {
    case FUNC_PRE_INIT:
      MCP41_ResolveBusAndPins();
      break;

    case FUNC_INIT:
      MCP41_InitSpiIfNeeded();
      break;

    case FUNC_ACTIVE:
      // Indiquer que le driver est actif si les broches sont configurées
      handled = (mcp41_cs_gpio >= 0);
      break;

    case FUNC_COMMAND:
      AddLog(LOG_LEVEL_DEBUG, PSTR(LOG_TAG ": FUNC_COMMAND - calling dispatcher"));
      handled = MCP41_CommandDispatcher();
      break;

#ifdef USE_WEBSERVER
    // Home page
    case FUNC_WEB_SENSOR:
      MCP41_WebSensor();
      break;

    case FUNC_WEB_COL_SENSOR:
      MCP41_WebButtons();
      break;

    // Page de configuration
    case FUNC_WEB_ADD_BUTTON:
      break;
#endif

    default:
      break;
  }
  return handled;
}

#endif  // USE_MCP41X
