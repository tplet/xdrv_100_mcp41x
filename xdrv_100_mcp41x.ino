/*
  xdrv_100_mcp41x.ino — Driver custom MCP41xxx (ex. MCP41010) pour Tasmota

  Fonctionnalités :
    - Rôle GPIO custom "MCP41 CS" (sélectionnable dans Configure Module)
    - Commandes :
        * MCP41 <0..255>  : écrit le wiper (8 bits). Sans argument => renvoie la dernière valeur + infos CS.
        * MCP41GET        : alias lecture (renvoie la dernière valeur).
        * MCP41CS <gpio>  : override runtime du CS ; -1 => retour à la config GUI ("MCP41 CS").
    - Persistance/re-applique au boot : la dernière valeur est stockée via une Rule Tasmota ("Rule3" par défaut)
      sous forme : on System#Boot do MCP41 <val> endon

  Prérequis build :
    - USE_SPI activé
    - Ce fichier placé dans tasmota/tasmota_xdrv_driver/

  A ajuster :
    - CMD_WRITE_WIPER : vérifie l’opcode exact pour TA référence MCP41xxx (datasheet)
    - SPISettings : fréquence, mode, etc. si nécessaire
    - SLOT de Rule utilisé pour la persistance (par défaut Rule3)

  Auteur : toi :)
*/

#ifdef USE_MCP41X

#ifndef USE_SPI
  #define USE_SPI
#endif

#include "tasmota.h"
#include <SPI.h>

// -----------------------------------------------------------------------------
// Déclaration du rôle GPIO custom pour le CS (apparaît dans le WebUI)
// IMPORTANT : ces symboles (kGpioUser / kGpioUserNames) ne doivent être définis
//             qu'une seule fois dans tout le firmware.
// -----------------------------------------------------------------------------

#define USE_CUSTOM_GPIO

#ifndef GPIO_MCP41_CS
  #define GPIO_MCP41_CS 590          // ID "user" libre (évite collisions)
#endif
#ifndef D_GPIO_MCP41_CS
  #define D_GPIO_MCP41_CS "MCP41 CS" // Libellé dans le menu GPIO
#endif

const uint16_t kGpioUser[] PROGMEM = {
  GPIO_MCP41_CS, 0                   // 0 = terminator
};

const char kGpioUserNames[] PROGMEM =
  D_GPIO_MCP41_CS "|";

// -----------------------------------------------------------------------------
// Configuration du driver
// -----------------------------------------------------------------------------

#define XDRV_100              100
#define LOG_TAG               "MCP41"

// Slot de Rule utilisé pour persister/re-appliquer au boot.
// Tasmota propose Rule1..Rule3. Choisis-en un libre.
#ifndef MCP41_RULE_SLOT
  #define MCP41_RULE_SLOT 3
#endif

// Exemple d’opcode d’écriture (à VALIDER avec TA datasheet MCP41xxx)
// Pour MCP41010 / famille MCP41xxx, la trame 16 bits est [opcode+addr][data].
// Beaucoup de docs citent 0x11 (Write Data, pot0). Vérifie et adapte au besoin.
#ifndef MCP41_CMD_WRITE_WIPER
  #define MCP41_CMD_WRITE_WIPER 0x11 // Parfois aussi noté B00010001
#endif

// Paramètres SPI (fréquence, mode, ordre des bits)
#ifndef MCP41_SPI_CLOCK
  #define MCP41_SPI_CLOCK 14000000     // 14 MHz
#endif

#ifndef MCP41_SPI_MODE
  #define MCP41_SPI_MODE SPI_MODE0
#endif

// -----------------------------------------------------------------------------
// Etat du driver
// -----------------------------------------------------------------------------

static int8_t  mcp41_cs_gpio      = -1;   // CS effectif (override ou config GUI)
static int8_t  mcp41_cs_override  = -1;   // -1 => pas d’override
static bool    mcp41_spi_inited   = false;
static uint8_t mcp41_last_value   = 0;    // dernière valeur écrite (volatile)

// -----------------------------------------------------------------------------
// Utilitaires
// -----------------------------------------------------------------------------

inline void MCP41_Select(bool en) {
  if (mcp41_cs_gpio < 0) return;
  digitalWrite(mcp41_cs_gpio, en ? LOW : HIGH);
}

static void MCP41_ResolveCsFromConfig() {
  int8_t new_cs = -1;

  if (mcp41_cs_override >= 0) {
    new_cs = mcp41_cs_override;                // priorité à l’override
  } else if (PinUsed(GPIO_MCP41_CS)) {
    new_cs = (int8_t)Pin(GPIO_MCP41_CS);       // via GUI ("MCP41 CS")
  }

  if (new_cs != mcp41_cs_gpio) {
    mcp41_cs_gpio = new_cs;
    if (mcp41_cs_gpio >= 0) {
      pinMode(mcp41_cs_gpio, OUTPUT);
      digitalWrite(mcp41_cs_gpio, HIGH);       // CS inactif
      AddLog(LOG_LEVEL_INFO, PSTR(LOG_TAG ": CS=GPIO%d (%s)"),
             mcp41_cs_gpio, (mcp41_cs_override >= 0 ? "override" : "GPIO_MCP41_CS"));
    } else {
      AddLog(LOG_LEVEL_INFO, PSTR(LOG_TAG ": CS non défini (GUI/override)"));
    }
    mcp41_spi_inited = false; // force ré-init si changement
  }
}

static void MCP41_InitSpiIfNeeded() {
  MCP41_ResolveCsFromConfig();
  if (mcp41_spi_inited || mcp41_cs_gpio < 0) return;

  SPI.begin();   // SPI matériel par défaut (SCK/MISO/MOSI de la carte)
  mcp41_spi_inited = true;
  AddLog(LOG_LEVEL_INFO, PSTR(LOG_TAG ": SPI prêt"));
}

// Ecrit le wiper (0..255) sur MCP41xxx
static bool MCP41_Write(uint8_t value) {
  if (mcp41_cs_gpio < 0) { AddLog(LOG_LEVEL_ERROR, PSTR(LOG_TAG ": CS indisponible")); return false; }
  if (!mcp41_spi_inited) MCP41_InitSpiIfNeeded();
  if (!mcp41_spi_inited) return false;

  SPI.beginTransaction(SPISettings(MCP41_SPI_CLOCK, MSBFIRST, MCP41_SPI_MODE));
  MCP41_Select(true);
  SPI.transfer((uint8_t)MCP41_CMD_WRITE_WIPER);  // <-- valide cet opcode
  SPI.transfer(value);
  MCP41_Select(false);
  SPI.endTransaction();

  mcp41_last_value = value;
  return true;
}

// -----------------------------------------------------------------------------
// Persistance via Rule: on System#Boot do MCP41 <last> endon
// -----------------------------------------------------------------------------

static void MCP41_UpdateBootRule(uint8_t value) {
  // Construit la chaîne de la Rule et l'active
  // Exemple: Rule3 on System#Boot do MCP41 128 endon
  char cmnd[96];
  snprintf_P(cmnd, sizeof(cmnd), PSTR("Rule%d on System#Boot do MCP41 %u endon"), MCP41_RULE_SLOT, (unsigned)value);
  ExecuteCommand(cmnd, SRC_IGNORE);

  snprintf_P(cmnd, sizeof(cmnd), PSTR("Rule%d 1"), MCP41_RULE_SLOT);
  ExecuteCommand(cmnd, SRC_IGNORE);

  AddLog(LOG_LEVEL_INFO, PSTR(LOG_TAG ": Boot rule set (Rule%d -> MCP41 %u)"), MCP41_RULE_SLOT, (unsigned)value);
}

// (Optionnel) Désactiver la rule si besoin (non utilisé ici)
// static void MCP41_DisableBootRule() {
//   char cmnd[24];
//   snprintf_P(cmnd, sizeof(cmnd), PSTR("Rule%d 0"), MCP41_RULE_SLOT);
//   ExecuteCommand(cmnd, SRC_IGNORE);
// }

// -----------------------------------------------------------------------------
// Commandes console
// -----------------------------------------------------------------------------

// MCP41 <0..255>  | MCP41  (sans argument => lecture)
static void Cmnd_MCP41() {
  if (XdrvMailbox.data_len == 0) {
    Response_P(PSTR("{\"MCP41\":{\"last\":%u,\"cs\":%d,\"source\":\"%s\"}}"),
               (unsigned)mcp41_last_value,
               (int)mcp41_cs_gpio,
               (mcp41_cs_override >= 0 ? "override" : (PinUsed(GPIO_MCP41_CS) ? "GPIO_MCP41_CS" : "unset")));
    return;
  }

  uint32_t val = XdrvMailbox.payload;
  if (val > 255) val = 255;

  MCP41_InitSpiIfNeeded();
  if (mcp41_cs_gpio < 0) {
    Response_P(PSTR("{\"MCP41\":\"CS missing (set MCP41 CS in GUI or use MCP41CS)\"}"));
    return;
  }

  bool ok = MCP41_Write((uint8_t)val);
  if (ok) {
    // Persiste via Rule et répond
    MCP41_UpdateBootRule((uint8_t)val);
    Response_P(PSTR("{\"MCP41\":%u}"), (unsigned)val);
  } else {
    Response_P(PSTR("{\"MCP41\":\"error\"}"));
  }
}

// MCP41GET -> renvoie la dernière valeur
static void Cmnd_MCP41GET() {
  Response_P(PSTR("{\"MCP41\":{\"last\":%u}}"), (unsigned)mcp41_last_value);
}

// MCP41CS <gpio>  (override ; -1 pour retour au GPIO du GUI)
static void Cmnd_MCP41CS() {
  if (XdrvMailbox.data_len == 0) {
    Response_P(PSTR("{\"MCP41CS\":{\"override\":%d,\"effective\":%d}}"),
               (int)mcp41_cs_override, (int)mcp41_cs_gpio);
    return;
  }

  int32_t gpio = XdrvMailbox.payload; // peut être -1
  if (gpio < -1 || gpio > 39) {
    Response_P(PSTR("{\"MCP41CS\":\"invalid\"}"));
    return;
  }

  mcp41_cs_override = (int8_t)gpio;  // -1 => override off
  MCP41_ResolveCsFromConfig();
  MCP41_InitSpiIfNeeded();

  Response_P(PSTR("{\"MCP41CS\":{\"override\":%d,\"effective\":%d}}"),
             (int)mcp41_cs_override, (int)mcp41_cs_gpio);
}

// Table & dispatch
static const char kMCP41Commands[] PROGMEM = "MCP41|MCP41GET|MCP41CS";
void (* const MCP41Command[])(void) PROGMEM = { Cmnd_MCP41, Cmnd_MCP41GET, Cmnd_MCP41CS };

static bool MCP41_CommandDispatcher() {
  return CommandMatcher(kMCP41Commands, MCP41Command);
}

// -----------------------------------------------------------------------------
// Entry point du driver
// -----------------------------------------------------------------------------

bool Xdrv100(uint8_t function) {
  bool handled = false;

  switch (function) {
    case FUNC_PRE_INIT:
      // Lit la config (GUI / override) dès le boot
      MCP41_ResolveCsFromConfig();
      break;

    case FUNC_INIT:
      // Init SPI si CS dispo
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

