// ------------------------- ATTENZIONE  ------------------------//
// SETTARE LOGICA RELAY_CORRETTA PER OPTOISOLATO O RELAY NORMALE //

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <Wire.h>

/*
 * TERMOSTATO INTELLIGENTE ESP32 - VERSIONE RELÈ SINGOLO
 * Framework: Arduino per ESP32
 * Platform: Espressif32
 * 
 * VERSIONE MODIFICATA CON RELÈ SINGOLO:
 * - Un unico relè sul pin 32 per heating/cooling
 * - Logica di controllo unificata basata su modalità
 * - Display semplificato per singolo output
 * - Mantenute tutte le features UX avanzate
 * 
 * COMPONENTI HARDWARE:
 * - ESP32-WROOM-32 (DevKit)
 * - LCD I2C 4x20 (PCF8574 backpack) con gestione backlight
 * - Encoder rotativo KY-040 + resistenze pull-up 4.7kΩ
 * - Modulo relay SINGOLO OPTOISOLATO (GTZ817C) - PIN 32
 * - Sensore DS18B20 OneWire
 * 
 * MODIFICHE PRINCIPALI:
 * - Rimosso RELAY_COOLER (pin 33)
 * - Unificata logica controllo per singolo relè
 * - Aggiornato display per mostrare stato singolo output
 * - Semplificati messaggi diagnostici
 */

// ===== FORWARD DECLARATIONS per PlatformIO =====
// Hardware setup functions
void setupHardware();
void setupLCD();
void setupTemperatureSensor();
void setupEncoder();
void setupEEPROM();
void scanI2CDevices();
void createCustomChars();

// Configuration management - SISTEMA TEMPORANEO
void loadConfiguration();
void saveConfiguration();
void loadTempConfiguration();
void saveTempConfiguration();
void cancelTempConfiguration();

// Temperature control - MODIFICATO PER RELÈ SINGOLO
void updateTemperatureReading();
void controlThermostatLogic();
void updateRelayState(bool shouldActivate);
void deactivateRelay();

// Input handling - POLLING VERSION
void readEncoderPolling();
void handleEncoderInput();
void processEncoderRotation();
void processButtonPress();

// Display management - 4x20 VERSION
void updateDisplayContent();
void displayMainScreen();
void displaySettingsMenu();
void displayTemperatureSetting();
void displayDeltaSetting();
void displayModeSetting();
void displaySystemStateSetting();
void displayDiagnosticScreen();

// Backlight management - SISTEMA AUTOMATICO + AUTO RETURN
void updateBacklightManagement();
void setBacklight(bool state, bool immediate = true);
void resetUserActivityTimer();
bool isUserActive();
void toggleBacklightMode();
void forceBacklightOn();
void getBacklightStatus();

// Helper functions
const char* getMenuValue(int index);
void createProgressBar(char* buffer, float value, float min, float max, int length);
void createTemperatureGraph(char* buffer, float current, float target, float delta);

// ===== CONFIGURAZIONI HARDWARE =====
// I2C Bus (condiviso LCD + eventuali expander)
#define I2C_SDA   21
#define I2C_SCL   22

// LCD I2C Address (tipicamente 0x27 o 0x3F)
#define LCD_I2C_ADDRESS  0x27

// Encoder rotativo KY-040 (connessione diretta + pull-up esterne 4.7kΩ)
#define ENCODER_CLK  25
#define ENCODER_DT   26
#define ENCODER_SW   27

// Sensore temperatura DS18B20 (OneWire)
#define TEMP_SENSOR_PIN  19

// MODIFICA: Modulo relay SINGOLO OPTOISOLATO (GTZ817C)
#define RELAY_OUTPUT  33  // Un solo relè per heating/cooling

// LED di stato sistema
#define LED_STATUS  23

// ===== CONFIGURAZIONI SISTEMA =====
#define TEMP_READ_INTERVAL    2000    // ms - intervallo lettura temperatura
#define DISPLAY_UPDATE_INTERVAL 500   // ms - aggiornamento display
#define DEBOUNCE_DELAY        50      // ms - debounce pulsante encoder
#define ENCODER_POLLING_INTERVAL 8    // ms - polling encoder ottimizzato (125Hz)

// CONFIGURAZIONI BACKLIGHT AUTOMATICO
#define BACKLIGHT_TIMEOUT     30000   // ms - 30 secondi timeout backlight
#define BACKLIGHT_FADE_STEPS  10      // Steps per fade out smooth (futuro)
#define BACKLIGHT_CHECK_INTERVAL 100  // ms - intervallo controlli backlight

#define TEMP_MIN             -50.0f   // °C - temperatura minima configurabile
#define TEMP_MAX             150.0f   // °C - temperatura massima configurabile
#define DELTA_MIN            0.5f     // °C - delta minimo configurabile
#define DELTA_MAX            5.0f     // °C - delta massimo configurabile

#define EEPROM_SIZE          512      // bytes - dimensione EEPROM allocata
#define EEPROM_SETTINGS_ADDR 0        // indirizzo base settings

// CONFIGURAZIONI MENU AVANZATE
#define MENU_ITEMS_COUNT     7        // 7 voci menu (0-6)
#define DISPLAY_BUFFER_SIZE  24       // Buffer sicuro

// ===== CONFIGURAZIONE RELAY OPTOISOLATO =====
// GTZ817C Optoaccoppiatore: logica invertita
// HIGH = Relay OFF, LOW = Relay ON
#define RELAY_LOGIC_INVERTED  false

// ===== CARATTERI PERSONALIZZATI =====
#define CHAR_THERMOMETER 0
#define CHAR_ARROW_UP    1
#define CHAR_ARROW_DOWN  2
#define CHAR_DEGREE      3

// ===== INIZIALIZZAZIONE OGGETTI =====
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, 20, 4);  // LCD 4x20
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature temperatureSensor(&oneWire);

// ===== STRUTTURE DATI =====
struct ThermostatConfig {
  float targetTemperature;    // Temperatura obiettivo (-50.0 - 150.0°C)
  float deltaTemperature;     // Isteresi temperatura (0.5 - 5.0°C)
  bool systemEnabled;         // Sistema attivo/disattivo
  uint8_t operatingMode;      // 0=HEATER, 1=COOLER
  bool isConfigured;          // Flag prima configurazione
  uint32_t configVersion;     // Versione configurazione per compatibilità
};

// ===== VARIABILI GLOBALI =====
ThermostatConfig config;        // Configurazione attiva (salvata)
ThermostatConfig tempConfig;    // Configurazione temporanea (in editing)
bool isEditingConfig = false;   // Flag modalità editing

// Variabili operative sistema - MODIFICATO PER RELÈ SINGOLO
float currentTemperature = 0.0f;
bool relayActive = false;       // MODIFICA: Stato del singolo relè
bool temperatureSensorError = false;
unsigned long lastTemperatureRead = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long systemStartTime = 0;

// Variabili encoder POLLING VERSION (NON più volatile)
int encoderPosition = 0;
bool encoderChanged = false;
int lastEncoderPosition = 0;
bool buttonPressed = false;
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;

// Variabili specifiche per polling encoder
bool lastCLKState = HIGH;
bool lastDTState = HIGH;
unsigned long lastEncoderPoll = 0;

// VARIABILI GESTIONE BACKLIGHT AUTOMATICO
unsigned long lastUserActivity = 0;    // Timestamp ultima attività utente
bool backlightActive = true;           // Stato attuale backlight
bool backlightAutoMode = true;         // Modalità automatica attiva
unsigned long backlightFadeStart = 0;  // Per fade graduale (futuro)
unsigned long lastBacklightCheck = 0;  // Race condition protection

// Sistema menu multi-livello esteso
enum MenuState {
  MAIN_DISPLAY,
  SETTINGS_MENU,
  TEMPERATURE_SETTING,
  DELTA_SETTING,
  MODE_SETTING,
  SYSTEM_STATE_SETTING,
  DIAGNOSTIC_SCREEN
};

MenuState currentMenuState = MAIN_DISPLAY;
int menuSelectionIndex = 0;
bool inSubMenu = false;

// Statistiche sistema
float temperatureHistory[4] = {0.0f, 0.0f, 0.0f, 0.0f};
int historyIndex = 0;
unsigned long totalOnTime = 0;
unsigned long lastRelayChange = 0;

// ===== FUNZIONI GESTIONE CONFIGURAZIONE TEMPORANEA =====
void loadTempConfiguration() {
  // Copia configurazione attiva in quella temporanea per editing
  tempConfig = config;
  
  // Validazione tempConfig per sicurezza
  tempConfig.targetTemperature = constrain(tempConfig.targetTemperature, TEMP_MIN, TEMP_MAX);
  tempConfig.deltaTemperature = constrain(tempConfig.deltaTemperature, DELTA_MIN, DELTA_MAX);
  tempConfig.operatingMode = constrain(tempConfig.operatingMode, 0, 1);
  
  isEditingConfig = true;
  Serial.println("📝 Configurazione temporanea caricata e validata");
}

void saveTempConfiguration() {
  // Validazione prima del salvataggio
  tempConfig.targetTemperature = constrain(tempConfig.targetTemperature, TEMP_MIN, TEMP_MAX);
  tempConfig.deltaTemperature = constrain(tempConfig.deltaTemperature, DELTA_MIN, DELTA_MAX);
  tempConfig.operatingMode = constrain(tempConfig.operatingMode, 0, 1);
  
  // Salva configurazione temporanea come definitiva
  config = tempConfig;
  saveConfiguration();
  isEditingConfig = false;
  Serial.println("💾 Configurazione temporanea validata e salvata definitivamente");
}

void cancelTempConfiguration() {
  // Annulla modifiche temporanee, ripristina originale
  tempConfig = config;  // Ripristina valori originali
  isEditingConfig = false;
  Serial.println("❌ Modifiche temporanee annullate, configurazione ripristinata");
}

// ===== FUNZIONI GESTIONE BACKLIGHT AUTOMATICO + AUTO RETURN =====
void updateBacklightManagement() {
  if (!backlightAutoMode) {
    return; // Modalità automatica disabilitata
  }
  
  unsigned long currentTime = millis();
  
  // Race condition protection - evita controlli troppo frequenti
  if (currentTime - lastBacklightCheck < BACKLIGHT_CHECK_INTERVAL) {
    return;
  }
  lastBacklightCheck = currentTime;
  
  unsigned long inactiveTime = currentTime - lastUserActivity;
  
  // Check se è tempo di spegnere backlight
  if (backlightActive && inactiveTime >= BACKLIGHT_TIMEOUT) {
    // Double-check: riverifica attività prima di spegnere
    if ((millis() - lastUserActivity) >= BACKLIGHT_TIMEOUT) {
      setBacklight(false);
      
      // Auto-ritorno main screen
      if (currentMenuState != MAIN_DISPLAY) {
        // Se era in editing, annulla modifiche temporanee
        if (isEditingConfig) {
          cancelTempConfiguration();
        }
        currentMenuState = MAIN_DISPLAY;
        menuSelectionIndex = 0;
        Serial.println("🏠 Auto-ritorno main screen per backlight OFF");
      }
      
      Serial.println("💡 Backlight spenta per inattività");
    }
  }
  
  // Check se riattivare backlight per attività recente
  if (!backlightActive && inactiveTime < BACKLIGHT_TIMEOUT) {
    setBacklight(true);
    Serial.println("💡 Backlight riattivata per attività utente");
  }
}

void setBacklight(bool state, bool immediate) {
  if (backlightActive == state) {
    return; // Già nello stato richiesto
  }
  
  backlightActive = state;
  
  if (immediate) {
    // Cambio immediato
    if (state) {
      lcd.backlight();
      Serial.println("💡 Backlight ON");
    } else {
      lcd.noBacklight();
      Serial.println("💡 Backlight OFF");
    }
  } else {
    // TODO: Implementazione fade graduale (futuro)
    // Per ora usa cambio immediato
    if (state) {
      lcd.backlight();
    } else {
      lcd.noBacklight();
    }
  }
}

void resetUserActivityTimer() {
  unsigned long currentTime = millis();
  
  // Se backlight era spenta, riaccendila immediatamente
  if (!backlightActive && backlightAutoMode) {
    setBacklight(true);
    Serial.println("💡 Backlight riattivata da interazione utente");
  }
  
  lastUserActivity = currentTime;
  
  // Debug: stampa solo se backlight era spenta (evita spam)
  static bool wasInactive = false;
  if (!backlightActive || wasInactive) {
    Serial.printf("👆 Attività utente rilevata (tempo: %lu)\n", currentTime);
    wasInactive = false;
  }
  
  if (currentTime - lastUserActivity > BACKLIGHT_TIMEOUT - 5000) { // 5 sec prima
    wasInactive = true;
  }
}

bool isUserActive() {
  return (millis() - lastUserActivity) < BACKLIGHT_TIMEOUT;
}

void toggleBacklightMode() {
  backlightAutoMode = !backlightAutoMode;
  
  if (backlightAutoMode) {
    Serial.println("💡 Modalità backlight: AUTOMATICA");
    lastUserActivity = millis(); // Reset timer
  } else {
    Serial.println("💡 Modalità backlight: MANUALE");
    setBacklight(true); // Accendi in modalità manuale
  }
}

void forceBacklightOn() {
  backlightAutoMode = false;
  setBacklight(true);
  Serial.println("💡 Backlight forzata SEMPRE ACCESA");
}

void getBacklightStatus() {
  Serial.println("💡 ===== STATUS BACKLIGHT =====");
  Serial.printf("Stato attuale: %s\n", backlightActive ? "ACCESA" : "SPENTA");
  Serial.printf("Modalità: %s\n", backlightAutoMode ? "AUTOMATICA" : "MANUALE");
  
  if (backlightAutoMode) {
    unsigned long inactiveTime = millis() - lastUserActivity;
    unsigned long remainingTime = (BACKLIGHT_TIMEOUT > inactiveTime) ? 
                                  (BACKLIGHT_TIMEOUT - inactiveTime) : 0;
    Serial.printf("Ultima attività: %lu ms fa\n", inactiveTime);
    Serial.printf("Tempo rimanente: %lu ms\n", remainingTime);
  }
  Serial.println("==============================");
}

// ===== FUNZIONI DI SETUP =====
void setupHardware() {
  Serial.println("⚙️ Inizializzazione hardware...");
  
  // Inizializzazione bus I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); // 400kHz per I2C veloce
  Serial.printf("🔌 I2C inizializzato - SDA:%d, SCL:%d\n", I2C_SDA, I2C_SCL);
  
  // Configurazione pin digitali CON PULL-UP ESTERNE 4.7kΩ
  pinMode(ENCODER_CLK, INPUT);     // NON più INPUT_PULLUP (pull-up esterne)
  pinMode(ENCODER_DT, INPUT);      // NON più INPUT_PULLUP (pull-up esterne)
  pinMode(ENCODER_SW, INPUT);      // NON più INPUT_PULLUP (pull-up esterne)
  pinMode(RELAY_OUTPUT, OUTPUT);   // MODIFICA: Un solo pin per relè
  pinMode(LED_STATUS, OUTPUT);
  
  // STATO SICURO per relay optoisolato GTZ817C
  // Logica invertita: HIGH = Relay OFF, LOW = Relay ON
  if (RELAY_LOGIC_INVERTED) {
    digitalWrite(RELAY_OUTPUT, HIGH);  // Relay OFF sicuro
    Serial.println("🔒 Relay singolo optoisolato (GTZ817C) - Stato sicuro: HIGH = OFF");
  } else {
    digitalWrite(RELAY_OUTPUT, LOW);   // Relay OFF normale
    Serial.println("🔒 Relay singolo normale - Stato sicuro: LOW = OFF");
  }
  
  digitalWrite(LED_STATUS, LOW);
  Serial.println("✅ Pin configurati correttamente");
  Serial.println("🔌 Pull-up esterne 4.7kΩ - interne disabilitate");
  Serial.println("🎛️ Relè singolo sul pin 32 per heating/cooling");
}

void setupLCD() {
  Serial.println("🖥️ Inizializzazione LCD I2C 4x20...");
  
  // Inizializzazione LCD I2C 4x20
  lcd.init();
  lcd.backlight();
  lcd.clear();
  
  // Verifica comunicazione I2C
  Wire.beginTransmission(LCD_I2C_ADDRESS);
  if (Wire.endTransmission() == 0) {
    Serial.printf("✅ LCD I2C 4x20 trovato all'indirizzo 0x%02X\n", LCD_I2C_ADDRESS);
  } else {
    Serial.printf("❌ ERRORE: LCD I2C non trovato all'indirizzo 0x%02X\n", LCD_I2C_ADDRESS);
  }
  
  // Crea caratteri personalizzati
  createCustomChars();
  
  // Messaggio di benvenuto 4x20
  lcd.setCursor(0, 0);
  lcd.print("   ESP32 THERMOSTAT  ");
  lcd.setCursor(0, 1);
  lcd.print("  SINGLE RELAY v1.0  ");
  lcd.setCursor(0, 2);
  lcd.print("    One Output       ");
  lcd.setCursor(0, 3);
  lcd.print("   Ready to Rock!    ");
  delay(3000);
  
  Serial.println("✅ LCD I2C 4x20 inizializzato");
}

void createCustomChars() {
  // Carattere termometro
  byte thermometer[8] = {
    B00100,
    B01010,
    B01010,
    B01110,
    B11111,
    B11111,
    B01110,
    B00000
  };
  
  // Carattere freccia su
  byte arrowUp[8] = {
    B00100,
    B01110,
    B11111,
    B00100,
    B00100,
    B00100,
    B00100,
    B00000
  };
  
  // Carattere freccia giù
  byte arrowDown[8] = {
    B00100,
    B00100,
    B00100,
    B00100,
    B11111,
    B01110,
    B00100,
    B00000
  };
  
  // Carattere grado personalizzato
  byte degree[8] = {
    B01100,
    B10010,
    B10010,
    B01100,
    B00000,
    B00000,
    B00000,
    B00000
  };
  
  lcd.createChar(CHAR_THERMOMETER, thermometer);
  lcd.createChar(CHAR_ARROW_UP, arrowUp);
  lcd.createChar(CHAR_ARROW_DOWN, arrowDown);
  lcd.createChar(CHAR_DEGREE, degree);
}

void setupTemperatureSensor() {
  Serial.println("🌡️ Inizializzazione sensore temperatura...");
  temperatureSensor.begin();
  
  // Configurazione precisione sensore (12-bit = 0.0625°C risoluzione)
  temperatureSensor.setResolution(12);
  temperatureSensor.setWaitForConversion(false); // Lettura asincrona
  
  Serial.printf("📊 Sensori DS18B20 trovati: %d\n", temperatureSensor.getDeviceCount());
  Serial.println("✅ Sensore temperatura configurato");
}

void setupEncoder() {
  Serial.println("🎛️ Configurazione encoder (POLLING MODE)...");
  
  // NESSUN INTERRUPT - solo polling
  // Inizializzazione stati encoder
  lastCLKState = digitalRead(ENCODER_CLK);
  lastDTState = digitalRead(ENCODER_DT);
  
  Serial.println("✅ Encoder configurato in modalità POLLING");
  Serial.printf("   └─ Polling interval: %dms (125Hz - alta velocità)\n", ENCODER_POLLING_INTERVAL);
  Serial.println("   └─ Direzione: ORARIO = +, ANTIORARIO = -");
  Serial.println("   └─ Pull-up esterne 4.7kΩ attive");
}

void setupEEPROM() {
  Serial.println("💾 Inizializzazione EEPROM...");
  
  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("❌ ERRORE: Inizializzazione EEPROM fallita!");
    return;
  }
  
  Serial.printf("✅ EEPROM inizializzata (%d bytes)\n", EEPROM_SIZE);
}

// ===== FUNZIONI I2C UTILITY =====
void scanI2CDevices() {
  Serial.println("🔍 Scansione bus I2C...");
  int deviceCount = 0;
  
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.printf("📍 Dispositivo I2C trovato: 0x%02X\n", address);
      deviceCount++;
      
      // Identifica dispositivi comuni
      switch (address) {
        case 0x27:
        case 0x3F:
          Serial.println("   └─ Probabile LCD I2C 4x20");
          break;
        case 0x20:
        case 0x21:
        case 0x22:
        case 0x23:
          Serial.println("   └─ Probabile PCF8574 I2C Expander");
          break;
        case 0x48:
        case 0x49:
        case 0x4A:
        case 0x4B:
          Serial.println("   └─ Probabile ADS1115 ADC");
          break;
        default:
          Serial.println("   └─ Dispositivo sconosciuto");
          break;
      }
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("❌ Nessun dispositivo I2C trovato!");
  } else {
    Serial.printf("✅ Trovati %d dispositivi I2C\n", deviceCount);
  }
}

// ===== FUNZIONI CONFIGURAZIONE =====
void loadConfiguration() {
  Serial.println("📖 Caricamento configurazione...");
  
  EEPROM.get(EEPROM_SETTINGS_ADDR, config);
  
  if (!config.isConfigured || config.configVersion != 1) {
    Serial.println("🔧 Prima configurazione - caricamento valori default");
    
    // Configurazione di fabbrica
    config.targetTemperature = 22.0f;
    config.deltaTemperature = 1.0f;
    config.systemEnabled = false;
    config.operatingMode = 0; // HEATER
    config.isConfigured = true;
    config.configVersion = 1;
    
    saveConfiguration();
  } else {
    Serial.println("✅ Configurazione caricata da EEPROM");
  }
  
  // Validazione e sanitizzazione valori
  config.targetTemperature = constrain(config.targetTemperature, TEMP_MIN, TEMP_MAX);
  config.deltaTemperature = constrain(config.deltaTemperature, DELTA_MIN, DELTA_MAX);
  config.operatingMode = constrain(config.operatingMode, 0, 1);
  
  // Copia in configurazione temporanea
  tempConfig = config;
  
  // Log configurazione caricata
  Serial.printf("🎯 Target: %.1f°C\n", config.targetTemperature);
  Serial.printf("📊 Delta: %.1f°C\n", config.deltaTemperature);
  Serial.printf("🔧 Modalità: %s\n", config.operatingMode == 0 ? "HEATER" : "COOLER");
  Serial.printf("⚡ Sistema: %s\n", config.systemEnabled ? "ATTIVO" : "DISATTIVO");
}

void saveConfiguration() {
  config.isConfigured = true;
  config.configVersion = 1;
  
  EEPROM.put(EEPROM_SETTINGS_ADDR, config);
  if (EEPROM.commit()) {
    Serial.println("💾 Configurazione salvata in EEPROM");
  } else {
    Serial.println("❌ ERRORE: Salvataggio EEPROM fallito!");
  }
}

// ===== GESTIONE TEMPERATURA =====
void updateTemperatureReading() {
  static bool conversionRequested = false;
  static unsigned long conversionStartTime = 0;
  
  if (!conversionRequested) {
    // Richiedi conversione asincrona
    temperatureSensor.requestTemperatures();
    conversionRequested = true;
    conversionStartTime = millis();
    return;
  }
  
  // Attendi completamento conversione (750ms per 12-bit)
  if (millis() - conversionStartTime < 750) {
    return;
  }
  
  float temperature = temperatureSensor.getTempCByIndex(0);
  conversionRequested = false;
  
  // Validazione lettura
  if (temperature == DEVICE_DISCONNECTED_C || temperature < -55.0f || temperature > 125.0f) {
    if (!temperatureSensorError) {
      Serial.println("❌ ERRORE: Sensore temperatura disconnesso o malfunzionante");
      temperatureSensorError = true;
    }
  } else {
    if (temperatureSensorError) {
      Serial.println("✅ Sensore temperatura ripristinato");
      temperatureSensorError = false;
    }
    
    // Aggiorna storico temperature
    temperatureHistory[historyIndex] = currentTemperature;
    historyIndex = (historyIndex + 1) % 4;
    
    currentTemperature = temperature;
    Serial.printf("🌡️ Temperatura: %.2f°C\n", currentTemperature);
  }
}

// ===== CONTROLLO RELAY E LOGICA TERMOSTATO - MODIFICATO PER RELÈ SINGOLO =====
void controlThermostatLogic() {
  // USA SEMPRE LA CONFIGURAZIONE ATTIVA (NON quella temporanea)
  if (!config.systemEnabled || temperatureSensorError) {
    deactivateRelay();
    return;
  }
  
  bool shouldActivateRelay = false;
  
  // Logica di controllo con isteresi unificata per singolo relè
  if (config.operatingMode == 0) { // HEATER MODE
    // Attiva relè quando temperatura scende sotto (target - delta)
    if (currentTemperature < (config.targetTemperature - config.deltaTemperature)) {
      shouldActivateRelay = true;
    } else if (currentTemperature > config.targetTemperature) {
      shouldActivateRelay = false;
    } else {
      shouldActivateRelay = relayActive; // Mantieni stato corrente (isteresi)
    }
  } else { // COOLER MODE
    // Attiva relè quando temperatura sale sopra (target + delta)
    if (currentTemperature > (config.targetTemperature + config.deltaTemperature)) {
      shouldActivateRelay = true;
    } else if (currentTemperature < config.targetTemperature) {
      shouldActivateRelay = false;
    } else {
      shouldActivateRelay = relayActive; // Mantieni stato corrente (isteresi)
    }
  }
  
  // Aggiornamento stato relè singolo
  updateRelayState(shouldActivateRelay);
}

// MODIFICA: Funzione semplificata per gestire un solo relè
void updateRelayState(bool shouldActivate) {
  if (shouldActivate != relayActive) {
    // Statistiche tempo attivazione
    if (relayActive && !shouldActivate) {
      totalOnTime += millis() - lastRelayChange;
    }
    if (!relayActive && shouldActivate) {
      lastRelayChange = millis();
    }
    
    relayActive = shouldActivate;
    
    // Modalità operativa per logging
    const char* mode = (config.operatingMode == 0) ? "HEATER" : "COOLER";
    
    // Logica per relay optoisolato GTZ817C
    if (RELAY_LOGIC_INVERTED) {
      // Logica invertita: LOW = Relay ON, HIGH = Relay OFF
      digitalWrite(RELAY_OUTPUT, relayActive ? LOW : HIGH);
      Serial.printf("🔌 OUTPUT (%s): %s (pin=%s - GTZ817C)\n", 
                    mode,
                    relayActive ? "ATTIVATO" : "DISATTIVATO",
                    relayActive ? "LOW" : "HIGH");
    } else {
      // Logica normale: HIGH = Relay ON, LOW = Relay OFF
      digitalWrite(RELAY_OUTPUT, relayActive ? HIGH : LOW);
      Serial.printf("🔌 OUTPUT (%s): %s (pin=%s - normale)\n", 
                    mode,
                    relayActive ? "ATTIVATO" : "DISATTIVATO",
                    relayActive ? "HIGH" : "LOW");
    }
  }
}

// MODIFICA: Funzione semplificata per disattivare il singolo relè
void deactivateRelay() {
  if (relayActive) {
    relayActive = false;
    
    if (RELAY_LOGIC_INVERTED) {
      // Relay OFF = pin HIGH per optoaccoppiatore GTZ817C
      digitalWrite(RELAY_OUTPUT, HIGH);
      Serial.println("🔌 Relè disattivato (GTZ817C: pin HIGH = OFF)");
    } else {
      // Relay OFF = pin LOW per relay normale
      digitalWrite(RELAY_OUTPUT, LOW);
      Serial.println("🔌 Relè disattivato (normale: pin LOW = OFF)");
    }
  }
}

// ===== GESTIONE ENCODER POLLING VERSION CON BACKLIGHT =====
void readEncoderPolling() {
  unsigned long currentTime = millis();
  
  // Controllo timing polling - ogni 8ms = 125Hz (alta frequenza)
  if (currentTime - lastEncoderPoll < ENCODER_POLLING_INTERVAL) {
    return;
  }
  lastEncoderPoll = currentTime;
  
  // Lettura stato corrente pin encoder
  bool currentCLK = digitalRead(ENCODER_CLK);
  bool currentDT = digitalRead(ENCODER_DT);
  
  // Rileva cambiamento su CLK (pin principale di trigger)
  if (currentCLK != lastCLKState) {
    
    // Fronte di discesa CLK = movimento encoder rilevato
    if (currentCLK == LOW) {
      // Reset timer attività utente
      resetUserActivityTimer();
      
      // Determina direzione basata su stato DT al momento del trigger
      if (currentDT == HIGH) {
        // SENSO ORARIO → incrementa (+)
        encoderPosition++;
        encoderChanged = true;
        Serial.printf("🎛️ Encoder: ORARIO (+) → pos=%d\n", encoderPosition);
      } else {
        // SENSO ANTIORARIO → decrementa (-)
        encoderPosition--;
        encoderChanged = true;
        Serial.printf("🎛️ Encoder: ANTIORARIO (-) → pos=%d\n", encoderPosition);
      }
    }
  }
  
  // Aggiorna stati precedenti per prossima iterazione
  lastCLKState = currentCLK;
  lastDTState = currentDT;
}

void handleEncoderInput() {
  // Gestione rotazione encoder (ora tramite polling)
  if (encoderChanged) {
    encoderChanged = false;
    processEncoderRotation();
  }
  
  // Gestione pressione pulsante con debounce software
  bool currentButtonState = digitalRead(ENCODER_SW);
  if (currentButtonState != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (currentButtonState != buttonPressed) {
      buttonPressed = currentButtonState;
      if (buttonPressed == LOW) { // Pulsante premuto (active LOW con pull-up)
        // Reset timer attività utente
        resetUserActivityTimer();
        
        processButtonPress();
        Serial.println("🎛️ Pulsante encoder premuto");
      }
    }
  }
  lastButtonState = currentButtonState;
}

void processEncoderRotation() {
  int delta = encoderPosition - lastEncoderPosition;
  lastEncoderPosition = encoderPosition;
  
  // Debug movimento encoder
  if (delta != 0) {
    Serial.printf("🎯 Delta: %s%d, Menu: %d\n", 
                  delta > 0 ? "+" : "", delta, currentMenuState);
  }
  
  switch (currentMenuState) {
    case MAIN_DISPLAY:
      if (abs(delta) > 0) {
        currentMenuState = SETTINGS_MENU;
        menuSelectionIndex = 0;
        // Carica configurazione temporanea per editing
        loadTempConfiguration();
        Serial.println("📋 Entrato in menu settings");
      }
      break;
      
    case SETTINGS_MENU:
      // Navigazione circolare corretta
      menuSelectionIndex += delta;
      if (menuSelectionIndex < 0) {
        menuSelectionIndex = MENU_ITEMS_COUNT - 1;  // = 6 (Cancel)
      } else if (menuSelectionIndex >= MENU_ITEMS_COUNT) {
        menuSelectionIndex = 0;  // = 0 (Temperature)
      }
      Serial.printf("📋 Menu selection: %d/%d (circolare corretta)\n", 
                    menuSelectionIndex, MENU_ITEMS_COUNT-1);
      break;
      
    case TEMPERATURE_SETTING:
      tempConfig.targetTemperature += delta * 0.5f; // Incrementi di 0.5°C
      tempConfig.targetTemperature = constrain(tempConfig.targetTemperature, TEMP_MIN, TEMP_MAX);
      Serial.printf("🎯 Target temp (temp): %.1f°C\n", tempConfig.targetTemperature);
      break;
      
    case DELTA_SETTING:
      tempConfig.deltaTemperature += delta * 0.1f; // Incrementi di 0.1°C
      tempConfig.deltaTemperature = constrain(tempConfig.deltaTemperature, DELTA_MIN, DELTA_MAX);
      Serial.printf("📊 Delta temp (temp): %.1f°C\n", tempConfig.deltaTemperature);
      break;
      
    case MODE_SETTING:
      if (delta != 0) { // Qualsiasi movimento cambia modalità
        tempConfig.operatingMode = (tempConfig.operatingMode == 0) ? 1 : 0;
        Serial.printf("🔧 Mode (temp): %s\n", tempConfig.operatingMode == 0 ? "HEATER" : "COOLER");
      }
      break;
      
    case SYSTEM_STATE_SETTING:
      if (delta != 0) { // Qualsiasi movimento toglia stato
        tempConfig.systemEnabled = !tempConfig.systemEnabled;
        Serial.printf("⚡ System (temp): %s\n", tempConfig.systemEnabled ? "ON" : "OFF");
      }
      break;
      
    case DIAGNOSTIC_SCREEN:
      // Scroll tra diverse info diagnostiche
      if (abs(delta) > 0) {
        currentMenuState = MAIN_DISPLAY;
        Serial.println("📋 Ritorno al main da diagnostica");
      }
      break;
  }
}

void processButtonPress() {
  switch (currentMenuState) {
    case MAIN_DISPLAY:
      currentMenuState = SETTINGS_MENU;
      menuSelectionIndex = 0;
      // Carica configurazione temporanea per editing
      loadTempConfiguration();
      Serial.println("📋 Entrato in settings dal main");
      break;
      
    case SETTINGS_MENU:
      // Bounds checking per sicurezza
      if (menuSelectionIndex >= 0 && menuSelectionIndex < MENU_ITEMS_COUNT) {
        switch (menuSelectionIndex) {
          case 0: 
            currentMenuState = TEMPERATURE_SETTING; 
            Serial.println("🎯 Impostazione temperatura (temporanea)");
            break;
          case 1: 
            currentMenuState = DELTA_SETTING; 
            Serial.println("📊 Impostazione delta (temporanea)");
            break;
          case 2: 
            currentMenuState = MODE_SETTING; 
            Serial.println("🔧 Impostazione modalità (temporanea)");
            break;
          case 3: 
            currentMenuState = SYSTEM_STATE_SETTING; 
            Serial.println("⚡ Impostazione sistema (temporanea)");
            break;
          case 4: 
            currentMenuState = DIAGNOSTIC_SCREEN;
            Serial.println("🔍 Aperta diagnostica");
            break;
          case 5: 
            // SAVE & EXIT
            currentMenuState = MAIN_DISPLAY;
            saveTempConfiguration();
            Serial.println("💾 Configurazione salvata, ritorno al main");
            break;
          case 6:
            // CANCEL
            currentMenuState = MAIN_DISPLAY;
            cancelTempConfiguration();
            Serial.println("❌ Modifiche annullate, ritorno al main");
            break;
          default:
            Serial.printf("❌ Indice menu non valido: %d\n", menuSelectionIndex);
            break;
        }
      } else {
        Serial.printf("❌ Indice menu fuori bounds: %d\n", menuSelectionIndex);
      }
      break;
      
    default:
      // Da qualsiasi sub-menu torna al menu settings
      currentMenuState = SETTINGS_MENU;
      Serial.println("📋 Ritorno al menu settings");
      break;
  }
}

// ===== FUNZIONI HELPER PER DISPLAY =====
const char* getMenuValue(int index) {
  static char valueStr[12]; // Buffer più sicuro
  
  // Bounds checking
  if (index < 0 || index >= MENU_ITEMS_COUNT) {
    strcpy(valueStr, "ERROR");
    return valueStr;
  }
  
  switch (index) {
    case 0: // Temperature - mostra valore temporaneo
      snprintf(valueStr, 12, "%6.1f°C", tempConfig.targetTemperature);
      break;
    case 1: // Delta - mostra valore temporaneo
      snprintf(valueStr, 12, "±%.1f°C", tempConfig.deltaTemperature);
      break;
    case 2: // Mode - mostra valore temporaneo
      strcpy(valueStr, tempConfig.operatingMode == 0 ? "HEAT " : "COOL ");
      break;
    case 3: // System - mostra valore temporaneo
      strcpy(valueStr, tempConfig.systemEnabled ? "ON  " : "OFF ");
      break;
    case 4: // Diagnostic
      strcpy(valueStr, "INFO ");
      break;
    case 5: // Save & Exit
      strcpy(valueStr, "SAVE ");
      break;
    case 6: // Cancel
      strcpy(valueStr, "CANCEL");
      break;
    default:
      strcpy(valueStr, "UNKNOW");
  }
  return valueStr;
}

void createProgressBar(char* buffer, float value, float minVal, float maxVal, int length) {
  float normalized = (value - minVal) / (maxVal - minVal);
  normalized = constrain(normalized, 0.0f, 1.0f);
  int barLength = (int)(normalized * length);
  
  // Bounds checking
  if (length < 0) length = 0;
  if (barLength < 0) barLength = 0;
  if (barLength > length) barLength = length;
  
  for (int i = 0; i < length; i++) {
    if (i < barLength) {
      buffer[i] = (char)255; // Carattere pieno
    } else {
      buffer[i] = (char)45;  // Carattere vuoto (-)
    }
  }
  buffer[length] = '\0';
}

void createTemperatureGraph(char* buffer, float current, float target, float delta) {
  // Crea mini-grafico 6 caratteri: target-delta, target, target+delta
  float range = delta * 4; // Range totale del grafico
  float min = target - range/2;
  float max = target + range/2;
  
  int pos = (int)((current - min) / (max - min) * 5);
  pos = constrain(pos, 0, 5);
  
  for (int i = 0; i < 6; i++) {
    if (i == pos) {
      buffer[i] = (char)CHAR_THERMOMETER; // Posizione temperatura attuale
    } else if (i == 2 || i == 3) {
      buffer[i] = '|'; // Target zone
    } else {
      buffer[i] = '-';
    }
  }
  buffer[6] = '\0';
}

// ===== GESTIONE DISPLAY 4x20 CON SISTEMA AVANZATO - MODIFICATO PER RELÈ SINGOLO =====
void updateDisplayContent() {
  switch (currentMenuState) {
    case MAIN_DISPLAY:
      displayMainScreen();
      break;
    case SETTINGS_MENU:
      displaySettingsMenu();
      break;
    case TEMPERATURE_SETTING:
      displayTemperatureSetting();
      break;
    case DELTA_SETTING:
      displayDeltaSetting();
      break;
    case MODE_SETTING:
      displayModeSetting();
      break;
    case SYSTEM_STATE_SETTING:
      displaySystemStateSetting();
      break;
    case DIAGNOSTIC_SCREEN:
      displayDiagnosticScreen();
      break;
  }
}

void displayMainScreen() {
  lcd.clear();
  
  // RIGA 1: Temperatura attuale e status (USA CONFIG ATTIVA)
  lcd.setCursor(0, 0);
  if (temperatureSensorError) {
    lcd.print("TEMP: ERROR!        ");
  } else {
    char tempStr[DISPLAY_BUFFER_SIZE];
    const char* status = (abs(currentTemperature - config.targetTemperature) <= config.deltaTemperature) ? "OK" : "!!";
    snprintf(tempStr, DISPLAY_BUFFER_SIZE, "TEMP:%7.2f%cC [%s]", currentTemperature, (char)223, status);
    tempStr[20] = '\0'; // Tronca per sicurezza display 20 char
    lcd.print(tempStr);
  }
  
  // RIGA 2: Target e isteresi (USA CONFIG ATTIVA)
  lcd.setCursor(0, 1);
  char targetStr[DISPLAY_BUFFER_SIZE];
  snprintf(targetStr, DISPLAY_BUFFER_SIZE, "TARGET:%6.1f%cC %c%.1f%cC", 
           config.targetTemperature, (char)223, (char)177, 
           config.deltaTemperature, (char)223);
  targetStr[20] = '\0'; // Tronca per sicurezza
  lcd.print(targetStr);
  
  // RIGA 3: Sistema, modalità e uptime (USA CONFIG ATTIVA)
  lcd.setCursor(0, 2);
  char systemStr[DISPLAY_BUFFER_SIZE];
  unsigned long uptime = (millis() - systemStartTime) / 1000;
  snprintf(systemStr, DISPLAY_BUFFER_SIZE, "SYS:%s MODE:%s T:%02d:%02d", 
           config.systemEnabled ? "ON " : "OFF",
           config.operatingMode == 0 ? "HEAT" : "COOL",
           (int)(uptime / 60), (int)(uptime % 60));
  systemStr[20] = '\0'; // Tronca per sicurezza
  lcd.print(systemStr);
  
  // RIGA 4: MODIFICATA - Mostra solo stato del singolo relè
  lcd.setCursor(0, 3);
  char relayStr[DISPLAY_BUFFER_SIZE];
  
  // Crea mini-grafico temperatura (6 caratteri)
  char tempGraph[7];
  if (!temperatureSensorError) {
    createTemperatureGraph(tempGraph, currentTemperature, config.targetTemperature, config.deltaTemperature);
  } else {
    strcpy(tempGraph, "------");
  }
  
  // MODIFICA: Mostra OUTPUT invece di H/C separati
  snprintf(relayStr, DISPLAY_BUFFER_SIZE, "OUTPUT:%s  MENU %s", 
           relayActive ? "ON " : "OFF",
           tempGraph);
  relayStr[20] = '\0'; // Tronca per sicurezza
  lcd.print(relayStr);
}

void displaySettingsMenu() {
  lcd.clear();
  
  // RIGA 1: Titolo con indicatore modifiche temporanee
  lcd.setCursor(0, 0);
  if (isEditingConfig) {
    lcd.print("== SETTINGS (TEMP) ==");  // Indica editing temporaneo
  } else {
    lcd.print("====== SETTINGS ====");
  }
  
  // Menu items con Save/Cancel (7 voci totali 0-6)
  const char* menuItems[] = {
    "Temperature",
    "Delta T    ", 
    "Mode       ",
    "System     ",
    "Diagnostic ",
    "Save & Exit",
    "Cancel     "
  };
  
  const int totalItems = sizeof(menuItems) / sizeof(menuItems[0]); // = 7
  
  // Mostra 3 opzioni contemporaneamente con scroll intelligente
  for (int i = 0; i < 3; i++) {
    int itemIndex = (menuSelectionIndex - 1 + i + totalItems) % totalItems;
    
    // Double-check bounds per sicurezza extra
    if (itemIndex < 0 || itemIndex >= totalItems) {
      itemIndex = 0; // Fallback sicuro
    }
    
    lcd.setCursor(0, i + 1);
    
    char menuLine[DISPLAY_BUFFER_SIZE];  // Buffer sicuro
    if (itemIndex == menuSelectionIndex) {
      // Opzione selezionata con freccia
      snprintf(menuLine, DISPLAY_BUFFER_SIZE, ">%-11s %s", 
               menuItems[itemIndex], getMenuValue(itemIndex));
    } else {
      // Opzioni non selezionate
      snprintf(menuLine, DISPLAY_BUFFER_SIZE, " %-11s %s", 
               menuItems[itemIndex], getMenuValue(itemIndex));
    }
    menuLine[20] = '\0'; // Tronca per display
    lcd.print(menuLine);
  }
}

void displayTemperatureSetting() {
  lcd.clear();
  
  // RIGA 1: Titolo con indicatore temporaneo
  lcd.setCursor(0, 0);
  lcd.print("== TEMPERATURE (T) ==");
  
  // RIGA 2: Valore corrente temporaneo
  lcd.setCursor(0, 1);
  char currentStr[DISPLAY_BUFFER_SIZE];
  snprintf(currentStr, DISPLAY_BUFFER_SIZE, "TEMP: %8.1f%cC", tempConfig.targetTemperature, (char)223);
  currentStr[20] = '\0';
  lcd.print(currentStr);
  
  // RIGA 3: Valore attivo + range
  lcd.setCursor(0, 2);
  char activeStr[DISPLAY_BUFFER_SIZE];
  snprintf(activeStr, DISPLAY_BUFFER_SIZE, "ACTIVE:%6.1f%cC    ", config.targetTemperature, (char)223);
  activeStr[20] = '\0';
  lcd.print(activeStr);
  
  // RIGA 4: Istruzioni + barra progresso
  lcd.setCursor(0, 3);
  char progressStr[DISPLAY_BUFFER_SIZE];
  
  // Barra progresso (10 caratteri)
  char progressBar[11];
  createProgressBar(progressBar, tempConfig.targetTemperature, TEMP_MIN, TEMP_MAX, 10);
  
  snprintf(progressStr, DISPLAY_BUFFER_SIZE, "STEP:0.5 [%s]", progressBar);
  progressStr[20] = '\0';
  lcd.print(progressStr);
}

void displayDeltaSetting() {
  lcd.clear();
  
  // RIGA 1: Titolo con indicatore temporaneo
  lcd.setCursor(0, 0);
  lcd.print("=== DELTA T (T) ====");
  
  // RIGA 2: Valore corrente temporaneo
  lcd.setCursor(0, 1);
  char currentStr[DISPLAY_BUFFER_SIZE];
  snprintf(currentStr, DISPLAY_BUFFER_SIZE, "HYST: %c%.1f%cC", (char)177, tempConfig.deltaTemperature, (char)223);
  currentStr[20] = '\0';
  lcd.print(currentStr);
  
  // RIGA 3: Valore attivo
  lcd.setCursor(0, 2);
  char activeStr[DISPLAY_BUFFER_SIZE];
  snprintf(activeStr, DISPLAY_BUFFER_SIZE, "ACTIVE: %c%.1f%cC    ", (char)177, config.deltaTemperature, (char)223);
  activeStr[20] = '\0';
  lcd.print(activeStr);
  
  // RIGA 4: Visualizzazione zone operative (usa tempConfig)
  lcd.setCursor(0, 3);
  char zoneStr[DISPLAY_BUFFER_SIZE];
  float target = tempConfig.targetTemperature;
  float delta = tempConfig.deltaTemperature;
  
  if (tempConfig.operatingMode == 0) { // HEATER
    snprintf(zoneStr, DISPLAY_BUFFER_SIZE, "ON<%.1f OFF>%.1f%cC", target-delta, target, (char)223);
  } else { // COOLER  
    snprintf(zoneStr, DISPLAY_BUFFER_SIZE, "OFF<%.1f ON>%.1f%cC", target, target+delta, (char)223);
  }
  zoneStr[20] = '\0';
  lcd.print(zoneStr);
}

void displayModeSetting() {
  lcd.clear();
  
  // RIGA 1: Titolo con indicatore temporaneo
  lcd.setCursor(0, 0);
  lcd.print("==== MODE (T) ======");
  
  // RIGA 2: Modalità corrente temporanea con icone
  lcd.setCursor(0, 1);
  char modeStr[20]; // Buffer per testo dopo icona
  if (tempConfig.operatingMode == 0) {
    lcd.write(CHAR_ARROW_UP);
    snprintf(modeStr, 20, " HEATING MODE");
  } else {
    lcd.write(CHAR_ARROW_DOWN);
    snprintf(modeStr, 20, " COOLING MODE");
  }
  lcd.print(modeStr);
  
  // RIGA 3: Modalità attiva
  lcd.setCursor(0, 2);
  char activeStr[DISPLAY_BUFFER_SIZE];
  snprintf(activeStr, DISPLAY_BUFFER_SIZE, "ACTIVE: %s      ", config.operatingMode == 0 ? "HEATING" : "COOLING");
  activeStr[20] = '\0';
  lcd.print(activeStr);
  
  // RIGA 4: MODIFICATA - Stato singolo relè invece di heater/cooler separati
  lcd.setCursor(0, 3);
  char statusStr[DISPLAY_BUFFER_SIZE];
  snprintf(statusStr, DISPLAY_BUFFER_SIZE, "ROTATE toggle OUT:%s", 
           relayActive ? "ON " : "OFF");
  statusStr[20] = '\0';
  lcd.print(statusStr);
}

void displaySystemStateSetting() {
  lcd.clear();
  
  // RIGA 1: Titolo con indicatore temporaneo
  lcd.setCursor(0, 0);
  lcd.print("== SYSTEM (T) ======");
  
  // RIGA 2: Stato corrente temporaneo
  lcd.setCursor(0, 1);
  char statusStr[DISPLAY_BUFFER_SIZE];
  snprintf(statusStr, DISPLAY_BUFFER_SIZE, "SYSTEM: %s", tempConfig.systemEnabled ? "ENABLED " : "DISABLED");
  statusStr[20] = '\0';
  lcd.print(statusStr);
  
  // RIGA 3: Stato attivo
  lcd.setCursor(0, 2);
  char activeStr[DISPLAY_BUFFER_SIZE];
  snprintf(activeStr, DISPLAY_BUFFER_SIZE, "ACTIVE: %s      ", config.systemEnabled ? "ENABLED" : "DISABLED");
  activeStr[20] = '\0';
  lcd.print(activeStr);
  
  // RIGA 4: Warning o statistiche
  lcd.setCursor(0, 3);
  if (!tempConfig.systemEnabled) {
    lcd.print("Will DISABLE relay  ");  // MODIFICA: "relay" singolare
  } else {
    char uptimeStr[DISPLAY_BUFFER_SIZE];
    unsigned long totalMin = totalOnTime / 60000;
    snprintf(uptimeStr, DISPLAY_BUFFER_SIZE, "Runtime: %lu min    ", totalMin);
    uptimeStr[20] = '\0';
    lcd.print(uptimeStr);
  }
}

void displayDiagnosticScreen() {
  lcd.clear();
  
  // RIGA 1: Titolo
  lcd.setCursor(0, 0);
  lcd.print("===== DIAGNOSTIC ===");
  
  // RIGA 2: Info hardware + backlight status
  lcd.setCursor(0, 2);
  char hwStr[DISPLAY_BUFFER_SIZE];
  snprintf(hwStr, DISPLAY_BUFFER_SIZE, "CPU:%dMHz BL:%s", ESP.getCpuFreqMHz(), 
           backlightActive ? "ON " : "OFF");
  hwStr[20] = '\0';
  lcd.print(hwStr);
  
  // RIGA 3: Uptime e contatori + timer backlight
  lcd.setCursor(0, 2);
  char uptimeStr[DISPLAY_BUFFER_SIZE];
  unsigned long uptime = millis() / 1000;
  unsigned long inactive = (millis() - lastUserActivity) / 1000;
  snprintf(uptimeStr, DISPLAY_BUFFER_SIZE, "UP:%02d:%02d IDLE:%02ds", 
           (int)(uptime/60), (int)(uptime%60), (int)inactive);
  uptimeStr[20] = '\0';
  lcd.print(uptimeStr);
  
  // RIGA 4: MODIFICATA - Status sistema con singolo relè
  lcd.setCursor(0, 3);
  char sensorStr[DISPLAY_BUFFER_SIZE];
  snprintf(sensorStr, DISPLAY_BUFFER_SIZE, "DS:%s BL:%s R:%s ED:%s", 
           temperatureSensorError ? "ERR" : "OK ",
           backlightAutoMode ? "AUTO" : "MAN ",
           relayActive ? "ON " : "OFF",  // MODIFICA: singolo relay status
           isEditingConfig ? "TMP" : "LIV");
  sensorStr[20] = '\0';
  lcd.print(sensorStr);
}

// ===== FUNZIONI PRINCIPALI =====
void setup() {
  // Salva tempo di avvio
  systemStartTime = millis();
  
  // Inizializzazione comunicazione seriale
  Serial.begin(115200);
  Serial.println("\n🚀 ===== TERMOSTATO INTELLIGENTE ESP32 =====");
  Serial.println("📅 VERSIONE RELÈ SINGOLO v1.0");
  Serial.printf("🔧 ESP32 Core Version: %s\n", ESP.getSdkVersion());
  
  // Informazioni board e chip
  Serial.printf("📱 Board: %s\n", ARDUINO_BOARD);
  Serial.printf("🖥️ Chip Model: %s\n", ESP.getChipModel());
  Serial.printf("🔧 Chip Revision: %d\n", ESP.getChipRevision());
  Serial.printf("💾 Flash Size: %d MB\n", ESP.getFlashChipSize() / (1024 * 1024));
  Serial.printf("⚡ CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
  
  // Modifiche implementate
  Serial.println("🔧 ===== MODIFICHE RELÈ SINGOLO =====");
  Serial.println("✅ Un solo relè sul pin 32");
  Serial.println("✅ Controllo unificato heating/cooling");
  Serial.println("✅ Display aggiornato per singolo output");
  Serial.println("✅ Logica semplificata ma completa");
  
  // Features UX avanzate mantenute
  Serial.println("✨ ===== FEATURES UX MANTENUTE =====");
  Serial.println("📱 LCD 4x20 interfaccia professionale");
  Serial.println("🎛️ Encoder polling ultra-stabile (125Hz)");
  Serial.println("💡 Backlight automatico + auto-ritorno main");
  Serial.println("📝 Sistema configurazione temporanea Save/Cancel");
  Serial.println("🔄 Menu navigazione circolare infinita");
  Serial.println("🔒 Relay optoisolato GTZ817C con sicurezza");
  Serial.println("📊 Diagnostica completa integrata");
  
  // Informazioni relay singolo
  if (RELAY_LOGIC_INVERTED) {
    Serial.println("🔌 Configurazione relay: SINGOLO OPTOISOLATO (GTZ817C)");
    Serial.println("   └─ Pin 32: HIGH = OFF, LOW = ON");
    Serial.println("   └─ Controllo unificato per heating/cooling");
  } else {
    Serial.println("🔌 Configurazione relay: SINGOLO NORMALE");
    Serial.println("   └─ Pin 32: HIGH = ON, LOW = OFF");
    Serial.println("   └─ Controllo unificato per heating/cooling");
  }
  
  // Informazioni encoder
  Serial.println("🎛️ Configurazione encoder: POLLING MODE OTTIMIZZATO");
  Serial.printf("   └─ Frequenza polling: %dms (%dHz)\n", 
                ENCODER_POLLING_INTERVAL, 1000/ENCODER_POLLING_INTERVAL);
  Serial.println("   └─ Direzione: ORARIO (+), ANTIORARIO (-)");
  Serial.println("   └─ Pull-up esterne 4.7kΩ, interne disabilitate");
  
  // Inizializzazione componenti hardware
  setupEEPROM();
  setupHardware();
  
  // Scansione bus I2C per debug
  scanI2CDevices();
  
  setupLCD();
  setupTemperatureSensor();
  setupEncoder();
  
  // Caricamento configurazione
  loadConfiguration();
  
  // Inizializza sistema backlight automatico
  lastUserActivity = millis();
  backlightActive = true;
  backlightAutoMode = true;
  lastBacklightCheck = millis();
  
  Serial.println("💡 Sistema backlight automatico attivato");
  Serial.printf("   └─ Timeout: %d secondi\n", BACKLIGHT_TIMEOUT / 1000);
  Serial.println("   └─ Auto-ritorno main screen su spegnimento");
  
  // Prima lettura temperatura
  updateTemperatureReading();
  
  // Display iniziale
  displayMainScreen();
  
  Serial.println("✅ Sistema inizializzato e operativo!");
  Serial.println("🎛️ Un solo relè per heating/cooling sul pin 32");
  Serial.println("🚀 SISTEMA PRONTO CON RELÈ SINGOLO!");
}

void loop() {
  unsigned long currentTime = millis();
  
  // POLLING ENCODER - PRIORITÀ MASSIMA
  readEncoderPolling();
  
  // Gestione input encoder e pulsante
  handleEncoderInput();
  
  // Lettura temperatura ogni TEMP_READ_INTERVAL ms
  if (currentTime - lastTemperatureRead >= TEMP_READ_INTERVAL) {
    updateTemperatureReading();
    lastTemperatureRead = currentTime;
  }
  
  // Controllo logica termostato (usa sempre config attiva, non temp)
  controlThermostatLogic();
  
  // Gestione backlight automatica + auto return main
  updateBacklightManagement();
  
  // Aggiornamento display ogni DISPLAY_UPDATE_INTERVAL ms
  if (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    updateDisplayContent();
    lastDisplayUpdate = currentTime;
  }
  
  // LED di stato sistema
  digitalWrite(LED_STATUS, config.systemEnabled && !temperatureSensorError);
  
  // Delay ottimizzato per responsività massima
  delay(2);
}