#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <Wire.h>

/*
 * TERMOSTATO INTELLIGENTE ESP32 - PlatformIO + Relay Optoisolati
 * Framework: Arduino per ESP32
 * Platform: Espressif32
 * 
 * VERSIONE CON ENCODER POLLING (risolve problemi alta velocità)
 * Pull-up esterne 4.7kΩ per stabilità massima
 * 
 * COMPONENTI HARDWARE:
 * - ESP32-WROOM-32 (DevKit)
 * - LCD I2C 16x2 (PCF8574 backpack)
 * - Encoder rotativo KY-040 + resistenze pull-up 4.7kΩ
 * - Modulo relay 2 canali OPTOISOLATI (GTZ817C)
 * - Sensore DS18B20 OneWire
 * 
 * FEATURES:
 * - Controllo temperatura con isteresi configurabile
 * - Menu navigabile con encoder rotativo (POLLING - alta velocità)
 * - Salvataggio configurazione in EEPROM
 * - Sistema di sicurezza relay optoisolati
 * - Monitoraggio I2C automatico
 * - Logica invertita per optoaccoppiatori GTZ817C
 * - Direzione corretta: ORARIO = +, ANTIORARIO = -
 */

// ===== FORWARD DECLARATIONS per PlatformIO =====
// Hardware setup functions
void setupHardware();
void setupLCD();
void setupTemperatureSensor();
void setupEncoder();
void setupEEPROM();
void scanI2CDevices();

// Configuration management
void loadConfiguration();
void saveConfiguration();

// Temperature control
void updateTemperatureReading();
void controlThermostatLogic();
void updateRelayState(int, bool, bool&, const char*);
void deactivateAllRelays();

// Input handling - POLLING VERSION
void readEncoderPolling();
void handleEncoderInput();
void processEncoderRotation();
void processButtonPress();

// Display management
void updateDisplayContent();
void displayMainScreen();
void displaySettingsMenu();
void displayTemperatureSetting();
void displayDeltaSetting();
void displayModeSetting();
void displaySystemStateSetting();

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

// Modulo relay 2 canali OPTOISOLATI (GTZ817C)
#define RELAY_HEATER  32
#define RELAY_COOLER  33

// LED di stato sistema
#define LED_STATUS  23

// ===== CONFIGURAZIONI SISTEMA =====
#define TEMP_READ_INTERVAL    2000    // ms - intervallo lettura temperatura
#define DISPLAY_UPDATE_INTERVAL 500   // ms - aggiornamento display
#define DEBOUNCE_DELAY        50      // ms - debounce pulsante encoder
#define ENCODER_POLLING_INTERVAL 8    // ms - polling encoder ottimizzato (125Hz)

#define TEMP_MIN             10.0f    // °C - temperatura minima configurabile
#define TEMP_MAX             35.0f    // °C - temperatura massima configurabile
#define DELTA_MIN            0.5f     // °C - delta minimo configurabile
#define DELTA_MAX            5.0f     // °C - delta massimo configurabile

#define EEPROM_SIZE          512      // bytes - dimensione EEPROM allocata
#define EEPROM_SETTINGS_ADDR 0        // indirizzo base settings

// ===== CONFIGURAZIONE RELAY OPTOISOLATI =====
// GTZ817C Optoaccoppiatori: logica invertita
// HIGH = Relay OFF, LOW = Relay ON
#define RELAY_LOGIC_INVERTED  true

// ===== INIZIALIZZAZIONE OGGETTI =====
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, 16, 2);
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature temperatureSensor(&oneWire);

// ===== STRUTTURE DATI =====
struct ThermostatConfig {
  float targetTemperature;    // Temperatura obiettivo (10.0 - 35.0°C)
  float deltaTemperature;     // Isteresi temperatura (0.5 - 5.0°C)
  bool systemEnabled;         // Sistema attivo/disattivo
  uint8_t operatingMode;      // 0=HEATER, 1=COOLER
  bool isConfigured;          // Flag prima configurazione
  uint32_t configVersion;     // Versione configurazione per compatibilità
};

// ===== VARIABILI GLOBALI =====
ThermostatConfig config;

// Variabili operative sistema
float currentTemperature = 0.0f;
bool heaterRelayActive = false;
bool coolerRelayActive = false;
bool temperatureSensorError = false;
unsigned long lastTemperatureRead = 0;
unsigned long lastDisplayUpdate = 0;

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

// Sistema menu multi-livello
enum MenuState {
  MAIN_DISPLAY,
  SETTINGS_MENU,
  TEMPERATURE_SETTING,
  DELTA_SETTING,
  MODE_SETTING,
  SYSTEM_STATE_SETTING
};

MenuState currentMenuState = MAIN_DISPLAY;
int menuSelectionIndex = 0;
bool inSubMenu = false;

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
  pinMode(RELAY_HEATER, OUTPUT);
  pinMode(RELAY_COOLER, OUTPUT);
  pinMode(LED_STATUS, OUTPUT);
  
  // STATO SICURO per relay optoisolati GTZ817C
  // Logica invertita: HIGH = Relay OFF, LOW = Relay ON
  if (RELAY_LOGIC_INVERTED) {
    digitalWrite(RELAY_HEATER, HIGH);  // Relay OFF sicuro
    digitalWrite(RELAY_COOLER, HIGH);  // Relay OFF sicuro
    Serial.println("🔒 Relay optoisolati (GTZ817C) - Stato sicuro: HIGH = OFF");
  } else {
    digitalWrite(RELAY_HEATER, LOW);   // Relay OFF normale
    digitalWrite(RELAY_COOLER, LOW);   // Relay OFF normale
    Serial.println("🔒 Relay normali - Stato sicuro: LOW = OFF");
  }
  
  digitalWrite(LED_STATUS, LOW);
  Serial.println("✅ Pin configurati correttamente");
  Serial.println("🔌 Pull-up esterne 4.7kΩ - interne disabilitate");
}

void setupLCD() {
  Serial.println("🖥️ Inizializzazione LCD I2C...");
  
  // Inizializzazione LCD I2C
  lcd.init();
  lcd.backlight();
  lcd.clear();
  
  // Verifica comunicazione I2C
  Wire.beginTransmission(LCD_I2C_ADDRESS);
  if (Wire.endTransmission() == 0) {
    Serial.printf("✅ LCD I2C trovato all'indirizzo 0x%02X\n", LCD_I2C_ADDRESS);
  } else {
    Serial.printf("❌ ERRORE: LCD I2C non trovato all'indirizzo 0x%02X\n", LCD_I2C_ADDRESS);
  }
  
  // Messaggio di benvenuto
  lcd.setCursor(0, 0);
  lcd.print("ESP32 Thermostat");
  lcd.setCursor(0, 1);
  lcd.print("Polling v2.0    ");
  delay(2000);
  
  Serial.println("✅ LCD I2C inizializzato");
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
          Serial.println("   └─ Probabile LCD I2C");
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
    currentTemperature = temperature;
    Serial.printf("🌡️ Temperatura: %.2f°C\n", currentTemperature);
  }
}

// ===== CONTROLLO RELAY E LOGICA TERMOSTATO =====
void controlThermostatLogic() {
  if (!config.systemEnabled || temperatureSensorError) {
    deactivateAllRelays();
    return;
  }
  
  bool shouldActivateHeater = false;
  bool shouldActivateCooler = false;
  
  // Logica di controllo con isteresi
  if (config.operatingMode == 0) { // HEATER MODE
    if (currentTemperature < (config.targetTemperature - config.deltaTemperature)) {
      shouldActivateHeater = true;
    } else if (currentTemperature > config.targetTemperature) {
      shouldActivateHeater = false;
    } else {
      shouldActivateHeater = heaterRelayActive; // Mantieni stato corrente (isteresi)
    }
  } else { // COOLER MODE
    if (currentTemperature > (config.targetTemperature + config.deltaTemperature)) {
      shouldActivateCooler = true;
    } else if (currentTemperature < config.targetTemperature) {
      shouldActivateCooler = false;
    } else {
      shouldActivateCooler = coolerRelayActive; // Mantieni stato corrente (isteresi)
    }
  }
  
  // Sistema di sicurezza: mai entrambi i relay attivi
  if (shouldActivateHeater && shouldActivateCooler) {
    Serial.println("🚨 EMERGENZA: Tentativo attivazione simultanea relay!");
    shouldActivateHeater = false;
    shouldActivateCooler = false;
  }
  
  // Aggiornamento stato relay con logica optoisolata
  updateRelayState(RELAY_HEATER, shouldActivateHeater, heaterRelayActive, "HEATER");
  updateRelayState(RELAY_COOLER, shouldActivateCooler, coolerRelayActive, "COOLER");
}

void updateRelayState(int relayPin, bool shouldActivate, bool &currentState, const char* relayName) {
  if (shouldActivate != currentState) {
    currentState = shouldActivate;
    
    // Logica per relay optoisolati GTZ817C
    if (RELAY_LOGIC_INVERTED) {
      // Logica invertita: LOW = Relay ON, HIGH = Relay OFF
      // shouldActivate=true  → pin=LOW  → optoaccoppiatore ON  → relay ON
      // shouldActivate=false → pin=HIGH → optoaccoppiatore OFF → relay OFF
      digitalWrite(relayPin, currentState ? LOW : HIGH);
      Serial.printf("🔌 %s: %s (pin=%s - GTZ817C)\n", 
                    relayName, 
                    currentState ? "ATTIVATO" : "DISATTIVATO",
                    currentState ? "LOW" : "HIGH");
    } else {
      // Logica normale: HIGH = Relay ON, LOW = Relay OFF
      digitalWrite(relayPin, currentState ? HIGH : LOW);
      Serial.printf("🔌 %s: %s (pin=%s - normale)\n", 
                    relayName, 
                    currentState ? "ATTIVATO" : "DISATTIVATO",
                    currentState ? "HIGH" : "LOW");
    }
  }
}

void deactivateAllRelays() {
  if (heaterRelayActive || coolerRelayActive) {
    heaterRelayActive = false;
    coolerRelayActive = false;
    
    if (RELAY_LOGIC_INVERTED) {
      // Relay OFF = pin HIGH per optoaccoppiatori GTZ817C
      digitalWrite(RELAY_HEATER, HIGH);
      digitalWrite(RELAY_COOLER, HIGH);
      Serial.println("🔌 Tutti i relay disattivati (GTZ817C: pin HIGH = OFF)");
    } else {
      // Relay OFF = pin LOW per relay normali
      digitalWrite(RELAY_HEATER, LOW);
      digitalWrite(RELAY_COOLER, LOW);
      Serial.println("🔌 Tutti i relay disattivati (normali: pin LOW = OFF)");
    }
  }
}

// ===== GESTIONE ENCODER POLLING VERSION =====
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
        Serial.println("📋 Entrato in menu settings");
      }
      break;
      
    case SETTINGS_MENU:
      menuSelectionIndex += delta;
      menuSelectionIndex = constrain(menuSelectionIndex, 0, 4);
      Serial.printf("📋 Menu selection: %d\n", menuSelectionIndex);
      break;
      
    case TEMPERATURE_SETTING:
      config.targetTemperature += delta * 0.5f; // Incrementi di 0.5°C
      config.targetTemperature = constrain(config.targetTemperature, TEMP_MIN, TEMP_MAX);
      Serial.printf("🎯 Target temp: %.1f°C\n", config.targetTemperature);
      break;
      
    case DELTA_SETTING:
      config.deltaTemperature += delta * 0.1f; // Incrementi di 0.1°C
      config.deltaTemperature = constrain(config.deltaTemperature, DELTA_MIN, DELTA_MAX);
      Serial.printf("📊 Delta temp: %.1f°C\n", config.deltaTemperature);
      break;
      
    case MODE_SETTING:
      if (delta != 0) { // Qualsiasi movimento cambia modalità
        config.operatingMode = (config.operatingMode == 0) ? 1 : 0;
        Serial.printf("🔧 Mode: %s\n", config.operatingMode == 0 ? "HEATER" : "COOLER");
      }
      break;
      
    case SYSTEM_STATE_SETTING:
      if (delta != 0) { // Qualsiasi movimento toglia stato
        config.systemEnabled = !config.systemEnabled;
        Serial.printf("⚡ System: %s\n", config.systemEnabled ? "ON" : "OFF");
      }
      break;
  }
}

void processButtonPress() {
  switch (currentMenuState) {
    case MAIN_DISPLAY:
      currentMenuState = SETTINGS_MENU;
      menuSelectionIndex = 0;
      Serial.println("📋 Entrato in settings dal main");
      break;
      
    case SETTINGS_MENU:
      switch (menuSelectionIndex) {
        case 0: 
          currentMenuState = TEMPERATURE_SETTING; 
          Serial.println("🎯 Impostazione temperatura");
          break;
        case 1: 
          currentMenuState = DELTA_SETTING; 
          Serial.println("📊 Impostazione delta");
          break;
        case 2: 
          currentMenuState = MODE_SETTING; 
          Serial.println("🔧 Impostazione modalità");
          break;
        case 3: 
          currentMenuState = SYSTEM_STATE_SETTING; 
          Serial.println("⚡ Impostazione sistema");
          break;
        case 4: 
          currentMenuState = MAIN_DISPLAY;
          saveConfiguration();
          Serial.println("💾 Configurazione salvata, ritorno al main");
          break;
      }
      break;
      
    default:
      currentMenuState = SETTINGS_MENU;
      saveConfiguration();
      Serial.println("💾 Configurazione salvata, ritorno al menu");
      break;
  }
}

// ===== GESTIONE DISPLAY =====
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
  }
}

void displayMainScreen() {
  lcd.clear();
  
  // Prima riga: temperatura attuale e stato sensore
  lcd.setCursor(0, 0);
  if (temperatureSensorError) {
    lcd.print("TEMP: ERROR!    ");
  } else {
    lcd.printf("TEMP: %5.1f%cC   ", currentTemperature, (char)223);
  }
  
  // Seconda riga: stato sistema e relay
  lcd.setCursor(0, 1);
  lcd.print(config.systemEnabled ? "ON " : "OFF");
  lcd.print(config.operatingMode == 0 ? " H:" : " C:");
  lcd.print(heaterRelayActive ? "ON " : "OFF");
  lcd.print(coolerRelayActive ? "ON " : "OFF");
  lcd.printf(" %4.1f%c", config.targetTemperature, (char)223);
}

void displaySettingsMenu() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("=== SETTINGS ===");
  
  lcd.setCursor(0, 1);
  const char* menuItems[] = {
    ">Set Temperature",
    ">Set Delta T    ",
    ">Set Mode       ",
    ">System ON/OFF  ",
    ">Save & Exit    "
  };
  lcd.print(menuItems[menuSelectionIndex]);
}

void displayTemperatureSetting() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SET TEMPERATURE:");
  lcd.setCursor(0, 1);
  lcd.printf("Target: %4.1f%cC  ", config.targetTemperature, (char)223);
}

void displayDeltaSetting() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SET DELTA T:    ");
  lcd.setCursor(0, 1);
  lcd.printf("Delta: %3.1f%cC   ", config.deltaTemperature, (char)223);
}

void displayModeSetting() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SET MODE:       ");
  lcd.setCursor(0, 1);
  lcd.print(config.operatingMode == 0 ? "Mode: HEATER   " : "Mode: COOLER   ");
}

void displaySystemStateSetting() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SYSTEM STATE:   ");
  lcd.setCursor(0, 1);
  lcd.print(config.systemEnabled ? "System: ON     " : "System: OFF    ");
}

// ===== FUNZIONI PRINCIPALI =====
void setup() {
  // Inizializzazione comunicazione seriale
  Serial.begin(115200);
  Serial.println("\n🚀 ===== TERMOSTATO INTELLIGENTE ESP32 =====");
  Serial.println("📅 PlatformIO + Encoder POLLING + Pull-up 4.7kΩ");
  Serial.printf("🔧 ESP32 Core Version: %s\n", ESP.getSdkVersion());
  
  // Informazioni board e chip
  Serial.printf("📱 Board: %s\n", ARDUINO_BOARD);
  Serial.printf("🖥️ Chip Model: %s\n", ESP.getChipModel());
  Serial.printf("🔧 Chip Revision: %d\n", ESP.getChipRevision());
  Serial.printf("💾 Flash Size: %d MB\n", ESP.getFlashChipSize() / (1024 * 1024));
  Serial.printf("⚡ CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
  
  // Verifica configurazione board
  #ifdef ARDUINO_ESP32_DEV
    Serial.println("✅ Configurazione: Generic ESP32 Dev Board");
  #elif defined(ARDUINO_UPESY_WROOM)
    Serial.println("✅ Configurazione: Upesy ESP32 WROOM");
  #elif defined(ARDUINO_NodeMCU_32S)
    Serial.println("✅ Configurazione: NodeMCU ESP32-S");
  #else
    Serial.println("⚠️ Configurazione: Board specifica non riconosciuta");
  #endif
  
  // Informazioni relay
  if (RELAY_LOGIC_INVERTED) {
    Serial.println("🔌 Configurazione relay: OPTOISOLATI (GTZ817C - logica invertita)");
    Serial.println("   └─ HIGH = Relay OFF, LOW = Relay ON");
  } else {
    Serial.println("🔌 Configurazione relay: NORMALI (logica diretta)");
    Serial.println("   └─ HIGH = Relay ON, LOW = Relay OFF");
  }
  
  // Informazioni encoder
  Serial.println("🎛️ Configurazione encoder: POLLING MODE OTTIMIZZATO");
  Serial.printf("   └─ Frequenza polling: %dms (%dHz)\n", 
                ENCODER_POLLING_INTERVAL, 1000/ENCODER_POLLING_INTERVAL);
  Serial.println("   └─ Direzione: ORARIO (+), ANTIORARIO (-)");
  Serial.println("   └─ Pull-up esterne 4.7kΩ, interne disabilitate");
  Serial.println("   └─ Ottimizzato per alta velocità di rotazione");
  
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
  
  // Prima lettura temperatura
  updateTemperatureReading();
  
  // Display iniziale
  displayMainScreen();
  
  Serial.println("✅ Sistema inizializzato e operativo!");
  Serial.println("📱 Menu: Ruota encoder (POLLING) o premi per navigare");
  Serial.println("🔒 Sicurezza relay optoisolati attiva");
  Serial.println("🎛️ Encoder polling attivo - stabilità garantita ad alta velocità");
  Serial.println("🔄 Direzione encoder: ORARIO = +, ANTIORARIO = -");
}

void loop() {
  unsigned long currentTime = millis();
  
  // ⚡ POLLING ENCODER - PRIORITÀ MASSIMA
  // Chiamata ogni ciclo per garantire responsività anche ad alta velocità
  readEncoderPolling();
  
  // Gestione input encoder e pulsante
  handleEncoderInput();
  
  // Lettura temperatura ogni TEMP_READ_INTERVAL ms
  if (currentTime - lastTemperatureRead >= TEMP_READ_INTERVAL) {
    updateTemperatureReading();
    lastTemperatureRead = currentTime;
  }
  
  // Controllo logica termostato con relay optoisolati
  controlThermostatLogic();
  
  // Aggiornamento display ogni DISPLAY_UPDATE_INTERVAL ms
  if (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    updateDisplayContent();
    lastDisplayUpdate = currentTime;
  }
  
  // LED di stato sistema
  digitalWrite(LED_STATUS, config.systemEnabled && !temperatureSensorError);
  
  // Delay ridotto per garantire polling veloce (8ms interno + 2ms loop = 10ms totali)
  delay(2);
}