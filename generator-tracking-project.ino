#include <Arduino.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Firebase_ESP_Client.h>
#include "time.h"

// Provide the token generation process info.
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Network credentials
#define WIFI_SSID "kaveenThivanka’s iPhone"
#define WIFI_PASSWORD "987654321"

// Insert Firebase project API Key
#define API_KEY "AIzaSyAAGAr-esUeltNoJput1rLuRZHBpdNk0ho"

// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "https://generator-tracking-ipc-project-default-rtdb.asia-southeast1.firebasedatabase.app/"

// Define Firebase Data object
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
int count = 0;
bool signupOK = false;

static const int RXPin = 16, TXPin = 17;
int lcdColumns = 16;
int lcdRows = 2;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

TaskHandle_t Task1;
TaskHandle_t Task2;

SemaphoreHandle_t sema;

float lat;
float lng;
float temperature;

int timestamp;
FirebaseJson json;

String databasePath;
String parentPath;

const int oneWireBus = 4;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

byte degreeSymbol[8] = {
  B00110,
  B01001,
  B01001,
  B00110,
  B00000,
  B00000,
  B00000,
  B00000
};

uint64_t chipId;

// Initialize WiFi
void initWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi ..");
  lcd.setCursor(0, 0);
  lcd.print("Connecting to");
  lcd.setCursor(11, 1);
  lcd.print("Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connection");
  lcd.setCursor(1, 1);
  lcd.print("Established");
  delay(3000);
  Serial.println();
}

// Function that gets current epoch time
unsigned long getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return (0);
  }
  time(&now);
  return now;
}

void setup() {
  Serial.begin(9600);
  ss.begin(9600);

  lcd.init();
  lcd.backlight();
  lcd.createChar(0, degreeSymbol);

  initWiFi();

  setenv("TZ", "Asia/Colombo", 1);
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("ok");
    signupOK = true;
  } else {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback;  //see addons/TokenHelper.h

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  sema = xSemaphoreCreateBinary();

  sensors.begin();

  xTaskCreatePinnedToCore(
    Task1code, /* Task function. */
    "Task1",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task1,    /* Task handle to keep track of created task */
    0);        /* pin task to core 0 */
  delay(500);

  xTaskCreatePinnedToCore(
    Task2code, /* Task function. */
    "Task2",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task2,    /* Task handle to keep track of created task */
    1);        /* pin task to core 1 */
  delay(500);
}

void Task1code(void *pvParameters) {
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  while (1) {
    if (ss.available() > 0) {
      if (gps.encode(ss.read())) {
        if (gps.location.isUpdated()) {
          xSemaphoreGive(sema);
          vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
      }
    }
  }
}

void Task2code(void *pvParameters) {
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());
  unsigned long previousMillis = 0;
  const long interval = 5000;
  chipId = ESP.getEfuseMac();
  Serial.printf("ESP32 Chip ID: %04X", (uint16_t)(chipId >> 32));
  Serial.printf("%08X\n", (uint32_t)chipId);
  databasePath = "/GeneratorData/" + String(chipId, HEX) + "/readings";
  while (1) {
    if (xSemaphoreTake(sema, portMAX_DELAY)) {
      unsigned long currentMillis = millis();

      if ((currentMillis - previousMillis >= interval) || previousMillis == 0) {
        previousMillis = currentMillis;
        performTempOperation();
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        performGPSOperation();
        vTaskDelay(5000 / portTICK_PERIOD_MS);
      }

      if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 60000 || sendDataPrevMillis == 0)) {
        sendDataPrevMillis = millis();
        //Get current timestamp
        timestamp = getTime();
        Serial.print("time: ");
        Serial.println(timestamp);

        parentPath = databasePath + "/" + String(timestamp);

        String formattedDate = String(gps.date.year()) + "-" + String(gps.date.month()) + "-" + String(gps.date.day());
        String formattedTime = String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());

        // Clear the JSON object before adding new data
        json.clear();
        json.set("latitude", gps.location.lat());
        json.set("longitude", gps.location.lng());
        json.set("satellitesInUse", gps.satellites.value());
        json.set("date", formattedDate);
        json.set("time", formattedTime);
        json.set("temperatureInCelcius", temperature);
        json.set("timestamp", String(timestamp));

        // Serial.printf("Set json... %s\n", Firebase.RTDB.setJSON(&fbdo, parentPath.c_str(), &json) ? "ok" : fbdo.errorReason().c_str());

        if (Firebase.RTDB.setJSON(&fbdo, parentPath.c_str(), &json)) {
          Serial.println("Data sent to Firebase successfully!");
        } else {
          Serial.println("Failed to send data to Firebase");
          Serial.println("Reason: " + fbdo.errorReason());
        }
      }
    }
  }
}

void performTempOperation() {
  sensors.requestTemperatures();
  float temperatureC = sensors.getTempCByIndex(0);
  Serial.print(temperatureC);
  temperature = temperatureC;
  Serial.println("ºC");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.setCursor(1, 1);
  String formattedNumber = String(temperatureC, 2);
  lcd.print(formattedNumber);
  lcd.write(0);
  lcd.print("C");
}

void performGPSOperation() {
  Serial.print("Latitude= ");
  Serial.print(gps.location.lat());
  Serial.print(" Longitude= ");
  Serial.println(gps.location.lng());
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Lat: ");
  lcd.print(gps.location.lat(), 6);
  lcd.setCursor(0, 1);
  lcd.print("Long: ");
  lcd.print(gps.location.lng(), 6);
}

void loop() {
}