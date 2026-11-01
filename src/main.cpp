/*-----------------------------------------------------------------------------------------------*
 * 7-Segment Flip-disc Clock by Marcin Saj https://flipo.io                                      *
 * https://github.com/marcinsaj/Flipo-Clock-4x7-Segment-Flip-Disc-Display                        *
 *                                                                                               *
 * Final Flip-disc Clock 4x7-Segment                                                             *
 * Setting Options:                                                                              *
 * - time - 12/24 hour                                                                           *
 * - rest period - sleep hour, wake hour                                                         *
 * - leading zero - on/off                                                                       *
 * - speed/delay flip disc effect - 0-99ms                                                       *
 * - temperature display - on/off                                                                *
 * - temperature degrees °F/°C                                                                   *
 * - humidity display - on/off                                                                   *
 * - temperature and/or humidity display frequency - 30/60 seconds                               *
 *                                                                                               *
 * Attention!!! - Firmware Update Instructions - https://bit.ly/4x7SEG-CLOCK-FIRMWARE-UPDATE     *
 * Settings Instructions - https://bit.ly/4x7SEG-CLOCK-SET                                       *
 *                                                                                               *
 * Setup:                                                                                        *
 * Assembly Instructions - https://bit.ly/Flip-Disc-Clock-Assembly                               *
 * Clock Diagram - https://bit.ly/4x7SEG-CLOCK-DIAGRAM                                           *
 * Dedicated Controller - https://bit.ly/AC1-FD                                                  *
 * 7-Segment Flip-disc Display - https://bit.ly/7SEG-FD                                          *
 * 3x1 Flip-disc Display - https://bit.ly/3x1DOT-FD                                              *
 * Arduino Nano Every - https://bit.ly/ARD-EVERY                                                 *
 * RTC Real Time Clock RX8025T - https://bit.ly/RX8025T                                          *
 * Temperature Sensor DHT22 - https://bit.ly/DHT22                                               *
 *-----------------------------------------------------------------------------------------------*/

#include <Arduino.h>
#include <FlipDisc.h>         // https://github.com/marcinsaj/FlipDisc
#include <RTC_RX8025T.h>      // https://github.com/marcinsaj/RTC_RX8025T
#include <TimeLib.h>          // https://github.com/PaulStoffregen/Time
#include <Wire.h>             // https://arduino.cc/en/Reference/Wire (included with Arduino IDE)
#include <OneButton.h>        // https://github.com/mathertel/OneButton
#include <EEPROM.h>           // https://www.arduino.cc/en/Reference/EEPROM
#include <Adafruit_Sensor.h>  // https://github.com/adafruit/Adafruit_Sensor
#include <DHT.h>              // https://github.com/adafruit/DHT-sensor-library
#include <WiFi.h>
#include <ESP32MQTTClient.h>  // https://github.com/cyijun/ESP32MQTTClient - wrapper around ESP-IDF's esp-mqtt
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <esp_sntp.h>
#include "time.h"
#include <arduino_secrets.h>

// Function prototypes (generated automatically by the Arduino IDE for .ino files)
void WatchButtons(void);
void ShortPressButton1(void);
void ShortPressButton3(void);
void LongPressButton1(void);
void LongPressButton2(void);
void LongPressButton3(void);
void ClearPressButtonFlags(void);
void DisplayRestPeriod(void);
void GetTimeRTC(void);
void DisplayTime(void);
void CheckRestPeriod(void);
void DisplayTimeAndTemperature(void);
void MeasureTemperatureAndHumidity(void);
void DisplayTemperature(void);
void DisplayHumidity(void);
void SettingSpeed(void);
void SettingTemp(void);
void SettingTime(void);
void HandleNetwork(void);
bool SyncTimeNTP(void);
void PublishState(void);
void SetupWebServer(void);
void HandleWebRoot(void);
void HandleWebSettings(void);
void HandleWebSync(void);
void ApplySettings(void);
String FormatHour(uint8_t hour);
String HtmlSelect(const char *name, const char *label, const String &options);
String HtmlOption(int value, const String &label, int selected);
void HandleMqttMessages(void);
void DisplayMqttMessage(const char *payload);
void CheckMqttHold(void);

#define DHT_PIN A0            // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22         // Sensor type

DHT dht(DHT_PIN, DHTTYPE);    // Initialize DHT sensor

// Attention: do not change! Changing these settings may physical damage the flip-disc displays.
// Pin declaration for a dedicated controller
#define EN_PIN A7  // Start & End SPI transfer data
#define CH_PIN A2  // Charging PSPS module - turn ON/OFF
#define PL_PIN A3  // Release the current pulse - turn ON/OFF 

// RTC
#define RTC_PIN A1 // RTC interrupt input

// Buttons - counting from the top
#define B1_PIN 10  // Top button
#define B2_PIN 9   // Middle button
#define B3_PIN 8   // Bottom button

// Initialize a new OneButton instance for a buttons 
// BUTTON_PIN - Input pin for the button
// false      - Button is active high
// false      - Disable internal pull-up resistor
OneButton button1(B1_PIN, false, false);
OneButton button2(B2_PIN, false, false);
OneButton button3(B3_PIN, false, false);

// The flags to store status of the settings
bool modeSettingsStatus = false;
bool timeSettingsStatus = false;
bool speedSettingsStatus = false;
bool tempSettingsStatus = false;

// Flags for storing button press status
bool shortPressButton1Status = false;
bool shortPressButton3Status = false;
bool longPressButton1Status = false;
bool longPressButton2Status = false;
bool longPressButton3Status = false;

// RTC interrupt flag
volatile bool interruptRtcStatus = false;

// When the clock enters Rest Period mode, 
// to avoid cyclic display of the rest period state, i.e. 4 horizontal lines, 
// the rest_period_counter is started. If the counter is different from 0, 
// the display is not updated
unsigned long rest_period_counter = 0;

// A flag that stores the time display status, if the flag is set, 
// the current time will be displayed
bool timeDisplayStatus = false;

// The flag specifies whether the currently available time 
// in the tm structure is up to date. If the time is current, 
// there is no need to read the time from the RTC again
bool currentTimeStatus = false;

// A flag that stores the temperature display status, if the flag is set, 
// the current temperature will be displayed
bool tempDisplayStatus = false;

float humidity = 0;
float temperature_celsius = 0;
float temperature_fahrenheit = 0;

// Temperature and humidity measurement status flag
bool measurementStatus = false;

// Declare structure that allows convenient access to the time elements:
// - tm.Hour - hours
// - tm.Minute - minutes
tmElements_t tm;

// An array to store individual digits for display
int digit[4] = {0, 0, 0, 0};

// Default disc delay/speed values ​​for Flip.Delay(value)
int flipSpeed[7] = {0, 5, 10, 25, 50, 75, 99};

// How long and after what time from displaying the temperature to display the humidity
const unsigned long wait_interval = 5000;

// Every 30 seconds - temperature and humidity display frequency
const unsigned long sequence_interval = 30000 - 3 * wait_interval;

// Time values for millis()
unsigned long sequence_start_time = 0;
unsigned long sequence_current_time = 0;

// Time, temperature and humidity sequence display status flags
bool sequenceRunning= false;         // Indicates if the sequence is currently running
bool sequenceSecondRun = false;      // Flag for second sequence if the temp/hum display interval is set to 30s

// Variable defining the current level of the time/temperature/humidity display sequence
uint8_t sequence_level = 0;

// Aliases for individual option settings
static const uint8_t HR12 = 0;   // Display time in 12 hour format
static const uint8_t HR24 = 1;   // Display time in 24 hour format
static const uint8_t OFF = 0;    // Turn ON temperature and humidity display 
static const uint8_t ON = 1;     // Turn OFF temperature and humidity display
static const uint8_t TPF = 0;    // Temperature in Fahrenheit
static const uint8_t TPC = 1;    // Temperature in Celsius 
static const uint8_t THFQ60 = 0; // Every 60 seconds - temperature and humidity display frequency
static const uint8_t THFQ30 = 1; // Every 30 seconds - temperature and humidity display frequency
static const uint8_t AM = 0;     // Time is AM
static const uint8_t PM = 1;     // Time is PM

// The values is stored in eeprom memory and read during setup
uint8_t flip_disc_delay_time = 0; // Flip disc delay/speed effect [ms]
uint8_t leading_zero = 0;         // Leading zero display ON/OFF
uint8_t rest_period = 0;          // Clock rest period ON/OFF
uint8_t sleep_hour = 0;           // Hour to turn off the clock
uint8_t wake_hour = 0;            // Hour to turn on the clock
uint8_t time_hr = 0;              // Time 12/24 hour clock
uint8_t temp_on_off = 0;          // Temperature ON/OFF
uint8_t temp_c_f = 0;             // Temparature C/F - Celsius/Fahrenheit
uint8_t hum_on_off = 0;           // Humidity ON/OFF
uint8_t temp_hum_fq = 0;          // Temperature and humidity display frequency - 30/60 seconds

// Eeprom addresses where settings are stored
static const uint16_t ee_delay_address = 0;        // Flip disc delay/speed effect
static const uint16_t ee_leading_zero_address = 1; // Leading zero display
static const uint16_t ee_rest_period_address = 2;  // Clock rest period ON/OFF
static const uint16_t ee_sleep_hour_address = 3;   // Hour to turn off the clock
static const uint16_t ee_wake_hour_address = 4;    // Hour to turn on the clock
static const uint16_t ee_time_hr_address = 5;      // Time format 12/24 hour  
static const uint16_t ee_temp_on_off_address = 6;  // Temperature ON/OFF
static const uint16_t ee_temp_c_f_address = 7;     // Temparature C/F - Celsius/Fahrenheit
static const uint16_t ee_hum_on_off_address = 8;   // Humidity ON/OFF
static const uint16_t ee_temp_hum_fq_address = 9;  // Temperature and humidity display frequency - 30/60 seconds

// Required for EEPROM.begin(eeprom_size) for Arduino Nano ESP32
static const uint8_t eeprom_size = 10;

// MQTT topics - MQTT_TOPIC receives words to display, the clock publishes its readings
// to MQTT_TOPIC/state and "online"/"offline" to MQTT_TOPIC/status (both retained)
#ifndef MQTT_TOPIC
#define MQTT_TOPIC "iot/flipdisc"
#endif
#define MQTT_STATE_TOPIC MQTT_TOPIC "/state"
#define MQTT_STATUS_TOPIC MQTT_TOPIC "/status"

// Set by scripts/git_version.py
#ifndef GIT_SHA
#define GIT_SHA "unknown"
#endif

ESP32MQTTClient mqttClient;

// How often the temperature and humidity are published to MQTT_STATE_TOPIC
const unsigned long mqtt_publish_interval = 60000;
unsigned long mqtt_publish_time = 0;
bool mqttPublishedOnce = false;

// MQTT messages arrive on the esp-mqtt task, but the flip-disc display must only be
// driven from loop(). The MQTT callback copies each payload into this queue and
// loop() takes them off and displays them, so the display is never touched by two tasks.
struct MqttMessage {
  char payload[32];
};
static const uint8_t mqtt_queue_length = 8;
QueueHandle_t mqttMessageQueue;

// How long a word, temperature or humidity shown from MQTT stays up before the time comes back
#ifndef MQTT_HOLD_MS
#define MQTT_HOLD_MS 10000
#endif
const unsigned long mqtt_hold_interval = MQTT_HOLD_MS;
unsigned long mqtt_hold_start_time = 0;
bool mqttHoldStatus = false;

// OTA and MQTT are started the first time WiFi connects, so the clock runs without WiFi
bool networkStarted = false;

// POSIX TZ string used to convert NTP time to the local time stored in the RTC.
// newlib doesn't understand IANA names, so this is America/Los_Angeles.
#ifndef TIMEZONE
#define TIMEZONE "PST8PDT,M3.2.0,M11.1.0"
#endif
#define NTP_SERVER "pool.ntp.org"

// The RTC is checked against NTP after the first NTP sync, then every hour so daylight saving
// changes and RTC drift are picked up, and whenever "Sync time from NTP" is pressed on the web page
const unsigned long ntp_check_interval = 3600000UL;
bool ntpSyncDone = false;
unsigned long ntp_last_check_time = 0; // millis() of the last RTC check
long ntp_last_difference = 0;          // Seconds the RTC was off at the last RTC check

// Between syncs the system clock runs on the ESP32's crystal, which drifts more than the RTC,
// so the RTC is only set if SNTP has synced recently. SNTP is set to resync every hour
const unsigned long ntp_max_sync_age = 2 * 3600000UL;
volatile unsigned long sntp_last_sync_time = 0; // millis() of the last SNTP sync, set on the lwIP task
volatile bool sntpSynced = false;

// Web config page, started with OTA and MQTT. handleClient() runs from loop(),
// so its handlers can safely change settings and drive the display
WebServer server(80);

/************************************************************************************************/
// Interrupt from RTC
#if defined(ARDUINO_ARCH_ESP32)
  void ARDUINO_ISR_ATTR rtcInterruptISR(void) // Required for Arduino Nano ESP32
  {interruptRtcStatus = true;}
#else
  void rtcInterruptISR(void)
  {interruptRtcStatus = true;}
#endif

/************************************************************************************************/
void setup() 
{
  Serial.begin(9600);
  
  // Required for Arduino Nano ESP32, initialize EEPROM with predefined size
  #if defined(ARDUINO_ARCH_ESP32)
    EEPROM.begin(eeprom_size);
  #endif

  // Attention: do not change! Changing these settings may physical damage the flip-disc displays.
  // Flip.Pin(...); it is the most important function and first to call before everything else. 
  // The function is used to declare pin functions.
  Flip.Pin(EN_PIN, CH_PIN, PL_PIN);
  
  pinMode(RTC_PIN, INPUT_PULLUP);

  // Initialize DHT22 sensor
  dht.begin();

  // RTC RX8025T initialization
  RTC_RX8025T.init();

  // Time update interrupt initialization. Interrupt generated by RTC (INT output): 
  // "INT_SECOND" - every second,
  // "INT_MINUTE" - every minute.
  RTC_RX8025T.initTUI(INT_MINUTE);

  // "INT_ON" - turn ON interrupt generated by RTC (INT output),
  // "INT_OFF" - turn OFF interrupt.
  RTC_RX8025T.statusTUI(INT_ON);
  
  // Assign an interrupt handler to the RTC output, 
  // an interrupt will be generated every minute to display the time
  #if defined(ARDUINO_ARCH_ESP32)
    attachInterrupt(RTC_PIN, rtcInterruptISR, FALLING); // Required for Arduino Nano ESP32
  #else
    attachInterrupt(digitalPinToInterrupt(RTC_PIN), rtcInterruptISR, FALLING);
  #endif

  // Attention: do not change! Changing these settings may physical damage the flip-disc displays.
  Flip.Init(D7SEG, D7SEG, D3X1, D7SEG, D7SEG);

  delay(3000); 
  Serial.println("FLIP DISC 7-SEGMENT CLOCK");

  // Function allows you to control one, two or three discs of the selected D3X1 display
  // - Flip.Display_3x1(module_number, disc1, disc2, disc3);
  Flip.Display_3x1(1, 0,0,0);

  // his function allows you to display numbers and symbols
  // Flip.Matrix_7Seg(data1,data2,data3,data4); 
  Flip.Matrix_7Seg(B,E,R,T);
  delay(1500);
  Flip.Matrix_7Seg(I,R,I,S);
  delay(1500);
  Flip.Matrix_7Seg(R,O,R,O);
  delay(1500);

  // Link the button functions
  button1.attachClick(ShortPressButton1);
  button3.attachClick(ShortPressButton3);
  button1.attachLongPressStart(LongPressButton1);
  button2.attachLongPressStart(LongPressButton2);
  button3.attachLongPressStart(LongPressButton3);

  // Read disc speed/delay value from eeprom memory
  flip_disc_delay_time = EEPROM.read(ee_delay_address);

  bool success_reading = false;

  // Check if the data read from memory is correct, 
  // if it is one of the defined values ​​0, 5, 10, 25, 50, 75, 99ms
  for (int i = 0; i < 7; i++)
  {
    if(flip_disc_delay_time == flipSpeed[i]) success_reading = true;
  }

  // If the read value is correct, set the speed/delay effect according to the read value
  if(success_reading == true) Flip.Delay(flip_disc_delay_time);
  else // If the data is incorrect, reset the time for the speed/delay effect
  {
    flip_disc_delay_time = 0;
    Flip.Delay(flip_disc_delay_time);  
  }

  // Read remaining setting options from memory
  leading_zero = EEPROM.read(ee_leading_zero_address);
  rest_period = EEPROM.read(ee_rest_period_address);
  sleep_hour = EEPROM.read(ee_sleep_hour_address);
  wake_hour = EEPROM.read(ee_wake_hour_address);
  time_hr = EEPROM.read(ee_time_hr_address);
  temp_on_off = EEPROM.read(ee_temp_on_off_address);
  temp_c_f = EEPROM.read(ee_temp_c_f_address);
  hum_on_off = EEPROM.read(ee_hum_on_off_address);
  temp_hum_fq = EEPROM.read(ee_temp_hum_fq_address);

  // If the read values ​​are incorrect, set the default values
  if(leading_zero != ON && leading_zero != OFF) leading_zero = ON;         // Turn ON leading zero display
  if(rest_period != OFF && rest_period != ON) rest_period = OFF;           // Clock rest period ON/OFF
  if(sleep_hour > 23) sleep_hour = 0;                                      // Set sleep hour to 0
  if(wake_hour > 23) wake_hour = 0;                                        // Set wake hour to 0
  if(time_hr != HR12 && time_hr != HR24) time_hr = HR12;                   // Set the time display to 12 hour format
  if(temp_on_off != ON && temp_on_off != OFF) temp_on_off = OFF;           // Turn off the temperature and humidity display, the other options have no effect
  if(temp_c_f != TPF && temp_c_f != TPC) temp_c_f = TPF;                   // Set temperature display in fahrenheit
  if(hum_on_off != ON && hum_on_off != OFF) hum_on_off = OFF;              // Turn off humidity display
  if(temp_hum_fq != THFQ60 && temp_hum_fq != THFQ30) temp_hum_fq = THFQ60; // Set the temperature and humidity display frequency to 60 seconds

  mqttMessageQueue = xQueueCreate(mqtt_queue_length, sizeof(MqttMessage));

  // WiFi connects in the background and reconnects automatically, see HandleNetwork()
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(SECRET_MQTT_CLIENT_NAME);
  WiFi.begin(SECRET_WIFI_SSID, SECRET_WIFI_PASSWORD);

  mqttClient.enableDebuggingMessages(); // Enable debugging messages sent to serial output
  mqttClient.enableLastWillMessage(MQTT_STATUS_TOPIC, "offline", true); // Broker publishes this if the clock drops off
  mqttClient.setURL(SECRET_MQTT_HOSTNAME, SECRET_MQTT_PORT, SECRET_MQTT_USER, SECRET_MQTT_PASSWORD);
  mqttClient.setMqttClientName(SECRET_MQTT_CLIENT_NAME);

  // OTA updates - hostname is SECRET_MQTT_CLIENT_NAME, password comes from secrets.ini.
  // ArduinoOTA.handle() runs in loop(), so these callbacks can safely use the display
  ArduinoOTA.setHostname(SECRET_MQTT_CLIENT_NAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() {
    Serial.println("OTA update started");
    Flip.Display_3x1(1, 0,0,0);
    Flip.Matrix_7Seg(CLR,O,T,A);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA error: %u\n", error);
    DisplayTime();
  });

  SetupWebServer();

  DisplayTime();
}

/************************************************************************************************/
// Start OTA and MQTT once WiFi first connects. After that WiFi and esp-mqtt reconnect on their own
void HandleNetwork(void)
{
  if(networkStarted == false)
  {
    if(WiFi.status() != WL_CONNECTED) return;

    Serial.printf("WiFi connected: %s\n", WiFi.localIP().toString().c_str());
    ArduinoOTA.begin(); // Also starts mDNS, so the clock is reachable as <hostname>.local
    mqttClient.loopStart(); // Starts the esp-mqtt task and returns immediately
    // Runs on the lwIP task after every SNTP sync - only record when it happened
    sntp_set_time_sync_notification_cb([](struct timeval *tv) {
      sntp_last_sync_time = millis();
      sntpSynced = true;
    });
    sntp_set_sync_interval(ntp_check_interval); // The core's default is every 3 hours
    configTzTime(TIMEZONE, NTP_SERVER); // Starts SNTP in the background
    server.begin();
    MDNS.addService("http", "tcp", 80);
    networkStarted = true;
  }

  ArduinoOTA.handle();
  server.handleClient();
  if(ntpSyncDone == false || millis() - ntp_last_check_time >= ntp_check_interval) SyncTimeNTP();
  PublishState();
}

/************************************************************************************************/
// Update the RTC from NTP if it differs. Called from loop() until the first check succeeds,
// then every ntp_check_interval. Returns false if there's no recent NTP time to check against
bool SyncTimeNTP(void)
{
  // Don't change the time underneath the settings menus
  if(modeSettingsStatus == true) return false;

  // Only trust the system clock if SNTP has synced recently
  if(sntpSynced == false || millis() - sntp_last_sync_time > ntp_max_sync_age)
  {
    // Try again in a minute rather than on every loop
    if(ntpSyncDone == true) ntp_last_check_time = millis() - ntp_check_interval + 60000;
    return false;
  }

  struct tm local;
  if(!getLocalTime(&local, 0)) return false;

  tmElements_t ntp_tm;
  ntp_tm.Second = local.tm_sec;
  ntp_tm.Minute = local.tm_min;
  ntp_tm.Hour = local.tm_hour;
  ntp_tm.Wday = local.tm_wday + 1;                    // TimeLib: Sunday is 1
  ntp_tm.Day = local.tm_mday;
  ntp_tm.Month = local.tm_mon + 1;                    // TimeLib: January is 1
  ntp_tm.Year = CalendarYrToTm(local.tm_year + 1900); // TimeLib: offset from 1970
  time_t ntp_time = makeTime(ntp_tm);

  time_t rtc_time = RTC_RX8025T.get();
  long difference = (long)(ntp_time - rtc_time);

  // The RTC only has one second resolution, so allow a second of difference
  if(abs(difference) > 1)
  {
    RTC_RX8025T.set(ntp_time);
    Serial.printf("RTC was off by %lds, set from NTP: %02d:%02d:%02d\n",
                  difference, ntp_tm.Hour, ntp_tm.Minute, ntp_tm.Second);

    // Redisplay the time the same way the RTC minute interrupt does
    currentTimeStatus = false;
    interruptRtcStatus = true;
  }
  else Serial.println("RTC matches NTP time");

  ntp_last_difference = difference;
  ntp_last_check_time = millis();
  ntpSyncDone = true;
  return true;
}

/************************************************************************************************/
// Publish the temperature and humidity to MQTT_STATE_TOPIC every mqtt_publish_interval.
// Runs from loop(), so reading the sensor can't overlap the display's own measurements
void PublishState(void)
{
  if(!mqttClient.isConnected()) return;
  if(mqttPublishedOnce == true && millis() - mqtt_publish_time < mqtt_publish_interval) return;

  mqtt_publish_time = millis();
  mqttPublishedOnce = true;

  // Raw readings, not the heat index the clock displays
  float celsius = dht.readTemperature();
  float web_humidity = dht.readHumidity();
  if(isnan(celsius) || isnan(web_humidity))
  {
    Serial.println("DHT read failed, not publishing state");
    return;
  }

  float fahrenheit = dht.convertCtoF(celsius);
  char payload[160];
  snprintf(payload, sizeof(payload),
           "{\"temperature_c\":%.1f,\"temperature_f\":%.1f,\"humidity\":%.1f,"
           "\"heat_index_c\":%.1f,\"heat_index_f\":%.1f}",
           celsius, fahrenheit, web_humidity,
           dht.computeHeatIndex(celsius, web_humidity, false),
           dht.computeHeatIndex(fahrenheit, web_humidity, true));
  mqttClient.publish(MQTT_STATE_TOPIC, payload, 0, true);
}

/************************************************************************************************/
// Web config page - status plus every option the buttons can set. There is no authentication,
// anyone on the network can change the settings
void SetupWebServer(void)
{
  server.on("/", HTTP_GET, HandleWebRoot);
  server.on("/settings", HTTP_POST, HandleWebSettings);
  server.on("/sync", HTTP_POST, HandleWebSync);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
}

// Hour label in the clock's current 12/24 hour format, e.g. "11 PM" or "23:00"
String FormatHour(uint8_t hour)
{
  if(time_hr == HR24) return (hour < 10 ? "0" : "") + String(hour) + ":00";

  uint8_t hour_12 = hour % 12;
  if(hour_12 == 0) hour_12 = 12;
  return String(hour_12) + (hour < 12 ? " AM" : " PM");
}

String HtmlOption(int value, const String &label, int selected)
{
  return "<option value=\"" + String(value) + "\"" + (value == selected ? " selected" : "") + ">" + label + "</option>";
}

String HtmlSelect(const char *name, const char *label, const String &options)
{
  return "<label>" + String(label) + "<select name=\"" + String(name) + "\">" + options + "</select></label>";
}

void HandleWebRoot(void)
{
  // Read into a local copy so the display's tm and currentTimeStatus are left alone
  tmElements_t rtc_tm;
  RTC_RX8025T.read(rtc_tm);

  char time_text[16];
  if(time_hr == HR24) snprintf(time_text, sizeof(time_text), "%02d:%02d:%02d", rtc_tm.Hour, rtc_tm.Minute, rtc_tm.Second);
  else
  {
    uint8_t hour_12 = rtc_tm.Hour % 12;
    if(hour_12 == 0) hour_12 = 12;
    snprintf(time_text, sizeof(time_text), "%d:%02d:%02d %s", hour_12, rtc_tm.Minute, rtc_tm.Second, rtc_tm.Hour < 12 ? "AM" : "PM");
  }

  // The DHT library caches readings for 2 seconds, so this doesn't disturb the display's measurements
  bool fahrenheit = (temp_c_f == TPF);
  float web_temperature = dht.readTemperature(fahrenheit);
  float web_humidity = dht.readHumidity();
  String climate_text = "Sensor not responding";
  if(!isnan(web_temperature) && !isnan(web_humidity))
  {
    float heat_index = dht.computeHeatIndex(web_temperature, web_humidity, fahrenheit);
    const char *unit = fahrenheit ? "&deg;F" : "&deg;C";
    climate_text = String(web_temperature, 1) + unit + " (feels like " + String(heat_index, 1) + unit + "), "
                 + String(web_humidity, 0) + "% humidity";
  }

  String ntp_text = "Waiting for NTP";
  if(ntpSyncDone == true)
  {
    if(abs(ntp_last_difference) > 1) ntp_text = "RTC was off by " + String(ntp_last_difference) + "s and was corrected";
    else ntp_text = "RTC matched NTP";
    ntp_text += " " + String((millis() - ntp_last_check_time) / 60000) + " min ago";
  }

  unsigned long uptime = millis() / 1000;
  char uptime_text[24];
  snprintf(uptime_text, sizeof(uptime_text), "%lud %luh %lum", uptime / 86400, (uptime / 3600) % 24, (uptime / 60) % 60);

  String message = "";
  String msg = server.arg("msg");
  if(msg == "saved") message = "Settings saved.";
  else if(msg == "synced") message = "Time checked against NTP. " + ntp_text + ".";
  else if(msg == "nosync") message = "There's no recent NTP time to check against, try again in a moment.";

  String options;
  String html;
  html.reserve(8000);

  html += F("<!doctype html><html><head><meta charset=\"utf-8\">"
            "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
            "<title>Flip-disc Clock</title><style>"
            ":root{color-scheme:light dark;--bg:#f6f6f4;--card:#fff;--fg:#1d1d1b;--muted:#6b6b66;--line:#e2e2dd;--accent:#c2410c;--on-accent:#fff}"
            "@media(prefers-color-scheme:dark){:root{--bg:#161615;--card:#21211f;--fg:#ededea;--muted:#9a9a93;--line:#34342f;--accent:#fb923c;--on-accent:#1d1d1b}}"
            "body{margin:0;background:var(--bg);color:var(--fg);font:16px/1.45 system-ui,-apple-system,sans-serif}"
            "main{max-width:520px;margin:0 auto;padding:24px 16px 48px}"
            "h1{font-size:1.4rem;margin:0 0 4px}h2{font-size:.8rem;text-transform:uppercase;letter-spacing:.06em;color:var(--muted);margin:28px 0 8px}"
            ".time{font:600 2.6rem/1.1 ui-monospace,SFMono-Regular,Menlo,monospace;font-variant-numeric:tabular-nums;margin:12px 0 4px}"
            ".card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:4px 16px}"
            "dl{display:grid;grid-template-columns:auto 1fr;gap:0 16px;margin:0}dt,dd{padding:10px 0;border-bottom:1px solid var(--line);margin:0}"
            "dt{color:var(--muted)}dd{text-align:right;overflow-wrap:anywhere}dl>:nth-last-child(-n+2){border-bottom:0}"
            "label{display:flex;justify-content:space-between;align-items:center;gap:16px;padding:10px 0;border-bottom:1px solid var(--line)}"
            "form .card label:last-child{border-bottom:0}"
            "select{font:inherit;padding:4px 8px;border-radius:6px;border:1px solid var(--line);background:var(--bg);color:var(--fg)}"
            "button{font:inherit;font-weight:600;padding:10px 18px;border-radius:8px;border:0;background:var(--accent);color:var(--on-accent);cursor:pointer;margin-top:12px}"
            "button.secondary{background:transparent;color:var(--accent);border:1px solid var(--accent)}"
            ".msg{background:var(--card);border-left:3px solid var(--accent);padding:10px 12px;border-radius:6px;margin:16px 0 0}"
            ".note{color:var(--muted);font-size:.85rem;margin:8px 0 0}"
            "</style></head><body><main><h1>Flip-disc Clock</h1>");

  if(message.length() > 0) html += "<p class=\"msg\">" + message + "</p>";

  html += "<div class=\"time\">" + String(time_text) + "</div><p class=\"note\">Time from the clock's RTC</p>";

  html += F("<h2>Status</h2><div class=\"card\"><dl>");
  html += "<dt>Climate</dt><dd>" + climate_text + "</dd>";
  html += "<dt>NTP</dt><dd>" + ntp_text + "</dd>";
  html += "<dt>MQTT</dt><dd>" + String(mqttClient.isConnected() ? "Connected" : "Disconnected") + "</dd>";
  // SSIDs can contain characters that are special in HTML
  String ssid = WiFi.SSID();
  ssid.replace("&", "&amp;");
  ssid.replace("<", "&lt;");
  ssid.replace(">", "&gt;");
  html += "<dt>WiFi</dt><dd>" + ssid + " (" + String(WiFi.RSSI()) + " dBm)</dd>";
  html += "<dt>IP address</dt><dd>" + WiFi.localIP().toString() + "</dd>";
  html += "<dt>Uptime</dt><dd>" + String(uptime_text) + "</dd>";
  html += "<dt>Firmware</dt><dd>" GIT_SHA " (built " __DATE__ " " __TIME__ ")</dd>";
  html += F("</dl></div>");

  html += F("<form method=\"post\" action=\"/sync\"><button class=\"secondary\">Sync time from NTP</button></form>");

  html += F("<form method=\"post\" action=\"/settings\"><h2>Time</h2><div class=\"card\">");
  html += HtmlSelect("time_hr", "Time format", HtmlOption(HR12, "12-hour", time_hr) + HtmlOption(HR24, "24-hour", time_hr));
  html += HtmlSelect("leading_zero", "Leading zero", HtmlOption(ON, "On", leading_zero) + HtmlOption(OFF, "Off", leading_zero));
  options = "";
  for(int i = 0; i < 7; i++) options += HtmlOption(flipSpeed[i], flipSpeed[i] == 0 ? "No delay" : String(flipSpeed[i]) + " ms", flip_disc_delay_time);
  html += HtmlSelect("flip_delay", "Flip delay", options);
  html += F("</div>");

  html += F("<h2>Rest period</h2><div class=\"card\">");
  html += HtmlSelect("rest_period", "Rest period", HtmlOption(ON, "On", rest_period) + HtmlOption(OFF, "Off", rest_period));
  String sleep_options, wake_options;
  for(int hour = 0; hour < 24; hour++)
  {
    sleep_options += HtmlOption(hour, FormatHour(hour), sleep_hour);
    wake_options += HtmlOption(hour, FormatHour(hour), wake_hour);
  }
  html += HtmlSelect("sleep_hour", "Sleep at", sleep_options);
  html += HtmlSelect("wake_hour", "Wake at", wake_options);
  html += F("</div>");

  html += F("<h2>Temperature &amp; humidity</h2><div class=\"card\">");
  html += HtmlSelect("temp_on_off", "Show temperature", HtmlOption(ON, "On", temp_on_off) + HtmlOption(OFF, "Off", temp_on_off));
  html += HtmlSelect("temp_c_f", "Unit", HtmlOption(TPF, "&deg;F", temp_c_f) + HtmlOption(TPC, "&deg;C", temp_c_f));
  html += HtmlSelect("hum_on_off", "Show humidity", HtmlOption(ON, "On", hum_on_off) + HtmlOption(OFF, "Off", hum_on_off));
  html += HtmlSelect("temp_hum_fq", "Show every", HtmlOption(THFQ60, "60 seconds", temp_hum_fq) + HtmlOption(THFQ30, "30 seconds", temp_hum_fq));
  html += F("</div><button>Save settings</button></form></main></body></html>");

  server.send(200, "text/html", html);
}

// Read an ON/OFF style option (0 or 1) from the form, keeping the current value if it's missing or invalid
static uint8_t WebArgBool(const char *name, uint8_t current)
{
  if(!server.hasArg(name)) return current;
  String value = server.arg(name);
  if(value == "0") return 0;
  if(value == "1") return 1;
  return current;
}

// Read an hour (0-23) from the form, keeping the current value if it's missing or invalid
static uint8_t WebArgHour(const char *name, uint8_t current)
{
  if(!server.hasArg(name)) return current;
  String value = server.arg(name);
  int hour = value.toInt();
  if(value.length() == 0 || hour < 0 || hour > 23 || (hour == 0 && value != "0")) return current;
  return hour;
}

void HandleWebSettings(void)
{
  time_hr = WebArgBool("time_hr", time_hr);
  leading_zero = WebArgBool("leading_zero", leading_zero);
  rest_period = WebArgBool("rest_period", rest_period);
  sleep_hour = WebArgHour("sleep_hour", sleep_hour);
  wake_hour = WebArgHour("wake_hour", wake_hour);
  temp_on_off = WebArgBool("temp_on_off", temp_on_off);
  temp_c_f = WebArgBool("temp_c_f", temp_c_f);
  hum_on_off = WebArgBool("hum_on_off", hum_on_off);
  temp_hum_fq = WebArgBool("temp_hum_fq", temp_hum_fq);

  // The flip delay must be one of the values the speed menu offers
  if(server.hasArg("flip_delay"))
  {
    int delay_time = server.arg("flip_delay").toInt();
    for(int i = 0; i < 7; i++)
    {
      if(delay_time == flipSpeed[i]) flip_disc_delay_time = delay_time;
    }
  }

  ApplySettings();

  server.sendHeader("Location", "/?msg=saved");
  server.send(303);
}

// Save every setting to EEPROM and redraw the display, as the button settings menus do on exit
void ApplySettings(void)
{
  EEPROM.write(ee_delay_address, flip_disc_delay_time);
  EEPROM.write(ee_leading_zero_address, leading_zero);
  EEPROM.write(ee_rest_period_address, rest_period);
  EEPROM.write(ee_sleep_hour_address, sleep_hour);
  EEPROM.write(ee_wake_hour_address, wake_hour);
  EEPROM.write(ee_time_hr_address, time_hr);
  EEPROM.write(ee_temp_on_off_address, temp_on_off);
  EEPROM.write(ee_temp_c_f_address, temp_c_f);
  EEPROM.write(ee_hum_on_off_address, hum_on_off);
  EEPROM.write(ee_temp_hum_fq_address, temp_hum_fq);

  // Required for Arduino Nano ESP32, saves the changes to the EEPROM
  #if defined(ARDUINO_ARCH_ESP32)
    EEPROM.commit();
  #endif

  Serial.println("Settings saved from the web page");

  Flip.Delay(flip_disc_delay_time);

  // Restart the time/temperature/humidity sequence and redisplay the same way
  // the RTC minute interrupt does, so the rest period is respected
  sequenceRunning = false;
  sequenceSecondRun = false;
  timeDisplayStatus = false;
  currentTimeStatus = false;
  interruptRtcStatus = true;
}

void HandleWebSync(void)
{
  server.sendHeader("Location", SyncTimeNTP() ? "/?msg=synced" : "/?msg=nosync");
  server.send(303);
}

/************************************************************************************************/
// Called by ESP32MQTTClient on the esp-mqtt task when the broker connection is established
void onMqttConnect(esp_mqtt_client_handle_t client)
{
  if(!mqttClient.isMyTurn(client)) return;

  // Replaces the retained "offline" last will set in setup()
  mqttClient.publish(MQTT_STATUS_TOPIC, "online", 0, true);

  // Runs on the esp-mqtt task - only queue the payload, never touch the display here
  mqttClient.subscribe(MQTT_TOPIC, [](const std::string &payload) {
    MqttMessage message;
    strlcpy(message.payload, payload.c_str(), sizeof(message.payload));
    if(xQueueSend(mqttMessageQueue, &message, 0) != pdTRUE)
      Serial.println("MQTT message queue full, dropping message");
  });
}

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
esp_err_t handleMQTT(esp_mqtt_event_handle_t event)
{
  mqttClient.onEventCallback(event);
  return ESP_OK;
}
#else
void handleMQTT(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
  (void)base;
  (void)event_id;
  auto *client = static_cast<ESP32MQTTClient *>(handler_args);
  auto *event = static_cast<esp_mqtt_event_handle_t>(event_data);
  client->onEventCallback(event);
}
#endif

/************************************************************************************************/
// Display any MQTT messages queued by the esp-mqtt task, called from loop()
void HandleMqttMessages(void)
{
  MqttMessage message;
  while(xQueueReceive(mqttMessageQueue, &message, 0) == pdTRUE)
  {
    DisplayMqttMessage(message.payload);
  }
}

void DisplayMqttMessage(const char *payload)
{
    // Hold whatever is shown for mqtt_hold_interval before going back to the time,
    // see CheckMqttHold(). A new message restarts the hold
    mqttHoldStatus = true;
    mqtt_hold_start_time = millis();

    if (strcmp(payload, "Temperature") == 0) {
      DisplayTemperature();
      return;
    } else if (strcmp(payload, "Humidity") == 0) {
      DisplayHumidity();
      return;
    }

    String word = payload;
    Serial.println("Displaying word: " + word);

    Flip.Display_3x1(1,0,0,0); // Clear dots

    // Words shorter than 4 characters, and characters that can't be displayed, are blank
    int flipValues[4] = {CLR, CLR, CLR, CLR};

    for (int i = 0; i < word.length() && i <= 3; i++) {
        char character = word[i];
        int asciiValue = int(character);  // Get the ASCII value of the character

        int flipValue = -1;

        if (asciiValue >= 65 && asciiValue <= 90) {
            // Convert A-Z
            flipValue = asciiValue - 54;
        } else if (asciiValue >= 97 && asciiValue <= 122) {
            // Convert a-z
            flipValue = asciiValue - 86;
        } else if (asciiValue >= 48 && asciiValue <= 57) {
            // Convert 0-9
            flipValue = asciiValue - 48;
        } else if (asciiValue == 32) {
            // Convert space
            flipValue = 10;
        } else if (asciiValue == 45) {
            // Convert -
            flipValue = HLM;
        }

        if (flipValue >= 0) {
            Serial.print("Segment: ");
            Serial.print(i+1);
            Serial.print(", Flip Value: ");
            Serial.println(flipValue);
            flipValues[i] = flipValue;
        }
    }

    Flip.Matrix_7Seg(flipValues[0], flipValues[1], flipValues[2], flipValues[3]);
}

/************************************************************************************************/
// Once the MQTT hold is over, go back to the time (or the rest period display)
void CheckMqttHold(void)
{
  if(millis() - mqtt_hold_start_time < mqtt_hold_interval) return;

  mqttHoldStatus = false;

  // Restart the time/temperature/humidity sequence the same way the RTC minute interrupt does.
  // Resetting rest_period_counter makes CheckRestPeriod() redraw the rest period display
  sequenceRunning = false;
  sequenceSecondRun = false;
  timeDisplayStatus = false;
  currentTimeStatus = false;
  rest_period_counter = 0;
  interruptRtcStatus = true;
}

/************************************************************************************************/
void loop(void)
{
  WatchButtons();
  // While an MQTT message is held on the display, the minute interrupt and the
  // time/temperature sequence wait until CheckMqttHold() ends the hold
  if(mqttHoldStatus == true) CheckMqttHold();
  else
  {
    if(interruptRtcStatus == true) CheckRestPeriod();
    if(timeDisplayStatus == true) DisplayTimeAndTemperature();
  }
  if(timeSettingsStatus == true) SettingTime();
  if(speedSettingsStatus == true) SettingSpeed();
  if(tempSettingsStatus == true) SettingTemp();
  HandleNetwork();
  HandleMqttMessages();
}

/************************************************************************************************/
// Keep watching the buttons
void WatchButtons(void)
{
  button1.tick();
  button2.tick();
  button3.tick();

  // If the time settings are not currently active,
  // and if a long press of the middle or top button is detected, set corresponding flag
  if(modeSettingsStatus == false)
  {
    if(longPressButton1Status == true) speedSettingsStatus = true;
    if(longPressButton2Status == true) timeSettingsStatus = true;
    if(longPressButton3Status == true) tempSettingsStatus = true;
  }
}

/************************************************************************************************/
// Button press handling functions
void ShortPressButton1(void){shortPressButton1Status = true;}
void ShortPressButton3(void){shortPressButton3Status = true;}
void LongPressButton1(void){longPressButton1Status = true;}
void LongPressButton2(void){longPressButton2Status = true;}
void LongPressButton3(void){longPressButton3Status = true;}

/************************************************************************************************/
// Button flags clearing function
void ClearPressButtonFlags(void)
{
  shortPressButton1Status = false;
  shortPressButton3Status = false;
  longPressButton1Status = false;
  longPressButton2Status = false;
  longPressButton3Status = false;
}

/************************************************************************************************/
void DisplayRestPeriod(void)
{
  Flip.Display_3x1(1, 0,0,0);
  Flip.Matrix_7Seg(HLM,HLM,HLM,HLM);
  Serial.println("Rest period");
}

/************************************************************************************************/
void GetTimeRTC(void)
{
  // Get the time from the RTC and save it to the tm structure
  RTC_RX8025T.read(tm);
  currentTimeStatus = true;
}

/************************************************************************************************/
void DisplayTime(void)
{  
  if(currentTimeStatus == false) GetTimeRTC();

  uint8_t hour_time = tm.Hour;
  uint8_t minute_time = tm.Minute;
  
  // 12-Hour conversion
  if(time_hr == HR12)
  {
    if(hour_time > 12) hour_time = hour_time - 12;
    if(hour_time == 0) hour_time = 12; 
  }

  // Extract individual digits for the display
  digit[0] = (hour_time / 10) % 10;
  
  // Check if the leading zero hiding option is enabled
  // CLR - clear the leading display
  if(digit[0] == 0 && leading_zero == OFF) digit[0] = CLR;

  digit[1] = (hour_time / 1) % 10;
  digit[2] = (minute_time / 10) % 10;
  digit[3] = (minute_time / 1) % 10;

  // The function is used to set the delay effect between flip discs. 
  Flip.Delay(flip_disc_delay_time);

  // Display the current time
  Flip.Display_3x1(1, 1,1,0);
  Flip.Matrix_7Seg(digit[0],digit[1],digit[2],digit[3]);
  
  // Print time to the serial monitor
  Serial.print("TIME: ");
  if(hour_time < 10) Serial.print("0");
  Serial.print(hour_time);
  Serial.print(":");
  if(minute_time < 10) Serial.print("0");
  Serial.println(minute_time);

  rest_period_counter = 0;
}

/************************************************************************************************/
void CheckRestPeriod(void)
{
  GetTimeRTC();
  uint8_t current_hour = tm.Hour;
  
  bool clockStatus = false;

  // If the rest period is turned off, the clock is always on
  if(rest_period == OFF) clockStatus = true;
  // This part of the code checks if the sleep_hour is greater than the wake_hour.
  // If so, it means that the time interval for the clock to be turned off passes through midnight
  else if(sleep_hour > wake_hour)
  {
    // This condition checks if the current_hour is within the range where the clock should be turned off
    if(current_hour >= sleep_hour || current_hour < wake_hour) clockStatus = false;
    else clockStatus = true;
  }
  // If sleep_hour is not greater than wake_hour, it means that the time interval 
  // in which the clock is to be turned off is within one day
  else 
  {
    // This condition checks if the current_hour is within the range where the clock should be turned off
    if(current_hour >= sleep_hour && current_hour < wake_hour) clockStatus = false;
    else clockStatus = true;
  }

  if(clockStatus == true) 
  {
    rest_period_counter = 0;
    timeDisplayStatus = true;
  }

  if(clockStatus == false) 
  {
    if(rest_period_counter == 0) DisplayRestPeriod();

    rest_period_counter++;
    timeDisplayStatus = false;
  }

  interruptRtcStatus = false;
}

/* The sequence is started every full minute after the RTC interrupt occurs or every 30 seconds. 
30 seconds is always counted down from the time of the RTC interrupt, this method was chosen 
to obtain the best visual effect. After the sequence is called, the millis() start time 
is recorded at the very beginning. The time is displayed first. Depending on the options 
that have been set: if temperature display has been enabled, then after 
the time interval "wait_interval" set by default to 5 seconds, the temperature will be displayed. 
Then after the time "wait_interval" the humidity will be displayed, if it has been enabled. 
Of course, if the temperature display has been disabled and the humidity display enabled, 
only humidity will be displayed - these are independent options. If the temperature 
and humidity display frequency has been set to 60 seconds, the sequence will end. 
If it is set to 30 seconds, a new sequence will start 30 seconds after the first sequence was started. 
After the second cycle, the sequence will end. The next loop will start at the next full minute.*/
void DisplayTimeAndTemperature(void)
{    
  if(sequenceRunning == false)     // If the sequence is not already running, initialize it 
  {
    sequence_start_time = millis(); // Record the start time
    sequence_level = 0;             // Start from the first state
    sequenceRunning = true;
  }

  // Update the current time
  sequence_current_time = millis();

  switch(sequence_level) 
  {
    case 0:
        DisplayTime();
        sequence_level = 1;
        sequence_start_time = sequence_current_time;  // Reset the start time for the next interval
      break;

    case 1:
      if(temp_on_off == OFF) sequence_level = 2;
      else if(temp_on_off == ON)
      {
        if(sequence_current_time - sequence_start_time >= wait_interval)
        {
          DisplayTemperature();
          sequence_level = 2; 
          sequence_start_time = sequence_current_time;  // Reset the start time for the next interval
        }
      }
      break;

    case 2:
      if(hum_on_off == OFF) sequence_level = 3;
      else if(hum_on_off == ON)
      {
        if(sequence_current_time - sequence_start_time >= wait_interval)
        {
          DisplayHumidity();
          sequence_level = 3; 
          sequence_start_time = sequence_current_time;  // Reset the start time for the next interval
        }
      }
      break;

    case 3:
      if(temp_on_off == OFF && hum_on_off == OFF) sequence_level = 5;
      else
      {
        if(sequence_current_time - sequence_start_time >= wait_interval)
        {
          DisplayTime();

          if(temp_hum_fq == THFQ30 && sequenceSecondRun == false) 
          {
            sequence_level = 4;
            sequence_start_time = sequence_current_time;  // Reset the start time for the next interval
          }
          else sequence_level = 5;
        }
      }
      break;   

    case 4:
      if(sequence_current_time - sequence_start_time >= sequence_interval)
      {
        sequence_level = 1;
        sequenceSecondRun = true; 
        sequence_start_time = sequence_current_time;  // Reset the start time for the next interval
      }

      break; 

    case 5:
      sequenceRunning = false;
      sequenceSecondRun = false;
      timeDisplayStatus = false;
      break;
  }
}

/************************************************************************************************/
void MeasureTemperatureAndHumidity(void)
{
  // Reading temperature and humidity
  humidity = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float celsius = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float fahrenheit = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(humidity) || isnan(celsius) || isnan(fahrenheit)) return;

  // Compute heat index in Fahrenheit (the default)
  temperature_fahrenheit = dht.computeHeatIndex(fahrenheit, humidity);
  // Compute heat index in Celsius (isFahreheit = false)
  temperature_celsius = dht.computeHeatIndex(celsius, humidity, false);

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print("%  Temperature: ");
  Serial.print(temperature_celsius);
  Serial.print("°C  ");
  Serial.print(temperature_fahrenheit);
  Serial.println("°F");

  // Rounding the temperature value ​​and preparing data for display
  temperature_fahrenheit = temperature_fahrenheit + 0.05;

  if(temperature_fahrenheit < 100) temperature_fahrenheit = temperature_fahrenheit * 10;

  temperature_celsius = (temperature_celsius + 0.05) * 10 ;
  humidity = humidity * 10;

  measurementStatus = true;
}

/************************************************************************************************/
void DisplayTemperature(void)
{
  MeasureTemperatureAndHumidity();

  uint8_t type_symbol = 0;
  float data_temp = 0;

  if(temp_c_f == TPF) {data_temp = temperature_fahrenheit; type_symbol = F;} 
  if(temp_c_f == TPC) {data_temp = temperature_celsius; type_symbol = C;} 

  int data_value = int (data_temp);
  
  uint8_t digit1 = (data_value / 100) % 10;
  uint8_t digit2 = (data_value / 10) % 10;
  uint8_t digit3 = (data_value / 1) % 10;

  if(leading_zero == OFF && digit1 == 0) digit1 = CLR;
  
  if(digit3 == 0) 
  {
    digit3 = DEG;
    Flip.Display_3x1(1, 0,0,0);
  }
  else Flip.Display_3x1(1, 0,0,1);

  Flip.Matrix_7Seg(digit1, digit2, digit3, type_symbol);
}

/************************************************************************************************/
void DisplayHumidity(void)
{
  // During the temperature and humidity display sequence, the temperature is always displayed first 
  // and then the measurement is taken. However, it may happen that the temperature display is turned off, 
  // then we have to take the measurement.
  if(temp_on_off == OFF || measurementStatus == false) 
  {
    MeasureTemperatureAndHumidity();
    // The flag is set to true after the measurement is taken 
    // and therefore we need to reset it.
    measurementStatus = false;
  }

  int data_hum = int (humidity);

  uint8_t digit1 = (data_hum / 100) % 10;
  uint8_t digit2 = (data_hum / 10) % 10;
  uint8_t digit3 = (data_hum / 1) % 10;

  uint8_t type_symbol = 0;

  if(leading_zero == OFF && digit1 == 0) digit1 = CLR;
  
  if(digit3 == 0) 
  {
    digit3 = PFH;
    type_symbol = PSH;
    Flip.Display_3x1(1, 0,0,0);
  }
  else 
  {
    type_symbol = H;
    Flip.Display_3x1(1, 0,0,1);
  }

  Flip.Matrix_7Seg(digit1, digit2, digit3, type_symbol);
}

/************************************************************************************************/
void SettingSpeed(void)
{
  ClearPressButtonFlags();

  // If the settings were enabled during the time/temperature/humidity sequence, 
  // we need to reset the sequence flag
  sequenceRunning = false;

  // Activate the settings mode
  modeSettingsStatus = true;

  Serial.println();
  Serial.println("FLIP DISC SPEED/DELAY EFFECT SETTINGS");
  
  Flip.Delay(0);
  Flip.Display_3x1(1, 0,0,0);                     // Clear dots
  Flip.Matrix_7Seg(S,P,E,D);
  delay(1500);
  Flip.Display_3x1(1, 1,1,0);                     // Set dots
  Flip.Display_7Seg(1,S); Flip.Display_7Seg(2,P); // Display "SP"

  int speed_index = 0;
  bool updateDisplay = true;

  // To correctly display the current settings we need to find speed_index, 
  // which is the place where the current value is in flipSpeed[] array. 
  // Then after entering the settings we start from the current value.
  for(int i = 0; i < 6; i++)
  {
    if(flip_disc_delay_time == flipSpeed[i]) speed_index = i;
  }

  do // Stay in the speed/delay settings until the value is set
  {
    WatchButtons();
    
    if(shortPressButton1Status == true || shortPressButton3Status == true)
    {      
      if(shortPressButton1Status == true) speed_index++; // Top button "+1"
      if(shortPressButton3Status == true) speed_index--; // Bottom button "-1"

      ClearPressButtonFlags();
      updateDisplay = true;
    }
    
    if(longPressButton2Status == true)
    {
      ClearPressButtonFlags();
      speedSettingsStatus = false;
    } 

    if(updateDisplay == true)
    {
      if(speed_index > 6) speed_index = 0;
      if(speed_index < 0) speed_index = 6;

      flip_disc_delay_time = flipSpeed[speed_index];
      Flip.Delay(flip_disc_delay_time);

      uint8_t digit3 = (flip_disc_delay_time / 10) % 10;
      uint8_t digit4 = (flip_disc_delay_time / 1 ) % 10;

      if(digit3 == 0) digit3 = CLR;

      Serial.print("Speed/delay: "); Serial.print(flipSpeed[speed_index]); Serial.println("ms");  

      Flip.Display_7Seg(3, digit3);
      Flip.Display_7Seg(4, digit4);

      updateDisplay = false;
    }
  } while(speedSettingsStatus == true); // Stay in the speed settings until all digits are set 

  Serial.println("Settings have been saved");
  Serial.println("------------------------");
  
  EEPROM.write(ee_delay_address, flip_disc_delay_time);
  
  // Required for Arduino Nano ESP32, saves the changes to the EEPROM
  #if defined(ARDUINO_ARCH_ESP32)
    EEPROM.commit(); 
  #endif

  modeSettingsStatus = false;
  timeDisplayStatus = false;
  currentTimeStatus = false;
  interruptRtcStatus = false;

  DisplayTime();
}

/************************************************************************************************/
void SettingTemp(void)
{
  ClearPressButtonFlags();

  // If the settings were enabled during the time/temperature/humidity sequence, 
  // we need to reset the sequence flag
  sequenceRunning = false;

  // Settings are active
  modeSettingsStatus = true;

  Serial.println();
  Serial.println("TEMPERATURE & HUMIDITY SETTINGS");

  Flip.Delay(0);
  Flip.Matrix_7Seg(T,E,M,P);
  Flip.Display_3x1(1, 0,0,0); // Clear the dots
  delay(1500); 
  Flip.Display_3x1(1, 1,1,0); // Set the dots

  uint8_t temp_settings_level = 0;
  bool updateDisplay = true;

  do // Stay in settings until all values are set
  {
    WatchButtons();

    if(shortPressButton1Status == true || shortPressButton3Status == true)
    {
      if(temp_settings_level == 0) temp_on_off = !temp_on_off;
      if(temp_settings_level == 1) temp_c_f = !temp_c_f;
      if(temp_settings_level == 2) hum_on_off = !hum_on_off;
      if(temp_settings_level == 3) temp_hum_fq = !temp_hum_fq; 

      updateDisplay = true;
      ClearPressButtonFlags();
    }

    if(longPressButton2Status == true)
    {
      temp_settings_level++;
      if(temp_settings_level <= 3) updateDisplay = true;        // Stay in settings
      if(temp_settings_level >  3) tempSettingsStatus = false;  // Exit settings

      ClearPressButtonFlags();
    }

  if(updateDisplay == true)
  {      
    if(temp_settings_level == 0)
    {
      Serial.print("Temperature display: ");
      if(temp_on_off == OFF) {Flip.Matrix_7Seg(T,P,O,F); Serial.println("OFF");}
      if(temp_on_off == ON) {Flip.Matrix_7Seg(T,P,O,N); Serial.println("ON");}
    }

    // If the temperature display has been disabled then 
    // skip the Celsius/Fahrenheit display format selection and go to the humidity display option
    if(temp_settings_level == 1)
    {
      if(temp_on_off == ON) 
      {
        Serial.print("Temperature: ");
        if(temp_c_f == TPF) {Flip.Matrix_7Seg(T,D,DEG,F); Serial.println("°F");}
        if(temp_c_f == TPC) {Flip.Matrix_7Seg(T,D,DEG,C); Serial.println("°C");}
      } else temp_settings_level = 2;
    } 

    if(temp_settings_level == 2)
    {
      Serial.print("Humidity display: ");
      if(hum_on_off == OFF) {Flip.Matrix_7Seg(H,U,O,F); Serial.println("OFF");}
      if(hum_on_off == ON) {Flip.Matrix_7Seg(H,U,O,N); Serial.println("ON");}
    }

    // If the temperature and humidity display has been disabled 
    // then skip the display frequency option and complete the settings
    if(temp_settings_level == 3)
    {
      if(temp_on_off == ON || hum_on_off == ON)
      {
        Serial.print("Temperature and/or Humidity display frequency: "); 
        if(temp_hum_fq == THFQ60) {Flip.Matrix_7Seg(F,Q,6,0); Serial.println("60 seconds");}
        if(temp_hum_fq == THFQ30) {Flip.Matrix_7Seg(F,Q,3,0); Serial.println("30 seconds");}
      } else tempSettingsStatus = false;
    } 

    updateDisplay = false;
  }  

  } while(tempSettingsStatus == true); // Stay in the speed settings until all digits are set

  Serial.println("Settings have been saved");
  Serial.println("------------------------");
  
  // Save settings
  EEPROM.write(ee_temp_on_off_address, temp_on_off);
  EEPROM.write(ee_temp_c_f_address, temp_c_f);
  EEPROM.write(ee_hum_on_off_address, hum_on_off);
  EEPROM.write(ee_temp_hum_fq_address, temp_hum_fq);

  // Required for Arduino Nano ESP32, saves the changes to the EEPROM
  #if defined(ARDUINO_ARCH_ESP32)
    EEPROM.commit(); 
  #endif

  modeSettingsStatus = false;
  timeDisplayStatus = false;
  currentTimeStatus = false;
  interruptRtcStatus = false;

  DisplayTime();
}

/************************************************************************************************/
void SettingTime(void)
{
  ClearPressButtonFlags();

  // If the settings were enabled during the time/temperature/humidity sequence, 
  // we need to reset the sequence flag
  sequenceRunning = false;

  // Settings are active
  modeSettingsStatus = true;
  
  Serial.println();
  Serial.println("TIME FORMAT SETTINGS");

  Flip.Delay(0);
  Flip.Display_3x1(1, 0,0,0); // Clear dots
  Flip.Matrix_7Seg(H,O,U,R);
  delay(1500);
  Flip.Display_3x1(1, 1,1,0); // Set dots

  uint8_t time_settings_level = 0;
  bool updateDisplay = true;

  do
  {
    WatchButtons();

    if(shortPressButton1Status == true || shortPressButton3Status == true)
    {
      if(time_settings_level == 0) time_hr = !time_hr;
      if(time_settings_level == 1) leading_zero = !leading_zero;

      ClearPressButtonFlags();
      updateDisplay = true;    
    }
    
    if(longPressButton2Status == true)
    {
      time_settings_level++;
      if(time_settings_level <= 1) updateDisplay = true;       // Stay in settings
      if(time_settings_level  > 1) timeSettingsStatus = false;  // Exit first part od time settings
      ClearPressButtonFlags();
    }

    if(updateDisplay == true)
    {
      updateDisplay = false;

      if(time_settings_level == 0)
      {
        Serial.print("Time format: ");
        if(time_hr == HR12) {Flip.Matrix_7Seg(H,R,1,2); Serial.println("12-hour");}
        if(time_hr == HR24) {Flip.Matrix_7Seg(H,R,2,4); Serial.println("24-hour");}
      }

      if(time_settings_level == 1)
      {
        Serial.print("Leading zero: ");
        if(leading_zero == ON)  {Flip.Matrix_7Seg(L,Z,O,N); Serial.println("ON"); }
        if(leading_zero == OFF) {Flip.Matrix_7Seg(L,Z,O,F); Serial.println("OFF");}
      }

      updateDisplay = false;
    } 
  } while(timeSettingsStatus == true);

  Serial.println();
  Serial.println("TIME SETTINGS");
  
  Flip.Delay(0);
  Flip.Display_3x1(1, 0,0,0); // Clear dots
  Flip.Matrix_7Seg(T,I,M,E);
  delay(1500);
  
  time_settings_level = 0;
  updateDisplay = true;
  timeSettingsStatus = true;
  int current_digit = 0;
  bool updateDigit = true;

  uint8_t hour_time = tm.Hour;
  uint8_t minute_time = tm.Minute;
  
  // 12-Hour conversion
  if(time_hr == HR12)
  {
    if(hour_time > 12) hour_time = hour_time - 12;
    if(hour_time == 0) hour_time = 12; 
  }

  // Extract individual digits for the display
  digit[0] = (hour_time / 10) % 10;
  digit[1] = (hour_time / 1) % 10;
  digit[2] = (minute_time / 10) % 10;
  digit[3] = (minute_time / 1) % 10;

  do  // Stay in the time settings until all digits are set
  {
    WatchButtons();
    
    if(shortPressButton1Status == true || shortPressButton3Status == true)
    {
      if(shortPressButton1Status == true) current_digit++; // Top button "+1"
      if(shortPressButton3Status == true) current_digit--; // Bottom button "-1"
      
      if(time_hr == HR12)
      {
        if(time_settings_level == 0) // First digit: 0-1
        {
          if(current_digit < 0) current_digit = 1;
          if(current_digit > 1) current_digit = 0;
          Serial.print(current_digit); Serial.println("---");
        }

        if(time_settings_level == 1) // Second digit: 1-9 or 0-2
        {
          // If the first digit is 0 then the second digit can be from 1 to 9
          if(digit[0] == 0)
          {
           if(current_digit < 1) current_digit = 9;
           if(current_digit > 9)  current_digit = 1;
          }

          // If the first digit is 1 then the second digit 
          // cannot be greater than 2 because the largest possible hour is 12
          if(digit[0] == 1)
          {
            if(current_digit > 2) current_digit = 0;
            if(current_digit < 0) current_digit = 2;
          }

          Serial.print("-"); Serial.print(current_digit); Serial.println("--");
        }
      }
      
      if(time_hr == HR24)
      {
        // First digit: 0-2
        if(time_settings_level == 0)
        {
          if(current_digit < 0) current_digit = 2;
          if(current_digit > 2) current_digit = 0;
          Serial.print(current_digit); Serial.println("---");
        }

        // Second digit: 0-9 or 0-3
        if(time_settings_level == 1)
        {
          // If the first digit is 0 or 1 then the second digit can be from 0 to 9
          if(digit[0] != 2)
          {
            if(current_digit < 0) current_digit = 9;
            if(current_digit > 9) current_digit = 0;
          }

          // If the first digit is 2 then the second digit 
          // cannot be greater than 3 because the largest possible hour is 23
          if(digit[0] == 2)
          {
            if(current_digit > 3) current_digit = 0;
            if(current_digit < 0) current_digit = 3;
          }

          Serial.print("-"); Serial.print(current_digit); Serial.println("--");
        }
      }  

      // Third digit: 0-5
      if(time_settings_level == 2)
      {
        if(current_digit < 0) current_digit = 5;
        if(current_digit > 5) current_digit = 0;
        Serial.print("--"); Serial.print(current_digit); Serial.println("-");
      }

      // Fourth digit: 0-9
      if(time_settings_level == 3)
      {
        if(current_digit < 0) current_digit = 9;
        if(current_digit > 9) current_digit = 0;
        Serial.print("---"); Serial.println(current_digit);
      }          

      // Update the display for only the currently set digit,
      // for more details see FlipDisc.h library
      Flip.Display_7Seg(time_settings_level + 1, current_digit);     
      digit[time_settings_level] = current_digit;

      ClearPressButtonFlags();
    }

    if(longPressButton2Status == true)
    {      
      time_settings_level++;      
      if(time_settings_level >  3) timeSettingsStatus = false;  // Exit settings
      if(time_settings_level <= 3) 
      {
        updateDigit = true;
        updateDisplay = true;
      }

      ClearPressButtonFlags();
    }

    if(updateDigit == true)
    { 
      // It may happen that the previous set time was e.g. 04:00. We start setting the time from the first digit on the left, 
      // so if we set the first digit to e.g. 1, then after entering the second digit of the hour, the number 4 could appear. 
      // It may also happen e.g. the old set time is 10:00. If we start the new time by setting the first digit to 0, 
      // then the second digit could also be displayed 0. 
      // Therefore, these conditions ensure that such an inaccuracy does not occur
      if(time_settings_level == 1 && time_hr == HR12)
      {
        if(digit[time_settings_level - 1] == 1 && digit[time_settings_level] > 2) digit[time_settings_level] = 0;
        if(digit[time_settings_level - 1] == 0 && digit[time_settings_level] == 0) digit[time_settings_level] = 1;
      }

      if(time_settings_level == 1 && time_hr == HR24)
      {
        if(digit[time_settings_level - 1] == 2 && digit[time_settings_level] > 3) digit[time_settings_level] = 0;
      }
      
      current_digit = digit[time_settings_level];
      updateDigit = false;
    }
    
    if(updateDisplay == true)
    {
      if(time_settings_level == 0) {Flip.Matrix_7Seg(digit[0],HLM,HLM,HLM); Serial.print(digit[0]); Serial.println("---");}
      if(time_settings_level == 1) {Flip.Matrix_7Seg(HLM,digit[1],HLM,HLM); Serial.print("-"); Serial.print(digit[1]); Serial.println("--");}
      if(time_settings_level == 2) {Flip.Matrix_7Seg(HLM,HLM,digit[2],HLM); Serial.print("--"); Serial.print(digit[2]); Serial.println("-");}
      if(time_settings_level == 3) {Flip.Matrix_7Seg(HLM,HLM,HLM,digit[3]); Serial.print("---"); Serial.println(digit[3]);}

      updateDisplay = false;
    }      

  } while(timeSettingsStatus == true); // Stay in the speed settings until all digits are set
  
  // The hour and minute settings are complete
  // Next settings are AM/PM time format for 12 hour time. 
  // The last option to set will be the rest period 
  // of the clock - the hour to turn off the clock and the hour to turn on the clock
  // sleep time & wake time

  Flip.Display_3x1(1, 1,1,0); // Set the dots

  updateDisplay = true;
  bool updateData = true;

  bool time_am_pm = 0;
  bool sleep_am_pm = 0;
  bool wake_am_pm = 0;

  int set_hour = 0;
  uint8_t digit_1 = 0;
  uint8_t digit_2 = 0;

  timeSettingsStatus = true;
  time_settings_level = 0;

  do
  {
    WatchButtons();

    if(shortPressButton1Status == true || shortPressButton3Status == true)
    {
      if(time_settings_level == 2 || time_settings_level == 4)
      {
        // Top button "+1", bottom button "-1"
        if(shortPressButton1Status == true) set_hour++;
        if(shortPressButton3Status == true) set_hour--;

        if(time_hr == HR24)
        {
          if(set_hour > 23) set_hour = 0;
          if(set_hour < 0) set_hour = 23;
        }

        if(time_hr == HR12)
        {
          if(set_hour > 12) set_hour = 1;
          if(set_hour < 1) set_hour = 12;
        } 
      }

      if(time_settings_level == 0) time_am_pm = !time_am_pm;
      if(time_settings_level == 1) rest_period = !rest_period;
      if(time_settings_level == 3) sleep_am_pm = !sleep_am_pm;
      if(time_settings_level == 5) wake_am_pm = !wake_am_pm;

      updateDisplay = true;
    }  
    
    if(longPressButton2Status == true)
    {      
      time_settings_level++;      
      if(time_settings_level >  5) timeSettingsStatus = false;  // Exit settings
      if(time_settings_level <= 5) 
      {
        updateData = true;
        updateDisplay = true;
      }

      ClearPressButtonFlags();
    }
    
    // If the time is set to 24 hours, there is no need to set the time format AM/PM
    if(time_hr == HR24) 
    {
      if(time_settings_level == 0) time_settings_level = 1;
      if(time_settings_level == 3) time_settings_level = 4;
      if(time_settings_level == 5) 
      {
        time_settings_level = 6;
        timeSettingsStatus = false;
      }
    }

    // RTC does not support 12 hour time, a simple trick was used to determine AM/PM time in 12 hour system. 
    // Time stored in RTC is always in 24 hour format, so with a simple conversion we can determine 
    // and prepare the time to display in 12 hour AM/PM format. 
    // If we have hours from 0 to 11, it is AM time, if from 12 to 23, it is PM.
    if(updateData == true)
    {
      if(time_hr == HR24)
      {
        if(time_settings_level == 2) set_hour = sleep_hour;
        if(time_settings_level == 4) set_hour = wake_hour; 
      }
      
      if(time_hr == HR12)
      {
        if(time_settings_level == 2)
        {
          if(sleep_hour >= 12) 
          {
            sleep_am_pm = PM;
            set_hour = sleep_hour - 12;
          }
          if(sleep_hour <  12) 
          {
            sleep_am_pm = AM;
            set_hour = sleep_hour; 
          }
        }
        
        if(time_settings_level == 4)
        {
          if(wake_hour >= 12) 
          {
            wake_am_pm = PM;
            set_hour = wake_hour - 12;
          }
          if(wake_hour <  12) 
          {
            wake_am_pm = AM;
            set_hour = wake_hour; 
          }
        }

        if(set_hour == 0) set_hour = 1;
      }

      updateData = false;
    }

    // If the clock rest period has been disabled, skip the rest of the options and exit the settings
    if(rest_period == OFF && time_settings_level == 2) 
    {
      timeSettingsStatus = false; 
      time_settings_level = 6;
    }

    if(updateDisplay == true)
    {      
      if(time_settings_level == 0)
      {
        Serial.print("Time format: ");
        if(time_am_pm == AM) {Flip.Matrix_7Seg(T,F,A,M); Serial.println("AM");}
        if(time_am_pm == PM) {Flip.Matrix_7Seg(T,F,P,M); Serial.println("PM");}
      }

      if(time_settings_level == 1)
      {
        Serial.print("Clock rest period: ");
        if(rest_period == OFF) {Flip.Matrix_7Seg(R,P,O,F); Serial.println("OFF");}
        if(rest_period == ON)  {Flip.Matrix_7Seg(R,P,O,N); Serial.println("ON");}
      }

      if(time_settings_level == 2)
      {
        // To display hour correctly on two displays, we need to split the entered data into tens and ones. 
        // If the first digit is 0, it will not be displayed.
        Serial.print("Sleep time: ");
        sleep_hour = set_hour;
        digit_1 = (set_hour / 10) % 10;
          
        if(digit_1 == 0) digit_1 = CLR;
        digit_2 = (set_hour / 1 ) % 10;
        Serial.println(set_hour);
        Flip.Matrix_7Seg(S,H,digit_1,digit_2);
      }

      if(time_settings_level == 3)
      {
        Serial.print("Sleep time: ");
        if(sleep_am_pm == AM) {Flip.Matrix_7Seg(S,F,A,M); Serial.println("AM");}
        if(sleep_am_pm == PM) {Flip.Matrix_7Seg(S,F,P,M); Serial.println("PM");}
      }

      if(time_settings_level == 4)
      {
        // To display hour correctly on two displays, we need to split the entered data into tens and ones. 
        // If the first digit is 0, it will not be displayed.
        Serial.print("Wake time: ");
        wake_hour = set_hour;
        digit_1 = (set_hour / 10) % 10;
          
        if(digit_1 == 0) digit_1 = CLR;
        digit_2 = (set_hour / 1 ) % 10;
        Serial.println(set_hour);
        Flip.Matrix_7Seg(W,H,digit_1,digit_2);
      }

      if(time_settings_level == 5)
      {
        Serial.print("Wake time: ");
        if(wake_am_pm == AM) {Flip.Matrix_7Seg(W,F,A,M); Serial.println("AM");}
        if(wake_am_pm == PM) {Flip.Matrix_7Seg(W,F,P,M); Serial.println("PM");}
      }

      updateDisplay = false;
    } 

    ClearPressButtonFlags();

  } while(timeSettingsStatus == true); // Stay in the speed settings until all digits are set

  Serial.println("Settings have been saved");
  Serial.println("------------------------");
  
  // Convert entered individual digits to the format supported by RTC
  hour_time = (digit[0] * 10) + digit[1];
  minute_time = (digit[2] * 10) + digit[3];


  // RTC does not support 12 hour time, a simple trick was used to determine AM/PM time in 12 hour system. 
  // Time stored in RTC is always in 24 hour format, so with a simple conversion we can determine 
  // and prepare the time to display in 12 hour AM/PM format. 
  // If we have hours from 0 to 11, it is AM time, if from 12 to 23, it is PM.
  if(time_hr == HR12)
  {
    if(time_am_pm == PM && hour_time != 12) hour_time = hour_time + 12;
    if(time_am_pm == AM && hour_time == 12) hour_time = 0;

    if(sleep_am_pm == PM && sleep_hour != 12) sleep_hour = sleep_hour + 12;
    if(sleep_am_pm == AM && sleep_hour == 12) sleep_hour = 0;

    if(wake_am_pm == PM && wake_hour != 12) wake_hour = wake_hour + 12;
    if(wake_am_pm == AM && wake_hour == 12) wake_hour = 0;
  }

  updateDisplay = true;
  
  do
  {
    WatchButtons();

    if(updateDisplay == true)
    {
      Flip.Display_3x1(1, 0,0,0);
      Flip.Matrix_7Seg(E,X,I,T);
      Serial.println("Long press the middle button to exit");
      updateDisplay = false;  
    }
  } while(longPressButton2Status == false);

  ClearPressButtonFlags();

  EEPROM.write(ee_time_hr_address, time_hr);
  EEPROM.write(ee_leading_zero_address, leading_zero);
  EEPROM.write(ee_rest_period_address, rest_period);
  EEPROM.write(ee_sleep_hour_address, sleep_hour);
  EEPROM.write(ee_wake_hour_address, wake_hour);

  // Required for Arduino Nano ESP32, saves the changes to the EEPROM
  #if defined(ARDUINO_ARCH_ESP32)
    EEPROM.commit(); 
  #endif
  
  Serial.println("Settings have been saved");
  Serial.println("------------------------");

  // setTime(hh, mm, ss, day, month, year) 
  // The date is skipped and the seconds are set by default to 0
  // We are only interested in hours and minutes
  setTime(hour_time, minute_time, 0, 0, 0, 0);

  // Set the RTC from the system time
  RTC_RX8025T.set(now());
  
  modeSettingsStatus = false;
  timeDisplayStatus = false;
  currentTimeStatus = false;
  interruptRtcStatus = false;

  DisplayTime();
}
