/*
    HydroNodeLPW.ino

    Precipitation measurement node with
    the Grove hall effect sensor.

    ===
    This code sleeps until the hall effect interrupt
    wakes it up.  Upon waking, it stores the interrupt
    count (number of tips) and the offset of the time
    the tip occurs (as a delta relative to the time
    the chip started running to the tip time).

    The array data_values, simply stores the millis() of
    the current tip (and subsequent tips should they
    occur during the communication sequence) and
    transmits the tip calibration data (pre-calibrated
    and stored in the config) along with the offset.

    RTC memory is used to store the interrupt count and
    time in millis.

    copyright (c) 2020 keith maull
    Website    :
    Author     : kmaull-ucar
    Create Time:
    Change Log :
*/
#include <WiFi.h>
#include "BluetoothSerial.h" // Header File for Serial BluetoothTurotial on: www.circuitdigest.com 
#include "nvs_flash.h"       // non-volatile ram
#include "SPIFFS.h"          // SPIFFS file system
#include <ArduinoJson.h>     // JSON decoding
#include "IoTwx.h"           // https://github.com/iotwx, core IoTwx


IoTwx                       node;
float                       calibrated_tip;
char*                       sensor;
char*                       topic;
int                         timezone;
int                         max_frequency       = 80;
int                         reset_interval;
int                         publish_interval;
unsigned long               start_millis        = 0;
const byte                  interrupt_pin       = 32;
volatile int                interrupt_count     = 0;
volatile RTC_DATA_ATTR int
data_count          = 0;
volatile RTC_DATA_ATTR unsigned long
interrupt_time[54];    // ~2 hours worth of data
portMUX_TYPE                mux                 = portMUX_INITIALIZER_UNLOCKED;


/*
   interrupt handler code that stores time of interrupt (in millis())
   in interrupt_time
*/
void IRAM_ATTR handleInterrupt() {
  portENTER_CRITICAL_ISR(&mux);
  interrupt_time[interrupt_count++] = millis();
  portEXIT_CRITICAL_ISR(&mux);
}


void setup() {
  File                      file;
  StaticJsonDocument<1024>  doc;
  char                      uuid[32];
  String                    mac = String((uint32_t)ESP.getEfuseMac(), HEX);

  strcpy(uuid, "ESP32P_AtomLite_"); strcat(uuid, (const char*) mac.c_str());

  Serial.begin(57600);
  Serial.println("[] This is the IoTwx HydroNode.  Initializing ...");

  start_millis = millis();
  init_led();     // inialize AtomLite LED

  node = IoTwx(wait_for_bluetooth_config(uuid, millis(), 90)); // initializes config.json
  if (node.isConfigured())
  {
    Serial.println("[] intializing hall effect interrupts");
    pinMode(interrupt_pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interrupt_pin), handleInterrupt, FALLING);

    Serial.println("[] deserializing SPIFFS JSON config file to local document object");
    file = SPIFFS.open("/config.json", FILE_READ);
    deserializeJson(doc, file);
    file.close();

    Serial.println("[] reading values from SPIFFS JSON config");
    calibrated_tip   = atof((const char*)doc["iotwx_calibrated_tip"]);
    timezone         = atoi((const char*)doc["iotwx_timezone"]);
    sensor           = strdup((const char*)doc["iotwx_sensor"]);
    topic            = strdup((const char*)doc["iotwx_topic"]);
    reset_interval   = 1000 * 60 * atoi((const char*)doc["iotwx_reset_interval"]);
    publish_interval = 1000 * 60 * atoi((const char*)doc["iotwx_publish_interval"]);
    max_frequency    = atoi((const char*)doc["iotwx_max_frequency"]);

    // set pin 32 for wakeup on hall effect
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_32, 0);

    // begin shutdown sequence, downthrottle, shutdown wifi and BT
    btStop(); Serial.println("[] BT disconnected for power reduction");
    setCpuFrequencyMhz(max_frequency); Serial.println(); Serial.print("[] CPU downthrottled to "); Serial.print(max_frequency); Serial.println("Mhz for power reduction");
    WiFi.mode(WIFI_OFF); Serial.println("[] Wifi shut off for power reduction");
  } else
    Serial.println("[] halting: internal JSON configuration corrupt");
}


void loop() {
  if ( interrupt_count > 0 ) {
    node.establishCommunications();

    while ( interrupt_count > 0 ) {
      // TODO : the datacount could change during an interrupt, making this go real bad
      node.publishMQTTMeasurement(topic, sensor, calibrated_tip, interrupt_time[interrupt_count] * .001); // offset is in s, hence millis/1000 TODO: check interrupt_time[interrupt_count]
      interrupt_count--;
    }
  }
  else {
    // publish a 0 tip measurement when there is nothing to report
    node.establishCommunications();
    node.publishMQTTMeasurement(topic, sensor, 0, 0);
    Serial.println("[info]: data published");
  }

  // restart the chip after config reset interval
  if ( millis() - start_millis > reset_interval ) esp_restart();

  Serial.println("[info]: sleeping"); Serial.println(millis() - start_millis); Serial.println(reset_interval); delay(2000);
  // TODO : esp_sleep_enable_timer_wakeup(5 * 60L * 1000000L);
  esp_light_sleep_start();
}
