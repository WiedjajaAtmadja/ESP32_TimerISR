/*
Minimal Stack Size: configMINIMAL_STACK_SIZE
*/
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <esp32-hal-cpu.h>

#define WIFI_SSID "Steff-IoT"
#define WIFI_PASSWD "steffiot123"
#define MQTT_BROKER  "broker.emqx.io"

#define MQTT_TOPIC_SUBSCRIBE     "dev/cmd/#"
#define MQTT_TOPIC_CMD_DOSING_PUMP "dev/cmd/dosingPump" 

#define DOSING_PUMP_COUNT 8
const uint8_t  arDosingPumpPins[DOSING_PUMP_COUNT] = {18, 19, 21, 22, 23, 25, 26, 27};
const u_int8_t arDosingPumpSpeeds[DOSING_PUMP_COUNT] = {1, 2, 3, 4, 0, 0, 0, 0};
volatile u_int8_t arDosingPumpCount[DOSING_PUMP_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};
hw_timer_t *hwTimer0 = NULL;

WiFiClient espClient;
PubSubClient mqtt(espClient);
volatile int g_nTimerCounter = 1;
#define MAX_TIMER_COUNTER 1000
boolean mqttConnect();
void connectToWiFi();

void IRAM_ATTR onTimerISR() {
  for (int i = 0; i < DOSING_PUMP_COUNT; i++) {
    if (arDosingPumpSpeeds[i]) {
      if (++arDosingPumpCount[i]>=arDosingPumpSpeeds[i]) {
        arDosingPumpCount[i] = 0;
        digitalWrite(arDosingPumpPins[i], !digitalRead(arDosingPumpPins[i]));
      }
    }
  }

  // for (int i = 0; i < DOSING_PUMP_COUNT; i++) 
  //   if (arDosingPumpSpeeds[i] && (g_nTimerCounter % arDosingPumpSpeeds[i] == 0)) {
  //     digitalWrite(arDosingPumpPins[i], !digitalRead(arDosingPumpPins[i]));
  //   }

  // if (++g_nTimerCounter >= MAX_TIMER_COUNTER) 
  //   g_nTimerCounter = 1;
}

static void task1(void *pvParameters) {
  connectToWiFi();
  mqttConnect();

  while (1) {
    // Serial.println("Task1 running");
    // digitalWrite(arDosingPumpPins[0], !digitalRead(arDosingPumpPins[0]));
    mqtt.loop();

    // digitalWrite(LED_BUILTIN, HIGH);    
    // vTaskDelay(100 / portTICK_PERIOD_MS);
    // digitalWrite(LED_BUILTIN, LOW);    
    // vTaskDelay(900 / portTICK_PERIOD_MS);
    vTaskDelay(1);
  }
}

void mqttCommandRecv(char* topic, byte* payload, unsigned int length)
{
  char szPayload[length + 1];
  strncpy(szPayload, (char*) payload, length);
  szPayload[length] = 0;
  Serial.printf("%s: %s\n", topic, (char*) szPayload);
  if (strcmp(topic, MQTT_TOPIC_CMD_DOSING_PUMP) == 0) {
    int nValue = atoi(szPayload);
    if (nValue == 0) {
      timerAlarmDisable(hwTimer0);
    } else {
      timerAlarmEnable(hwTimer0);
    }
  }
}

boolean mqttConnect() {
  char g_szDeviceId[30];
  sprintf(g_szDeviceId, "esp32_%08X",(uint32_t)ESP.getEfuseMac());
  mqtt.setServer(MQTT_BROKER, 1883);
  mqtt.setCallback(mqttCommandRecv);
  Serial.printf("Connecting to %s clientId: %s\n", MQTT_BROKER, g_szDeviceId);

  boolean fMqttConnected = false;
  for (int i=0; i<3 && !fMqttConnected; i++) {
    Serial.print("Connecting to mqtt broker...");
    fMqttConnected = mqtt.connect(g_szDeviceId);
    if (fMqttConnected == false) {
      Serial.print(" fail, rc=");
      Serial.println(mqtt.state());
      delay(1000);
    }
  }

  if (fMqttConnected)
  {
    Serial.println(" success");
    mqtt.subscribe(MQTT_TOPIC_SUBSCRIBE);
    Serial.printf("Subcribe topic: %s\n", MQTT_TOPIC_SUBSCRIBE);
    // mqttPublish(MQTT_TOPIC_PUBLISH_DEVICE_STATUS, "1");
  }
  return mqtt.connected();
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi with IP: "+WiFi.localIP().toString());
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  for (int_fast8_t i = 0; i < DOSING_PUMP_COUNT; i++) {
    pinMode(arDosingPumpPins[i], OUTPUT);
    digitalWrite(arDosingPumpPins[i], LOW);
  }
  while(!Serial);
  // delay(500);
  Serial.println("System booting...");  
  Serial.printf("CPU Model: %s, Freq: %d MHz, Cores: %d\n", ESP.getChipModel(),
                getCpuFrequencyMhz(), ESP.getChipCores());
  Serial.printf("XTAL Freq: %d MHz\n", getXtalFrequencyMhz());
  Serial.printf("APB Freq: %d MHz\n", getApbFrequency() / 1000000);
  Serial.printf("Free Heap: %d Bytes\n", ESP.getFreeHeap());
  Serial.printf("Flash Chip Size: %d MB\n", ESP.getFlashChipSize() / (1024 * 1024));
  Serial.printf("Main Task started on Core: %d\n", xPortGetCoreID());
  // connectToWiFi();
  // mqttConnect();

  int nTargetTimerFreqInKHz = 10;
  Serial.printf("Task1 started on Core: %d\n", xPortGetCoreID());
  hwTimer0 = timerBegin(0, 80, true); // Clock source: APB_CLK=80MHz/80 = 1MHz
  timerAttachInterrupt(hwTimer0, &onTimerISR, true);
  timerAlarmWrite(hwTimer0, (1000/(2*nTargetTimerFreqInKHz)), true); // every 100 KHz
  timerAlarmEnable(hwTimer0);
  xTaskCreatePinnedToCore(task1, "Task1", 4*1024, NULL, 2, NULL, 0);//ARDUINO_RUNNING_CORE);
}

void loop() {
  // if (millis() % 1000 == 0) {
  //   digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  // }
  // mqtt.loop();
  vTaskDelay(0);
}
