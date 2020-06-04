#include <WiFi.h>
#include <ArduinoOSC.h>

OscWiFi osc;

const int recv_port = 7703;
const int send_port = 7703;
const char* host = "192.168.2.248";


////////////////Set OSC Comands here//////////////////
const char* comand =  "/buzzer/rot";


//////////////////Define your SSID and Password - if on AP mode this will be used as settings for AP if in Wifi mode this will be used to connect to an existing WiFi network//////////////
const char* ssid = "Netgear";
const char* password =  "0x01234%&/";


// Digital input
const byte interruptPin = 16;

unsigned long lastDetection = 0;
unsigned long debounceTime = 500; // ms

SemaphoreHandle_t syncSemaphore;

void IRAM_ATTR handleInterrupt() {
    xSemaphoreGiveFromISR(syncSemaphore, NULL);
}

void setup() {

  Serial.begin(115200);
  Serial.println("Monitoring interrupts: ");

  /////Wifi connect
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");

  /////osc init
  osc.begin(recv_port);

  syncSemaphore = xSemaphoreCreateBinary();

  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, FALLING);

}

void loop() {

  xSemaphoreTake(syncSemaphore, portMAX_DELAY);

  if ((millis() - lastDetection > debounceTime) && !gpio_get_level((gpio_num_t)interruptPin)) {

    Serial.println("Shake detected");
    osc.send(host, send_port, "/buzzer/rot", 1);
    lastDetection = millis();
  }
}
