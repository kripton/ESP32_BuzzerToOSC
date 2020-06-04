///// INCLUDES /////
#include <WiFi.h>
#include <ArduinoOSC.h>
#include <Adafruit_NeoPixel.h>
#include <ESPAsyncE131.h>

///// CONFIG / CONSTANTS /////
const char* wifi_ssid = "MySSID";
const char* wifi_password =  "MyPW";

IPAddress   my_ip(192, 168, 2, 252);
IPAddress   my_gw(0, 0, 0, 0);
IPAddress   my_netmask(255, 255, 255, 0);

const char* osc_command =  "/buzzer/rot";
const char* osc_host = "192.168.2.248";
const int   osc_port = 7703;

#define     PIN_BUZZER   16
#define     PIN_LEDSTRIP 21
#define     NUMLEDS      20
#define     FIRSTDMXCHAN  1

//////////////////Define your SSID and Password - if on AP mode this will be used as settings for AP if in Wifi mode this will be used to connect to an existing WiFi network//////////////
const char* ssid = "MySSID";
const char* password =  "MyPW";


// Digital input
const byte interruptPin = 16;

unsigned long lastDetection = 0;
unsigned long debounceTime = 500; // ms

///// Globals /////
OscWiFi osc;
Adafruit_NeoPixel pixels(NUMLEDS, PIN_LEDSTRIP, NEO_GRB + NEO_KHZ800);
unsigned long lastDetection = 0;
SemaphoreHandle_t syncSemaphore;

///// FUNCTIONS /////
void IRAM_ATTR handleInterrupt() {
    xSemaphoreGiveFromISR(syncSemaphore, NULL);
}

void setup() {
  // Initialize the LED string in RED
  pixels.begin();
  pixels.clear();
  for(int i=0; i < NUMLEDS; i++) {
    pixels.setPixelColor(i, pixels.Color((i * 10) + 10, 0, 0));
  }
  pixels.show();

  // Initialize the serial console for status messages
  Serial.begin(115200);
  Serial.println("BUZZER initializing");

  // Wifi config + connect
  WiFi.config(my_ip, my_gw, my_netmask);
  WiFi.begin(wifi_ssid, wifi_password);
  Serial.println("Connecting to WiFi ...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  Serial.println("Connected to the WiFi network");

  // Set the LEDs to GREEN briefly
  pixels.clear();
  for(int i = 0; i < NUMLEDS; i++) {
    pixels.setPixelColor(i, pixels.Color(0, i, 0));
  }
  pixels.show();
  delay(500);

  // Init OSC
  osc.begin(osc_port);

  // Init Buzzer input
  syncSemaphore = xSemaphoreCreateBinary();
  pinMode(PIN_BUZZER, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BUZZER), handleInterrupt, FALLING);

  // TODO: Set up DMX receiver

  // Clear LEDs
  pixels.clear();
  pixels.show();
}

// Looooooooop
void loop() {
  xSemaphoreTake(syncSemaphore, portMAX_DELAY);
  if (((millis() - lastDetection) > debounceTime) && !gpio_get_level((gpio_num_t)PIN_BUZZER)) {
    Serial.println("Buzzer input detected");
    osc.send(osc_host, osc_port, "/buzzer/rot", 1);
    lastDetection = millis();
  }
}
