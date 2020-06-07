///// INCLUDES /////
#include <WiFi.h>
#include <ArduinoOSC.h>
#include <NeoPixelBus.h>
#include <ESPAsyncE131.h>

///// CONFIG / CONSTANTS /////
const char* wifi_ssid = "MySSID";
const char* wifi_password =  "MyPW";

IPAddress   my_ip(172, 17, 206, 31);
IPAddress   my_gw(0, 0, 0, 0);
IPAddress   my_netmask(255, 255, 0, 0);

const char* osc_command_trigger =  "/buzzer/red/trigger";
const char* osc_command_ping =  "/buzzer/red/ping";
const char* osc_host = "255.255.255.255";
const int   osc_port = 6206;

#define     PIN_BUZZER   16
#define     PIN_BATADC   36
#define     PIN_LEDSTRIP 21
#define     NUMLEDS      20

#define     E131_UNIVERSE  17
#define     E131_STARTCHAN  1

//////////////////Define your SSID and Password - if on AP mode this will be used as settings for AP if in Wifi mode this will be used to connect to an existing WiFi network//////////////
const char* ssid = "MySSID";
const char* password =  "MyPW";


// Digital input
const byte interruptPin = 16;

unsigned long lastDetection = 0;
unsigned long debounceTime = 500; // ms
unsigned int  pingCount = 0;
unsigned int  e131_okay = 0;


///// Globals /////
e131_packet_t e131_packet;
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> pixels(NUMLEDS, PIN_LEDSTRIP);
ESPAsyncE131 e131(1);
unsigned long lastDetection = 0;
SemaphoreHandle_t syncSemaphore;

///// FUNCTIONS /////
void IRAM_ATTR handleInterrupt() {
  xSemaphoreGiveFromISR(syncSemaphore, NULL);
}

void setup() {
  // Initialize the LED string in RED
  pixels.Begin();
  pixels.ClearTo(RgbColor(0, 0, 0));
  for(int i=0; i < NUMLEDS; i++) {
    pixels.SetPixelColor(i, RgbColor((i * 10) + 10, 0, 0));
  }
  pixels.Show();

  // Initialize the serial console for status messages
  Serial.begin(115200);
  Serial.println("BUZZER initializing");

  // Wifi config + connect
  WiFi.mode(WIFI_STA);
  WiFi.config(my_ip, my_gw, my_netmask);
  WiFi.begin(wifi_ssid, wifi_password);
  Serial.println("Connecting to WiFi ...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  Serial.println("Connected to the WiFi network");

  // Set the LEDs to GREEN briefly
  pixels.ClearTo(RgbColor(0, 0, 0));
  for(int i = 0; i < NUMLEDS; i++) {
    pixels.SetPixelColor(i, RgbColor(0, i, 0));
  }
  pixels.Show();
  delay(500);

  // Init Buzzer input
  syncSemaphore = xSemaphoreCreateBinary();
  pinMode(PIN_BUZZER, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BUZZER), handleInterrupt, FALLING);

  // Set up DMX receiver
  e131.begin(E131_MULTICAST, E131_UNIVERSE); 

  // Clear LEDs
  pixels.ClearTo(RgbColor(0, 0, 0));
  pixels.Show();
}

// Looooooooop
void loop() {
  // DMX input to LEDs
  //Serial.printf("E131 empty: %d\n", e131.isEmpty());
  if (!e131.isEmpty()) {
    e131.pull(&e131_packet);     // Pull packet from ring buffer

/*
    Serial.printf("Universe %u / %u Channels | Packet#: %u / Errors: %u / CH1: %u\n",
      htons(e131_packet.universe),                 // The Universe for this packet
      htons(e131_packet.property_value_count) - 1, // Start code is ignored, we're interested in dimmer data
      e131.stats.num_packets,                 // Packet counter
      e131.stats.packet_errors,               // Packet error counter
      e131_packet.property_values[1]             // Dimmer data for Channel 1
    );
*/
    e131_okay = 1;
    pixels.ClearTo(RgbColor(0, 0, 0));
    for(int i = 0; i < NUMLEDS; i++) {
      int chan = E131_STARTCHAN + i*3;
      pixels.SetPixelColor(i, RgbColor(e131_packet.property_values[chan], e131_packet.property_values[chan+1], e131_packet.property_values[chan+2]));
    }
    pixels.Show();
  }

  //xSemaphoreTake(syncSemaphore, portMAX_DELAY);
  // Buzzer input
  if (((millis() - lastDetection) > debounceTime) && !gpio_get_level((gpio_num_t)PIN_BUZZER)) {
    Serial.println("Buzzer input detected");
    OscWiFi.send(osc_host, osc_port, osc_command_trigger, 1);
    lastDetection = millis();
  }

  pingCount++;
  if (pingCount > 100) {
    pingCount = 0;
    unsigned long adcVal = analogRead(PIN_BATADC);
    // 0 = 0V, 4096 = 3.3V
    // Voltage divider with 220k (high side) and 100k (low side)
    // => 4096 = 10.56V
    float batVolt = adcVal * 10.56 / 4096;
    Serial.printf("BAT ADC: %u BAT VOLT: %f E1.31_Okay: %u\n", analogRead(PIN_BATADC), batVolt, e131_okay);
    OscWiFi.send(osc_host, osc_port, osc_command_ping, WiFi.localIP().toString(), batVolt, e131_okay);
    e131_okay = 0;
  }

  delay(20);
}
