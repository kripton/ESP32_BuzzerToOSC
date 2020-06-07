///// INCLUDES /////
#include <ESP8266WiFi.h>
#include <ArduinoOSCWiFi.h>
#include <NeoPixelBus.h>
#include <ESPAsyncE131.h>
#include <LittleFS.h>

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

// Locally attached LED pixel string (P1)
#define     LED1_DATA     22
#define     LED1_NUMLEDS  20

// Voltage divider central tap for BAT
#define     BATADC_PIN    36

// Input pin for Buzzer button
#define     BUZZER1_PIN   16

// E131 input via WiFi
#define     E131_UNIVERSE  17
#define     E131_STARTCHAN  1

//////////////////Define your SSID and Password - if on AP mode this will be used as settings for AP if in Wifi mode this will be used to connect to an existing WiFi network//////////////
const char* ssid = "MySSID";
const char* password =  "MyPW";


// Digital input
const byte interruptPin = 16;

unsigned long lastDetection = 0;

///// Globals /////
e131_packet_t e131_packet;
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> pixels(LED1_NUMLEDS, LED1_DATA);
ESPAsyncE131 e131(1);
volatile unsigned long debounceTime = 500; // ms
volatile unsigned long lastDetection = 0;
volatile unsigned int  pingCount = 0;
volatile unsigned int  e131_okay = 0;

///// FUNCTIONS /////
void IRAM_ATTR handleInterrupt() {
  lastDetection = millis();
}

void setup() {
  delay(2000);
  Serial.begin(921600);
  Serial.println("MT3000 getting ready :)");

  // Initialize the LED string in RED
  pixels.Begin();
  pixels.ClearTo(RgbColor(0, 0, 0));
  for(int i=0; i < LED1_NUMLEDS; i++) {
    pixels.SetPixelColor(i, RgbColor((i * 10) + 10, 0, 0));
  }
  pixels.Show();

  Serial.println("PostLED");

  if (!LittleFS.begin()){
      Serial.println("An Error has occurred while mounting SPIFFS");
      return;
  }

  // Wifi config + connect
  WiFi.mode(WIFI_STA);
  WiFi.config(my_ip, my_gw, my_netmask);
  WiFi.begin(wifi_ssid, wifi_password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  // Set the LEDs to GREEN briefly
  pixels.ClearTo(RgbColor(0, 0, 0));
  for(int i = 0; i < LED1_NUMLEDS; i++) {
    pixels.SetPixelColor(i, RgbColor(0, i, 0));
  }
  pixels.Show();
  delay(500);

  // Init Buzzer input
  pinMode(BUZZER1_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUZZER1_PIN), handleInterrupt, FALLING);

  // Set up DMX receiver
  e131.begin(E131_MULTICAST, E131_UNIVERSE); 

  // Clear LEDs
  pixels.ClearTo(RgbColor(0, 0, 0));
  pixels.Show();
}

// Looooooooop
void loop() {
  // DMX input to LEDs
  Serial.printf("E131 empty: %d\n", e131.isEmpty());
  if (!e131.isEmpty()) {
    e131.pull(&e131_packet);     // Pull packet from ring buffer

    //memcpy(serialBuffer, e131_packet.property_values+1, 512);


    Serial.printf("Universe %u / %u Channels | Packet#: %u / Errors: %u / CH1: %u\n",
      htons(e131_packet.universe),                 // The Universe for this packet
      htons(e131_packet.property_value_count) - 1, // Start code is ignored, we're interested in dimmer data
      e131.stats.num_packets,                 // Packet counter
      e131.stats.packet_errors,               // Packet error counter
      e131_packet.property_values[1]             // Dimmer data for Channel 1
    );

    e131_okay = 1;
    pixels.ClearTo(RgbColor(0, 0, 0));
    for(int i = 0; i < LED1_NUMLEDS; i++) {
      int chan = E131_STARTCHAN + i*3;
      pixels.SetPixelColor(i, RgbColor(e131_packet.property_values[chan], e131_packet.property_values[chan+1], e131_packet.property_values[chan+2]));
    }
    pixels.Show();
  }

  // Buzzer input
  if (((millis() - lastDetection) > debounceTime) && !digitalRead(BUZZER1_PIN)) {
    //Serial.println("Buzzer input detected");
    OscWiFi.send(osc_host, osc_port, osc_command_trigger, 1);
    lastDetection = millis();
  }

  pingCount++;
  if (pingCount > 100) {
    pingCount = 0;
    unsigned long adcVal = analogRead(BATADC_PIN);
    // 0 = 0V, 4096 = 3.3V
    // Voltage divider with 220k (high side) and 100k (low side)
    // => 4096 = 10.56V
    float batVolt = adcVal * 10.56 / 4096;
    //Serial.printf("BAT ADC: %u BAT VOLT: %f E1.31_Okay: %u\n", analogRead(PIN_BATADC), batVolt, e131_okay);
    OscWiFi.send(osc_host, osc_port, osc_command_ping, (float)batVolt, (unsigned int)e131_okay);
    e131_okay = 0;
  }

  //memset(serialBuffer, 0, 520);
  Serial.println("Sending frame ...");

  delay(5);
}
<<<<<<< HEAD:src/ESP32_BuzzerToOSC.ino

void writeDmx() {
  uint8_t zero = 0;

  // activate the driver and give it some time to stabilize
  digitalWrite(DMX1_DIRPIN, HIGH);
  delayMicroseconds(500);

  // Send BREAK
  uart_set_line_inverse(DMX1_UART, UART_INVERSE_DISABLE);
  delayMicroseconds(200);

  // Send MARK_AFTER_BREAK
  uart_set_line_inverse(DMX1_UART, UART_INVERSE_TXD);
  delayMicroseconds(20);

  // Send one universe of data
  uart_write_bytes(DMX1_UART, (const char*)&zero, 1); // start byte
  uart_write_bytes(DMX1_UART, (const char*)serialBuffer, 512);

  // Wait until the TX buffer has been sent completely
  delay(3);
  // and turn off the driver again
  digitalWrite(DMX1_DIRPIN, LOW);
}
=======
>>>>>>> a744e1b (Move from ESP32 (heltec WiFi LoRa) to ESP8266 (Wemos D1 Mini) and drop DMX output for now):src/ESP8266_BuzzerController.ino
