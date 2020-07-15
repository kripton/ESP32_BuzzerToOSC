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

// Local DMX outputs (X1, X2)
#define     DMX1_UART      UART_NUM_1
#define     DMX1_UARTPIN   2
#define     DMX1_DIRPIN   17
#define     DMX2_UART      UART_NUM_2
#define     DMX2_UARTPIN  12
#define     DMX2_DIRPIN   13

// Locally attached LED pixel string (P1)
#define     LED1_DATA     22
#define     LED1_CLK      23
#define     LED1_NUMLEDS  20

// Voltage divider central tap for BAT
#define     BATADC_PIN    36

// Input pin for Buzzer button
#define     BUZZER1_PIN   16

// TEMP: E131 input
#define     E131_UNIVERSE  17
#define     E131_STARTCHAN  1


// Low-Level serial driver
#include "driver/uart.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#define BUF_SIZE (1024)

unsigned long debounceTime = 500; // ms
unsigned int  pingCount = 0;
unsigned int  e131_okay = 0;

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

// Serial handler, buffer and configs
static QueueHandle_t uart1_queue;
static uint8_t serialBuffer[520];
uart_config_t uart_config_data = {
  .baud_rate   =   250000,
  .data_bits   =   UART_DATA_8_BITS,
  .parity      =   UART_PARITY_DISABLE,
  .stop_bits   =   UART_STOP_BITS_2,
  .flow_ctrl   =   UART_HW_FLOWCTRL_DISABLE
};

///// FUNCTIONS /////
void IRAM_ATTR handleInterrupt() {
  lastDetection = millis();
}

void setup() {
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

  // Set up our serial port
  //pinMode(13, OUTPUT);
  uart_set_pin(DMX1_UART, DMX1_UARTPIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  //uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 10, &uart1_queue, 0);
  uart_driver_install(DMX1_UART, BUF_SIZE * 2, 0, 10, &uart1_queue, 0); // No TX buffer => write calls will block
  uart_param_config(DMX1_UART, &uart_config_data);

  // Set up trigger output
  pinMode(DMX1_DIRPIN, OUTPUT);

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
  syncSemaphore = xSemaphoreCreateBinary();
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

    memcpy(serialBuffer, e131_packet.property_values+1, 512);

/*
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
  if (((millis() - lastDetection) > debounceTime) && !gpio_get_level((gpio_num_t)BUZZER1_PIN)) {
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
    OscWiFi.send(osc_host, osc_port, osc_command_ping, batVolt, e131_okay);
    e131_okay = 0;
  }

  //memset(serialBuffer, 0, 520);
  Serial.println("Sending frame ...");
  writeDmx();

  delay(5);
}

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
