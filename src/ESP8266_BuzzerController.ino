///// INCLUDES /////
#define AC_USE_LITTLEFS
#define FORMAT_ON_FAIL  true        // Autoconnect
#define AUTOCONNECT_STARTUPTIME 10

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoOSCWiFi.h>
#include <NeoPixelBus.h>
#include <ArtnetWiFi.h>
#include <LittleFS.h>
#include <time.h>
#include <AutoConnect.h>

///// CONFIG / CONSTANTS /////
const char* wifi_ssid = "MySSID";
const char* wifi_password =  "MyPW";

IPAddress   my_ip(172, 17, 206, 31);
IPAddress   my_gw(0, 0, 0, 0);
IPAddress   my_netmask(255, 255, 0, 0);

// Will be overridden by the values from the AutoConnect config page later
// See data/params.json
char* osc_command_trigger = "/buzzer/placeholder/trigger";
char* osc_command_ping    = "/buzzer/placeholder/ping";
char* osc_host            = "255.255.255.255";
int   osc_port            = 6206;
int   dmx_universe       = 0;
int   dmx_startchan      = 0;

#define ONBOARD_LED       2

// Locally attached LED pixel string (P1)
// !!! On the ESP8266, NeoPixel uses pin 3 !!!
// see https://github.com/Makuna/NeoPixelBus/wiki/ESP8266-NeoMethods
#define     LED1_DATA     3
#define     LED1_NUMLEDS  20

// Voltage divider central tap for BAT
#define     BATADC_PIN    36

// Input pin for Buzzer button
#define     BUZZER1_PIN   5

///// Globals /////
ArtnetWiFiReceiver artnet;
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> pixels(LED1_NUMLEDS, LED1_DATA);
ESP8266WebServer Server;
AutoConnect      Portal(Server);
AutoConnectConfig Config;
volatile unsigned long debounceTime = 500; // ms
volatile unsigned long lastDetection = 0;
volatile unsigned long triggered = false;
volatile unsigned long handled = 0;
volatile unsigned int  pingCount = 0;
volatile unsigned int  dmx_in_okay = 0;

///// FUNCTIONS /////
void IRAM_ATTR handleInterrupt() {
  if ((millis() - lastDetection) > debounceTime) {
    lastDetection = millis();
    triggered = true;
  }
}

// Redirect "/" to "/_ac"
void rootPage() {
  Server.sendHeader("Location", String("/_ac"), true);
  Server.send ( 302, "text/plain", "");
}

// Handle parameter changes and save them persistently
String onSettingsSavePage(AutoConnectAux& aux, PageArgument& args) {
  LittleFS.begin();
  File param = LittleFS.open("/params.json", "w");

  // The page passed via "aux" parameter is /params_save
  // We need the parameters of the /params page
  AutoConnectAux* paramsPage;
  paramsPage = Portal.aux("/params");

  // Save all elements and values
  paramsPage->saveElement(param);

  param.close();
  LittleFS.end();

  return String();
}

void artNetCallback(const unsigned char* data, short unsigned int size) {
  // set led
  // artnet data size per packet is 512 max
  // so there is max 170 pixel per packet (per universe)

  dmx_in_okay = 1;
  pixels.ClearTo(RgbColor(0, 0, 0));
  for(int i = 0; i < LED1_NUMLEDS; i++) {
    int chan = dmx_startchan + i*3;
    pixels.SetPixelColor(i, RgbColor(data[chan], data[chan+1], data[chan+2]));
  }
  pixels.Show();
}

void setup() {
  Serial.begin(921600);
  Serial.println("MT3000 getting ready :)");

  // Initialize the LED string in RED
  pixels.Begin();
  pixels.ClearTo(RgbColor(0, 0, 0));
  for(int i=0; i < LED1_NUMLEDS; i++) {
    pixels.SetPixelColor(i, RgbColor(255, 0, 0));
  }
  pixels.Show();

  if (!LittleFS.begin()){
      Serial.println("An Error has occurred while mounting LittleFS");
      return;
  }

  Server.on("/", rootPage);
  Config.autoReconnect = true;
  Config.apid = "WiFiBuzzer";
  Config.psk  = "";
  Portal.config(Config);
 
  // Second one contains the success message
  const char params_save_page[] = R"raw(
    {
      "title": "Buzzer Params",
      "uri": "/params_save",
      "menu": false,
      "element": [
        {
          "name": "caption2",
          "type": "ACText",
          "value": "Parameters saved! Please reset the board to apply!"
        }
      ]
    }
]
)raw";

  // Load the form elements and values from the file system
  AutoConnectAux* settingsPage;
  File paramsFile = LittleFS.open("/params.json", "r");
  Portal.load(paramsFile);
  paramsFile.close();
  LittleFS.end();
  settingsPage = Portal.aux("/params");

  // Also save the parameter values in the variables the program works with
  AutoConnectSelect& params_name = settingsPage->getElement<AutoConnectSelect>("param_name");
  sprintf(osc_command_trigger, "/buzzer/%s/trigger", params_name.value().c_str());
  sprintf(osc_command_ping, "/buzzer/%s/ping", params_name.value().c_str());
  AutoConnectInput& params_osc_dest_host = settingsPage->getElement<AutoConnectInput>("param_osc_dest_host");
  sprintf(osc_host, "%s", params_osc_dest_host.value.c_str());
  AutoConnectInput& params_osc_dest_port = settingsPage->getElement<AutoConnectInput>("param_osc_dest_port");
  osc_port = params_osc_dest_port.value.toInt();
  AutoConnectInput& param_dmx_universe = settingsPage->getElement<AutoConnectInput>("param_dmx_universe");
  dmx_universe = param_dmx_universe.value.toInt();
  AutoConnectInput& param_dmx_start = settingsPage->getElement<AutoConnectInput>("param_dmx_start");
  dmx_startchan = param_dmx_start.value.toInt();

  Portal.load(params_save_page);
  AutoConnectAux* settingsSavePage;
  settingsSavePage = Portal.aux("/params_save");
  settingsSavePage->on(onSettingsSavePage);

  Portal.begin();

  // Set the LEDs to GREEN briefly
  pixels.ClearTo(RgbColor(0, 0, 0));
  for(int i = 0; i < LED1_NUMLEDS; i++) {
    pixels.SetPixelColor(i, RgbColor(0, 255, 0));
  }
  pixels.Show();
  delay(500);

  // Set the LEDs to the configured color of the buzzer
  pixels.ClearTo(RgbColor(0, 0, 0));
  RgbColor col = RgbColor(0, 0, 0);
  if (params_name.value() == "red") {
    col = RgbColor(255, 0, 0);
  } else if (params_name.value() == "blue") {
    col = RgbColor(0, 0, 255);
  } else if (params_name.value() == "green") {
    col = RgbColor(0, 255, 0);
  } else if (params_name.value() == "yellow") {
    col = RgbColor(255, 255, 0);
  } else if (params_name.value() == "magenta") {
    col = RgbColor(255, 0, 255);
  } else if (params_name.value() == "cyan") {
    col = RgbColor(0, 255, 255);
  }
  for(int i = 0; i < LED1_NUMLEDS; i++) {
    pixels.SetPixelColor(i, col);
  }
  pixels.Show();
  delay(500);

  // Init Buzzer input
  pinMode(BUZZER1_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUZZER1_PIN), handleInterrupt, FALLING);

  // Set up DMX receiver
  artnet.begin();
  // artnet.subscribe_net(0);     // optionally you can change
  // artnet.subscribe_subnet(0);  // optionally you can change

  // if Artnet packet comes to this universe, this function (lambda) is called
  artnet.subscribe(dmx_universe, artNetCallback);

  // Clear LEDs
  pixels.ClearTo(RgbColor(0, 0, 0));
  pixels.Show();
}

// Looooooooop
void loop() {

  // Send a "0"-OSC-message so the "1"-message is actually change
  if (handled && ((millis() - handled) > 100)) {
    handled = 0;
    OscWiFi.send(osc_host, osc_port, osc_command_trigger, 0);
  }

  // Buzzer input
  //Serial.printf("NOW: %lu, lastDetection@: %lu, BUZZER1_PIN: %d\n", millis(), lastDetection, digitalRead(BUZZER1_PIN));
  if (triggered) {
    triggered = false;
    handled = millis();
    //Serial.println("Buzzer input detected");
    OscWiFi.send(osc_host, osc_port, osc_command_trigger, 255);
  }

  artnet.parse();  // check if artnet packet has come and execute callback

  Portal.handleClient();

  pingCount++;
  if (pingCount > 1000) {
    pingCount = 0;
    unsigned long adcVal = analogRead(BATADC_PIN);
    // 0 = 0V, 4096 = 3.3V
    // Voltage divider with 220k (high side) and 100k (low side)
    // => 4096 = 10.56V
    float batVolt = adcVal * 10.56 / 4096;
    //Serial.printf("BAT ADC: %u BAT VOLT: %f E1.31_Okay: %u\n", analogRead(PIN_BATADC), batVolt, dmx_in_okay);
    OscWiFi.send(osc_host, osc_port, osc_command_ping, (float)batVolt, (unsigned int)dmx_in_okay);
    dmx_in_okay = 0;
  }

  //memset(serialBuffer, 0, 520);
  //Serial.println("Sending frame ...");

  delay(1);
}
