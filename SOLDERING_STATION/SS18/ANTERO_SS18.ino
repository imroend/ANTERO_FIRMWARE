#include "SettingMenu.h"
#include "WELCOME.h"
#include "SOLDER.h"
#include "ICON.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <SPI.h>
#include <PID_v1.h>             // https://github.com/mblythe86/C-PID-Library/tree/master/PID_v1
#include <EEPROM.h>             // for storing user settings into EEPROM
#include "OneButton.h"
#include <RotaryEncoder.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "DINProBold12.h"
#include "DINProBold15.h"
#include "DINProBold16.h"
#include "DINProBold24.h"
#define EEPROM_SIZE 512
#define EEPROM_MAGIC_ADDR 255
#define EEPROM_MAGIC_VALUE 0x42

#define GFXFF  1       // Aktifkan GFX font support
#define SMOOTH_FONT       // Jika ingin smooth font (opsional)
#define DIN12 DINProBold12
#define DIN16 DINProBold16
#define DIN15 DINProBold15
#define DIN24 DINProBold24


#define SCREEN_W 160
#define SCREEN_H 128


// --- Penyesuaian posisi dan ukuran grafik berdasarkan lebar layar ---
#define GRAPH_X 22                          // Posisi X grafik dinamis
#define GRAPH_Y 6                          // Posisi Y grafik dinamis
#define GRAPH_MARGIN_RIGHT 15                       // Margin kanan untuk label
#define GRAPH_DRAW_W ((SCREEN_W - GRAPH_X) - GRAPH_MARGIN_RIGHT)      // Lebar area gambar (75% dari layar)
#define GRAPH_BG_W GRAPH_DRAW_W + 2
#define GRAPH_H (SCREEN_H - 40)                           // Tinggi grafik = 1/2 layar

//#define TFT_SDA  23
//#define TFT_SCLK 18
//#define TFT_CS    15  // Chip select control pin
//#define TFT_DC    4  // Data Command control pin
//#define TFT_RST   2
#define LED_PIN  33         // GPIO yang kamu pakai
#define LED_CHANNEL 1       // Gunakan channel 0
#define PWM_FREQ 500       // Frekuensi PWM 5 kHz
#define PWM_RES 8           // Resolusi 8-bit (0-255)

#define HEATER_CHANNEL   2
#define HEATER_PWM_FREQ  1000
#define HEATER_PWM_RES   12    // 12-bit: 0–4095




// Type of MOSFET
#define P_MOSFET                // P_MOSFET or N_MOSFET
#define FF18 &FreeSansBold9pt7b
#define GFXFF 1

#define BUFF_SIZE 64
#define RED2RED 0
#define GREEN2GREEN 1
#define BLUE2BLUE 2
#define BLUE2RED 3
#define GREEN2RED 4
#define RED2GREEN 5
#define RAINBOW 6
// Warna tampilan curve
#define GRID_COLOR TFT_DARKGREY                      // Warna garis grid
#define LINE_COLOR TFT_YELLOW                      // Warna garis suhu
#define SETPOINT_COLOR TFT_RED                     // Warna setpoint suhu
#define DOT_COLOR TFT_YELLOW                        // Warna titik suhu real-time
#define PWM_COLOR TFT_GREEN                        // Warna grafik PWM
#define FRAME_COLOR TFT_DARKGREY                     // Warna frame di 7 Segment
#define CH_COLOR TFT_YELLOW                           // Warna nilai Ch aktif
// Pins
#define SENSOR_PIN              34       // tip temperature sense
#define VIN_PIN                 36        // input voltage sense
#define VCC_PIN                 35        //
#define BUZZER_PIN              13// buzzer
#define BUTTON_PIN              0     //RESET WIFI
#define BUTTON_UP               21
#define BUTTON_DOWN             32
#define encoderButton           27        // rotary encoder switch
#define encoderPinA             14        // rotary encoder 1
#define encoderPinB             12        // rotary encoder 2
#define CONTROL_PIN             26        // heater MOSFET PWM control
#define SWITCH_PIN              5       // handle switch controll
#define SWITCH_CHANNEL_PANEL    13     // FOR CHANGE CHANNEL switch ANALOG SERI
//#define SWITCH_BOOST            19      // boost switch external
#define SWITCH_CHANNEL_TOUCH    22    //  FOR CHANGE CHANNEL switch TOUCH
#define SWITCH_HANDLE  25    //  FOR CHANGE CHANNEL switch TOUCH

// Rentang suhu dan PWM CURVE
#define MIN_TEMP 0                                 // Suhu minimum
#define MIN_PWM 0                                 // pwm minimum
#define MAX_PWM 100                                // Nilai PWM maksimum
// Default temperature control values (recommended soldering temperature: 300-380°C)
#define TEMP_MIN      150      // min selectable temperature
#define TEMP_MAX      450      // max selectable temperature
#define TEMP_SETPOINT 200      // default start setpoint

#define TEMP_SLEEP    100       // temperature in sleep mode
#define TEMP_BOOST    50       // temperature increase in boost mode
#define TEMP_STEP     5       // rotary encoder temp change steps
#define SETCHANNEL    0      // set Channel   0 = channael 1 ,1 = channel 2 , 2 = cannel 3 
#define MAX_CHANNEL   4
#define TEMP_CH_DEFAULT 200
// Default tip temperature calibration values
#define HANDLE_TYPE   1      // Handle tip name   0 = T115 , 1 = T210 , 2 = T245
#define BRIGHTNESS    150
#define TIPMAX         3
#define TEMP200       200       // temperature at ADC = 200
#define TEMP280       280       // temperature at ADC = 280
#define TEMP360       360       // temperature at ADC = 360
#define MAX_VALUE_DIRRECT 200
// value calibration
// Default timer values (0 = disabled)
#define TIME2SLEEP    0        // time to enter sleep mode in minutes
#define TIMEOFBOOST   10
// Control values
#define TIME2SETTLE   100   // time in microseconds to allow OpAmp output to settle
#define SMOOTHIE      0.05      // OpAmp output smooth factor (1=no smoothing; 0.05 default)
#define PID_ENABLE    true     // enable PID control
#define BEEP_ENABLE   true      // enable/disable buzzer 
#define LOCK_ENABLE   false     // status locked key
#define MAINSCREEN    0        // type of main screen (0: NORMAL; 1: 7SEGMENT 2: CURVE)
#define ECREVERSE     false     // enable/disable rotary encoder reverse
// MOSFET control definitions
#define USE_P_CHANNEL false  // true jika pakai P-Channel, false untuk N-Channel
#define HEATER_OFF  MAX_PWM_VALUE
#define HEATER_PWM  Output
#define NUM_READINGS 32


char buf[100];



unsigned long lastMirrorTime = 0;
const unsigned long mirrorInterval = 200;  // Kirim setiap 200ms

int measurementPeriod = 100;
bool dilepas = false;

volatile bool short_button1;
volatile bool long_button1;
volatile bool multiClick;
volatile bool short_button2;
volatile bool long_button2;
volatile bool short_button_up;
volatile bool long_button_up;
volatile bool short_button_down;
volatile bool long_button_down;
bool otaMode = false;

// -------------
int colR, colG, colB;
int coarWd = 96;
int colBarY0 = 5;
int colBarYlRold, colGold, colBold;
int setTimermode = 0;
int colBarY = 40;
int colBarHt = 30;

int lastRotaryPos = 0;

// Default values that can be changed by the user and stored in the EEPROM
uint16_t  SetpointTemp = TEMP_SETPOINT;
uint16_t  SleepTemp    = TEMP_SLEEP;
uint16_t  TempMin      = TEMP_MIN;
uint16_t  TempMax      = TEMP_MAX;
uint8_t   BoostTemp    = TEMP_BOOST;
uint8_t   timeOfBoost  = TIMEOFBOOST;
uint8_t   time2sleep   = TIME2SLEEP;
uint8_t   MainScrType  = MAINSCREEN;
uint8_t   SetChannel   = SETCHANNEL;
uint8_t   handleType   = HANDLE_TYPE;
bool      beepEnable   = BEEP_ENABLE;
bool      lockEnable   = LOCK_ENABLE;
bool      PIDenable    = PID_ENABLE;
int       Brightness   = BRIGHTNESS;
uint16_t TempChannel[MAX_CHANNEL] = {200, 200, 200, 200};  // default awal
const char* versionUrl = "http://yourserver.com/version.txt";

// Variabel data
int LastSetTemp;
int SetTemp;

float tempData[GRAPH_DRAW_W];                     // Array data suhu
float setPointData[GRAPH_DRAW_W];                  // Array data suhu
int timeData[GRAPH_DRAW_W];                       // Waktu (ms sejak mulai naik)
int pwmData[GRAPH_DRAW_W];                        // Data PWM (0–100)
float tMax = TempMax;                             // Nilai suhu maksimum
float tMin = MIN_TEMP;                             // Nilai suhu minimum
int dataindex = 0;                                 // Indeks data saat ini
float smoothTemp = MIN_TEMP;                       // Suhu dengan smoothing
float alpha = 0.1;                                 // Faktor smoothing
int cal;
bool risingStarted = false;                        // Flag apakah suhu mulai naik
unsigned long startTime = 0;                       // Waktu awal kenaikan
int elapsedTime;                                   // Waktu sejak mulai naik
bool setpointReached = false;                      // Flag tercapai setpoint
int setpointReachedX = -1;                         // Posisi X saat tercapai
unsigned long setpointReachedTime = 0;             // Waktu saat setpoint tercapai
// Variabel untuk waktu
unsigned long handleDuration = 0;
bool handleIsUp = false;
unsigned long handleLiftStartMillis = 0;
unsigned long buzzerStart = 0;
unsigned long buzzerDuration = 0;

bool buzzerActive = false;
int pwmVal = 0;                                     // Nilai PWM sekarang
int pwmDir = 1;                                     // Arah perubahan PWM (naik/turun)
const int MAX_PWM_VALUE = (1 << HEATER_PWM_RES) - 1;  // 4095
const int pwmResolution = PWM_RES;
const int maxBrightness = (1 << pwmResolution) - 1;  // 1023 untuk 10-bit
// Fungsi pemetaan float seperti map() tapi untuk float
float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



#define MIRROR_ROWS_PER_LOOP 2
unsigned long lastMirror = 0;
int mirrorRow = 0;


int handlesw;

int colour = TFT_RED;
int rgb = TFT_BLUE;

// Variables for pin change interrupt
volatile bool     handleMoved;

// Variables for temperature control
// Define the aggressive and conservative PID tuning parameters

uint16_t ShowTemp, gap, pwm;
double Setpoint, Input, Output, CurrentTemp, RawTemp;
double Kp = 18, Ki = 0, Kd = 9;
PID myPID(&Input, &Output, &Setpoint , Kp, Ki, Kd, DIRECT);

// State variables
bool      inSleepMode = false;
bool      inOffMode   = false;
bool      inBoostMode = false;
bool      inMode      = false;
bool      isWorky     = false;
bool      wasStable   = false;
bool      TipIsPresent = true;
bool      SetupMenu   = false;
bool      kedip       = false;

String tipNames[TIPMAX] = {"T115", "T210", "T245"};
uint16_t CalTemp[TIPMAX][3] = {
  {TEMP200, TEMP280, TEMP360}
};

uint8_t CurrentTip = 0;
uint8_t NumberOfTips = TIPMAX;

// Timing variables
uint32_t  sleepmillis;
uint32_t  boostmillis;
uint32_t  buttonmillis;
uint8_t   goneSeconds;
uint8_t   SensorCounter = 255;



uint16_t denoiseAnalog(byte port) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 100; i++) {
    sum += analogRead(port);
    delayMicroseconds(100);  // opsional untuk stabilisasi ADC
  }
  return sum / 100;
}

TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h
TFT_eSprite sprite = TFT_eSprite(&tft); // Sprite object
uint16_t screenBuffer[TFT_WIDTH * TFT_HEIGHT];
RotaryEncoder encoder(encoderPinA, encoderPinB, RotaryEncoder::LatchMode::TWO03);
// Variables will change:
OneButton button1(encoderButton, true);//buton encoder
OneButton button2(SWITCH_CHANNEL_TOUCH, true);
OneButton button_up(BUTTON_UP, true);
OneButton button_down(BUTTON_DOWN, true);


Preferences preferences;
WebServer webServer(80);            // untuk mode konfigurasi


String wifi_ssid = "";
String wifi_pass = "";
bool connectToSavedWiFi() {
  preferences.begin("wifi", true);
  wifi_ssid = preferences.getString("ssid", "");
  wifi_pass = preferences.getString("pass", "");
  preferences.end();

  if (wifi_ssid == "" || wifi_pass == "") return false;

  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());

  Serial.print("Connecting to WiFi ");
  Serial.println(wifi_ssid);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) {
    delay(500);
    Serial.print(".");
  }

  return WiFi.status() == WL_CONNECTED;
}


#include <DNSServer.h>
DNSServer dnsServer;
const byte DNS_PORT = 53;
const char* customHostname = "anterotools";
const char* defaultAPSSID = "ANTERO SS18";
const char* defaultAPPassword = "12345678";


void startConfigPortal() {
  WiFi.mode(WIFI_AP);
  IPAddress apIP(192, 168, 4, 1);
  WiFi.softAP(defaultAPSSID, defaultAPPassword);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  Serial.print("AP IP address: ");
  Serial.println(apIP);
  // 🔽 Mulai DNS redirect SEMUA domain ke ESP32
  dnsServer.start(53, "*", apIP); // Mulai DNS redirect

  webServer.on("/", HTTP_GET, []() {
    String page = R"rawliteral(
  <!DOCTYPE html>
  <html lang="id">
  <head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>ANTERO WiFi Setup</title>
    <style>
      body {
        background-color: #121212;
        color: #ffffff;
        font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
        text-align: center;
        margin: 0;
        padding: 0;
      }
      .container {
        max-width: 320px;
        margin: auto;
        padding-top: 40px;
      }
      h2 {
        color: #03dac6;
        margin-bottom: 20px;
      }
      form {
        background-color: #1e1e1e;
        padding: 20px;
        border-radius: 10px;
        box-shadow: 0 0 10px #00000088;
      }
      input[type='text'],
      input[type='password'] {
        width: 100%;
        padding: 10px;
        margin: 8px 0;
        box-sizing: border-box;
        background-color: #2a2a2a;
        border: 1px solid #444;
        color: #fff;
        border-radius: 5px;
      }
      input[type='submit'] {
        width: 100%;
        padding: 10px;
        background-color: #03dac6;
        border: none;
        color: black;
        font-weight: bold;
        border-radius: 5px;
        cursor: pointer;
        margin-top: 10px;
      }
      input[type='submit']:hover {
        background-color: #018786;
      }
      .info {
        margin-top: 30px;
        font-size: 13px;
        color: #bbb;
        line-height: 1.5;
      }
      .info b {
        color: #ffffff;
      }
      a {
        color: #03dac6;
        text-decoration: none;
      }
      a:hover {
        text-decoration: underline;
      }
    </style>
  </head>
  <body>
    <div class="container">
      <h2>ANTERO WiFi SETUP</h2>
      <form action='/save' method='POST'>
        SSID:<br><input name='ssid' type='text' required><br>
        Password:<br><input name='pass' type='password'><br><br>
        <input type='submit' value='Save & Reboot'>
      </form>
      <div class="info">
        <p>Hubungkan perangkat Anda ke WiFi berikut:</p>
        <p><b>SSID:</b> ANTERO SOLDERING STATION<br>
        <b>Password:</b> 12345678</p>
        <p>Jika halaman ini tidak muncul otomatis,<br>
        silakan buka manual di:<br>
        <a href='http://192.168.4.1'><b>http://192.168.4.1</b></a></p>
      </div>
    </div>
  </body>
  </html>
  )rawliteral";

    webServer.send(200, "text/html", page);
  });

  webServer.on("/save", HTTP_POST, []() {
    if (webServer.hasArg("ssid") && webServer.hasArg("pass")) {
      preferences.begin("wifi", false);
      preferences.putString("ssid", webServer.arg("ssid"));
      preferences.putString("pass", webServer.arg("pass"));
      preferences.end();
      webServer.send(200, "text/html", "<h3>Saved! Rebooting...</h3>");
      delay(2000);
      ESP.restart();
    }
  });
  webServer.onNotFound([]() {
    webServer.sendHeader("Location", "/", true);
    webServer.send(302, "text/plain", "");
  });

  webServer.begin();
  Serial.println("Config portal started.");
}

// Fungsi tampilan pesan sukses ganti WiFi
void showWiFiChangedSuccess() {
  sprite.fillSprite(TFT_BLACK);
  sprite.setTextDatum(MC_DATUM);
  sprite.setTextColor(TFT_GREEN);
  sprite.loadFont(DIN16);
  sprite.drawString("WiFi berhasil diganti!", SCREEN_W / 2, SCREEN_H / 2);
  sprite.unloadFont();
  sprite.pushSprite(0, 0);
  delay(3000);
}

void setup(void) {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);


  // ⬇️ CEK TOMBOL RESET WIFI
  delay(1000);  // beri waktu 1 detik untuk tekan BOOT saat startup
  if (digitalRead(BUTTON_PIN) == LOW) {
    Serial.println("Tombol BOOT ditekan. Reset WiFi...");
    preferences.begin("wifi", false);
    preferences.clear();
    preferences.end();
    delay(1000);  // biar sempat lihat pesan di Serial
    ESP.restart();
  }
  preferences.begin("wifi", false);
  bool wifiChanged = preferences.getBool("wifiChanged", false);
  if (wifiChanged) {
    showWiFiChangedSuccess();
    preferences.putBool("wifiChanged", false);  // reset flag supaya notif hanya sekali
  }
  preferences.end();




  pinMode(SENSOR_PIN, INPUT);
  pinMode(SWITCH_HANDLE, INPUT);
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(VIN_PIN, INPUT);
  pinMode(VCC_PIN, INPUT);
  pinMode(encoderButton, INPUT_PULLUP);
  //pinMode(SWITCH_BOOST,   INPUT_PULLUP);
  pinMode(BUZZER_PIN,   OUTPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  heaterOff(); // this shuts off the heater
  digitalWrite(BUZZER_PIN, LOW);        // must be LOW when buzzer not in use

  ledcSetup(HEATER_CHANNEL, HEATER_PWM_FREQ, HEATER_PWM_RES);  // Setup PWM
  ledcAttachPin(CONTROL_PIN, HEATER_CHANNEL);                   // Hubungkan pin

  ledcSetup(LED_CHANNEL, PWM_FREQ, pwmResolution);  // Atur channel
  ledcAttachPin(LED_PIN, LED_CHANNEL);        // Sambungkan pin ke channel
  EEPROM.begin(EEPROM_SIZE);
  if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC_VALUE) {
    //EEPROM belum diinisialisasi, lakukan inisialisasi default
    eraseEEPROM();
    resetDefaultSettings();
    updateEEPROM();
  }
  // Setelah itu, baca dari EEPROM seperti biasa
  getEEPROM();
  ledcWrite(LED_CHANNEL, Brightness);        // tulis PWM
  analogReadResolution(10);
  analogSetPinAttenuation(SENSOR_PIN, ADC_11db );
  SetTemp  = SetpointTemp;
  button1.attachClick(click1);
  button2.attachClick(click2);
  button_up.attachClick(click3);
  button_down.attachClick(click4);
  button1.attachLongPressStart(longPress1);
  button2.attachLongPressStart(longPress2);
  button_up.attachDuringLongPress(longPress3);
  button_down.attachDuringLongPress(longPress4);
  button1.attachDoubleClick(multiClick1);
  myPID.SetOutputLimits(0, MAX_PWM_VALUE);  // 0 sampai 4095
  myPID.SetMode(AUTOMATIC);

  tft.init();
  tft.fillScreen(TFT_BLACK);
  sprite.fillSprite(TFT_BLACK);
  tft.setRotation(1);
  sprite.createSprite(SCREEN_W, SCREEN_H);
  sprite.setSwapBytes(true);
  welcomebeep();
  sprite.pushImage(0, 0, SOGUN_AFIEN_WIDTH, SOGUN_AFIEN_HEIGHT, SOGUN_AFIEN);
  //sprite.pushImage(0, 0, GARUDA_WIDTH, GARUDA_HEIGHT, garuda);

  sprite.fillRoundRect(28, 118, 105, 8, 4, TFT_DARKGREY);
  sprite.pushSprite(0, 0);
  for (int i = 0; i < 100; i += 5) {
    sprite.fillRoundRect(28, 118, 10 + i, 8, 4, TFT_GREEN);
    sprite.pushSprite(0, 0);
    delay(100);
  }
  delay(3000);
  SetpointTemp = TempChannel[SetChannel];
  setRotaryMainscreen(TempMin, TempMax, TEMP_STEP, SetpointTemp);
  // reset sleep timer
  sleepmillis = millis();
}
void loop() {
  if (buzzerActive && millis() - buzzerStart >= buzzerDuration) {
    noTone(BUZZER_PIN);
    buzzerActive = false;
  }
  if (SetupMenu) {
    handleButtonsSettings();
    handleEncoderSettings();
  }
  else {
    handleButtonsMainScreen();
    handleEncoderMainScreen();
  }
  SLEEPCheck();       // check and activate/deactivate sleep modes
  SENSORCheck();
  Thermostat();
  MainScreen();
  //plotting();

  
  if (WiFi.getMode() == WIFI_AP) {
    dnsServer.processNextRequest();   // WAJIB untuk DNS
    webServer.handleClient();
  }
  //mirrorScreenToPC();


}

void mirrorScreenToPC() {
  if (millis() - lastMirror < 5) return;  // Maks: setiap 5ms
  lastMirror = millis();

  if (mirrorRow >= TFT_HEIGHT) {
    mirrorRow = 0;
    return;
  }

  sprite.readRect(0, mirrorRow, TFT_WIDTH, 1, screenBuffer);

  for (int i = 0; i < TFT_WIDTH; i++) {
    Serial.write(highByte(screenBuffer[i]));
    Serial.write(lowByte(screenBuffer[i]));
  }

  mirrorRow++;
}
void startBeep(unsigned long durationMs) {
  if (beepEnable) {
    tone(BUZZER_PIN, 3000);
    buzzerStart = millis();
    buzzerDuration = durationMs;
    buzzerActive = true;
  }
}

void click1() {
  short_button1 = true;
  if (!lockEnable) {
    startBeep(10);
  }
}
void multiClick1() {
  multiClick = true;
  startBeep(300);
}
void longPress1() {
  long_button1 = true;
  if (!lockEnable) {
    startBeep(10);
  }
}
void click2() {
  short_button2 = true;
  if (!lockEnable) {
    startBeep(10);
  }
}

void longPress2() {
  long_button2 = true;
  if (!lockEnable) {
    startBeep(300);
  }
}
void click3() {
  short_button_up = true;
  if (SetTemp < TempMax && SetTemp > TempMin && !lockEnable) {
    startBeep(10);
  }
}

void longPress3() {
  long_button_up = true;
  if (SetTemp < TempMax && SetTemp > TempMin && !lockEnable) {
    startBeep(10);
  }
}
void click4() {
  short_button_down = true;
  if (SetTemp < TempMax && SetTemp > TempMin && !lockEnable) {
    startBeep(10);
  }
}

void longPress4() {
  long_button_down = true;
  if (SetTemp < TempMax && SetTemp > TempMin && !lockEnable) {
    startBeep(10);
  }
}

void handleButtonsMainScreen() {
  button1.tick();
  button2.tick();
  button_up.tick();
  button_down.tick();
  handlesw = analogRead(SWITCH_HANDLE);
  // === Di luar menu Setup ===
  if (!SetupMenu) {
    // === Toggle Lock dengan multiClick ===
    if (multiClick) {
      lockEnable = !lockEnable;
      multiClick = false;
    }

    if (short_button2 && !lockEnable) {
      SetChannel = (SetChannel + 1) % MAX_CHANNEL;
      SetTemp = TempChannel[SetChannel];  // langsung ambil dari array
      startBeep(10);
      short_button2 = false;
      EEPROM.write(21, SetChannel);
      EEPROM.commit();
    }

    else if (handlesw > 30 && handlesw < 500 && !lockEnable) {
      //inBoostMode = !inBoostMode;
      startBeep(10);
      delay(300);
      if (inBoostMode) boostmillis = millis();
    }

    else if (handlesw < 30 && !lockEnable) {
      //SetChannel = (SetChannel + 1) % MAX_CHANNEL;
      startBeep(10);
      delay(300);
      SetTemp = TempChannel[SetChannel];  // langsung dari array
      EEPROM.write(21, SetChannel);
      EEPROM.commit();
    }


    else if (short_button_up && !lockEnable) {
      SetTemp++;
      if (SetTemp > TempMax) SetTemp = TempMax;
      short_button_up = false;
    }

    else if (long_button_up && !lockEnable) {
      SetTemp += TEMP_STEP;
      delay(50);
      if (SetTemp > TempMax) SetTemp = TempMax;
      long_button_up = false;
    }

    else if (short_button_down && !lockEnable) {
      SetTemp--;
      if (SetTemp < TempMin) SetTemp = TempMin;
      short_button_down = false;
    }

    else if (long_button_down && !lockEnable) {
      SetTemp -= TEMP_STEP;
      delay(50);
      if (SetTemp < TempMin) SetTemp = TempMin;
      long_button_down = false;
    }

    if (long_button2 && !lockEnable) {
      TempChannel[SetChannel] = SetTemp;
      saveAllTempChannelsToEEPROM();
      long_button2 = false;
    }


    // Toggle boost dari short_button1
    if (short_button1 && !lockEnable && !multiClick) {
      inBoostMode = !inBoostMode;
      if (inBoostMode) boostmillis = millis();
      short_button1 = false;
    }

    // MASUK ke SetupMenu (long press)
    if (long_button1 && !short_button1 && !lockEnable) {
      LastSetTemp = SetTemp;
      SetupMenu = true;
      startTime = 0;
      Setpoint = 0;
      SetTemp = 0;
      analogWrite(CONTROL_PIN, 0);
      Output = 0;
      enterSettingsMenu();
      long_button1 = false;
    }
  }

  // === BOOST timeout ===
  if (inBoostMode && timeOfBoost) {
    goneSeconds = (millis() - boostmillis) / 1000;
    if (goneSeconds >= timeOfBoost) {
      inBoostMode = false;
      startBeep(300);
    }
  }
}

void handleEncoderMainScreen() {
  encoder.tick();
  int newPos = encoder.getPosition();

  if (newPos != lastRotaryPos) {
    int delta = newPos - lastRotaryPos;
    lastRotaryPos = newPos;

    if (!lockEnable) {
      // Pengaturan suhu (di luar menu)
      SetTemp += delta * 10;
      SetTemp = constrain(SetTemp, TempMin, TempMax);
      setRotaryMainscreen(TempMin, TempMax, 10, SetTemp);
    }
  }
}


void setRotaryMainscreen(int minVal, int maxVal, int step, int currentValue) {
  // Hitung nilai posisi berdasarkan step
  int position = constrain(currentValue / step, minVal / step, maxVal / step);
  encoder.setPosition(position);
  lastRotaryPos = position;
  Serial.print("Set rotary to position: ");
  Serial.println(lastRotaryPos);  // Debug
}



void SLEEPCheck() {
  static bool handleMoved = false;
  static bool lastHandleState = LOW;
  static unsigned long handleStableTime = 0;

  bool handleState = digitalRead(SWITCH_PIN);

  // Deteksi perubahan state untuk debounce
  if (handleState != lastHandleState) {
    handleStableTime = millis();
    lastHandleState = handleState;
  }

  unsigned long stableDuration = millis() - handleStableTime;

  // Jika gagang diangkat dan stabil minimal 50ms
  if (handleState == HIGH && stableDuration > 50) {

    if (!handleMoved) {
      startBeep(100);
      handleMoved = true;
      sleepmillis = millis();
      inSleepMode = false;
      inOffMode = false;
      // Catat waktu mulai saat gagang diangkat

    }
  }

  // Jika gagang ditaruh dan stabil >300ms
  if (handleState == LOW && stableDuration > 300) {
    handleMoved = false;

    goneSeconds = (millis() - sleepmillis) / 1000;

    if (CurrentTemp >= SleepTemp) {
      if (time2sleep == 0 && !inSleepMode) {
        startBeep(300);
        heaterOff();
        inSleepMode = true;
        inOffMode = false;
      }

      if (time2sleep > 0 && goneSeconds >= time2sleep && !inSleepMode) {
        startBeep(300);
        heaterOff();
        inSleepMode = true;
        inOffMode = false;
      }
    } else {
      inSleepMode = false;
      inOffMode = true;
    }
  }

  static bool lastSleepState = true;
  bool sleeping = inOffMode || inSleepMode;


  if (handleState == HIGH && !handleIsUp) {
    handleLiftStartMillis = millis();
    handleIsUp = true;
  }

  // Jika gagang baru saja ditaruh
  if (handleState == LOW && handleIsUp) {
    handleDuration = 0;
    handleLiftStartMillis = 0;
    handleIsUp = false;
  }

  // Hitung durasi jika gagang sedang diangkat
  if (handleIsUp) {
    handleDuration = millis() - handleLiftStartMillis;
  }

}



void SENSORCheck() {
  // 1. Matikan heater sejenak untuk pengukuran ADC
  heaterOff();
  delayMicroseconds(TIME2SETTLE);  // Stabilkan voltase sensor

  // 2. Baca dan haluskan nilai ADC mentah dari sensor
  double temp = denoiseAnalog(SENSOR_PIN);  // nilai ADC smoothed

  // 3. Lanjutkan kontrol heater menggunakan PWM
  if (Output > MAX_PWM_VALUE) Output = MAX_PWM_VALUE;

  if (USE_P_CHANNEL) {
    // Untuk P-Channel, PWM LOW = aktif
    ledcAttachPin(CONTROL_PIN, HEATER_CHANNEL);
    ledcWrite(HEATER_CHANNEL, MAX_PWM_VALUE - Output);
  } else {
    // Untuk N-Channel, PWM HIGH = aktif
    ledcAttachPin(CONTROL_PIN, HEATER_CHANNEL);
    ledcWrite(HEATER_CHANNEL, Output);
  }


  // 4. Filter tambahan ke RawTemp
  RawTemp += (temp - RawTemp) * SMOOTHIE;

  // 5. Hitung suhu dari kalibrasi
  calculateTemp();  // Gunakan CalTemp untuk konversi ke °C

  // 6. Hitung persentase PWM untuk indikator tampilan
  if (PIDenable) {
    pwm = (uint8_t)((Output / MAX_PWM_VALUE) * 100.0);
  } else {
    pwm = map(Output, 0, MAX_VALUE_DIRRECT, 0, 100);
  }

  // 7. Update ShowTemp agar tampilannya stabil dan tidak lompat
  if ((ShowTemp != Setpoint) || (abs(ShowTemp - CurrentTemp) > 20)) {
    ShowTemp = CurrentTemp;
  }
  if (abs(ShowTemp - Setpoint) <= 10) {
    ShowTemp = Setpoint;  // Jika mendekati target, stabilkan di target
  }

  // 8. Status suhu stabil (untuk kalibrasi atau indikator READY)
  gap = abs(Setpoint - ShowTemp);

  if (gap < 10) {
    if (!wasStable) {
      startBeep(300);
      isWorky = true;
      wasStable = true;
    }
  } else {
    isWorky = false;
    wasStable = false;
  }

  // 9. Deteksi kehadiran solder tip
  if (ShowTemp > 500 || ShowTemp < 0) {
    TipIsPresent = false;
    heaterOff();
    Output = 0;
  }
  else if (!TipIsPresent && ShowTemp < 500 && ShowTemp > 1) {
    TipIsPresent = true;
    RawTemp = denoiseAnalog(SENSOR_PIN);
  }
}

void calculateTemp() {
  if (RawTemp <= CalTemp[CurrentTip][0]) {
    // Rentang bawah (0 - TEMP200)
    CurrentTemp = map(RawTemp, 0, CalTemp[CurrentTip][0], 21, 200);
  } else if (RawTemp <= CalTemp[CurrentTip][1]) {
    // Rentang tengah (TEMP200 - TEMP280)
    CurrentTemp = map(RawTemp,
                      CalTemp[CurrentTip][0], CalTemp[CurrentTip][1],
                      200, 280);
  } else {
    // Rentang atas (TEMP280 - TEMP360)
    CurrentTemp = map(RawTemp,
                      CalTemp[CurrentTip][1], CalTemp[CurrentTip][2],
                      280, 360);
  }
}

// controls the heater
void Thermostat() {
  // Tetapkan Setpoint berdasarkan mode aktif
  if      (inOffMode)   Setpoint = 0;
  else if (inSleepMode) Setpoint = 0;
  else if (inBoostMode) Setpoint = SetTemp + BoostTemp;
  else                  Setpoint = SetTemp;

  // Hitung selisih suhu
  gap = abs(Setpoint - CurrentTemp);

  // Proteksi: jika sensor error atau tip tidak terdeteksi
  if (ShowTemp >= 550 || ShowTemp <= 0 || !TipIsPresent) {
    heaterOff();
    Output = 0;
    return;
  }

  // === Kontrol Pemanas ===
  if (PIDenable) {
    Input = CurrentTemp;
    myPID.Compute();  // Output akan diatur oleh PID
  } else {
    // Mode direct control
    if ((CurrentTemp + 0.5) < Setpoint && gap <= 10) {
      Output = MAX_PWM_VALUE * 0.25;  // Pemanasan ringan (25%)
    } else if (CurrentTemp < Setpoint && gap > 15) {
      Output = MAX_PWM_VALUE;         // Pemanasan cepat (100%)
    } else {
      Output = 0;                     // Matikan jika sudah cukup
    }
  }

  // Kirim PWM ke heater
  if (Output > 0) {
    heaterOn((uint16_t)Output);  // PWM dinamis
  } else {
    heaterOff();                 // Matikan total
  }
}


void welcomebeep() {
  tone(BUZZER_PIN, 2000); // Send 1KHz sound signal...
  delay(50);        // ...for 1 sec
  noTone(BUZZER_PIN);     // Stop sound...
  delay(100);        // ...for 1sec
  tone(BUZZER_PIN, 2500); // Send 1KHz sound signal...
  delay(50);        // ...for 1 sec
  noTone(BUZZER_PIN);     // Stop sound...
  delay(100);        // ...for 1se
  tone(BUZZER_PIN, 3000); // Send 1KHz sound signal...
  delay(50);        // ...for 1 sec
  noTone(BUZZER_PIN);     // Stop sound...
}

void getEEPROM() {
  SetpointTemp = (EEPROM.read(0) << 8)  | EEPROM.read(1);
  SleepTemp    = (EEPROM.read(2) << 8)  | EEPROM.read(3);
  TempMin      = (EEPROM.read(4) << 8)  | EEPROM.read(5);
  TempMax      = (EEPROM.read(6) << 8)  | EEPROM.read(7);

  // Baca array TempChannel[] dari EEPROM
  for (int i = 0; i < MAX_CHANNEL; i++) {
    int addr = 30 + (i * 2);  // mulai dari alamat 30, tiap channel 2 byte
    uint8_t high = EEPROM.read(addr);
    uint8_t low  = EEPROM.read(addr + 1);
    TempChannel[i] = (high << 8) | low;

    // Validasi nilai jika diperlukan
    if (TempChannel[i] < 100 || TempChannel[i] > 500) {
      TempChannel[i] = 200;
    }
  }

  BoostTemp    =  EEPROM.read(14);
  timeOfBoost  =  EEPROM.read(15);
  time2sleep   =  EEPROM.read(16);
  MainScrType  =  EEPROM.read(17);
  beepEnable   =  EEPROM.read(18);
  lockEnable   =  EEPROM.read(19);
  Brightness   =  EEPROM.read(20);

  SetChannel   =  EEPROM.read(21);
  SetChannel   = constrain(SetChannel, 0, MAX_CHANNEL - 1);  // Validasi

  PIDenable    =  EEPROM.read(22);
  CurrentTip   =  EEPROM.read(23);

  // Kalibrasi
  uint16_t counter = 24;
  for (uint8_t i = 0; i < NumberOfTips; i++) {
    for (uint8_t j = 0; j < 3; j++) {
      uint8_t high = EEPROM.read(counter++);
      uint8_t low  = EEPROM.read(counter++);
      CalTemp[i][j] = (high << 8) | low;
    }
  }
}
#include <EEPROM.h>

void eraseEEPROM() {
  EEPROM.begin(512); // atau ukuran EEPROM kamu
  for (int i = 0; i < 512; i++) {
    EEPROM.write(i, 0xFF);  // kosongkan
  }
  EEPROM.commit();
  Serial.println("EEPROM erased!");
}


void updateEEPROM() {
  EEPROM.write(0, SetpointTemp >> 8);
  EEPROM.write(1, SetpointTemp & 0xFF);
  EEPROM.write(2, SleepTemp >> 8);
  EEPROM.write(3, SleepTemp & 0xFF);
  EEPROM.write(4, TempMin >> 8);
  EEPROM.write(5, TempMin & 0xFF);
  EEPROM.write(6, TempMax >> 8);
  EEPROM.write(7, TempMax & 0xFF);

  // Simpan array TempChannel[] ke EEPROM mulai dari alamat 30
  for (int i = 0; i < MAX_CHANNEL; i++) {
    int addr = 30 + (i * 2);
    EEPROM.write(addr,     TempChannel[i] >> 8);
    EEPROM.write(addr + 1, TempChannel[i] & 0xFF);
  }

  EEPROM.write(14, BoostTemp);
  EEPROM.write(15, timeOfBoost);
  EEPROM.write(16, time2sleep);
  EEPROM.write(17, MainScrType);
  EEPROM.write(18, beepEnable);
  EEPROM.write(19, lockEnable);
  EEPROM.write(20, Brightness);
  EEPROM.write(21, SetChannel);
  EEPROM.write(22, PIDenable);
  EEPROM.write(23, CurrentTip);

  // Kalibrasi tip
  uint16_t counter = 24;
  for (uint8_t i = 0; i < NumberOfTips; i++) {
    for (uint8_t j = 0; j < 3; j++) {
      EEPROM.write(counter++, CalTemp[i][j] >> 8);
      EEPROM.write(counter++, CalTemp[i][j] & 0xFF);
    }
  }

  EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);  // Tandai EEPROM valid
  EEPROM.commit();
  delay(100);
}

void saveAllTempChannelsToEEPROM() {
  for (int i = 0; i < MAX_CHANNEL; i++) {
    int addr = 30 + i * 2;
    EEPROM.write(addr,     TempChannel[i] >> 8);
    EEPROM.write(addr + 1, TempChannel[i] & 0xFF);
  }
  EEPROM.commit();
}

void MainScreen() {
  if (SetupMenu == false) {
    if (MainScrType == 0) {
      sprite.fillSprite(TFT_BLACK);
      sprite.pushImage(150, 1, T210_WIDTH, T210_HEIGHT, T210);
      sprite.drawBitmap(2, 22,
                        (inSleepMode || inOffMode) ? cofee : home_,
                        (inSleepMode || inOffMode) ? COFEE_WIDTH : HOME_WIDTH,
                        (inSleepMode || inOffMode) ? COFEE_HEIGHT : HOME_HEIGHT,
                        TFT_WHITE);

      sprite.drawBitmap(2, 45,
                        beepEnable ? speaker_on : speaker_off,
                        beepEnable ? SPEAKER_ON_WIDTH : SPEAKER_OFF_WIDTH,
                        beepEnable ? SPEAKER_ON_HEIGHT : SPEAKER_OFF_HEIGHT,
                        TFT_WHITE);

      sprite.drawBitmap(2, 69,
                        lockEnable ? lock : unlock,
                        lockEnable ? LOCK_WIDTH : UNLOCK_WIDTH,
                        lockEnable ? LOCK_HEIGHT : UNLOCK_HEIGHT,
                        TFT_WHITE);
      int readingSetpoint = (inSleepMode) ? SleepTemp :
                            (inBoostMode) ? SetTemp + BoostTemp :
                            SetTemp;

      int power = pwm;
      sprite.setTextDatum(L_BASELINE);

      // Draw linear power bar
      linearMeter(power, 20, 93, 1, 10, 0, 100, GREEN2RED);
      RGBfont(ShowTemp, TempMax, BLUE2RED);

      // Power label
      sprite.setTextSize(0);
      sprite.setTextFont(1);
      sprite.setTextColor(colour);
      sprite.drawString("PWR", 0, 95, 1);

      // PWM percentage
      sprite.setCursor(125, 95);
      sprite.print(power);
      sprite.print("%");

      // Channel display
      const uint8_t maxDisplayChannel = (MAX_CHANNEL <= 3) ? MAX_CHANNEL : 3;
      const uint16_t frameY = 105;
      const uint16_t frameH = 23;
      const uint16_t frameW = 50;
      const uint16_t gap = (SCREEN_W - (frameW * maxDisplayChannel)) / (maxDisplayChannel + 1);

      sprite.setTextDatum(CC_DATUM);

      sprite.loadFont(DIN24);
      for (int i = 0; i < maxDisplayChannel; i++) {
        uint16_t frameX = gap + i * (frameW + gap);
        sprite.drawRoundRect(frameX, frameY, frameW, frameH, 3, FRAME_COLOR);
        sprite.setTextColor((SetChannel == i) ? CH_COLOR : TFT_DARKGREY);
        int centerX = frameX + frameW / 2;
        int centerY = frameY + frameH / 2 + 2;

        char buf[4];
        sprintf(buf, "%d", TempChannel[i]);
        sprite.drawString(buf, centerX, centerY);
      }
      sprite.setTextSize(1);
      sprite.setTextDatum(TL_DATUM);
      sprite.setTextColor(TFT_WHITE);
      sprite.loadFont(DIN16);
      sprite.drawString("SET :", 0, 3);
      char buf[8];
      snprintf(buf, sizeof(buf), "%03dC", readingSetpoint);
      sprite.drawString(buf, 40, 3);
      // Posisi tengah layar untuk status
      int ystatus = 30;
      int xstatus = SCREEN_W / 2;
      sprite.setTextDatum(MC_DATUM); // Tengah horizontal dan vertikal
      if (inSleepMode) {
        sprite.setTextColor(rgb);
        sprite.drawString("SLEEP..!", xstatus, ystatus);
      } else if (inOffMode || !TipIsPresent) {
        sprite.setTextColor(TFT_GREEN);
        sprite.drawString("OFF", xstatus, ystatus);
      } else {
        sprite.setTextColor(rgb);
        sprite.drawString("ACT TEMP", xstatus, ystatus);
      }

      // Bagian bawah selalu ditampilkan
      sprite.loadFont(DIN24);
      sprite.setTextColor(rgb);
      sprite.setTextDatum(TL_DATUM);
      int circleX = 123, circleY = 45;
      int tempX = 130, tempY = 43;

      sprite.drawString("C", tempX, tempY);
      sprite.fillCircle(circleX, circleY, 4, rgb);
      sprite.fillCircle(circleX, circleY, 2, TFT_BLACK);

      sprite.setTextColor(TFT_WHITE);
      sprite.setTextDatum(TR_DATUM);
      sprite.loadFont(DIN16);
      int x = 145;
      int y = 3; // Untuk FreeFont, koordinat y harus disesuaikan secara eksperimen

      if (lockEnable) {
        sprite.drawString("LOCKED", x, y);
      } else if (!TipIsPresent) {
        sprite.drawString("NO TIP", x, y);
      } else if (ShowTemp > 500) {
        sprite.drawString("ERROR", x, y);
      } else if (inOffMode) {
        sprite.drawString("OFF", x, y);
      } else if (inSleepMode) {
        sprite.drawString("COOL", x, y);
      } else if (inBoostMode) {
        sprite.drawString("BOOST", x, y);
      } else if (isWorky) {
        sprite.drawString("READY", x, y);
      } else {
        sprite.drawString("HEAT", x, y);
      }

      if (TipIsPresent) {
        char handleText[10];
        tipNames[constrain(CurrentTip, 0, 2)].toCharArray(handleText, sizeof(handleText));

        sprite.loadFont(DIN12);
        sprite.setTextColor(TFT_WHITE);
        sprite.setTextSize(1);               // Wajib 1 untuk FreeFont

        sprite.drawString(handleText, 148, 80); // Y diubah dari 80 jadi 92 karena FreeFont baseline lebih tinggi
      }
      sprite.unloadFont();


      int xTemp = 120;
      int yTemp = 65;
      sprite.setTextDatum(MR_DATUM);
      sprite.setTextSize(0);

      if (inOffMode || !TipIsPresent) {
        sprite.setTextColor(TFT_GREEN);
        sprite.drawString("000", xTemp, yTemp, 7);
      } else {
        sprite.setTextColor(rgb);
        char buf[10];
        snprintf(buf, sizeof(buf), "%03d", ShowTemp);
        sprite.drawString(buf, xTemp, yTemp, 7);
      }
      sprite.pushSprite(0, 0); // Tampilkan
    } else {
      CurveScreen(); // Jika bukan MainScrType = 0
    }
  }
}

void CurveScreen() {
  static bool lastSleepState = true;  // asumsi awal: dalam sleep
  bool isSleeping = !handleIsUp;;
  sprite.fillSprite(TFT_BLACK);

  // Set setpoint tergantung mode
  if (inSleepMode) SetpointTemp = SleepTemp;
  else if (inBoostMode) SetpointTemp = SetTemp + BoostTemp;
  else SetpointTemp = SetTemp;

  int temp = ShowTemp;
  pwmVal = pwm;

  // Reset data saat keluar dari sleep
  if (lastSleepState && !isSleeping) {
    dataindex = 0;
    tMin = 1000;
    tMax = 0;
    setpointReachedTime = 0;
    setpointReachedX = -1;
    memset(tempData, 0, sizeof(tempData));
    memset(setPointData, 0, sizeof(setPointData));
    memset(timeData, 0, sizeof(timeData));
    memset(pwmData, 0, sizeof(pwmData));
    startTime = millis();
    risingStarted = false;
    setpointReached = false;

  }

  // Reset saat masuk sleep/off
  if (!lastSleepState && isSleeping) {
    dataindex = 0;
    startTime = 0;
    risingStarted = false;
  }

  lastSleepState = isSleeping;

  // Filter suhu
  if (temp > TempMax) temp = MIN_TEMP;
  smoothTemp = alpha * temp + (1 - alpha) * smoothTemp;
  smoothTemp = constrain(smoothTemp, MIN_TEMP, TempMax);
  if (smoothTemp > tMax) tMax = smoothTemp;
  if (smoothTemp < tMin) tMin = smoothTemp;

  // Proses data hanya saat aktif
  if (!isSleeping) {
    if (!risingStarted) {
      risingStarted = true;
      startTime = millis();
    }

    if (!setpointReached && smoothTemp >= SetpointTemp) {
      setpointReached = true;
      setpointReachedX = dataindex;
      setpointReachedTime = millis() - startTime;
    }

    unsigned long nowTime = millis() - startTime;

    if (dataindex < GRAPH_DRAW_W) {
      tempData[dataindex] = smoothTemp;
      setPointData[dataindex] = SetpointTemp;
      pwmData[dataindex] = pwmVal;
      timeData[dataindex] = nowTime;
      dataindex++;
    } else {
      memmove(&tempData[0], &tempData[1], sizeof(float) * (GRAPH_DRAW_W - 1));
      memmove(&setPointData[0], &setPointData[1], sizeof(float) * (GRAPH_DRAW_W - 1));
      memmove(&timeData[0], &timeData[1], sizeof(int) * (GRAPH_DRAW_W - 1));
      memmove(&pwmData[0], &pwmData[1], sizeof(int) * (GRAPH_DRAW_W - 1));
      tempData[GRAPH_DRAW_W - 1] = smoothTemp;
      setPointData[GRAPH_DRAW_W - 1] = SetpointTemp;
      pwmData[GRAPH_DRAW_W - 1] = pwmVal;
      timeData[GRAPH_DRAW_W - 1] = nowTime;
    }
  }

  // --- Gambar latar gradasi biru ---
  for (int i = 0; i < GRAPH_H; i++) {
    uint8_t blue = 255 - (uint8_t)((float)i * 255.0 / GRAPH_H);
    uint16_t c = tft.color565(0, 0, blue);
    sprite.drawFastHLine(GRAPH_X, GRAPH_Y + i, GRAPH_BG_W, c);
  }

  // --- Grid Horizontal dan Label ---
  for (int i = 0; i <= 5; i++) {
    int tempVal = TempMax - i * ((TempMax - MIN_TEMP) / 5);
    int y_suhu = GRAPH_Y + map(tempVal, MIN_TEMP, TempMax, GRAPH_H, 0);
    // --- Gambar garis grid horizontal ---
    for (int x = GRAPH_X; x < GRAPH_X + GRAPH_BG_W; x += 6) {
      sprite.drawFastHLine(x, y_suhu, 3, GRID_COLOR);
    }
    // --- Label kiri: suhu (pakai font DIN12) ---
    sprite.loadFont(DIN12);
    sprite.setTextDatum(MR_DATUM);
    sprite.setTextColor(TFT_WHITE);
    sprite.drawString(String(tempVal), GRAPH_X - 2, y_suhu);
    sprite.unloadFont();
  }


  if (dataindex == 0) {
    sprite.loadFont(DIN16);
    sprite.setTextColor(TFT_WHITE);
    sprite.setTextDatum(TL_DATUM);
    int w = sprite.textWidth("SLEEP..!");
    int x = GRAPH_X + (GRAPH_BG_W >> 1) - (w >> 1);  // Tengah grafik akurat
    sprite.drawString("SLEEP..!", x, GRAPH_H / 2);

    // --- Label suhu dll ---
    LabelTempCurve();
    sprite.loadFont(DIN12);
    sprite.setTextDatum(MC_DATUM);
    sprite.setTextColor(TFT_WHITE);
    int wTime = sprite.textWidth("Time: 0s");
    int xTime = GRAPH_X + (GRAPH_BG_W >> 1) - (wTime >> 1);  // Tengah grafik akurat
    sprite.setCursor(xTime, GRAPH_H - 3);
    sprite.print("Time: 0s");
    sprite.unloadFont();
    sprite.pushSprite(0, 0);
    return;
  }


  // --- Grafik SetPoint ---
  int lastX0 = -1, lastY0 = -1;
  for (int i = 1; i < min(dataindex, GRAPH_DRAW_W); i++) {
    int x0 = GRAPH_X + i;
    int y0 = GRAPH_Y + GRAPH_H - mapf(setPointData[i], MIN_TEMP, TempMax, 0, GRAPH_H);
    if (lastX0 >= 0) {
      sprite.drawLine(lastX0, lastY0, x0, y0, SETPOINT_COLOR);
      sprite.drawLine(lastX0, lastY0 + 1, x0, y0 + 1, SETPOINT_COLOR);
    }
    lastX0 = x0;
    lastY0 = y0;
  }

  // --- Grafik suhu ---
  int lastX = -1, lastY = -1;
  for (int i = 0; i < min(dataindex, GRAPH_DRAW_W); i++) {
    int x = GRAPH_X + i;
    int y = GRAPH_Y + GRAPH_H - mapf(tempData[i], MIN_TEMP, TempMax, 0, GRAPH_H);
    if (lastX >= 0) {
      sprite.drawLine(lastX, lastY, x, y, LINE_COLOR);
      sprite.drawLine(lastX, lastY + 1, x, y + 1, LINE_COLOR);
    }
    lastX = x;
    lastY = y;
  }

  // --- Grafik PWM ---
  /*
    int lastX1 = -1, lastY1 = -1;
    for (int i = 1; i < min(dataindex, GRAPH_DRAW_W); i++) {
    int x1 = GRAPH_X + i;
    int y1 = GRAPH_Y + GRAPH_H - mapf(pwmData[i], 0, MAX_PWM, 0, GRAPH_H);
    if (lastX1 >= 0) {
      sprite.drawLine(lastX1, lastY1, x1, y1, PWM_COLOR);
      sprite.drawLine(lastX1, lastY1 + 1, x1, y1 + 1, PWM_COLOR);
    }
    lastX1 = x1;
    lastY1 = y1;
    }
  */

  // --- Titik indikator ---
  int dataI = dataindex - 1;
  int dotX = GRAPH_X + dataI;
  int dotY = GRAPH_Y + GRAPH_H - mapf(tempData[dataI], MIN_TEMP, TempMax, 0, GRAPH_H);
  int spY  = GRAPH_Y + GRAPH_H - mapf(setPointData[dataI], MIN_TEMP, TempMax, 0, GRAPH_H);
  int pwmY = GRAPH_Y + GRAPH_H - mapf(pwmData[dataI], 0, MAX_PWM, 0, GRAPH_H);

  sprite.fillCircle(dotX, dotY, 2, DOT_COLOR);
  sprite.fillCircle(dotX, spY, 2, SETPOINT_COLOR);
  //sprite.fillCircle(dotX, pwmY, 2, PWM_COLOR);


  // --- Label ---

  auto drawLabel = [&](int x, int y, const char* text, uint16_t color, float plotValue, float maxValue) {
    sprite.loadFont(DIN12);
    int labelX = x - 12;
    int labelY = (plotValue >= maxValue) ? (y + 12) : (y - 12);
    if (labelX < 0) labelX = 0;
    if (labelX > GRAPH_X + GRAPH_DRAW_W - 30) labelX = GRAPH_X + GRAPH_DRAW_W - 30;
    if (labelY < 0) labelY = 0;
    if (labelY > GRAPH_Y + GRAPH_H - 12) labelY = GRAPH_Y + GRAPH_H - 12;
    sprite.setTextColor(color);
    sprite.setTextDatum(TL_DATUM);
    sprite.setCursor(labelX, labelY);
    sprite.print(text);
    sprite.unloadFont();
  };

  dtostrf(setPointData[dataI], 4, 0, buf);
  drawLabel(dotX, spY, buf, SETPOINT_COLOR, setPointData[dataI], TempMax);
  dtostrf(tempData[dataI], 4, 0, buf);
  drawLabel(dotX, dotY, buf, TFT_YELLOW, tempData[dataI], TempMax);
  //dtostrf(pwmData[dataI], 4, 0, buf);
  //drawLabel(dotX, pwmY, buf, PWM_COLOR, pwmData[dataI], MAX_PWM);

  // --- Waktu / Timer ---
  unsigned long timeMs = timeData[dataI];
  unsigned long timeSec = timeMs / 1000;
  String timeStr;
  if (timeSec < 60)
    timeStr = "Time: " + String(timeSec) + "s";
  else {
    int minutes = timeSec / 60;
    int seconds = timeSec % 60;
    timeStr = "Time: " + String(minutes) + ":" + (seconds < 10 ? "0" : "") + String(seconds);
  }

  int offsetY = 12; // geser 12 pixel dari garis
  int timeY;

  // Cek apakah suhu < TempMax
  if (tempData[dataI] >= TempMax) {
    timeY = dotY + offsetY;  // teks di bawah garis
  } else {
    timeY = dotY - offsetY;  // teks di atas garis
  }
  // Clamp agar tidak keluar area grafik
  if (timeY < GRAPH_Y) timeY = GRAPH_Y;
  if (timeY > GRAPH_Y + GRAPH_H - 10) timeY = GRAPH_Y + GRAPH_H - 10;
  // Posisi X tetap center area grafik
  int w = sprite.textWidth(timeStr);
  int timeX = GRAPH_X + (GRAPH_BG_W >> 1) - (w >> 1);  // Tengah grafik akurat
  sprite.loadFont(DIN12);
  sprite.setTextDatum(TL_DATUM);  // Posisi diatur manual
  sprite.setTextColor(TFT_WHITE);
  sprite.setCursor(timeX, timeY);
  sprite.print(timeStr);
  // --- Label Tercapai ---
  if (setpointReached && setpointReachedX >= 0) {
    sprite.setTextColor(TFT_WHITE);

    int w = sprite.textWidth("Tercapai %.1fs");
    int x = GRAPH_X + (GRAPH_BG_W >> 1) - (w >> 1);  // Tetap di tengah grafik

    // Hitung posisi Y berdasarkan spY
    int offsetY = 12;
    int y;
    if (setPointData[dataI] >= TempMax) {
      y = spY + offsetY;
    } else {
      y = spY - offsetY;
    }

    // Clamp supaya tidak keluar area
    if (y < GRAPH_Y) y = GRAPH_Y;
    if (y > GRAPH_Y + GRAPH_H - 10) y = GRAPH_Y + GRAPH_H - 10;

    sprite.setCursor(x - 10, y);
    sprite.printf("Reached %.1fs", setpointReachedTime / 1000.0);
  }

  sprite.unloadFont();

  // --- Batas kiri-kanan ---
  sprite.drawFastVLine(GRAPH_X - 1, GRAPH_Y, GRAPH_H, GRID_COLOR);
  sprite.drawFastVLine(GRAPH_X + GRAPH_BG_W, GRAPH_Y, GRAPH_H, GRID_COLOR);

  LabelTempCurve();
  sprite.pushSprite(0, 0); // ✅ Tampilkan seluruh curve
}
void LabelTempCurve() {
  // --- Bar PWM vertikal di sisi kanan grafik ---
  int barX = GRAPH_X + GRAPH_DRAW_W + 6;       // Posisi X (kanan luar grafik)
  int barW = 10;                           // Lebar bar
  int barBase = GRAPH_Y + GRAPH_H;       // Dasar grafik (bawah)
  int barTop = GRAPH_Y + GRAPH_H - mapf(pwm, 0, MAX_PWM, 0, GRAPH_H);                   // Posisi atas bar sesuai nilai PWM
  int barH = barBase - barTop;             // Tinggi bar

  // Background bar (DARKGREY) dari atas ke bawah
  sprite.fillRect(barX, GRAPH_Y, barW, GRAPH_H, TFT_DARKGREY);

  // Bar PWM aktif (PWM_COLOR)
  sprite.fillRect(barX, barTop, barW, barH, PWM_COLOR);

  // --- Label bawah (Info Channel, Temp, PWM) ---
  int labelTopY = GRAPH_Y + GRAPH_H + 5;
  int labelSpacing = 15;  // Sedikit lebih tinggi karena DIN16 lebih besar

  // Posisi titik dua
  int colonLeftX  = 30;
  int valueLeftX  = colonLeftX + 5;

  int colonRightX = 118;
  int valueRightX = colonRightX + 5;

  sprite.loadFont(DIN15);
  sprite.setTextColor(TFT_WHITE);
  sprite.setTextDatum(TL_DATUM);

  char bufVal[16];
  int x, y;
  // === KIRI: Set ===
  y = labelTopY ;
  sprite.setTextColor(SETPOINT_COLOR);
  sprite.setCursor(colonLeftX - sprite.textWidth("Set") - 3, y);
  sprite.print("Set");
  sprite.setCursor(colonLeftX, y);
  sprite.print(":");
  snprintf(bufVal, sizeof(bufVal), "%dC", (int)SetpointTemp);
  sprite.setCursor(valueLeftX, y);
  sprite.print(bufVal);

  // === KIRI: CH ===
  y = labelTopY + labelSpacing;
  sprite.setTextColor(TFT_WHITE);
  sprite.setCursor(colonLeftX - sprite.textWidth("CH ") - 3, y);
  sprite.print("CH ");
  sprite.setCursor(colonLeftX, y);
  sprite.print(":");
  snprintf(bufVal, sizeof(bufVal), "%d", SetChannel + 1);
  sprite.setCursor(valueLeftX, y);
  sprite.print(bufVal);

  // === KANAN: Pwr ===
  y = labelTopY;
  sprite.setTextColor(PWM_COLOR);
  sprite.setCursor(colonRightX - sprite.textWidth("Temp"), y);
  sprite.print("Pwr");
  sprite.setCursor(colonRightX, y);
  sprite.print(":");
  snprintf(bufVal, sizeof(bufVal), "%d%%", pwmVal);
  sprite.setCursor(valueRightX, y);
  sprite.print(bufVal);

  // === KANAN: Temp ===
  y = labelTopY + labelSpacing;
  sprite.setTextColor(TFT_YELLOW);
  sprite.setCursor(colonRightX - sprite.textWidth("Temp") - 3, y);
  sprite.print("Temp");
  sprite.setCursor(colonRightX, y);
  sprite.print(":");
  snprintf(bufVal, sizeof(bufVal), "%dC", (int)smoothTemp);
  sprite.setCursor(valueRightX, y);
  sprite.print(bufVal);
  sprite.unloadFont();
}


void RGBfont(int val, int n, byte s) {
  for (int b = 1; b <= n; b++) {
    if (val > 0 && b <= val) { // Fill in coloured blocks
      switch (s) {
        case 0: rgb = TFT_RED; break; // Fixed colour
        case 1: rgb = TFT_GREEN; break; // Fixed colour
        case 2: rgb = TFT_BLUE; break; // Fixed colour
        case 3: rgb = rainbowColor(map(b, 0, n, 127,   0)); break; // Blue to red
        case 4: rgb = rainbowColor(map(b, 0, n,  63,   0)); break; // Green to red
        case 5: rgb = rainbowColor(map(b, 0, n,   0,  63)); break; // Red to green
        case 6: rgb = rainbowColor(map(b, 0, n,   0, 159)); break; // Rainbow (red to violet)
      }
    }
  }
}

void linearMeter(int val, int x, int y, int w, int h, int g, int n, byte s) {
  // Variable to save "value" text colour from scheme and set default

  // Draw n colour blocks
  for (int b = 1; b <= n; b++) {
    if (val > 0 && b <= val) { // Fill in coloured blocks
      switch (s) {
        case 0: colour = TFT_RED; break; // Fixed colour
        case 1: colour = TFT_GREEN; break; // Fixed colour
        case 2: colour = TFT_BLUE; break; // Fixed colour
        case 3: colour = rainbowColor(map(b, 0, n, 127,   0)); break; // Blue to red
        case 4: colour = rainbowColor(map(b, 0, n,  63,   0)); break; // Green to red
        case 5: colour = rainbowColor(map(b, 0, n,   0,  63)); break; // Red to green
        case 6: colour = rainbowColor(map(b, 0, n,   0, 159)); break; // Rainbow (red to violet)
      }
      //sprite.fillRect(x , y - b * (h + g), w, h, colour); //keatas
      sprite.fillRect(x + b * (w + g), y, w, h, colour); // kesamping

    }
    else // Fill in blank segments
    {
      //sprite.fillRect(x , y - b * (h + g), w, h, TFT_DARKGREY); //keatas
      sprite.fillRect(x + b * (w + g), y, w, h, TFT_DARKGREY); //kesamping
    }
  }
}

/***************************************************************************************
** Function name:           rainbowColor
** Description:             Return a 16 bit rainbow colour
***************************************************************************************/
// If 'spectrum' is in the range 0-159 it is converted to a spectrum colour
// from 0 = red through to 127 = blue to 159 = violet
// Extending the range to 0-191 adds a further violet to red band

uint16_t rainbowColor(uint8_t spectrum) {
  spectrum = spectrum % 192;

  uint8_t red   = 0; // Red is the top 5 bits of a 16 bit colour spectrum
  uint8_t green = 0; // Green is the middle 6 bits, but only top 5 bits used here
  uint8_t blue  = 0; // Blue is the bottom 5 bits

  uint8_t sector = spectrum >> 5;
  uint8_t amplit = spectrum & 0x1F;

  switch (sector)
  {
    case 0:
      red   = 0x1F;
      green = amplit; // Green ramps up
      blue  = 0;
      break;
    case 1:
      red   = 0x1F - amplit; // Red ramps down
      green = 0x1F;
      blue  = 0;
      break;
    case 2:
      red   = 0;
      green = 0x1F;
      blue  = amplit; // Blue ramps up
      break;
    case 3:
      red   = 0;
      green = 0x1F - amplit; // Green ramps down
      blue  = 0x1F;
      break;
    case 4:
      red   = amplit; // Red ramps up
      green = 0;
      blue  = 0x1F;
      break;
    case 5:
      red   = 0x1F;
      green = 0;
      blue  = 0x1F - amplit; // Blue ramps down
      break;
  }

  return red << 11 | green << 6 | blue;
}
void resetDefaultSettings() {
  SetpointTemp = TEMP_SETPOINT;
  SleepTemp    = TEMP_SLEEP;
  TempMin      = TEMP_MIN;
  TempMax      = TEMP_MAX;

  // Reset semua channel ke default
  for (int i = 0; i < MAX_CHANNEL; i++) {
    TempChannel[i] = TEMP_CH_DEFAULT;
  }

  BoostTemp    = TEMP_BOOST;
  timeOfBoost  = TIMEOFBOOST;
  time2sleep   = TIME2SLEEP;
  MainScrType  = MAINSCREEN;
  handleType   = HANDLE_TYPE;
  beepEnable   = BEEP_ENABLE;
  lockEnable   = LOCK_ENABLE;
  PIDenable    = PID_ENABLE;
  Brightness   = BRIGHTNESS;
  ledcWrite(LED_CHANNEL, Brightness);
}


void heaterOn(uint16_t pwmValue) {
  if (pwmValue > MAX_PWM_VALUE) pwmValue = MAX_PWM_VALUE;

  ledcAttachPin(CONTROL_PIN, HEATER_CHANNEL);

  if (USE_P_CHANNEL) {
    // Untuk P-Channel: logika terbalik (semakin besar PWM, semakin off)
    ledcWrite(HEATER_CHANNEL, MAX_PWM_VALUE - pwmValue);
  } else {
    // Untuk N-Channel: logika normal
    ledcWrite(HEATER_CHANNEL, pwmValue);
  }
}

void heaterOff() {
  // Matikan PWM
  ledcWrite(HEATER_CHANNEL, 0);
  ledcDetachPin(CONTROL_PIN);

  pinMode(CONTROL_PIN, OUTPUT);

  if (USE_P_CHANNEL) {
    // Untuk P-Channel: HIGH = OFF
    digitalWrite(CONTROL_PIN, HIGH);
  } else {
    // Untuk N-Channel: LOW = OFF
    digitalWrite(CONTROL_PIN, LOW);
  }
}

void plotting() {
  Serial.print(Input);     // Suhu aktual
  Serial.print(",");
  Serial.print(Setpoint);  // Suhu target
  Serial.print(",");
  Serial.print(Output);    // Output PID
  Serial.print(",");
  Serial.println(handleDuration / 1000.0); // dalam detik desimal
}
