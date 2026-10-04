/*
  G5500 ESP32-S3 Controller v1.6.23 RECONNECT SAFE TFT + XPT2046 TOUCH CAL + HC-05
  Copyright (c) UA1CFM
  Added:
    - PSTROTATOR / Yaesu GS-232B serial command handling
    - RGB activity indication:
        GREEN = RX command received from PSTROTATOR or Look4Sat/TCP
        RED   = TX response sent to PSTROTATOR or Look4Sat/TCP
        BLUE  = short heartbeat every 2 seconds
    - Non-blocking LED timing (millis), no delay used for RGB indication
    - Multi-control:
        UART0 USB = PSTROTATOR / GS-232 (Windows COM number may vary)
        TCP 4533 = Look4Sat / Hamlib rotctld + GS-232B
        Web UI remains available

  Default onboard addressable RGB LED pin: GPIO48.
  If the selected ESP32-S3 board defines RGB_BUILTIN, that definition is used.
*/


#include <time.h>
#include "esp_system.h"
#include <Arduino.h>
#if ARDUINO_USB_MODE
  #include <HWCDC.h>
  #if !ARDUINO_USB_CDC_ON_BOOT
    HWCDC HWCDCSerial;
  #endif
#endif
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>
#include "cat_image.h"

static const int PIN_AZ_ADC=4, PIN_EL_ADC=5;

// ILI9341 320x240 + XPT2046, shared SPI bus.
// TFT: MOSI=10, SCK=11, CS=12, DC=13, RST=14.
// TOUCH: MISO=2, CS=8, IRQ=9; shares MOSI/SCK with TFT.
static const int PIN_TFT_MOSI=10;
static const int PIN_TFT_SCK =11;
static const int PIN_TFT_MISO=2;
static const int PIN_TFT_CS  =12;
static const int PIN_TFT_DC  =13;
static const int PIN_TFT_RST =14;
static const int PIN_TOUCH_CS=8;
static const int PIN_TOUCH_IRQ=9;

// Default XPT2046 raw calibration for common 2.8" modules.
// If a particular panel is shifted, only these four limits need correction.
static const int TOUCH_RAW_X_MIN=240;
static const int TOUCH_RAW_X_MAX=3850;
static const int TOUCH_RAW_Y_MIN=240;
static const int TOUCH_RAW_Y_MAX=3850;

static bool touchCalValid=false;
static bool touchSwapXY=true;
static int touchX0Raw=TOUCH_RAW_Y_MIN;
static int touchX1Raw=TOUCH_RAW_Y_MAX;
static int touchY0Raw=TOUCH_RAW_X_MAX;
static int touchY1Raw=TOUCH_RAW_X_MIN;

// HC-05 Bluetooth Classic module
// TXD -> GPIO6 (ESP RX), RXD <- GPIO7 (ESP TX), STATE -> GPIO3
static const int PIN_HC05_STATE=3;
static const int PIN_HC05_RX=6;
static const int PIN_HC05_TX=7;
static const uint32_t HC05_BAUD=9600;

static const int PIN_LEFT=15, PIN_RIGHT=16, PIN_UP=17, PIN_DOWN=18;

#ifdef RGB_BUILTIN
static const int PIN_RGB = RGB_BUILTIN;
#else
static const int PIN_RGB = 48;   // Common onboard WS2812/NeoPixel pin on ESP32-S3 DevKit boards
#endif

static const uint8_t RGB_LEVEL = 40;       // LED brightness 0..255
static const uint32_t RGB_FLASH_MS = 55;   // TX/RX pulse duration
static const uint32_t RGB_HEARTBEAT_MS = 2000;
static const uint32_t RGB_HEARTBEAT_ON_MS = 30;

static const uint16_t TCP_PORT=4533;
static const char* AP_SSID="G5500-Rotator";
static const char* AP_PASS="";  // default: open AP; set 8..63 char password from Web UI

Adafruit_ILI9341 tft(&SPI, PIN_TFT_DC, PIN_TFT_CS, PIN_TFT_RST);
XPT2046_Touchscreen touch(PIN_TOUCH_CS, PIN_TOUCH_IRQ);
Preferences prefs;
WebServer web(80);
WiFiServer tcpServer(TCP_PORT);
WiFiClient rotctldClient;
String rotctldLine;

struct Calibration {
  // GS-232 two-point AZ calibration is intentionally separate for P36 and P45.
  int az36_0=0, az36Max=3100;
  int az45_0=0, az45Max=3100;
  // EL calibration is shared because the elevation range does not depend on P36/P45.
  int el0=0, elMax=3100;
  bool valid=false;
} cal;

// Exactly two calibration algorithms:
//   0 = GS-232 two-point calibration (O/O2/F/F2)
//   1 = multi-point piecewise-linear calibration
// Within each algorithm AZ has TWO independent data sets: P36 (0..360) and P45 (0..450).
enum CalibrationMode : uint8_t { CAL_GS232_2POINT=0, CAL_MULTI_POINT=1 };
CalibrationMode calibrationMode=CAL_GS232_2POINT;

static const uint8_t AZ36_CAL_N=5, AZ45_CAL_N=6, EL_CAL_N=5;
static const float az36CalDeg[AZ36_CAL_N]={0,90,180,270,360};
static const float az45CalDeg[AZ45_CAL_N]={0,90,180,270,360,450};
static const float elCalDeg[EL_CAL_N]={0,45,90,135,180};

int az36CalMv[AZ36_CAL_N]={-1,-1,-1,-1,-1};
int az45CalMv[AZ45_CAL_N]={-1,-1,-1,-1,-1,-1};
int elCalMv[EL_CAL_N]={-1,-1,-1,-1,-1};

bool az36MultiValid=false, az45MultiValid=false, elMultiValid=false;
bool az36LinearValid=false, az45LinearValid=false, elLinearValid=false;

// Explicit two-point endpoint flags.
bool az36OSet=false, az36FSet=false;
bool az45OSet=false, az45FSet=false;
bool elOSet=false, elFSet=false;

String wifiSsid,wifiPass,apPass;
float currentAz=0,currentEl=0,targetAz=0,targetEl=0;
bool targetValid=false;          // no movement until at least one real target has been received
bool azTrack=false;              // LVBTrack-style axis tracking flag
bool elTrack=false;              // LVBTrack-style axis tracking flag
uint32_t lastTargetRxMs=0;       // diagnostics only; never used to stop motion

// GS-232B timed sequence support (TCP). Kept compact to save RAM.
static const uint8_t GSSEQ_MAX=16;
float gsSeqAz[GSSEQ_MAX], gsSeqEl[GSSEQ_MAX];
uint8_t gsSeqCount=0, gsSeqIndex=0;
uint16_t gsSeqIntervalSec=0;
bool gsSeqHasEl=false, gsSeqActive=false;
uint32_t gsSeqNextMs=0;
uint8_t gsAzMode=45;       // 45 = 450-degree mode, 36 = 360-degree mode
uint8_t gsSpeed=4;          // X1..X4 accepted; relay hardware has fixed speed

static const int8_t RSSI_EMPTY = 127;
int8_t rssiHour[60], rssiDay[288], rssiWeek[336], rssiMonth[360], rssiYear[365];
uint16_t rssiHourPos=0,rssiHourCount=0,rssiDayPos=0,rssiDayCount=0;
uint16_t rssiWeekPos=0,rssiWeekCount=0,rssiMonthPos=0,rssiMonthCount=0,rssiYearPos=0,rssiYearCount=0;
int32_t acc1m=0,acc5m=0,acc30m=0,acc2h=0,acc24h=0;
uint16_t cnt1m=0,cnt5m=0,cnt30m=0,cnt2h=0,cnt24h=0;
uint32_t rssiLastSampleMs=0,rssiT1m=0,rssiT5m=0,rssiT30m=0,rssiT2h=0,rssiT24h=0;
static const uint32_t RSSI_SAMPLE_MS=10000UL;
static const uint32_t WIFI_RECONNECT_MS=30000UL;       // wait between reconnect attempts
static const uint32_t WIFI_RECONNECT_TIMEOUT_MS=8000UL; // one STA attempt timeout
static uint32_t wifiReconnectMs=0;
static uint32_t wifiReconnectTryStartedMs=0;
static bool wifiReconnectTryActive=false;

uint32_t lastTargetCommandMs=0,lastDisplayMs=0;

String serialCmd;
String hc05Cmd;
HardwareSerial HC05Serial(1);
enum GsReplyPort : uint8_t { GS_REPLY_USB=0, GS_REPLY_HC05=1 };
static GsReplyPort gsReplyPort=GS_REPLY_USB;
enum RgbState : uint8_t { RGB_IDLE, RGB_RX, RGB_TX, RGB_HEARTBEAT };
RgbState rgbState = RGB_IDLE;
uint32_t rgbUntilMs = 0;
uint32_t lastHeartbeatMs = 0;
bool pendingTxFlash = false;

enum TftPage : uint8_t {
  TFT_PAGE_DASH=0,
  TFT_PAGE_CAL_HOME=1,
  TFT_PAGE_CAL_AZ=2,
  TFT_PAGE_CAL_EL=3,
  TFT_PAGE_TOUCH_CAL=4,
  TFT_PAGE_IMAGE=5
};
static TftPage tftPage=TFT_PAGE_DASH;
static uint32_t lastTouchMs=0;
static uint32_t imagePageStartedMs=0;
static const uint32_t IMAGE_PAGE_DURATION_MS=3000;
static bool tftNeedsRedraw=true;

// Dashboard anti-flicker cache.
// Dynamic regions are repainted only when their visible value actually changes.
static float dashLastAz=-9999.0f, dashLastEl=-9999.0f;
static float dashLastTargetAz=-9999.0f, dashLastTargetEl=-9999.0f;
static bool dashLastTargetValid=false;
static int dashLastAzBar=-1, dashLastElBar=-1;
static String dashLastAzState="", dashLastElState="";
static String dashLastCal="", dashLastWifi="", dashLastBt="", dashLastIp="", dashLastTouch="";
static uint8_t touchCalStep=0;
static int touchCalRawX[4]={0,0,0,0};
static int touchCalRawY[4]={0,0,0,0};
static bool touchWaitRelease=false;
static uint32_t touchHoldStartMs=0;
static bool touchHoldArmed=false;

static void saveCal();
static void updatePosition();
static void dbgPrint(const String &s);
static void dbgPrintln(const String &s);
static void processGs232Command(String cmd);
static void captureGs232Cal(const String &cmd);
static void initLocalDisplay();
static void updateLocalDisplay();
static void serviceTouch();
static void drawTftPage();
static void loadTouchCalibration();
static void saveTouchCalibration();
static void startTouchCalibration();
static void serviceTouchCalibration();
static void serviceTouchRecoveryHold();
static void showHeaderImage();
static void serviceHeaderImage();
static void serviceWifiReconnect();

static float clampf(float v,float lo,float hi){ return v<lo?lo:(v>hi?hi:v); }
static int adcAvgMv(int pin){ uint32_t s=0; for(int i=0;i<25;i++){s+=analogReadMilliVolts(pin);delayMicroseconds(150);} return s/25; }

// ADC diagnostic only. This DOES NOT change the position calculation.
static void adcDiagRead(int pin,int &raw,int &mv){
  uint32_t sr=0, sm=0;
  for(int i=0;i<25;i++){
    sr += analogRead(pin);
    sm += analogReadMilliVolts(pin);
    delayMicroseconds(150);
  }
  raw=(int)(sr/25);
  mv=(int)(sm/25);
}
static float mapCal(int raw,int a,int b,float maxd){ if(a==b)return 0; return clampf((raw-a)*maxd/(float)(b-a),0,maxd); }

static bool calPointsMonotonic(const int *adc,uint8_t n){
  if(n<2) return false;
  for(uint8_t i=0;i<n;i++) if(adc[i]<0) return false;
  bool inc=adc[n-1]>adc[0];
  if(adc[n-1]==adc[0]) return false;
  for(uint8_t i=1;i<n;i++){
    if(inc && adc[i]<=adc[i-1]) return false;
    if(!inc && adc[i]>=adc[i-1]) return false;
  }
  return true;
}
static float mapPiecewise(int raw,const int *adc,const float *deg,uint8_t n){
  if(!calPointsMonotonic(adc,n)) return 0.0f;
  bool inc=adc[n-1]>adc[0];
  if((inc && raw<=adc[0]) || (!inc && raw>=adc[0])) return deg[0];
  if((inc && raw>=adc[n-1]) || (!inc && raw<=adc[n-1])) return deg[n-1];
  for(uint8_t i=0;i<n-1;i++){
    bool inside=inc ? (raw>=adc[i] && raw<=adc[i+1]) : (raw<=adc[i] && raw>=adc[i+1]);
    if(inside){
      float t=(raw-adc[i])/(float)(adc[i+1]-adc[i]);
      return deg[i] + t*(deg[i+1]-deg[i]);
    }
  }
  return deg[n-1];
}

static bool activeAzLinearValid(){ return gsAzMode==36 ? az36LinearValid : az45LinearValid; }
static bool activeAzMultiValid(){ return gsAzMode==36 ? az36MultiValid : az45MultiValid; }
static int activeAz2Zero(){ return gsAzMode==36 ? cal.az36_0 : cal.az45_0; }
static int activeAz2Full(){ return gsAzMode==36 ? cal.az36Max : cal.az45Max; }
static bool activeAzOSet(){ return gsAzMode==36 ? az36OSet : az45OSet; }
static bool activeAzFSet(){ return gsAzMode==36 ? az36FSet : az45FSet; }
static float activeAzMaxDeg(){ return gsAzMode==36 ? 360.0f : 450.0f; }

static void refreshMultiCalValidity(){
  az36MultiValid=calPointsMonotonic(az36CalMv,AZ36_CAL_N);
  az45MultiValid=calPointsMonotonic(az45CalMv,AZ45_CAL_N);
  elMultiValid=calPointsMonotonic(elCalMv,EL_CAL_N);
}

static bool selectedCalibrationValid(){
  if(calibrationMode==CAL_MULTI_POINT) return activeAzMultiValid() && elMultiValid;
  return activeAzLinearValid() && elLinearValid;
}

static const char* calibrationModeName(){
  return calibrationMode==CAL_MULTI_POINT ? "MULTI-POINT" : "GS232 2-POINT";
}

static void setCalibrationMode(CalibrationMode m){
  calibrationMode=m;
  prefs.begin("g5500",false);
  prefs.putUChar("calmode",(uint8_t)calibrationMode);
  prefs.end();
}

static void setGsAzMode(uint8_t m){
  if(m!=36 && m!=45) return;

  // Repeating the already selected range must not interrupt tracking.
  if(m==gsAzMode){
    prefs.begin("g5500",false);
    prefs.putUChar("azmode",gsAzMode);
    prefs.end();
    cal.valid=selectedCalibrationValid();
    return;
  }

  // SAFETY: prevent an unreachable old target after 450 -> 360 switch.
  stopAzimuth();
  gsSeqActive=false;

  gsAzMode=m;
  prefs.begin("g5500",false);
  prefs.putUChar("azmode",gsAzMode);
  prefs.end();

  updatePosition();
  targetAz=currentAz; // cancel stale AZ target without disturbing EL target
  cal.valid=selectedCalibrationValid();

  dbgPrint("[AZ MODE] P");
  dbgPrint(String(gsAzMode));
  dbgPrintln(" selected; AZ stopped and stale AZ target cancelled");
}

static void captureGs232Cal(const String &cmd){
  int az=adcAvgMv(PIN_AZ_ADC);
  int el=adcAvgMv(PIN_EL_ADC);

  if(cmd=="O"){
    if(gsAzMode==36){ cal.az36_0=az; az36OSet=true; }
    else            { cal.az45_0=az; az45OSet=true; }
  }else if(cmd=="F"){
    if(gsAzMode==36){ cal.az36Max=az; az36FSet=true; }
    else            { cal.az45Max=az; az45FSet=true; }
  }else if(cmd=="O2"){
    cal.el0=el; elOSet=true;
  }else if(cmd=="F2"){
    cal.elMax=el; elFSet=true;
  }else{
    return;
  }

  az36LinearValid=(az36OSet && az36FSet && abs(cal.az36Max-cal.az36_0)>100);
  az45LinearValid=(az45OSet && az45FSet && abs(cal.az45Max-cal.az45_0)>100);
  elLinearValid=(elOSet && elFSet && abs(cal.elMax-cal.el0)>100);
  cal.valid=selectedCalibrationValid();
  saveCal();

  dbgPrint("[CAL GS232] "); dbgPrint(cmd);
  dbgPrint(" P"); dbgPrint(String(gsAzMode));
  dbgPrint(" AZ0="); dbgPrint(String(activeAz2Zero()));
  dbgPrint(" AZFULL="); dbgPrint(String(activeAz2Full()));
  dbgPrint(" EL0="); dbgPrint(String(cal.el0));
  dbgPrint(" EL180="); dbgPrintln(String(cal.elMax));
}

// -----------------------------------------------------------------------------
// Web diagnostic console.
// The same diagnostic stream that is written to Native USB Serial/JTAG is
// mirrored into a small volatile RAM ring buffer for the /diaglog web page.
// -----------------------------------------------------------------------------
static const uint16_t DIAG_MAX_LINES = 160;
static const uint16_t DIAG_MAX_LINE_LEN = 220;
String diagLines[DIAG_MAX_LINES];
uint16_t diagHead = 0;
uint16_t diagCount = 0;
String diagPending;

static void diagStoreLine(String line){
  line.replace("\r","");
  if(line.length() > DIAG_MAX_LINE_LEN){
    line = line.substring(0,DIAG_MAX_LINE_LEN-3) + "...";
  }
  diagLines[diagHead] = line;
  diagHead = (diagHead + 1) % DIAG_MAX_LINES;
  if(diagCount < DIAG_MAX_LINES) diagCount++;
}

static void diagMirror(const String &s,bool newline){
  for(size_t i=0;i<s.length();i++){
    char ch=s[i];
    if(ch=='\r') continue;
    if(ch=='\n'){
      diagStoreLine(diagPending);
      diagPending="";
    }else{
      diagPending += ch;
      if(diagPending.length() >= DIAG_MAX_LINE_LEN){
        diagStoreLine(diagPending);
        diagPending="";
      }
    }
  }
  if(newline){
    diagStoreLine(diagPending);
    diagPending="";
  }
}

static void diagClear(){
  for(uint16_t i=0;i<DIAG_MAX_LINES;i++) diagLines[i]="";
  diagHead=0;
  diagCount=0;
  diagPending="";
}

static String diagSnapshot(){
  String out;
  out.reserve(8192);
  uint16_t start=(diagHead + DIAG_MAX_LINES - diagCount) % DIAG_MAX_LINES;
  for(uint16_t i=0;i<diagCount;i++){
    uint16_t idx=(start+i)%DIAG_MAX_LINES;
    out += diagLines[idx];
    out += '\n';
  }
  if(diagPending.length()) out += diagPending;
  return out;
}

static void dbgPrint(const String &s){
  diagMirror(s,false);
#if ARDUINO_USB_MODE
  if(HWCDCSerial) HWCDCSerial.print(s);
#endif
}
static void dbgPrintln(const String &s){
  diagMirror(s,true);
#if ARDUINO_USB_MODE
  if(HWCDCSerial) HWCDCSerial.println(s);
#endif
}
static void dbgPstRx(const String &s){
  dbgPrint("[PST RX] ");
  dbgPrintln(s);
}
static void dbgPstTx(const String &s){
  dbgPrint("[PST TX] ");
  String t=s;
  t.replace("\r","");
  t.replace("\n","");
  dbgPrintln(t);
}


static esp_reset_reason_t bootResetReason = ESP_RST_UNKNOWN;
static String bootResetReasonText = "UNKNOWN";

static String resetReasonToText(esp_reset_reason_t r){
  switch(r){
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXTERNAL";
    case ESP_RST_SW:        return "SOFTWARE";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}


static const uint8_t RESET_HISTORY_MAX = 20;

static bool syncClockFromNtp(uint32_t timeoutMs=3500){
  if(WiFi.status()!=WL_CONNECTED) return false;
  configTime(0,0,"pool.ntp.org","time.nist.gov");
  uint32_t t0=millis();
  time_t now=0;
  while(millis()-t0<timeoutMs){
    now=time(nullptr);
    if(now>1700000000) return true;  // sane Unix time
    delay(100);
  }
  return false;
}

static void addResetHistoryEntry(){
  prefs.begin("g5500rst",false);

  uint8_t head=prefs.getUChar("head",0);
  uint8_t count=prefs.getUChar("count",0);
  if(head>=RESET_HISTORY_MAX) head=0;
  if(count>RESET_HISTORY_MAX) count=RESET_HISTORY_MAX;

  time_t now=time(nullptr);
  uint64_t epoch=(now>1700000000) ? (uint64_t)now : 0ULL;

  char kt[8], kr[8], kc[8];
  snprintf(kt,sizeof(kt),"t%02u",head);
  snprintf(kr,sizeof(kr),"r%02u",head);
  snprintf(kc,sizeof(kc),"c%02u",head);

  prefs.putULong64(kt,epoch);
  prefs.putString(kr,bootResetReasonText);
  prefs.putInt(kc,(int)bootResetReason);

  head=(head+1)%RESET_HISTORY_MAX;
  if(count<RESET_HISTORY_MAX) count++;

  prefs.putUChar("head",head);
  prefs.putUChar("count",count);
  prefs.end();
}

static String resetHistoryJson(){
  prefs.begin("g5500rst",true);
  uint8_t head=prefs.getUChar("head",0);
  uint8_t count=prefs.getUChar("count",0);
  if(head>=RESET_HISTORY_MAX) head=0;
  if(count>RESET_HISTORY_MAX) count=RESET_HISTORY_MAX;

  String j="{\"count\":"+String(count)+",\"entries\":[";
  for(uint8_t n=0;n<count;n++){
    int idx=(int)head-1-(int)n;
    while(idx<0) idx+=RESET_HISTORY_MAX;

    char kt[8], kr[8], kc[8];
    snprintf(kt,sizeof(kt),"t%02d",idx);
    snprintf(kr,sizeof(kr),"r%02d",idx);
    snprintf(kc,sizeof(kc),"c%02d",idx);

    uint64_t epoch=prefs.getULong64(kt,0ULL);
    String reason=prefs.getString(kr,"UNKNOWN");
    int code=prefs.getInt(kc,0);

    if(n) j+=",";
    j+="{\"epoch\":"+String((unsigned long long)epoch)+
       ",\"reason\":\""+reason+"\",\"code\":"+String(code)+"}";
  }
  j+="]}";
  prefs.end();
  return j;
}

static void clearResetHistory(){
  prefs.begin("g5500rst",false);
  prefs.clear();
  prefs.end();
}

static void updatePosition();
static void allStop();
static void moveDir(char d);
static void setReferenceTarget(float az,float el,bool setAz,bool setEl);
static void refreshMultiCalValidity();

static void rgbSet(uint8_t r,uint8_t g,uint8_t b){
  rgbLedWrite(PIN_RGB,r,g,b);
}
static void rgbStart(RgbState s){
  rgbState=s;
  if(s==RGB_RX){ rgbSet(0,RGB_LEVEL,0); rgbUntilMs=millis()+RGB_FLASH_MS; }
  else if(s==RGB_TX){ rgbSet(RGB_LEVEL,0,0); rgbUntilMs=millis()+RGB_FLASH_MS; }
  else if(s==RGB_HEARTBEAT){ rgbSet(0,0,RGB_LEVEL); rgbUntilMs=millis()+RGB_HEARTBEAT_ON_MS; }
  else { rgbSet(0,0,0); rgbUntilMs=0; }
}
static void flashRx(){
  // RX gets shown first. A TX response generated immediately afterwards is queued.
  rgbStart(RGB_RX);
}
static void flashTx(){
  if(rgbState==RGB_RX && (int32_t)(rgbUntilMs-millis())>0) pendingTxFlash=true;
  else rgbStart(RGB_TX);
}
static void updateRgb(){
  uint32_t now=millis();
  if(rgbState!=RGB_IDLE && (int32_t)(now-rgbUntilMs)>=0){
    rgbSet(0,0,0);
    rgbState=RGB_IDLE;
    if(pendingTxFlash){
      pendingTxFlash=false;
      rgbStart(RGB_TX);
      return;
    }
  }
  if(rgbState==RGB_IDLE && now-lastHeartbeatMs>=RGB_HEARTBEAT_MS){
    lastHeartbeatMs=now;
    rgbStart(RGB_HEARTBEAT);
  }
}
static void sendReply(const String &s){
  if(gsReplyPort==GS_REPLY_HC05){
    HC05Serial.print(s);
    dbgPrint("[HC05 TX] ");
    dbgPrintln(s);
  }else{
    Serial0.print(s);
    dbgPstTx(s);
  }
  flashTx();
}

static int roundedAz(){ return constrain((int)lroundf(currentAz),0,450); }
static int roundedEl(){ return constrain((int)lroundf(currentEl),0,180); }

static void stopAzimuth(){
  digitalWrite(PIN_LEFT,LOW);
  digitalWrite(PIN_RIGHT,LOW);
  azTrack=false;
}
static void stopElevation(){
  digitalWrite(PIN_UP,LOW);
  digitalWrite(PIN_DOWN,LOW);
  elTrack=false;
}

static void processGs232Command(String cmd){
  cmd.trim();
  if(!cmd.length()) return;
  cmd.toUpperCase();
  flashRx();

  // Refresh angles immediately before answering a position request.
  if(cmd=="C" || cmd=="B" || cmd=="C2") updatePosition();

  if(cmd=="R"){ gsSeqActive=false; moveDir('R'); return; }
  if(cmd=="L"){ gsSeqActive=false; moveDir('L'); return; }
  if(cmd=="U"){ gsSeqActive=false; moveDir('U'); return; }
  if(cmd=="D"){ gsSeqActive=false; moveDir('D'); return; }
  if(cmd=="A"){ gsSeqActive=false; stopAzimuth(); return; }
  if(cmd=="E"){ gsSeqActive=false; stopElevation(); return; }
  if(cmd=="S"){ gsSeqActive=false; allStop(); return; }

  if(cmd=="C"){
    char b[20];
    snprintf(b,sizeof(b),"AZ=%03d\r\n",roundedAz());
    sendReply(String(b));
    return;
  }
  if(cmd=="B"){
    char b[20];
    snprintf(b,sizeof(b),"EL=%03d\r\n",roundedEl());
    sendReply(String(b));
    return;
  }
  if(cmd=="C2"){
    char b[32];
    snprintf(b,sizeof(b),"AZ=%03d EL=%03d\r\n",roundedAz(),roundedEl());
    sendReply(String(b));
    return;
  }

  // GS-232B position command used by PSTROTATOR: Waaa eee
  if(cmd[0]=='W'){
    gsSeqActive=false;
    float az,el;
    if(sscanf(cmd.c_str()+1,"%f %f",&az,&el)==2){
      setReferenceTarget(clampf(az,0,gsAzMode==36?360.0f:450.0f),clampf(el,0,180),true,true);
      lastTargetCommandMs=millis(); lastTargetRxMs=millis();
    }
    return;
  }

  // Azimuth-only GS-232 command: Maaa
  if(cmd[0]=='M' && cmd.length()>1){
    gsSeqActive=false;
    float az=cmd.substring(1).toFloat();
    setReferenceTarget(clampf(az,0,gsAzMode==36?360.0f:450.0f),currentEl,true,false);
    lastTargetCommandMs=millis(); lastTargetRxMs=millis();
    return;
  }

  // GS-232 azimuth operating range. This also defines the full-scale angle
  // captured by F in the GS232 2-POINT calibration mode.
  if(cmd=="P36"){ setGsAzMode(36); return; }
  if(cmd=="P45"){ setGsAzMode(45); return; }

  // Standard GS-232 calibration commands.
  // O  = current AZ feedback becomes 0 degrees
  // F  = current AZ feedback becomes full scale (360 or 450, from P36/P45)
  // O2 = current EL feedback becomes 0 degrees
  // F2 = current EL feedback becomes 180 degrees
  if(cmd=="O" || cmd=="O2" || cmd=="F" || cmd=="F2"){
    captureGs232Cal(cmd);
    return;
  }

  // Basic help response. Z is intentionally not implemented.
  if(cmd=="H"){
    sendReply("GS-232B: R L A C M S O F P36 P45\r\n");
    return;
  }
  if(cmd=="H2"){
    sendReply("GS-232B: U D E C2 W S O2 F2 B\r\n");
    return;
  }
  if(cmd=="H3"){
    sendReply(String("CAL=")+calibrationModeName()+"\r\n");
    return;
  }
}
static void handlePstRotatorSerial(){
  while(Serial0.available()){
    char c=(char)Serial0.read();
    if(c=='\r' || c=='\n'){
      if(serialCmd.length()){
        gsReplyPort=GS_REPLY_USB;
        dbgPstRx(serialCmd);
        processGs232Command(serialCmd);
        serialCmd="";
      }
    }else if(c>=32 && c<=126){
      if(serialCmd.length()<80) serialCmd+=c;
      else serialCmd=""; // protect against a malformed/never-terminated line
    }
  }
}


static bool hc05Connected(){
  return digitalRead(PIN_HC05_STATE)==HIGH;
}

static void handleHc05Serial(){
  while(HC05Serial.available()){
    char c=(char)HC05Serial.read();
    if(c=='\r' || c=='\n'){
      if(hc05Cmd.length()){
        gsReplyPort=GS_REPLY_HC05;
        dbgPrint("[HC05 RX] ");
        dbgPrintln(hc05Cmd);
        processGs232Command(hc05Cmd);
        hc05Cmd="";
      }
    }else if(c>=32 && c<=126){
      if(hc05Cmd.length()<80) hc05Cmd+=c;
      else hc05Cmd="";
    }
  }
}


static void updatePosition(){
  // factory-calibrated ESP32-S3 millivolts -> selected mechanical calibration -> angle.
  // AZ table is selected by P36/P45; EL table is shared.
  int az=adcAvgMv(PIN_AZ_ADC), el=adcAvgMv(PIN_EL_ADC);
  float azMax=activeAzMaxDeg();

  if(calibrationMode==CAL_MULTI_POINT){
    if(gsAzMode==36 && az36MultiValid) currentAz=mapPiecewise(az,az36CalMv,az36CalDeg,AZ36_CAL_N);
    else if(gsAzMode==45 && az45MultiValid) currentAz=mapPiecewise(az,az45CalMv,az45CalDeg,AZ45_CAL_N);
    else currentAz=clampf(az*azMax/3100.0f,0,azMax);

    if(elMultiValid) currentEl=mapPiecewise(el,elCalMv,elCalDeg,EL_CAL_N);
    else currentEl=clampf(el*180.0f/3100.0f,0,180);
  }else{
    if(activeAzLinearValid()) currentAz=mapCal(az,activeAz2Zero(),activeAz2Full(),azMax);
    else currentAz=clampf(az*azMax/3100.0f,0,azMax);

    if(elLinearValid) currentEl=mapCal(el,cal.el0,cal.elMax,180.0f);
    else currentEl=clampf(el*180.0f/3100.0f,0,180);
  }
  cal.valid=selectedCalibrationValid();
}
static void allStop(){
  digitalWrite(PIN_LEFT,LOW);
  digitalWrite(PIN_RIGHT,LOW);
  digitalWrite(PIN_UP,LOW);
  digitalWrite(PIN_DOWN,LOW);
  azTrack=false;
  elTrack=false;
}
// Movement logic ported from the supplied LVBTrack.c reference.
// No global Tracking ON/OFF state:
// STOP cancels current motion only; the next valid target can start motion again.
// A new target chooses the direction once. The periodic update only stops
// the axis when the destination is reached or crossed.
static void setReferenceTarget(float az,float el,bool setAz,bool setEl){
  updatePosition();

  if(setAz){
    targetAz=az;
    targetValid=true;

    float cur=currentAz;
    float tgt=targetAz;

    if(tgt>cur){
      digitalWrite(PIN_RIGHT,HIGH);
      digitalWrite(PIN_LEFT,LOW);
      azTrack=true;
    }else if(tgt<cur){
      digitalWrite(PIN_LEFT,HIGH);
      digitalWrite(PIN_RIGHT,LOW);
      azTrack=true;
    }else{
      digitalWrite(PIN_LEFT,LOW);
      digitalWrite(PIN_RIGHT,LOW);
      azTrack=false;
    }
  }

  if(setEl){
    targetEl=el;
    targetValid=true;

    float cur=currentEl;
    float tgt=targetEl;

    if(tgt>cur){
      digitalWrite(PIN_UP,HIGH);
      digitalWrite(PIN_DOWN,LOW);
      elTrack=true;
    }else if(tgt<cur){
      digitalWrite(PIN_DOWN,HIGH);
      digitalWrite(PIN_UP,LOW);
      elTrack=true;
    }else{
      digitalWrite(PIN_DOWN,LOW);
      digitalWrite(PIN_UP,LOW);
      elTrack=false;
    }
  }
}

static void moveDir(char d){
  // Match LVBTrack.c more closely:
  // manual AZ cancels only AZ tracking; manual EL cancels only EL tracking.
  // The other axis may continue its automatic movement.
  if(d=='L'){
    azTrack=false;
    digitalWrite(PIN_RIGHT,LOW);
    digitalWrite(PIN_LEFT,HIGH);
  }else if(d=='R'){
    azTrack=false;
    digitalWrite(PIN_LEFT,LOW);
    digitalWrite(PIN_RIGHT,HIGH);
  }else if(d=='U'){
    elTrack=false;
    digitalWrite(PIN_DOWN,LOW);
    digitalWrite(PIN_UP,HIGH);
  }else if(d=='D'){
    elTrack=false;
    digitalWrite(PIN_UP,LOW);
    digitalWrite(PIN_DOWN,HIGH);
  }else if(d=='A'){
    stopAzimuth();
  }else if(d=='E'){
    stopElevation();
  }else if(d=='S'){
    allStop();
  }
}
static String motion(){
  bool l=digitalRead(PIN_LEFT);
  bool r=digitalRead(PIN_RIGHT);
  bool u=digitalRead(PIN_UP);
  bool d=digitalRead(PIN_DOWN);

  if(u && r) return "UP+RIGHT";
  if(u && l) return "UP+LEFT";
  if(d && r) return "DOWN+RIGHT";
  if(d && l) return "DOWN+LEFT";
  if(l) return "LEFT";
  if(r) return "RIGHT";
  if(u) return "UP";
  if(d) return "DOWN";
  return "STOP";
}
static void controlRotator(){
  // Direct LVBTrack.c principle: only stop a motion that was started
  // by a target command. Do not continuously steer around the target.
  if(azTrack){
    float cur=currentAz;
    float tgt=targetAz;

    if((tgt>=cur && digitalRead(PIN_LEFT)) ||
       (tgt<=cur && digitalRead(PIN_RIGHT))){
      digitalWrite(PIN_LEFT,LOW);
      digitalWrite(PIN_RIGHT,LOW);
      azTrack=false;
    }
  }

  if(elTrack){
    float cur=currentEl;
    float tgt=targetEl;

    if((tgt>=cur && digitalRead(PIN_DOWN)) ||
       (tgt<=cur && digitalRead(PIN_UP))){
      digitalWrite(PIN_DOWN,LOW);
      digitalWrite(PIN_UP,LOW);
      elTrack=false;
    }
  }
}
static void initRssiHistory(){
  memset(rssiHour,RSSI_EMPTY,sizeof(rssiHour));
  memset(rssiDay,RSSI_EMPTY,sizeof(rssiDay));
  memset(rssiWeek,RSSI_EMPTY,sizeof(rssiWeek));
  memset(rssiMonth,RSSI_EMPTY,sizeof(rssiMonth));
  memset(rssiYear,RSSI_EMPTY,sizeof(rssiYear));
  prefs.begin("g5500rssi",true);
  size_t n=prefs.getBytesLength("year");
  if(n==sizeof(rssiYear)) prefs.getBytes("year",rssiYear,sizeof(rssiYear));
  rssiYearPos=prefs.getUShort("ypos",0);
  rssiYearCount=prefs.getUShort("ycnt",0);
  prefs.end();
  if(rssiYearPos>=365) rssiYearPos=0;
  if(rssiYearCount>365) rssiYearCount=365;
  uint32_t now=millis();
  rssiT1m=rssiT5m=rssiT30m=rssiT2h=rssiT24h=now;
}
static void rssiPush(int8_t *buf,uint16_t size,uint16_t &pos,uint16_t &count,int8_t v){
  buf[pos]=v; pos=(uint16_t)((pos+1)%size); if(count<size) count++;
}
static int8_t rssiAvg(int32_t sum,uint16_t cnt){
  if(!cnt) return RSSI_EMPTY;
  int v=(int)lroundf((float)sum/(float)cnt);
  if(v<-127)v=-127; if(v>0)v=0; return (int8_t)v;
}
static void persistRssiYear(){
  prefs.begin("g5500rssi",false);
  prefs.putBytes("year",rssiYear,sizeof(rssiYear));
  prefs.putUShort("ypos",rssiYearPos); prefs.putUShort("ycnt",rssiYearCount);
  prefs.end();
}
static void serviceRssiHistory(){
  uint32_t now=millis();
  if(now-rssiLastSampleMs<RSSI_SAMPLE_MS) return;
  rssiLastSampleMs=now;
  if(WiFi.status()==WL_CONNECTED){
    int r=WiFi.RSSI(); if(r<-127)r=-127; if(r>0)r=0;
    acc1m+=r;acc5m+=r;acc30m+=r;acc2h+=r;acc24h+=r;
    cnt1m++;cnt5m++;cnt30m++;cnt2h++;cnt24h++;
  }
  if(now-rssiT1m>=60000UL){rssiPush(rssiHour,60,rssiHourPos,rssiHourCount,rssiAvg(acc1m,cnt1m));acc1m=0;cnt1m=0;rssiT1m=now;}
  if(now-rssiT5m>=300000UL){rssiPush(rssiDay,288,rssiDayPos,rssiDayCount,rssiAvg(acc5m,cnt5m));acc5m=0;cnt5m=0;rssiT5m=now;}
  if(now-rssiT30m>=1800000UL){rssiPush(rssiWeek,336,rssiWeekPos,rssiWeekCount,rssiAvg(acc30m,cnt30m));acc30m=0;cnt30m=0;rssiT30m=now;}
  if(now-rssiT2h>=7200000UL){rssiPush(rssiMonth,360,rssiMonthPos,rssiMonthCount,rssiAvg(acc2h,cnt2h));acc2h=0;cnt2h=0;rssiT2h=now;}
  if(now-rssiT24h>=86400000UL){rssiPush(rssiYear,365,rssiYearPos,rssiYearCount,rssiAvg(acc24h,cnt24h));acc24h=0;cnt24h=0;rssiT24h=now;persistRssiYear();}
}
static String rssiHistoryJson(const int8_t *buf,uint16_t size,uint16_t pos,uint16_t count,const char *range){
  String j; j.reserve(2600); j="{\"range\":\"";j+=range;j+="\",\"values\":[";
  int minv=0,maxv=-127;long sum=0;uint16_t valid=0;uint16_t start=(count<size)?0:pos;
  for(uint16_t i=0;i<count;i++){uint16_t idx=(uint16_t)((start+i)%size);int8_t v=buf[idx];if(i)j+=',';if(v==RSSI_EMPTY)j+="null";else{j+=String((int)v);int iv=(int)v;if(!valid||iv<minv)minv=iv;if(!valid||iv>maxv)maxv=iv;sum+=iv;valid++;}}
  j+="],\"count\":"+String(count);
  if(valid)j+=",\"min\":"+String(minv)+",\"max\":"+String(maxv)+",\"avg\":"+String((float)sum/valid,1);
  else j+=",\"min\":null,\"max\":null,\"avg\":null";
  j+="}"; return j;
}

static void loadPrefs(){
  prefs.begin("g5500",true);
  wifiSsid=prefs.getString("ssid","");
  wifiPass=prefs.getString("pass","");
  apPass=prefs.getString("appass",AP_PASS);

  // v1.6.23 format 4: independent P36/P45 + explicit two-point endpoint flags.
  uint8_t calFmt=prefs.getUChar("calfmt",0);

  cal.az36_0=0; cal.az36Max=3100;
  cal.az45_0=0; cal.az45Max=3100;
  cal.el0=0; cal.elMax=3100;

  az36OSet=az36FSet=az45OSet=az45FSet=false;
  elOSet=elFSet=false;
  az36LinearValid=az45LinearValid=elLinearValid=false;

  for(uint8_t i=0;i<AZ36_CAL_N;i++) az36CalMv[i]=-1;
  for(uint8_t i=0;i<AZ45_CAL_N;i++) az45CalMv[i]=-1;
  for(uint8_t i=0;i<EL_CAL_N;i++) elCalMv[i]=-1;

  if(calFmt==4){
    cal.az36_0=prefs.getInt("a36z",0);      cal.az36Max=prefs.getInt("a36f",3100);
    cal.az45_0=prefs.getInt("a45z",0);      cal.az45Max=prefs.getInt("a45f",3100);
    cal.el0=prefs.getInt("el0mv",0);        cal.elMax=prefs.getInt("elmaxmv",3100);

    az36OSet=prefs.getBool("a36os",false);   az36FSet=prefs.getBool("a36fs",false);
    az45OSet=prefs.getBool("a45os",false);   az45FSet=prefs.getBool("a45fs",false);
    elOSet=prefs.getBool("elos",false);      elFSet=prefs.getBool("elfs",false);

    az36CalMv[0]=prefs.getInt("a36m0",-1);   az36CalMv[1]=prefs.getInt("a36m90",-1);
    az36CalMv[2]=prefs.getInt("a36m180",-1); az36CalMv[3]=prefs.getInt("a36m270",-1);
    az36CalMv[4]=prefs.getInt("a36m360",-1);

    az45CalMv[0]=prefs.getInt("a45m0",-1);   az45CalMv[1]=prefs.getInt("a45m90",-1);
    az45CalMv[2]=prefs.getInt("a45m180",-1); az45CalMv[3]=prefs.getInt("a45m270",-1);
    az45CalMv[4]=prefs.getInt("a45m360",-1); az45CalMv[5]=prefs.getInt("a45m450",-1);

    elCalMv[0]=prefs.getInt("elm0",-1);      elCalMv[1]=prefs.getInt("elm45",-1);
    elCalMv[2]=prefs.getInt("elm90",-1);     elCalMv[3]=prefs.getInt("elm135",-1);
    elCalMv[4]=prefs.getInt("elm180",-1);

  }else if(calFmt==3){
    // Migration from 1.5.58. Its per-endpoint state was unreliable.
    // Preserve measured 2-point values but force endpoints to be captured again.
    cal.az36_0=prefs.getInt("a36z",0);       cal.az36Max=prefs.getInt("a36f",3100);
    cal.az45_0=prefs.getInt("a45z",0);       cal.az45Max=prefs.getInt("a45f",3100);
    cal.el0=prefs.getInt("el0mv",0);         cal.elMax=prefs.getInt("elmaxmv",3100);

    // AZ MULTI was independent in 1.5.58 and is safe to preserve.
    az36CalMv[0]=prefs.getInt("a36m0",-1);   az36CalMv[1]=prefs.getInt("a36m90",-1);
    az36CalMv[2]=prefs.getInt("a36m180",-1); az36CalMv[3]=prefs.getInt("a36m270",-1);
    az36CalMv[4]=prefs.getInt("a36m360",-1);

    az45CalMv[0]=prefs.getInt("a45m0",-1);   az45CalMv[1]=prefs.getInt("a45m90",-1);
    az45CalMv[2]=prefs.getInt("a45m180",-1); az45CalMv[3]=prefs.getInt("a45m270",-1);
    az45CalMv[4]=prefs.getInt("a45m360",-1); az45CalMv[5]=prefs.getInt("a45m450",-1);

    // EL MULTI endpoints in 1.5.58 could be inherited from 2-point.
    // Keep only the unambiguous middle points; recapture 0° and 180°.
    elCalMv[0]=-1;
    elCalMv[1]=prefs.getInt("elm45",-1);
    elCalMv[2]=prefs.getInt("elm90",-1);
    elCalMv[3]=prefs.getInt("elm135",-1);
    elCalMv[4]=-1;

  }else if(calFmt==2){
    // Migration from the earlier single-AZ format with explicit flags when available.
    int oldAz0=prefs.getInt("az0mv",0);
    int oldAzMax=prefs.getInt("azmaxmv",3100);
    float oldMaxDeg=prefs.getFloat("az2maxdeg",450.0f);
    bool oldO=prefs.getBool("az2oset",false);
    bool oldF=prefs.getBool("az2fset",false);

    if(oldMaxDeg<400.0f){
      cal.az36_0=oldAz0; cal.az36Max=oldAzMax; az36OSet=oldO; az36FSet=oldF;
    }else{
      cal.az45_0=oldAz0; cal.az45Max=oldAzMax; az45OSet=oldO; az45FSet=oldF;
    }

    cal.el0=prefs.getInt("el0mv",0);
    cal.elMax=prefs.getInt("elmaxmv",3100);
    elOSet=prefs.getBool("el2oset",false);
    elFSet=prefs.getBool("el2fset",false);

    int oldAz[6];
    oldAz[0]=prefs.getInt("azm0",-1); oldAz[1]=prefs.getInt("azm90",-1);
    oldAz[2]=prefs.getInt("azm180",-1); oldAz[3]=prefs.getInt("azm270",-1);
    oldAz[4]=prefs.getInt("azm360",-1); oldAz[5]=prefs.getInt("azm450",-1);
    for(uint8_t i=0;i<AZ36_CAL_N;i++) az36CalMv[i]=oldAz[i];
    for(uint8_t i=0;i<AZ45_CAL_N;i++) az45CalMv[i]=oldAz[i];

    elCalMv[0]=prefs.getInt("elm0",-1);      elCalMv[1]=prefs.getInt("elm45",-1);
    elCalMv[2]=prefs.getInt("elm90",-1);     elCalMv[3]=prefs.getInt("elm135",-1);
    elCalMv[4]=prefs.getInt("elm180",-1);
  }

  uint8_t savedMode=prefs.getUChar("calmode",255);
  uint8_t savedAzMode=prefs.getUChar("azmode",45);
  prefs.end();

  az36LinearValid=(az36OSet && az36FSet && abs(cal.az36Max-cal.az36_0)>100);
  az45LinearValid=(az45OSet && az45FSet && abs(cal.az45Max-cal.az45_0)>100);
  elLinearValid=(elOSet && elFSet && abs(cal.elMax-cal.el0)>100);

  refreshMultiCalValidity();
  gsAzMode=(savedAzMode==36)?36:45;

  if(savedMode==CAL_GS232_2POINT || savedMode==CAL_MULTI_POINT){
    calibrationMode=(CalibrationMode)savedMode;
  }else{
    bool anyMulti=(az36MultiValid || az45MultiValid) && elMultiValid;
    calibrationMode=anyMulti ? CAL_MULTI_POINT : CAL_GS232_2POINT;
  }

  cal.valid=selectedCalibrationValid();
  if(calFmt!=4) saveCal();
}

static void saveCal(){
  az36LinearValid=(az36OSet && az36FSet && abs(cal.az36Max-cal.az36_0)>100);
  az45LinearValid=(az45OSet && az45FSet && abs(cal.az45Max-cal.az45_0)>100);
  elLinearValid=(elOSet && elFSet && abs(cal.elMax-cal.el0)>100);

  refreshMultiCalValidity();
  cal.valid=selectedCalibrationValid();

  prefs.begin("g5500",false);
  prefs.putUChar("calfmt",4);
  prefs.putUChar("calmode",(uint8_t)calibrationMode);
  prefs.putUChar("azmode",gsAzMode);

  prefs.putInt("a36z",cal.az36_0); prefs.putInt("a36f",cal.az36Max);
  prefs.putInt("a45z",cal.az45_0); prefs.putInt("a45f",cal.az45Max);
  prefs.putBool("a36os",az36OSet); prefs.putBool("a36fs",az36FSet);
  prefs.putBool("a45os",az45OSet); prefs.putBool("a45fs",az45FSet);

  prefs.putInt("el0mv",cal.el0); prefs.putInt("elmaxmv",cal.elMax);
  prefs.putBool("elos",elOSet); prefs.putBool("elfs",elFSet);

  prefs.putInt("a36m0",az36CalMv[0]); prefs.putInt("a36m90",az36CalMv[1]);
  prefs.putInt("a36m180",az36CalMv[2]); prefs.putInt("a36m270",az36CalMv[3]);
  prefs.putInt("a36m360",az36CalMv[4]);

  prefs.putInt("a45m0",az45CalMv[0]); prefs.putInt("a45m90",az45CalMv[1]);
  prefs.putInt("a45m180",az45CalMv[2]); prefs.putInt("a45m270",az45CalMv[3]);
  prefs.putInt("a45m360",az45CalMv[4]); prefs.putInt("a45m450",az45CalMv[5]);

  prefs.putInt("elm0",elCalMv[0]); prefs.putInt("elm45",elCalMv[1]);
  prefs.putInt("elm90",elCalMv[2]); prefs.putInt("elm135",elCalMv[3]);
  prefs.putInt("elm180",elCalMv[4]);
  prefs.end();
}

static void saveWifi(String s,String p){
  prefs.begin("g5500",false);prefs.putString("ssid",s);prefs.putString("pass",p);prefs.end();
}
static void saveApPassword(const String &p){
  prefs.begin("g5500",false);prefs.putString("appass",p);prefs.end();
}

static const char WEB_PAGE[] PROGMEM = R"HTML(
<!doctype html><html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>G5500 Controller v1.6.23 - UA1CFM</title>
<style>
:root{--bg:#081018;--p:#111d28;--p2:#162635;--t:#f4f7fa;--m:#96a9b7;--a:#24d18d;--s:#e74c3c;--b:#2b4557}
*{box-sizing:border-box}body{margin:0;background:linear-gradient(#081018,#0b1721);color:var(--t);font-family:Arial,sans-serif}
.wrap{max-width:980px;margin:auto;padding:16px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:14px}

.nav{display:flex;gap:8px;flex-wrap:wrap;margin:8px 0 12px;padding:8px;background:#0b1720;border:1px solid #294153;border-radius:12px;position:sticky;top:0;z-index:20}
.nav a{display:inline-block;padding:9px 12px;border-radius:9px;background:#173044;color:#fff;text-decoration:none;font-weight:700;font-size:14px}
.nav a.active{background:#167a5c}.nav a:hover{background:#235b78}
.card{background:var(--p);border:1px solid var(--b);border-radius:16px;padding:16px;margin-top:14px}
.big{font-size:54px;font-weight:700}.targetbig{font-size:34px;font-weight:700;color:var(--a)}.sep{font-size:30px;color:var(--m);font-weight:400}.unit{font-size:25px;color:var(--m)}.label{color:var(--m);font-size:13px;text-transform:uppercase;letter-spacing:.08em}
.bar{height:16px;border-radius:8px;background:#071018;overflow:hidden;margin-top:12px}.fill{height:100%;background:linear-gradient(90deg,#24d18d,#33b8ff)}
.controls{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;-webkit-user-select:none;user-select:none;-webkit-touch-callout:none}.btn{border:0;border-radius:14px;min-height:62px;background:#235b78;color:white;font-size:19px;font-weight:700;-webkit-user-select:none;user-select:none;-webkit-touch-callout:none;touch-action:none;-webkit-tap-highlight-color:transparent}.btn *{pointer-events:none;-webkit-user-select:none;user-select:none;-webkit-touch-callout:none}.movebtn{cursor:pointer}
.stop{background:var(--s);font-size:23px;min-height:78px}.good{background:#167a5c}.warn{background:#81541d}.blank{visibility:hidden}
.row{display:flex;gap:10px;flex-wrap:wrap}.field{flex:1;min-width:160px}input{width:100%;padding:12px;border-radius:10px;border:1px solid #355063;background:#07121a;color:white;font-size:16px}
.kv{display:grid;grid-template-columns:1fr 1fr;gap:8px}.kv div{background:#0b1720;padding:9px;border-radius:9px}
.status{display:inline-flex;align-items:center;gap:7px;padding:7px 10px;border-radius:999px;background:#0a151e;border:1px solid #294153;font-size:13px}
.dot{width:9px;height:9px;border-radius:50%;background:#697f8e}.dot.on{background:var(--a);box-shadow:0 0 10px #24d18d}
.rbtn{min-height:38px;font-size:13px;padding:7px 10px}.rbtn.active{outline:2px solid #24d18d}canvas{width:100%;height:260px;background:#071018;border-radius:10px;margin-top:10px}
.calhead{display:flex;align-items:center;justify-content:space-between;gap:12px;margin-top:10px}
.callive{font-size:34px;font-weight:700;white-space:nowrap}.callive span:last-child{font-size:18px;color:var(--m)}
.cal-toggle{width:100%;min-height:52px;background:#162635;border:1px solid #355063;border-radius:12px;color:var(--t);font-size:18px;font-weight:700;text-align:left;padding:0 14px;cursor:pointer}
.cal-toggle .arrow{float:right;transition:transform .18s ease}
.cal-toggle.open .arrow{transform:rotate(180deg)}
.cal-body{display:none;padding-top:10px}
.cal-body.open{display:block}
.btn.done{background:#1f7a46;border-color:#35b56e;color:#fff;box-shadow:0 0 10px rgba(36,209,141,.25)}
.calreset{background:#9b2c2c;min-height:46px;font-size:15px;padding:8px 12px}
.seg{display:grid;grid-template-columns:1fr 1fr;gap:6px;margin:8px 0}.seg .btn{min-height:42px;font-size:14px;border-radius:10px}.minirow{display:grid;grid-template-columns:repeat(3,1fr);gap:6px}.minirow .btn{min-height:42px;font-size:13px;border-radius:10px;padding:6px}.calbox{background:#0b1720;border:1px solid #294153;border-radius:12px;padding:10px;margin-top:8px}.calbox h3{margin:0 0 8px;font-size:15px}.calvalues{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;font-size:12px}.calvalues div{background:#071018;border-radius:8px;padding:7px}.hide{display:none!important}.rangebadge{display:inline-block;padding:4px 8px;border-radius:999px;background:#173044;color:#fff;font-size:12px;font-weight:700}
@media(max-width:700px){.grid{grid-template-columns:1fr}.big{font-size:46px}.targetbig{font-size:30px}.sep{font-size:26px}}
</style></head><body><div class="wrap">
<div style="font-size:13px;color:#96a9b7;margin:2px 0 6px">G5500 Controller v1.6.23 &copy; UA1CFM</div>
<h1>YAESU G-5500 / ESP32-S3</h1>
<nav class="nav">
<a class="active" href="/">УПРАВЛЕНИЕ</a>
<a href="/adc">ADC TEST</a>
<a href="/resetdiag">RESET DIAG</a>
<a href="/diaglog">DIAG CONSOLE</a>
</nav>
<div class="grid">
<div class="card"><div class="label">Азимут — текущее / задание</div><div><span id="az" class="big">--.-</span><span class="sep"> / </span><span id="azt" class="targetbig">---</span><span class="unit">°</span></div><div class="bar"><div id="azbar" class="fill"></div></div><div>0° <span id="azmaxlabel" style="float:right">450°</span></div></div>
<div class="card"><div class="label">Элевация — текущее / задание</div><div><span id="el" class="big">--.-</span><span class="sep"> / </span><span id="elt" class="targetbig">---</span><span class="unit">°</span></div><div class="bar"><div id="elbar" class="fill"></div></div><div>0° <span style="float:right">180°</span></div></div>
</div>
<div class="card">
<div style="display:flex;justify-content:space-between;gap:10px;flex-wrap:wrap">
<div><div class="label">Движение</div><div id="motion" style="font-size:23px;font-weight:700">STOP</div></div>
<div><span class="status"><span id="caldot" class="dot"></span><span id="caltxt">Calibration</span></span></div></div>
<div class="controls" style="margin-top:14px">
<button class="btn blank">-</button><button class="btn movebtn" data-dir="U"><span>▲ UP</span></button><button class="btn blank">-</button>
<button class="btn movebtn" data-dir="L"><span>◀ LEFT</span></button><button class="btn stop" onclick="st()">STOP</button><button class="btn movebtn" data-dir="R"><span>RIGHT ▶</span></button>
<button class="btn blank">-</button><button class="btn movebtn" data-dir="D"><span>▼ DOWN</span></button><button class="btn blank">-</button>
</div></div>
<div class="card"><div class="label">Цель</div><div class="row" style="margin-top:10px">
<div class="field"><label id="tazlbl">AZ 0..450°</label><input id="taz" type="number" step="0.1" min="0" max="450"></div>
<div class="field"><label>EL 0..180°</label><input id="tel" type="number" step="0.1" min="0" max="180"></div>
<button class="btn good" onclick="go()">GO</button></div></div>
<div class="card">
<button id="caltoggle" class="cal-toggle" onclick="toggleCalMenu()">Калибровка <span class="arrow">▼</span></button>
<div id="calbody" class="cal-body">
<div class="label">АЛГОРИТМ</div>
<div class="seg">
<button id="mode2" class="btn" onclick="setCalMode('2')">GS-232 2-POINT</button>
<button id="modem" class="btn" onclick="setCalMode('multi')">MULTI-POINT</button>
</div>
<div class="label">ДИАПАЗОН АЗИМУТА — таблицы P36/P45 независимы</div>
<div class="seg">
<button id="azm36" class="btn" onclick="setAzMode(36)">P36 · 360°</button>
<button id="azm45" class="btn" onclick="setAzMode(45)">P45 · 450°</button>
</div>
<div style="display:flex;justify-content:space-between;gap:8px;align-items:center;margin:6px 0 10px">
<span>Активно: <b id="calmode">-</b></span><span id="azrange" class="rangebadge">P--</span>
</div>

<div id="panel2" class="calbox">
<h3>GS-232 2-POINT</h3>
<div class="muted" style="font-size:12px;margin-bottom:8px">AZ: O = 0°, F = конец выбранного диапазона. EL: O2 = 0°, F2 = 180°.</div>
<div class="minirow">
<button id="b2_azo" class="btn warn" onclick="cal2('o')">O · AZ 0°</button>
<button id="b2_azf" class="btn warn" onclick="cal2('f')">F · AZ FULL</button>
<button class="btn calreset" onclick="resetCal('az','2')">RESET AZ</button>
<button id="b2_elo" class="btn warn" onclick="cal2('o2')">O2 · EL 0°</button>
<button id="b2_elf" class="btn warn" onclick="cal2('f2')">F2 · EL 180°</button>
<button class="btn calreset" onclick="resetCal('el','2')">RESET EL</button>
</div>
<div class="calvalues" style="margin-top:8px">
<div>AZ 0<br><b id="az2p0">-</b> mV</div>
<div>AZ FULL<br><b id="az2pf">-</b> mV</div>
<div>EL 0 / 180<br><b id="el2p0">-</b> / <b id="el2pf">-</b> mV</div>
</div>
</div>

<div id="panelm" class="calbox hide">
<div class="calhead" style="margin-top:0"><h3 style="margin:0">MULTI-POINT AZ</h3><div class="callive"><span id="calaz">--.-</span><span>°</span></div></div>
<div class="minirow" style="margin-top:8px">
<button id="b_az0" class="btn warn" onclick="cal('az0')">0°</button>
<button id="b_az90" class="btn warn" onclick="cal('az90')">90°</button>
<button id="b_az180" class="btn warn" onclick="cal('az180')">180°</button>
<button id="b_az270" class="btn warn" onclick="cal('az270')">270°</button>
<button id="b_az360" class="btn warn" onclick="cal('az360')">360°</button>
<button id="b_az450" class="btn warn" onclick="cal('az450')">450°</button>
</div>
<div class="calvalues" style="margin-top:8px">
<div>0° <b id="azp0">-</b></div><div>90° <b id="azp90">-</b></div><div>180° <b id="azp180">-</b></div>
<div>270° <b id="azp270">-</b></div><div>360° <b id="azp360">-</b></div><div id="azv450">450° <b id="azp450">-</b></div>
</div>
<button class="btn calreset" style="width:100%;margin-top:8px" onclick="resetCal('az','multi')">RESET AZ <span id="resetazrange">P--</span></button>

<div class="calhead" style="margin-top:12px"><h3 style="margin:0">MULTI-POINT EL · общая</h3><div class="callive"><span id="calel">--.-</span><span>°</span></div></div>
<div class="minirow" style="margin-top:8px">
<button id="b_el0" class="btn warn" onclick="cal('el0')">0°</button>
<button id="b_el45" class="btn warn" onclick="cal('el45')">45°</button>
<button id="b_el90" class="btn warn" onclick="cal('el90')">90°</button>
<button id="b_el135" class="btn warn" onclick="cal('el135')">135°</button>
<button id="b_el180" class="btn warn" onclick="cal('el180')">180°</button>
<button class="btn calreset" onclick="resetCal('el','multi')">RESET EL</button>
</div>
<div class="calvalues" style="margin-top:8px">
<div>0° <b id="elp0">-</b></div><div>45° <b id="elp45">-</b></div><div>90° <b id="elp90">-</b></div>
<div>135° <b id="elp135">-</b></div><div>180° <b id="elp180">-</b></div>
</div>
</div>
</div></div>
<div class="card"><div class="label">История RSSI домашнего Wi-Fi</div>
<div class="row" style="margin-top:10px"><button class="btn rbtn active" data-r="hour">1 HOUR</button><button class="btn rbtn" data-r="day">1 DAY</button><button class="btn rbtn" data-r="week">1 WEEK</button><button class="btn rbtn" data-r="month">1 MONTH</button><button class="btn rbtn" data-r="year">1 YEAR</button></div>
<canvas id="rchart" width="900" height="260"></canvas>
<div class="kv" style="margin-top:10px"><div>MIN: <b id="rmin">-</b></div><div>MAX: <b id="rmax">-</b></div><div>AVG: <b id="ravg">-</b></div><div>Points: <b id="rcount">0</b></div></div></div>
<div class="card"><div class="label">Подключение</div><div class="kv" style="margin-top:10px">
<div>Home IP: <b id="ip">-</b></div><div>AP IP: <b>192.168.4.1</b></div><div>TCP rotctld: <b>4533</b></div><div>Mode: <b id="mode">-</b></div><div>Home Wi-Fi RSSI: <b id="rssi">-</b></div><div>Signal: <b id="sig">-</b></div><div>HC-05 Bluetooth: <b id="btstate">-</b></div><div>HC-05 UART: <b>9600 baud</b></div></div>
<div class="field" style="margin-top:10px"><label>Wi‑Fi SSID</label><input id="ssid"></div>
<div class="field" style="margin-top:8px"><label>Wi‑Fi password</label><input id="pass" type="password"></div>
<button class="btn good" style="margin-top:10px" onclick="wifi()">Save Wi‑Fi + reboot</button>
<div class="field" style="margin-top:14px"><label>Access Point password (8..63 chars; empty = open AP)</label><input id="appass" type="password" minlength="8" maxlength="63" placeholder="leave empty for open AP"></div>
<button class="btn good" style="margin-top:10px" onclick="saveAp()">Save AP password + reboot</button></div>
</div>
<script>
async function q(u){try{return await (await fetch(u,{cache:'no-store'})).text()}catch(e){return''}}
async function mv(d){await q('/api/move?d='+d)}
async function st(){await q('/api/move?d=S')}
async function stopAxis(d){
  if(d==='L'||d==='R') await q('/api/move?d=A');
  else if(d==='U'||d==='D') await q('/api/move?d=E');
}
async function go(){let a=taz.value,e=tel.value;if(a!==''&&e!=='')await q('/api/target?az='+encodeURIComponent(a)+'&el='+encodeURIComponent(e))}
function toggleCalMenu(){
  const body=document.getElementById('calbody');
  const btn=document.getElementById('caltoggle');
  const open=!body.classList.contains('open');
  body.classList.toggle('open',open);
  btn.classList.toggle('open',open);
}
async function cal(p){await q('/api/cal?p='+p);setTimeout(upd,150)}
async function cal2(p){await q('/api/cal2?p='+p);setTimeout(upd,150)}
async function setCalMode(m){await q('/api/calmode?m='+m);setTimeout(upd,150)}
async function setAzMode(m){await q('/api/azmode?m='+m);setTimeout(upd,150)}
async function resetCal(axis,cm){
  const name=axis==='az'?'АЗИМУТА':'ЭЛЕВАЦИИ';
  const mn=cm==='2'?'GS-232 2-POINT':'MULTI-POINT';
  const rr=axis==='az'?' '+(window._azMode===36?'P36':'P45'):'';
  if(!confirm('Сбросить '+mn+' калибровку '+name+rr+'?')) return;
  await q('/api/calreset?axis='+axis+'&mode='+cm);
  setTimeout(upd,150);
}
function markCalBtn(id, ok){
  const b=document.getElementById(id);
  if(!b) return;
  b.classList.toggle('done', !!ok);
  b.classList.toggle('warn', !ok);
}
async function wifi(){if(!ssid.value)return;if(confirm('Сохранить Wi‑Fi и перезагрузить контроллер?'))await q('/api/wifi?ssid='+encodeURIComponent(ssid.value)+'&pass='+encodeURIComponent(pass.value))}
async function saveAp(){
  const p=appass.value;
  if(p.length>0 && p.length<8){alert('Пароль AP должен быть минимум 8 символов или пустым для открытой точки доступа.');return;}
  if(confirm(p.length?'Сохранить пароль точки доступа и перезагрузить контроллер?':'Сделать точку доступа открытой и перезагрузить контроллер?'))
    await q('/api/ap?pass='+encodeURIComponent(p));
}
function pct(v,m){v=Math.max(0,Math.min(m,v));return(v/m*100).toFixed(1)+'%'}
async function upd(){try{let j=await (await fetch('/api/status',{cache:'no-store'})).json();
az.textContent=Number(j.az).toFixed(1);el.textContent=Number(j.el).toFixed(1);
azt.textContent=j.targetValid?Number(j.targetAz).toFixed(1):'---';
elt.textContent=j.targetValid?Number(j.targetEl).toFixed(1):'---';
calaz.textContent=Number(j.az).toFixed(1);calel.textContent=Number(j.el).toFixed(1);
const azmx=j.azMode===36?360:450;azbar.style.width=pct(j.az,azmx);azmaxlabel.textContent=azmx+'°';taz.max=azmx;tazlbl.textContent='AZ 0..'+azmx+'°';elbar.style.width=pct(j.el,180);
motion.textContent=j.motion;
window._azMode=Number(j.azMode)||45;
const selok=!!j.selectedCalOk;
caltxt.textContent=(j.calMode==='multi'?'MULTI-POINT':'GS-232 2-POINT')+' · P'+j.azMode+(selok?' OK':' NOT CALIBRATED');
caldot.className='dot '+(selok?'on':'');
calmode.textContent=j.calMode==='multi'?'MULTI-POINT':'GS-232 2-POINT';
mode2.classList.toggle('good',j.calMode==='2');modem.classList.toggle('good',j.calMode==='multi');
azm36.classList.toggle('good',j.azMode===36);azm45.classList.toggle('good',j.azMode===45);
azrange.textContent='P'+j.azMode+' · '+(j.azMode===36?'360°':'450°');
resetazrange.textContent='P'+j.azMode;
panel2.classList.toggle('hide',j.calMode!=='2');panelm.classList.toggle('hide',j.calMode!=='multi');
b_az450.classList.toggle('hide',j.azMode===36);azv450.classList.toggle('hide',j.azMode===36);
az2p0.textContent=j.az2p0;az2pf.textContent=j.az2pf;el2p0.textContent=j.el2p0;el2pf.textContent=j.el2pf;
markCalBtn('b2_azo',j.az2p0>=0);markCalBtn('b2_azf',j.az2pf>=0);
markCalBtn('b2_elo',j.el2p0>=0);markCalBtn('b2_elf',j.el2pf>=0);
azp0.textContent=j.azp0;azp90.textContent=j.azp90;azp180.textContent=j.azp180;azp270.textContent=j.azp270;azp360.textContent=j.azp360;azp450.textContent=j.azp450;
elp0.textContent=j.elp0;elp45.textContent=j.elp45;elp90.textContent=j.elp90;elp135.textContent=j.elp135;elp180.textContent=j.elp180;
markCalBtn('b_az0',j.azp0>=0);markCalBtn('b_az90',j.azp90>=0);markCalBtn('b_az180',j.azp180>=0);
markCalBtn('b_az270',j.azp270>=0);markCalBtn('b_az360',j.azp360>=0);markCalBtn('b_az450',j.azMode===45&&j.azp450>=0);
markCalBtn('b_el0',j.elp0>=0);markCalBtn('b_el45',j.elp45>=0);markCalBtn('b_el90',j.elp90>=0);
markCalBtn('b_el135',j.elp135>=0);markCalBtn('b_el180',j.elp180>=0);
ip.textContent=j.ip;mode.textContent=j.wifiMode;
rssi.textContent=(j.rssi===null?'-':j.rssi+' dBm');
sig.textContent=j.signal||'-';
btstate.textContent=j.btConnected?'CONNECTED':'DISCONNECTED';
btstate.style.color=j.btConnected?'#24d18d':'#e5a94f';
if(document.activeElement!==ssid && !ssid.dataset.dirty) ssid.value=j.ssid||'';
}catch(e){}}
let rrange='hour';
function drawRssi(j){
 const c=document.getElementById('rchart'),x=c.getContext('2d'),w=c.width,h=c.height;
 x.clearRect(0,0,w,h);x.fillStyle='#071018';x.fillRect(0,0,w,h);x.strokeStyle='#294153';x.lineWidth=1;x.font='12px Arial';x.fillStyle='#96a9b7';
 const top=15,bottom=h-28,left=46,right=w-12;
 for(let db=-30;db>=-100;db-=10){let y=top+(-30-db)/70*(bottom-top);x.beginPath();x.moveTo(left,y);x.lineTo(right,y);x.stroke();x.fillText(db+' dBm',3,y+4);}
 let a=j.values||[],n=a.length;x.strokeStyle='#24d18d';x.lineWidth=2;x.beginPath();let pen=false;
 for(let i=0;i<n;i++){let v=a[i];if(v===null){pen=false;continue}let px=left+(n<=1?0:i/(n-1))*(right-left);let py=top+(-30-Math.max(-100,Math.min(-30,v)))/70*(bottom-top);if(!pen){x.moveTo(px,py);pen=true}else x.lineTo(px,py);}x.stroke();
 x.fillStyle='#96a9b7';x.fillText('oldest',left,bottom+18);x.fillText('now',right-24,bottom+18);
 rmin.textContent=j.min===null?'-':j.min+' dBm';rmax.textContent=j.max===null?'-':j.max+' dBm';ravg.textContent=j.avg===null?'-':j.avg+' dBm';rcount.textContent=j.count||0;
}
async function loadRssi(){try{let j=await (await fetch('/api/rssi?range='+rrange,{cache:'no-store'})).json();drawRssi(j)}catch(e){}}
document.querySelectorAll('.rbtn').forEach(b=>b.addEventListener('click',()=>{rrange=b.dataset.r;document.querySelectorAll('.rbtn').forEach(q=>q.classList.remove('active'));b.classList.add('active');loadRssi();}));
setInterval(loadRssi,30000);loadRssi();
setInterval(upd,500);upd();

ssid.addEventListener('input',function(){ssid.dataset.dirty='1';});

document.querySelectorAll('.btn').forEach(function(b){
  b.addEventListener('contextmenu',function(e){e.preventDefault();});
  b.addEventListener('dragstart',function(e){e.preventDefault();});
  b.addEventListener('selectstart',function(e){e.preventDefault();});
});

document.querySelectorAll('.movebtn').forEach(function(b){
  let active=false;

  b.addEventListener('touchstart',function(e){
    e.preventDefault();
  },{passive:false});

  b.addEventListener('pointerdown',function(e){
    e.preventDefault();
    active=true;
    try{ b.setPointerCapture(e.pointerId); }catch(x){}
    mv(b.dataset.dir);
  });

  function release(e){
    if(e) e.preventDefault();
    if(active){
      active=false;
      stopAxis(b.dataset.dir);
    }
  }

  b.addEventListener('pointerup',release);
  b.addEventListener('pointercancel',release);
  b.addEventListener('lostpointercapture',release);
  b.addEventListener('contextmenu',function(e){e.preventDefault();});
});
</script></body></html>
)HTML";


static const char ADC_TEST_PAGE[] PROGMEM = R"HTML(
<!doctype html><html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>G5500 ADC TEST v1.6.23</title>
<style>
:root{--bg:#081018;--p:#111d28;--t:#f4f7fa;--m:#96a9b7;--a:#24d18d;--b:#2b4557;--az:#33b8ff;--el:#ffb347}
*{box-sizing:border-box}body{margin:0;background:linear-gradient(#081018,#0b1721);color:var(--t);font-family:Arial,sans-serif}
.wrap{max-width:980px;margin:auto;padding:16px}.card{background:var(--p);border:1px solid var(--b);border-radius:16px;padding:16px;margin-top:14px}
h1{margin:.25em 0}.muted{color:var(--m)}.grid{display:grid;grid-template-columns:1fr 1fr;gap:14px}
.kv{display:grid;grid-template-columns:1fr 1fr;gap:8px}.kv div{background:#0b1720;padding:10px;border-radius:9px}
.val{font-size:25px;font-weight:700}.az{color:var(--az)}.el{color:var(--el)}
.btn{border:0;border-radius:12px;min-height:42px;padding:8px 14px;background:#235b78;color:white;font-size:15px;font-weight:700;cursor:pointer}
.good{background:#167a5c}.warn{background:#81541d}
canvas{width:100%;height:360px;background:#071018;border-radius:10px;margin-top:10px}
a{color:#24d18d;text-decoration:none;font-weight:700}
@media(max-width:700px){.grid{grid-template-columns:1fr}.kv{grid-template-columns:1fr}}
.nav{display:flex;gap:8px;flex-wrap:wrap;margin:8px 0 14px;padding:8px;background:#0b1720;border:1px solid #294153;border-radius:12px;position:sticky;top:0;z-index:20}.nav a{display:inline-block;padding:9px 12px;border-radius:9px;background:#173044;color:#fff;text-decoration:none;font-weight:700;font-size:14px}.nav a.active{background:#167a5c}.nav a:hover{background:#235b78}\n</style></head><body><div class="wrap">
<div class="muted" style="font-size:13px">G5500 Controller v1.6.23 &copy; UA1CFM</div>
<h1>ADC TEST — ESP32-S3</h1>
<nav class="nav"><a href="/">УПРАВЛЕНИЕ</a><a class="active" href="/adc">ADC TEST</a><a href="/resetdiag">RESET DIAG</a><a href="/diaglog">DIAG CONSOLE</a></nav>

<div class="card">
<div><b>Назначение:</b> диагностический замер ADC. Он не меняет калибровку AZ/EL и не управляет реле.</div>
<div class="muted" style="margin-top:7px">RAW — обычный 12-битный analogRead(). CAL mV — калиброванное значение analogReadMilliVolts(). LINEAR mV — условная идеальная прямая 0…3100 mV для ADC_11db. Δ показывает отличие заводской калиброванной шкалы от этой прямой, а не абсолютную погрешность без внешнего эталонного вольтметра.</div>
<div style="margin-top:10px"><button id="collectBtn" class="btn good" onclick="toggleCollect()">AUTO COLLECT: ON</button> <button class="btn warn" onclick="clearData()">CLEAR GRAPH</button></div>
</div>

<div class="grid">
<div class="card"><div class="az"><b>AZ — GPIO4</b></div><div class="kv" style="margin-top:10px">
<div>RAW ADC<br><span id="azraw" class="val az">-</span></div>
<div>CAL mV<br><span id="azmv" class="val az">-</span></div>
<div>LINEAR mV<br><span id="azlin" class="val">-</span></div>
<div>Δ mV<br><span id="azdmv" class="val">-</span></div>
<div>Δ %<br><span id="azpct" class="val">-</span></div>
<div>Samples<br><span id="azn" class="val">0</span></div>
</div></div>
<div class="card"><div class="el"><b>EL — GPIO5</b></div><div class="kv" style="margin-top:10px">
<div>RAW ADC<br><span id="elraw" class="val el">-</span></div>
<div>CAL mV<br><span id="elmv" class="val el">-</span></div>
<div>LINEAR mV<br><span id="ellin" class="val">-</span></div>
<div>Δ mV<br><span id="eldmv" class="val">-</span></div>
<div>Δ %<br><span id="elpct" class="val">-</span></div>
<div>Samples<br><span id="eln" class="val">0</span></div>
</div></div>
</div>

<div class="card">
<div><b>График RAW ADC → напряжение</b></div>
<div class="muted">Синяя линия/точки — AZ, оранжевая — EL, серая — условная линейная шкала 0…3100 mV. Для заполнения всей кривой медленно проведите соответствующую ось по диапазону вручную; страница автоматически набирает точки.</div>
<canvas id="chart" width="920" height="360"></canvas>
</div>
</div>
<script>
let collect=true, azPts=[], elPts=[], MAXPTS=900;
function toggleCollect(){collect=!collect;collectBtn.textContent='AUTO COLLECT: '+(collect?'ON':'OFF');collectBtn.className='btn '+(collect?'good':'warn')}
function clearData(){azPts=[];elPts=[];draw();azn.textContent='0';eln.textContent='0'}
function addPoint(a,raw,mv){
  if(!collect)return;
  let p={x:Number(raw),y:Number(mv)};
  if(a.length){
    let q=a[a.length-1];
    if(Math.abs(q.x-p.x)<2 && Math.abs(q.y-p.y)<2)return;
  }
  a.push(p);
  if(a.length>MAXPTS)a.shift();
}
function fmt(v,n=0){return Number(v).toFixed(n)}
function updateAxis(prefix,raw,mv,arr){
  let lin=raw*3100/4095, d=mv-lin, pct=lin>50?100*d/lin:0;
  document.getElementById(prefix+'raw').textContent=raw;
  document.getElementById(prefix+'mv').textContent=mv;
  document.getElementById(prefix+'lin').textContent=fmt(lin,1);
  document.getElementById(prefix+'dmv').textContent=(d>=0?'+':'')+fmt(d,1);
  document.getElementById(prefix+'pct').textContent=(pct>=0?'+':'')+fmt(pct,2)+'%';
  document.getElementById(prefix+'n').textContent=arr.length;
}
function draw(){
  const c=chart,x=c.getContext('2d'),w=c.width,h=c.height,left=58,right=w-18,top=18,bottom=h-38;
  x.clearRect(0,0,w,h);x.fillStyle='#071018';x.fillRect(0,0,w,h);
  x.strokeStyle='#294153';x.fillStyle='#96a9b7';x.font='12px Arial';x.lineWidth=1;
  for(let r=0;r<=4095;r+=512){let px=left+r/4095*(right-left);x.beginPath();x.moveTo(px,top);x.lineTo(px,bottom);x.stroke();if(r<4095)x.fillText(r,px-12,bottom+18)}
  for(let mv=0;mv<=3100;mv+=500){let py=bottom-mv/3100*(bottom-top);x.beginPath();x.moveTo(left,py);x.lineTo(right,py);x.stroke();x.fillText(mv+'mV',3,py+4)}
  x.fillText('4095',right-28,bottom+18);
  // Linear reference
  x.strokeStyle='#788a96';x.setLineDash([6,6]);x.beginPath();x.moveTo(left,bottom);x.lineTo(right,top);x.stroke();x.setLineDash([]);
  function series(a,color){
    if(!a.length)return;
    let s=a.slice().sort((p,q)=>p.x-q.x);
    x.strokeStyle=color;x.fillStyle=color;x.lineWidth=2;x.beginPath();
    s.forEach((p,i)=>{let px=left+p.x/4095*(right-left),py=bottom-Math.max(0,Math.min(3100,p.y))/3100*(bottom-top);if(i)x.lineTo(px,py);else x.moveTo(px,py)});
    x.stroke();
    for(let i=0;i<s.length;i+=Math.max(1,Math.floor(s.length/180))){let p=s[i],px=left+p.x/4095*(right-left),py=bottom-Math.max(0,Math.min(3100,p.y))/3100*(bottom-top);x.fillRect(px-1.5,py-1.5,3,3)}
  }
  series(azPts,'#33b8ff');series(elPts,'#ffb347');
  x.fillStyle='#33b8ff';x.fillText('AZ',right-95,top+14);x.fillStyle='#ffb347';x.fillText('EL',right-60,top+14);x.fillStyle='#96a9b7';x.fillText('RAW ADC',right-80,bottom+32);
}
async function upd(){
  try{
    let j=await (await fetch('/api/adcdiag',{cache:'no-store'})).json();
    addPoint(azPts,j.azRaw,j.azMv); addPoint(elPts,j.elRaw,j.elMv);
    updateAxis('az',j.azRaw,j.azMv,azPts); updateAxis('el',j.elRaw,j.elMv,elPts);
    draw();
  }catch(e){}
}
setInterval(upd,350);upd();draw();
</script></body></html>
)HTML";



static const char RESET_DIAG_PAGE[] PROGMEM = R"HTML(
<!doctype html><html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>G5500 RESET DIAG v1.6.23</title>
<style>
:root{--bg:#081018;--p:#111d28;--t:#f4f7fa;--m:#96a9b7;--a:#24d18d;--b:#2b4557;--w:#e5a94f;--red:#d9534f}
*{box-sizing:border-box}body{margin:0;background:linear-gradient(#081018,#0b1721);color:var(--t);font-family:Arial,sans-serif}
.wrap{max-width:920px;margin:auto;padding:16px}.card{background:var(--p);border:1px solid var(--b);border-radius:16px;padding:16px;margin-top:14px}
h1{margin:.25em 0}.muted{color:var(--m)}a{color:var(--a);text-decoration:none;font-weight:700}
.reason{font-size:38px;font-weight:800;color:var(--a);margin-top:8px}
.code{font-size:20px;font-weight:700}.kv{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:10px}
.kv div{background:#0b1720;padding:10px;border-radius:9px}
.note{line-height:1.45}.warn{color:var(--w)}
table{width:100%;border-collapse:collapse;margin-top:10px}
th,td{padding:9px 7px;border-bottom:1px solid #294153;text-align:left;font-size:14px}
th{color:var(--m);font-weight:700}
.btn{border:0;border-radius:9px;padding:10px 14px;font-weight:700;cursor:pointer}
.danger{background:#8f2e2b;color:#fff}
@media(max-width:650px){.kv{grid-template-columns:1fr}.reason{font-size:32px}th,td{font-size:12px;padding:7px 4px}}
.nav{display:flex;gap:8px;flex-wrap:wrap;margin:8px 0 14px;padding:8px;background:#0b1720;border:1px solid #294153;border-radius:12px;position:sticky;top:0;z-index:20}.nav a{display:inline-block;padding:9px 12px;border-radius:9px;background:#173044;color:#fff;text-decoration:none;font-weight:700;font-size:14px}.nav a.active{background:#167a5c}.nav a:hover{background:#235b78}\n</style></head><body><div class="wrap">
<div class="muted" style="font-size:13px">G5500 Controller v1.6.23 &copy; UA1CFM</div>
<h1>RESET DIAG</h1>
<nav class="nav"><a href="/">УПРАВЛЕНИЕ</a><a href="/adc">ADC TEST</a><a class="active" href="/resetdiag">RESET DIAG</a><a href="/diaglog">DIAG CONSOLE</a></nav>

<div class="card">
<div class="muted">Причина последней загрузки ESP32-S3</div>
<div id="reason" class="reason">--</div>
<div class="kv">
<div>Reset code<br><span id="code" class="code">--</span></div>
<div>Обновление<br><span id="stamp" class="code">--</span></div>
</div>
</div>

<div class="card">
<div style="display:flex;align-items:center;justify-content:space-between;gap:12px;flex-wrap:wrap">
  <div>
    <b>История перезагрузок</b>
    <div class="muted">Последние 20 запусков. Время хранится как UTC и показывается браузером в вашем местном часовом поясе.</div>
  </div>
  <button class="btn danger" onclick="clearHist()">ОЧИСТИТЬ ИСТОРИЮ</button>
</div>
<div style="overflow-x:auto">
<table>
<thead><tr><th>#</th><th>Дата и время</th><th>Причина</th><th>Код</th></tr></thead>
<tbody id="hist"><tr><td colspan="4" class="muted">Загрузка...</td></tr></tbody>
</table>
</div>
<div id="timeNote" class="muted" style="margin-top:8px"></div>
</div>

<div class="card note">
<b>Расшифровка:</b><br><br>
<b>POWERON</b> — обычное включение питания.<br>
<b>BROWNOUT</b> — просадка питания ESP32.<br>
<b>EXTERNAL</b> — внешний аппаратный reset.<br>
<b>SOFTWARE</b> — программный перезапуск.<br>
<b>PANIC</b> — аварийное исключение программы.<br>
<b>INT_WDT / TASK_WDT / WDT</b> — сработал watchdog.<br>
<b>DEEPSLEEP</b> — выход из deep sleep.<br><br>
<span class="warn">Если домашний Wi‑Fi недоступен при загрузке, NTP-время получить нельзя — такая запись будет показана как «Время не синхронизировано».</span>
</div>
</div>
<script>
async function upd(){
  try{
    let j=await (await fetch('/api/resetreason',{cache:'no-store'})).json();
    reason.textContent=j.reason||'UNKNOWN';
    code.textContent=j.code;
    stamp.textContent=new Date().toLocaleTimeString();
  }catch(e){reason.textContent='NO CONNECTION'}
}
async function histUpd(){
  try{
    let j=await (await fetch('/api/resethistory',{cache:'no-store'})).json();
    let h='';
    let unsynced=0;
    (j.entries||[]).forEach((e,i)=>{
      let when='Время не синхронизировано';
      if(Number(e.epoch)>0) when=new Date(Number(e.epoch)*1000).toLocaleString();
      else unsynced++;
      h+='<tr><td>'+(i+1)+'</td><td>'+when+'</td><td><b>'+e.reason+'</b></td><td>'+e.code+'</td></tr>';
    });
    hist.innerHTML=h||'<tr><td colspan="4" class="muted">История пуста</td></tr>';
    timeNote.textContent=unsynced?('Без точного времени: '+unsynced+' записей'):'';
  }catch(e){hist.innerHTML='<tr><td colspan="4">Ошибка загрузки истории</td></tr>'}
}
async function clearHist(){
  if(!confirm('Очистить всю историю перезагрузок?')) return;
  await fetch('/api/resethistory/clear',{cache:'no-store'});
  histUpd();
}
setInterval(upd,1000);upd();histUpd();
</script></body></html>
)HTML";


static const char DIAG_PAGE[] PROGMEM = R"HTML(
<!doctype html><html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>G5500 Diagnostic Console v1.6.23</title>
<style>
:root{--bg:#071018;--border:#294153;--text:#d8f7df;--muted:#91a5b2}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:white;font-family:Arial,sans-serif}
.wrap{max-width:1200px;margin:auto;padding:14px}
.top{display:flex;gap:10px;align-items:center;flex-wrap:wrap;margin-bottom:10px}
h2{margin:0 14px 0 0;font-size:21px}
button,a.btn{border:1px solid var(--border);background:#173044;color:white;border-radius:9px;padding:9px 13px;font-weight:700;text-decoration:none;cursor:pointer}
button.good{background:#126b50}
button.danger{background:#a93d3d}
label{color:var(--muted);font-size:14px;display:flex;gap:7px;align-items:center}
#state{color:var(--muted);margin-left:auto;font-size:13px}
#term{height:calc(100vh - 92px);min-height:420px;overflow:auto;white-space:pre-wrap;word-break:break-word;background:#020608;color:var(--text);border:1px solid var(--border);border-radius:10px;padding:12px;font:14px/1.42 Consolas,"Courier New",monospace}
@media(max-width:700px){#state{width:100%;margin-left:0}#term{height:calc(100vh - 140px);font-size:12px}}
.nav{display:flex;gap:8px;flex-wrap:wrap;margin:8px 0 14px;padding:8px;background:#0b1720;border:1px solid #294153;border-radius:12px;position:sticky;top:0;z-index:20}.nav a{display:inline-block;padding:9px 12px;border-radius:9px;background:#173044;color:#fff;text-decoration:none;font-weight:700;font-size:14px}.nav a.active{background:#167a5c}.nav a:hover{background:#235b78}\n</style></head><body><div class="wrap">
<div class="top">
<h2>G5500 Web Diagnostic Console</h2>
<a class="btn" href="/">УПРАВЛЕНИЕ</a>
<a class="btn" href="/adc">ADC TEST</a>
<a class="btn" href="/resetdiag">RESET DIAG</a>
<a class="btn" href="/diaglog" style="background:#167a5c">DIAG CONSOLE</a>
<button class="good" onclick="refreshNow()">Refresh</button>
<button class="danger" onclick="clearLog()">Clear RAM log</button>
<label><input id="auto" type="checkbox" checked> Auto scroll</label>
<label><input id="pause" type="checkbox"> Pause</label>
<span id="state">connecting...</span>
</div>
<pre id="term">Loading...</pre>
</div>
<script>
let last='';
let prevRawLines=[];
let stampedLines=[];

function pcStamp(){
  const d=new Date();
  const p=n=>String(n).padStart(2,'0');
  const ms=String(d.getMilliseconds()).padStart(3,'0');
  return '['+p(d.getHours())+':'+p(d.getMinutes())+':'+p(d.getSeconds())+'.'+ms+']';
}

function stampSnapshot(raw){
  if(!raw)return '';

  let lines=raw.replace(/\r/g,'').split('\n');
  if(lines.length && lines[lines.length-1]==='')lines.pop();

  // Preserve time labels for lines still present after the ESP32 RAM ring-buffer shifts.
  let overlap=0;
  const max=Math.min(prevRawLines.length,lines.length);
  for(let k=max;k>=0;k--){
    let ok=true;
    for(let i=0;i<k;i++){
      if(prevRawLines[prevRawLines.length-k+i]!==lines[i]){ok=false;break;}
    }
    if(ok){overlap=k;break;}
  }

  const nextStamped=[];
  for(let i=0;i<overlap;i++){
    nextStamped.push(stampedLines[stampedLines.length-overlap+i]);
  }
  for(let i=overlap;i<lines.length;i++){
    nextStamped.push(pcStamp()+' | '+lines[i]);
  }

  prevRawLines=lines;
  stampedLines=nextStamped;
  return nextStamped.join('\n');
}

async function loadLog(){
  if(pause.checked)return;
  try{
    const r=await fetch('/api/diaglog',{cache:'no-store'});
    if(!r.ok)throw new Error('HTTP '+r.status);
    const t=await r.text();
    if(t!==last){
      const nearBottom=(term.scrollHeight-term.scrollTop-term.clientHeight)<80;
      term.textContent=t ? stampSnapshot(t) : '(RAM diagnostic log is empty)';
      if(auto.checked && nearBottom)term.scrollTop=term.scrollHeight;
      last=t;
    }
    state.textContent='LIVE • PC '+new Date().toLocaleTimeString();
  }catch(e){
    state.textContent='ERROR: '+e.message;
  }
}
async function refreshNow(){await loadLog();if(auto.checked)term.scrollTop=term.scrollHeight}
async function clearLog(){
  if(!confirm('Очистить только диагностический RAM-буфер?'))return;
  await fetch('/api/diaglog/clear',{cache:'no-store'});
  last='';
  prevRawLines=[];
  stampedLines=[];
  await loadLog();
}
setInterval(loadLog,500);
loadLog();
</script></body></html>
)HTML";

static void setupWeb(){
  web.on("/",HTTP_GET,[]{web.send_P(200,"text/html; charset=utf-8",WEB_PAGE);});
  web.on("/diaglog",HTTP_GET,[]{web.send_P(200,"text/html; charset=utf-8",DIAG_PAGE);});
  web.on("/api/diaglog",HTTP_GET,[]{
    web.send(200,"text/plain; charset=utf-8",diagSnapshot());
  });
  web.on("/api/diaglog/clear",HTTP_GET,[]{
    diagClear();
    web.send(200,"text/plain","OK");
  });
  web.on("/resetdiag",HTTP_GET,[]{web.send_P(200,"text/html; charset=utf-8",RESET_DIAG_PAGE);});
  web.on("/adc",HTTP_GET,[]{web.send_P(200,"text/html; charset=utf-8",ADC_TEST_PAGE);});
  web.on("/api/adcdiag",HTTP_GET,[]{
    int azRaw=0,azMv=0,elRaw=0,elMv=0;
    adcDiagRead(PIN_AZ_ADC,azRaw,azMv);
    adcDiagRead(PIN_EL_ADC,elRaw,elMv);
    String j="{\"azRaw\":"+String(azRaw)+",\"azMv\":"+String(azMv)+",\"elRaw\":"+String(elRaw)+",\"elMv\":"+String(elMv)+"}";
    web.send(200,"application/json",j);
  });
  web.on("/api/move",HTTP_GET,[]{
    gsSeqActive=false;
    String d=web.arg("d");
    moveDir(d.length()?d[0]:'S');
    web.send(200,"text/plain","OK");
  });
  web.on("/api/target",HTTP_GET,[]{
    gsSeqActive=false;
    setReferenceTarget(clampf(web.arg("az").toFloat(),0,activeAzMaxDeg()),clampf(web.arg("el").toFloat(),0,180),true,true);
    lastTargetCommandMs=millis(); lastTargetRxMs=millis();
    web.send(200,"text/plain","OK");
  });
  web.on("/api/cal",HTTP_GET,[]{
    // MULTI-POINT. AZ is written only to the currently selected P36/P45 table.
    String p=web.arg("p"); int a=adcAvgMv(PIN_AZ_ADC), e=adcAvgMv(PIN_EL_ADC);
    if(p.startsWith("az")){
      int *tbl = (gsAzMode==36) ? az36CalMv : az45CalMv;
      uint8_t n = (gsAzMode==36) ? AZ36_CAL_N : AZ45_CAL_N;
      int idx=-1;
      if(p=="az0") idx=0; else if(p=="az90") idx=1; else if(p=="az180") idx=2;
      else if(p=="az270") idx=3; else if(p=="az360") idx=4; else if(p=="az450") idx=5;
      if(idx<0 || idx>=n){ web.send(400,"text/plain","POINT NOT IN ACTIVE AZ RANGE"); return; }
      tbl[idx]=a;
    }else if(p=="el0")elCalMv[0]=e; else if(p=="el45")elCalMv[1]=e; else if(p=="el90")elCalMv[2]=e;
    else if(p=="el135")elCalMv[3]=e; else if(p=="el180")elCalMv[4]=e;
    else {web.send(400,"text/plain","BAD POINT");return;}
    saveCal();
    web.send(200,"text/plain","MULTI SAVED");
  });

  web.on("/api/cal2",HTTP_GET,[]{
    String p=web.arg("p"); p.toUpperCase();
    if(p=="O" || p=="O2" || p=="F" || p=="F2"){
      captureGs232Cal(p);
      web.send(200,"text/plain","2-POINT SAVED");
    }else web.send(400,"text/plain","BAD COMMAND");
  });

  web.on("/api/calmode",HTTP_GET,[]{
    String m=web.arg("m"); m.toLowerCase();
    if(m=="2" || m=="gs232" || m=="2point") setCalibrationMode(CAL_GS232_2POINT);
    else if(m=="multi") setCalibrationMode(CAL_MULTI_POINT);
    else {web.send(400,"text/plain","BAD MODE");return;}
    cal.valid=selectedCalibrationValid();
    web.send(200,"text/plain",calibrationModeName());
  });

  web.on("/api/azmode",HTTP_GET,[]{
    int m=web.arg("m").toInt();
    if(m!=36 && m!=45){ web.send(400,"text/plain","BAD AZ MODE"); return; }
    setGsAzMode((uint8_t)m);
    web.send(200,"text/plain",m==36?"P36":"P45");
  });

  web.on("/api/calreset",HTTP_GET,[]{
    String axis=web.arg("axis"); axis.toLowerCase();
    String m=web.arg("mode"); m.toLowerCase();

    if(m=="multi"){
      if(axis=="az"){
        if(gsAzMode==36){ for(uint8_t i=0;i<AZ36_CAL_N;i++) az36CalMv[i]=-1; az36MultiValid=false; }
        else { for(uint8_t i=0;i<AZ45_CAL_N;i++) az45CalMv[i]=-1; az45MultiValid=false; }
      }else if(axis=="el"){
        for(uint8_t i=0;i<EL_CAL_N;i++) elCalMv[i]=-1;
        elMultiValid=false;
      }else{ web.send(400,"text/plain","BAD AXIS"); return; }
    }else if(m=="2"){
      if(axis=="az"){
        if(gsAzMode==36){
          cal.az36_0=0; cal.az36Max=3100;
          az36OSet=false; az36FSet=false; az36LinearValid=false;
        }else{
          cal.az45_0=0; cal.az45Max=3100;
          az45OSet=false; az45FSet=false; az45LinearValid=false;
        }
      }else if(axis=="el"){
        cal.el0=0; cal.elMax=3100;
        elOSet=false; elFSet=false; elLinearValid=false;
      }else{ web.send(400,"text/plain","BAD AXIS"); return; }
    }else{
      web.send(400,"text/plain","BAD MODE"); return;
    }

    saveCal();
    web.send(200,"text/plain","RESET OK");
  });
  web.on("/api/wifi",HTTP_GET,[]{saveWifi(web.arg("ssid"),web.arg("pass"));web.send(200,"text/plain","Saved");delay(500);ESP.restart();});
  web.on("/api/ap",HTTP_GET,[]{
    String p=web.arg("pass");
    if(p.length()!=0 && (p.length()<8 || p.length()>63)){
      web.send(400,"text/plain","AP password must be empty or 8..63 characters");
      return;
    }
    saveApPassword(p);
    web.send(200,"text/plain",p.length()?"AP password saved":"AP set to open");
    delay(500);
    ESP.restart();
  });
  web.on("/api/resetreason",HTTP_GET,[]{
    String j="{\"reason\":\""+bootResetReasonText+"\",\"code\":"+String((int)bootResetReason)+"}";
    web.send(200,"application/json",j);
  });
  web.on("/api/resethistory",HTTP_GET,[]{
    web.send(200,"application/json",resetHistoryJson());
  });
  web.on("/api/resethistory/clear",HTTP_GET,[]{
    clearResetHistory();
    web.send(200,"text/plain","OK");
  });
  web.on("/api/rssi",HTTP_GET,[]{
    String r=web.arg("range");
    if(r=="day") web.send(200,"application/json",rssiHistoryJson(rssiDay,288,rssiDayPos,rssiDayCount,"day"));
    else if(r=="week") web.send(200,"application/json",rssiHistoryJson(rssiWeek,336,rssiWeekPos,rssiWeekCount,"week"));
    else if(r=="month") web.send(200,"application/json",rssiHistoryJson(rssiMonth,360,rssiMonthPos,rssiMonthCount,"month"));
    else if(r=="year") web.send(200,"application/json",rssiHistoryJson(rssiYear,365,rssiYearPos,rssiYearCount,"year"));
    else web.send(200,"application/json",rssiHistoryJson(rssiHour,60,rssiHourPos,rssiHourCount,"hour"));
  });
  web.on("/api/status",HTTP_GET,[]{updatePosition();
    bool staOk = (WiFi.status()==WL_CONNECTED);
    String staIp = staOk ? WiFi.localIP().toString() : "-";
    String apIp = WiFi.softAPIP().toString();
    String ip = staOk ? staIp : apIp;
    String mode = staOk ? "AP + Wi-Fi Client" : "Access Point";
    int rssi = staOk ? WiFi.RSSI() : 0;
    String signal = !staOk ? "-" : (rssi>=-55 ? "Excellent" : (rssi>=-67 ? "Good" : (rssi>=-75 ? "Fair" : "Weak")));
    String rssiJson = staOk ? String(rssi) : "null";
    uint32_t targetAge = targetValid ? (millis()-lastTargetRxMs) : 0;
    const int *azTbl = (gsAzMode==36) ? az36CalMv : az45CalMv;
    int azN = (gsAzMode==36) ? AZ36_CAL_N : AZ45_CAL_N;
    bool azLin = activeAzLinearValid();
    bool azMul = activeAzMultiValid();
    int az450v = (azN>5) ? azTbl[5] : -1;
    int az2z = activeAzOSet() ? activeAz2Zero() : -1;
    int az2f = activeAzFSet() ? activeAz2Full() : -1;
    cal.valid=selectedCalibrationValid();

    String j="{\"az\":"+String(currentAz,2)+
      ",\"el\":"+String(currentEl,2)+
      ",\"targetAz\":"+String(targetAz,2)+
      ",\"targetEl\":"+String(targetEl,2)+
      ",\"targetValid\":"+(targetValid?"true":"false")+
      ",\"targetAgeMs\":"+String(targetAge)+
      ",\"calibrated\":"+(cal.valid?"true":"false")+
      ",\"selectedCalOk\":"+(selectedCalibrationValid()?"true":"false")+
      ",\"azMode\":"+String(gsAzMode)+
      ",\"azLinear\":"+(azLin?"true":"false")+
      ",\"azLinear36\":"+(az36LinearValid?"true":"false")+
      ",\"azLinear45\":"+(az45LinearValid?"true":"false")+
      ",\"elLinear\":"+(elLinearValid?"true":"false")+
      ",\"azMulti\":"+(azMul?"true":"false")+
      ",\"azMulti36\":"+(az36MultiValid?"true":"false")+
      ",\"azMulti45\":"+(az45MultiValid?"true":"false")+
      ",\"elMulti\":"+(elMultiValid?"true":"false")+
      ",\"calMode\":\""+String(calibrationMode==CAL_MULTI_POINT?"multi":"2")+"\""+
      ",\"az2p0\":"+String(az2z)+",\"az2pf\":"+String(az2f)+
      ",\"el2p0\":"+String(elOSet?cal.el0:-1)+",\"el2pf\":"+String(elFSet?cal.elMax:-1)+
      ",\"motion\":\""+motion()+"\""+
      ",\"azp0\":"+String(azTbl[0])+",\"azp90\":"+String(azTbl[1])+",\"azp180\":"+String(azTbl[2])+
      ",\"azp270\":"+String(azTbl[3])+",\"azp360\":"+String(azTbl[4])+",\"azp450\":"+String(az450v)+
      ",\"elp0\":"+String(elCalMv[0])+",\"elp45\":"+String(elCalMv[1])+",\"elp90\":"+String(elCalMv[2])+
      ",\"elp135\":"+String(elCalMv[3])+",\"elp180\":"+String(elCalMv[4])+
      ",\"ip\":\""+ip+"\",\"apIp\":\""+apIp+"\",\"staIp\":\""+staIp+
      "\",\"wifiMode\":\""+mode+"\",\"ssid\":\""+wifiSsid+"\",\"rssi\":"+rssiJson+
      ",\"signal\":\""+signal+"\",\"btConnected\":"+(hc05Connected()?"true":"false")+
      ",\"resetReason\":\""+bootResetReasonText+"\"}";
    web.send(200,"application/json",j);});
  web.begin();
}

static int parseGsNumbers(const String &s, float *vals, int maxVals){
  char buf[128];
  String a=s;
  a.trim();
  a.toCharArray(buf,sizeof(buf));
  int n=0;
  char *save=nullptr;
  char *tok=strtok_r(buf," ,\t",&save);
  while(tok && n<maxVals){
    vals[n++]=atof(tok);
    tok=strtok_r(nullptr," ,\t",&save);
  }
  return n;
}

static void gsSeqStart(){
  if(gsSeqCount==0) return;
  gsSeqIndex=0;
  float az=clampf(gsSeqAz[0],0,activeAzMaxDeg());
  float el=gsSeqHasEl?clampf(gsSeqEl[0],0,180):currentEl;
  setReferenceTarget(az,el,true,gsSeqHasEl);
  lastTargetCommandMs=millis(); lastTargetRxMs=millis();
  gsSeqNextMs=millis()+(uint32_t)gsSeqIntervalSec*1000UL;
  gsSeqActive=true;
}

static void serviceGsSequence(){
  if(!gsSeqActive) return;
  // Internal timed sequence; this is not a newly received external target.
  lastTargetCommandMs=millis();

  if((int32_t)(millis()-gsSeqNextMs)>=0){
    if(gsSeqIndex+1>=gsSeqCount){
      gsSeqActive=false;
      return;
    }
    gsSeqIndex++;
    float az=clampf(gsSeqAz[gsSeqIndex],0,activeAzMaxDeg());
    float el=gsSeqHasEl?clampf(gsSeqEl[gsSeqIndex],0,180):currentEl;
    setReferenceTarget(az,el,true,gsSeqHasEl);
    gsSeqNextMs=millis()+(uint32_t)gsSeqIntervalSec*1000UL;
  }
}

static void rotctldSend(const String &s){
  bool sent=false;
  if(rotctldClient && rotctldClient.connected()){
    rotctldClient.print(s);
    flashTx();
    sent=true;
  }
#if ARDUINO_USB_MODE
  if(sent){
    dbgPrint("[TCP TX] ");
    String t=s;
    t.replace("\r","");
    t.replace("\n"," | ");
    dbgPrintln(t);
  }
#endif
}

static void processRotctldCommand(String cmd){
  cmd.trim();
  if(!cmd.length()) return;

  flashRx();

#if ARDUINO_USB_MODE
  dbgPrint("[TCP RX] ");
  dbgPrintln(cmd);
#endif

  updatePosition();

  String lower = cmd;
  lower.toLowerCase();

  // GS-232B over TCP in addition to Hamlib/Look4Sat.
  // Command coverage follows the documented GS-232B command groups.
  String upper = cmd;
  upper.toUpperCase();

  if(upper=="C"){
    char b[20]; snprintf(b,sizeof(b),"AZ=%03d\r\n",roundedAz());
    rotctldSend(String(b)); return;
  }
  if(upper=="B"){
    char b[20]; snprintf(b,sizeof(b),"EL=%03d\r\n",roundedEl());
    rotctldSend(String(b)); return;
  }
  if(upper=="C2"){
    char b[32]; snprintf(b,sizeof(b),"AZ=%03d EL=%03d\r\n",roundedAz(),roundedEl());
    rotctldSend(String(b)); return;
  }

  if(upper=="R"){ gsSeqActive=false; moveDir('R'); return; }
  if(upper=="L"){ gsSeqActive=false; moveDir('L'); return; }
  if(upper=="U"){ gsSeqActive=false; moveDir('U'); return; }
  if(upper=="D"){ gsSeqActive=false; moveDir('D'); return; }
  if(upper=="A"){ gsSeqActive=false; stopAzimuth(); return; }
  if(upper=="E"){ gsSeqActive=false; stopElevation(); return; }

  // Mxxx or timed Mttt xxx xxx ...
  if(upper.length()>1 && upper[0]=='M'){
    float v[GSSEQ_MAX+1];
    int n=parseGsNumbers(upper.substring(1),v,GSSEQ_MAX+1);
    gsSeqActive=false;
    if(n==1){
      setReferenceTarget(clampf(v[0],0,gsAzMode==36?360.0f:450.0f),currentEl,true,false);
      lastTargetCommandMs=millis(); lastTargetRxMs=millis();
    }else if(n>=2){
      gsSeqIntervalSec=(uint16_t)constrain((int)v[0],1,999);
      gsSeqCount=(uint8_t)min(n-1,(int)GSSEQ_MAX);
      gsSeqHasEl=false;
      for(uint8_t i=0;i<gsSeqCount;i++) gsSeqAz[i]=v[i+1];
      // GS-232B waits for T before stepping; first point becomes target.
      setReferenceTarget(clampf(gsSeqAz[0],0,activeAzMaxDeg()),currentEl,true,false);
      lastTargetCommandMs=millis(); lastTargetRxMs=millis();
    }else{
      gsSeqCount=0;
      rotctldSend("?>\r\n");
    }
    return;
  }

  // Wxxx yyy or timed Wttt xxx yyy xxx yyy ...
  if(upper.length()>1 && upper[0]=='W'){
    float v[1+GSSEQ_MAX*2];
    int n=parseGsNumbers(upper.substring(1),v,1+GSSEQ_MAX*2);
    gsSeqActive=false;
    if(n==2){
      setReferenceTarget(clampf(v[0],0,gsAzMode==36?360.0f:450.0f),clampf(v[1],0,180),true,true);
      lastTargetCommandMs=millis(); lastTargetRxMs=millis();
#if ARDUINO_USB_MODE
      dbgPrint("[TCP GS232 TARGET] AZ="); dbgPrint(String(targetAz,1));
      dbgPrint(" EL="); dbgPrintln(String(targetEl,1));
#endif
    }else if(n>=3 && ((n-1)%2)==0){
      gsSeqIntervalSec=(uint16_t)constrain((int)v[0],1,999);
      gsSeqCount=(uint8_t)min((n-1)/2,(int)GSSEQ_MAX);
      gsSeqHasEl=true;
      for(uint8_t i=0;i<gsSeqCount;i++){
        gsSeqAz[i]=v[1+i*2];
        gsSeqEl[i]=v[2+i*2];
      }
      setReferenceTarget(clampf(gsSeqAz[0],0,activeAzMaxDeg()),clampf(gsSeqEl[0],0,180),true,true);
      lastTargetCommandMs=millis(); lastTargetRxMs=millis();
    }else{
      gsSeqCount=0;
      rotctldSend("?>\r\n");
    }
    return;
  }

  if(upper=="T"){
    gsSeqStart();
    return;
  }

  if(upper=="N"){
    char b[32];
    snprintf(b,sizeof(b),"N=%u/%u\r\n",(unsigned)(gsSeqCount?gsSeqIndex+1:0),(unsigned)gsSeqCount);
    rotctldSend(String(b));
    return;
  }

  if(upper=="P36"){ setGsAzMode(36); return; }
  if(upper=="P45"){ setGsAzMode(45); return; }

  // X1..X4 are accepted for software compatibility. The G-5500 relay
  // outputs in this controller are ON/OFF, so physical speed is unchanged.
  if(upper.length()==2 && upper[0]=='X' && upper[1]>='1' && upper[1]<='4'){
    gsSpeed=(uint8_t)(upper[1]-'0');
    return;
  }

  // Standard GS-232 two-point calibration commands.
  if(upper=="O" || upper=="O2" || upper=="F" || upper=="F2"){
    captureGs232Cal(upper);
    return;
  }

  if(upper=="H"){
    rotctldSend("R L A C M T N S O F X1 X2 X3 X4\r\n");
    return;
  }
  if(upper=="H2"){
    rotctldSend("U D E C2 W T N S O2 F2 B\r\n");
    return;
  }
  if(upper=="H3"){
    String s="P45 P36 MODE=";
    s += (gsAzMode==45 ? "450" : "360");
    s += " CAL=";
    s += calibrationModeName();
    s += "\r\n";
    rotctldSend(s);
    return;
  }

  // Position query:
  // Hamlib short: p
  // Hamlib long:  \get_pos
  // Look4Sat / compatible bridges: get_pos
  if(cmd=="p" || lower=="\\get_pos" || lower=="get_pos"){
    rotctldSend(String(currentAz,6) + "\n" + String(currentEl,6) + "\n");
    return;
  }

  // Set position. Accept ALL common forms:
  //   P 180.0 30.0
  //   \set_pos 180.0 30.0
  //   set_pos 180.0 30.0   <-- Look4Sat has used this form
  bool isSet = false;
  int valueStart = -1;

  if(cmd.length()>=2 && cmd[0]=='P' && cmd[1]==' '){
    isSet=true;
    valueStart=2;
  }else if(lower.startsWith("\\set_pos ")){
    isSet=true;
    valueStart=9;
  }else if(lower.startsWith("set_pos ")){
    isSet=true;
    valueStart=8;
  }

  if(isSet){
    float az=0, el=0;
    String args = cmd.substring(valueStart);

    // R4UAB DDE Client sends decimal values using the Windows locale,
    // e.g. "P 187,30 22,40".  sscanf("%f") expects a decimal point in
    // the C locale, so normalize decimal commas before parsing.
    args.replace(',', '.');

    if(sscanf(args.c_str(),"%f %f",&az,&el)==2){
      setReferenceTarget(clampf(az,0,activeAzMaxDeg()),clampf(el,0,180),true,true);
      lastTargetCommandMs=millis(); lastTargetRxMs=millis();

#if ARDUINO_USB_MODE
      dbgPrint("[TCP TARGET] AZ=");
      dbgPrint(String(targetAz,1));
      dbgPrint(" EL=");
      dbgPrintln(String(targetEl,1));
#endif

      rotctldSend("RPRT 0\n");
    }else{
      rotctldSend("RPRT -1\n");
    }
    return;
  }

  // Stop. Accept Hamlib and Look4Sat-style spelling.
  if(cmd=="S" || lower=="\\stop" || lower=="stop"){
    gsSeqActive=false;
    allStop();
    rotctldSend("RPRT 0\n");
    return;
  }

  // Quit current network session.
  if(cmd=="q" || cmd=="Q" || lower=="\\quit" || lower=="quit"){
    rotctldSend("RPRT 0\n");
    delay(5);
    rotctldClient.stop();
    return;
  }

  // Minimal info support.
  if(cmd=="_" || lower=="\\get_info" || lower=="get_info"){
    rotctldSend("G5500 ESP32-S3 v1.6.23 UA1CFM Look4Sat bridge\n");
    return;
  }

  // Some clients probe capabilities/help. Do not tear down the session.
  if(cmd=="?" || lower=="\\dump_caps" || lower=="dump_caps"){
    rotctldSend("RPRT 0\n");
    return;
  }

#if ARDUINO_USB_MODE
  dbgPrint("[TCP UNKNOWN] ");
  dbgPrintln(cmd);
#endif
  rotctldSend("RPRT -1\n");
}

static void handleRotctldTcp(){
  // Accept a client if none is currently connected.
  if(!rotctldClient || !rotctldClient.connected()){
    WiFiClient c = tcpServer.available();
    if(c){
      if(rotctldClient) rotctldClient.stop();
      rotctldClient = c;
      rotctldLine = "";
      rotctldLine.reserve(96);
#if ARDUINO_USB_MODE
      dbgPrintln("[TCP] Look4Sat/rotctld client connected");
#endif
    }else{
      return;
    }
  }

  // Non-blocking line receiver.
  while(rotctldClient.connected() && rotctldClient.available()){
    char ch=(char)rotctldClient.read();

    if(ch=='\r' || ch=='\n'){
      if(rotctldLine.length()){
        String cmd=rotctldLine;
        rotctldLine="";
        processRotctldCommand(cmd);
      }
    }else if(ch>=32 && ch<=126){
      if(rotctldLine.length()<95) rotctldLine += ch;
      else rotctldLine="";
    }
  }
}

static void startWifi(){
  // SAFE Wi-Fi logic:
  // 1) Start the controller AP.
  // 2) If saved home Wi-Fi credentials exist, try STA only ONCE.
  // 3) If home Wi-Fi is unavailable, stop STA completely and return to AP-only.
  // This prevents repeated STA scans from making the SoftAP disappear.

  WiFi.persistent(false);
  WiFi.setSleep(false);

  // Start with AP only.
  WiFi.mode(WIFI_AP);
  delay(250);

  IPAddress apIP(192,168,4,1);
  IPAddress apGW(192,168,4,1);
  IPAddress apMask(255,255,255,0);
  WiFi.softAPConfig(apIP,apGW,apMask);

  bool apOk = apPass.length()>=8 ? WiFi.softAP(AP_SSID,apPass.c_str()) : WiFi.softAP(AP_SSID);
  delay(300);

#if ARDUINO_USB_MODE
  dbgPrintln("[V1559] AP-only start");
  dbgPrint("[V1559] AP start = ");
  dbgPrintln(apOk ? "OK" : "FAILED");
  dbgPrint("[V1559] AP IP = ");
  dbgPrintln(WiFi.softAPIP().toString());
#endif

  // Try home Wi-Fi once, only if credentials are stored.
  if(wifiSsid.length()){
#if ARDUINO_USB_MODE
    dbgPrint("[V1559] one STA attempt: ");
    dbgPrintln(wifiSsid);
#endif

    WiFi.mode(WIFI_AP_STA);
    delay(100);
    WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());

    uint32_t t0=millis();
    while(WiFi.status()!=WL_CONNECTED && millis()-t0<8000){
      delay(100);
    }

    if(WiFi.status()==WL_CONNECTED){
#if ARDUINO_USB_MODE
      dbgPrint("[V1559] LAN connected = ");
      dbgPrintln(WiFi.localIP().toString());
      dbgPrint("[V1559] AP remains = ");
      dbgPrintln(WiFi.softAPIP().toString());
#endif
    }else{
      // Home network not available: completely stop STA and restore AP-only.
      WiFi.disconnect(true, false);
      delay(150);
      WiFi.mode(WIFI_AP);
      delay(150);

      WiFi.softAPConfig(apIP,apGW,apMask);
      if(apPass.length()>=8) WiFi.softAP(AP_SSID,apPass.c_str()); else WiFi.softAP(AP_SSID);
      delay(250);

#if ARDUINO_USB_MODE
      dbgPrintln("[V1559] LAN unavailable -> AP ONLY");
      dbgPrint("[V1559] AP restored = ");
      dbgPrintln(WiFi.softAPIP().toString());
#endif
    }
  }else{
#if ARDUINO_USB_MODE
    dbgPrintln("[V1559] no saved LAN credentials -> AP ONLY");
#endif
  }

  tcpServer.begin();
  setupWeb();

#if ARDUINO_USB_MODE
  dbgPrintln("[WIFI] safe auto-reconnect: 30 s interval, 8 s attempt, AP-only fallback");
#endif
}


static void restoreApOnlyAfterReconnectFail(){
  // Stop the failed STA side completely and return to the same stable
  // AP-only state used by startWifi(). No long delay is used here because
  // rotator control must continue to run.
  WiFi.disconnect(true, false);
  WiFi.mode(WIFI_AP);

  IPAddress apIP(192,168,4,1);
  IPAddress apGW(192,168,4,1);
  IPAddress apMask(255,255,255,0);
  WiFi.softAPConfig(apIP,apGW,apMask);

  if(apPass.length()>=8) WiFi.softAP(AP_SSID,apPass.c_str());
  else WiFi.softAP(AP_SSID);
}

static void serviceWifiReconnect(){
  if(!wifiSsid.length()) return;

  uint32_t now=millis();

  // LAN came back: remain in AP+STA and arm the timer for a future loss.
  if(WiFi.status()==WL_CONNECTED){
    if(wifiReconnectTryActive){
#if ARDUINO_USB_MODE
      dbgPrint("[WIFI] reconnect OK, LAN IP = ");
      dbgPrintln(WiFi.localIP().toString());
#endif
    }
    wifiReconnectTryActive=false;
    wifiReconnectMs=now;
    return;
  }

  // A reconnect attempt is running. Give it at most 8 seconds.
  // This is non-blocking: the normal control loop continues to execute.
  if(wifiReconnectTryActive){
    if((uint32_t)(now-wifiReconnectTryStartedMs)<WIFI_RECONNECT_TIMEOUT_MS) return;

#if ARDUINO_USB_MODE
    dbgPrintln("[WIFI] reconnect timeout -> restore AP ONLY");
#endif

    wifiReconnectTryActive=false;
    wifiReconnectMs=now;
    restoreApOnlyAfterReconnectFail();
    return;
  }

  // First observation after boot/loss: start the 30-second retry interval.
  if(wifiReconnectMs==0){
    wifiReconnectMs=now;
    return;
  }

  if((uint32_t)(now-wifiReconnectMs)<WIFI_RECONNECT_MS) return;

  // One controlled STA attempt. AP remains available while the attempt runs.
#if ARDUINO_USB_MODE
  dbgPrint("[WIFI] reconnect attempt: ");
  dbgPrintln(wifiSsid);
#endif

  if(WiFi.getMode()!=WIFI_AP_STA) WiFi.mode(WIFI_AP_STA);

  wifiReconnectTryActive=true;
  wifiReconnectTryStartedMs=now;
  WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
}

static void loadTouchCalibration(){
  prefs.begin("g5500",true);
  touchCalValid=prefs.getBool("tcalok",false);
  touchSwapXY=prefs.getBool("tswap",true);
  touchX0Raw=prefs.getInt("tx0",TOUCH_RAW_Y_MIN);
  touchX1Raw=prefs.getInt("tx1",TOUCH_RAW_Y_MAX);
  touchY0Raw=prefs.getInt("ty0",TOUCH_RAW_X_MAX);
  touchY1Raw=prefs.getInt("ty1",TOUCH_RAW_X_MIN);
  prefs.end();
  if(abs(touchX1Raw-touchX0Raw)<500 || abs(touchY1Raw-touchY0Raw)<500){
    touchCalValid=false;
  }
}

static void saveTouchCalibration(){
  prefs.begin("g5500",false);
  prefs.putBool("tcalok",touchCalValid);
  prefs.putBool("tswap",touchSwapXY);
  prefs.putInt("tx0",touchX0Raw);
  prefs.putInt("tx1",touchX1Raw);
  prefs.putInt("ty0",touchY0Raw);
  prefs.putInt("ty1",touchY1Raw);
  prefs.end();
}

// -----------------------------------------------------------------------------
// TFT + XPT2046 local UI
// -----------------------------------------------------------------------------
static const uint16_t TFT_BG     = ILI9341_BLACK;
static const uint16_t TFT_PANEL  = 0x18E3;
static const uint16_t TFT_BORDER = 0x4208;
static const uint16_t TFT_TEXT   = ILI9341_WHITE;
static const uint16_t TFT_MUTED  = 0x9CF3;
static const uint16_t TFT_AZ     = ILI9341_CYAN;
static const uint16_t TFT_EL     = ILI9341_YELLOW;
static const uint16_t TFT_OK     = ILI9341_GREEN;
static const uint16_t TFT_WARN   = ILI9341_ORANGE;
static const uint16_t TFT_STOP   = ILI9341_RED;
static const uint16_t TFT_BLUE   = ILI9341_BLUE;
static const uint16_t TFT_TARGET = ILI9341_MAGENTA;

static void tftText(int16_t x,int16_t y,uint8_t size,uint16_t fg,uint16_t bg,const String &txt){
  tft.setTextWrap(false);
  tft.setTextSize(size);
  tft.setTextColor(fg,bg);
  tft.setCursor(x,y);
  tft.print(txt);
}

static void tftButton(int x,int y,int w,int h,const String &label,bool active=false,bool saved=false){
  uint16_t fill=TFT_PANEL;
  uint16_t frame=TFT_WARN;       // normal calibration button: colored frame
  if(active){
    fill=TFT_BLUE;
    frame=TFT_AZ;                // selected mode/range: bright cyan frame
  }
  if(saved){
    fill=0x03E0;
    frame=TFT_OK;                // captured calibration point: green frame
  }

  tft.fillRoundRect(x,y,w,h,5,fill);

  // Two-pixel frame so it is clearly visible on the 2.4" TFT.
  tft.drawRoundRect(x,y,w,h,5,frame);
  if(w>4 && h>4) tft.drawRoundRect(x+1,y+1,w-2,h-2,4,frame);

  int tw=(int)label.length()*6*2;
  int tx=x+(w-tw)/2;
  int ty=y+(h-16)/2;
  tftText(tx,ty,2,TFT_TEXT,fill,label);
}

static String tftMotionText(){
  String m=motion();
  if(m=="RIGHT") return "RIGHT >";
  if(m=="LEFT")  return "< LEFT";
  if(m=="UP")    return "UP ^";
  if(m=="DOWN")  return "DOWN v";
  return "STOP";
}

static String tftCalibrationText(){
  if(calibrationMode==CAL_MULTI_POINT){
    return selectedCalibrationValid() ? "MULTI OK" : "MULTI";
  }
  return selectedCalibrationValid() ? "2PT OK" : "2-POINT";
}

static void resetDashboardCache(){
  dashLastAz=-9999.0f; dashLastEl=-9999.0f;
  dashLastTargetAz=-9999.0f; dashLastTargetEl=-9999.0f;
  dashLastTargetValid=!targetValid; // force first target redraw
  dashLastAzBar=-1; dashLastElBar=-1;
  dashLastAzState=""; dashLastElState="";
  dashLastCal=""; dashLastWifi=""; dashLastBt=""; dashLastIp=""; dashLastTouch="";
}

static void drawDashboardStatic(){
  tft.fillScreen(TFT_BG);
  resetDashboardCache();
  tft.fillRoundRect(3,3,314,29,5,TFT_BLUE);
  tftText(10,9,2,TFT_TEXT,TFT_BLUE,"G-5500 ROTATOR");
  tftText(190,9,2,TFT_TEXT,TFT_BLUE,"(c)UA1CFM");

  tft.drawRoundRect(3,36,154,139,6,TFT_BORDER);
  tft.drawRoundRect(163,36,154,139,6,TFT_BORDER);
  tft.fillRect(4,37,152,21,TFT_PANEL);
  tft.fillRect(164,37,152,21,TFT_PANEL);
  tftText(39,42,2,TFT_TEXT,TFT_PANEL,"AZIMUTH");
  tftText(188,42,2,TFT_TEXT,TFT_PANEL,"ELEVATION");

  // TARGET label and value are intentionally on ONE line.
  // DELTA is not used in this TFT branch.
  tftText(8,112,2,TFT_MUTED,TFT_BG,"TARGET ");
  tftText(168,112,2,TFT_MUTED,TFT_BG,"TARGET ");

  tft.drawRoundRect(11,143,138,12,3,TFT_BORDER);
  tft.drawRoundRect(171,143,138,12,3,TFT_BORDER);

  // Short large range labels.
  tftText(12,160,2,TFT_MUTED,TFT_BG,gsAzMode==36?"P36":"P45");
  tftText(172,160,2,TFT_MUTED,TFT_BG,"EL");

  // Bottom controls/status. All visible labels are size 2.
  tft.drawRoundRect(3,180,118,26,4,TFT_WARN);
  tft.drawRoundRect(4,181,116,24,3,TFT_WARN);
  tft.drawRoundRect(126,180,103,26,4,TFT_BORDER);
  tft.drawRoundRect(234,180,83,26,4,TFT_BORDER);
  tft.drawRoundRect(3,210,194,26,4,TFT_BORDER);
  tft.drawRoundRect(202,210,115,26,4,TFT_WARN);
  tft.drawRoundRect(203,211,113,24,3,TFT_WARN);

  tftText(8,186,2,TFT_MUTED,TFT_BG,"CAL");
  tftText(131,186,2,TFT_MUTED,TFT_BG,"WIFI");
  tftText(239,186,2,TFT_MUTED,TFT_BG,"BT");
  tftText(8,216,2,TFT_MUTED,TFT_BG,"IP");
  tftText(207,216,2,TFT_MUTED,TFT_BG,"TOUCH");
}

static void updateDashboard(){
  // 4 Hz is enough for a rotator display and avoids wasting SPI bandwidth.
  if(millis()-lastDisplayMs<500 && !tftNeedsRedraw) return;
  lastDisplayMs=millis();

  if(tftNeedsRedraw){
    drawDashboardStatic();
    tftNeedsRedraw=false;
  }

  // AZ / EL: show integer degrees only.
  int azShown=(int)lroundf(currentAz);
  int elShown=(int)lroundf(currentEl);

  if((int)dashLastAz!=azShown){
    tft.fillRect(10,63,140,37,TFT_BG);
    tft.setTextSize(4); tft.setTextColor(TFT_AZ,TFT_BG); tft.setCursor(16,67);
    tft.print(azShown); tft.setTextSize(2); tft.print((char)247);
    dashLastAz=(float)azShown;
  }

  if((int)dashLastEl!=elShown){
    tft.fillRect(170,63,140,37,TFT_BG);
    tft.setTextSize(4); tft.setTextColor(TFT_EL,TFT_BG); tft.setCursor(176,67);
    tft.print(elShown); tft.setTextSize(2); tft.print((char)247);
    dashLastEl=(float)elShown;
  }

  // TARGET only, integer degrees, on the SAME line as "TARGET -".
  int taShown=(int)lroundf(targetAz);
  int teShown=(int)lroundf(targetEl);

  bool needTarget =
      (targetValid!=dashLastTargetValid) ||
      (targetValid && (
        (int)dashLastTargetAz!=taShown ||
        (int)dashLastTargetEl!=teShown));

  if(needTarget){
    tft.fillRect(100,109,50,24,TFT_BG);
    tft.fillRect(260,109,50,24,TFT_BG);

    if(targetValid){
      tftText(100,112,2,TFT_TARGET,TFT_BG,String(taShown)+String((char)247));
      tftText(260,112,2,TFT_TARGET,TFT_BG,String(teShown)+String((char)247));
    }else{
      tftText(100,112,2,TFT_MUTED,TFT_BG,"---");
      tftText(260,112,2,TFT_MUTED,TFT_BG,"---");
    }

    dashLastTargetValid=targetValid;
    dashLastTargetAz=(float)taShown;
    dashLastTargetEl=(float)teShown;
  }

  // Progress bars: only change when width changes.
  float azMax=activeAzMaxDeg();
  int azw=(int)constrain(currentAz/azMax*134.0f,0.0f,134.0f);
  int elw=(int)constrain(currentEl/180.0f*134.0f,0.0f,134.0f);

  if(azw!=dashLastAzBar){
    // Instead of clearing the whole bar every time, change only the difference.
    if(dashLastAzBar<0){
      tft.fillRect(13,145,134,8,TFT_BG);
      if(azw>0) tft.fillRect(13,145,azw,8,TFT_AZ);
    }else if(azw>dashLastAzBar){
      tft.fillRect(13+dashLastAzBar,145,azw-dashLastAzBar,8,TFT_AZ);
    }else if(azw<dashLastAzBar){
      tft.fillRect(13+azw,145,dashLastAzBar-azw,8,TFT_BG);
    }
    dashLastAzBar=azw;
  }

  if(elw!=dashLastElBar){
    if(dashLastElBar<0){
      tft.fillRect(173,145,134,8,TFT_BG);
      if(elw>0) tft.fillRect(173,145,elw,8,TFT_EL);
    }else if(elw>dashLastElBar){
      tft.fillRect(173+dashLastElBar,145,elw-dashLastElBar,8,TFT_EL);
    }else if(elw<dashLastElBar){
      tft.fillRect(173+elw,145,dashLastElBar-elw,8,TFT_BG);
    }
    dashLastElBar=elw;
  }

  // Motion text: update only on state change.
  // Read each axis directly from its GPIOs so simultaneous motion
  // (e.g. UP+RIGHT) is displayed correctly.
  bool left  = digitalRead(PIN_LEFT);
  bool right = digitalRead(PIN_RIGHT);
  bool up    = digitalRead(PIN_UP);
  bool down  = digitalRead(PIN_DOWN);

  bool azMoving = left || right;
  bool elMoving = up || down;

  String azState = right ? "RIGHT" :
                   left  ? "LEFT"  : "STOP";

  String elState = up   ? "UP"   :
                   down ? "DOWN" : "STOP";

  if(azState!=dashLastAzState){
    tft.fillRect(60,158,90,17,TFT_BG);
    tftText(64,160,2,azMoving?TFT_OK:TFT_STOP,TFT_BG,azState);
    dashLastAzState=azState;
  }

  if(elState!=dashLastElState){
    tft.fillRect(218,158,92,17,TFT_BG);
    tftText(222,160,2,elMoving?TFT_OK:TFT_STOP,TFT_BG,elState);
    dashLastElState=elState;
  }

  // Bottom status fields: repaint only when text changes.
  String calText=(calibrationMode==CAL_MULTI_POINT)?"MULTI":"2PT";
  if(calText!=dashLastCal){
    tft.fillRect(48,183,68,20,TFT_BG);
    tftText(50,186,2,selectedCalibrationValid()?TFT_OK:TFT_WARN,TFT_BG,calText);
    dashLastCal=calText;
  }

  String wifiText;
  uint16_t wifiColor;
  if(WiFi.status()==WL_CONNECTED){
    wifiText=String(WiFi.RSSI());
    wifiColor=TFT_OK;
  }else{
    wifiText="AP";
    wifiColor=TFT_WARN;
  }
  if(wifiText!=dashLastWifi){
    tft.fillRect(183,183,40,20,TFT_BG);
    tftText(184,186,2,wifiColor,TFT_BG,wifiText);
    dashLastWifi=wifiText;
  }

  bool bt=digitalRead(PIN_HC05_STATE);
  String btText=bt?"ON":"OFF";
  if(btText!=dashLastBt){
    tft.fillRect(271,183,40,20,TFT_BG);
    tftText(272,186,2,bt?TFT_OK:TFT_MUTED,TFT_BG,btText);
    dashLastBt=btText;
  }

  String ip=(WiFi.status()==WL_CONNECTED)?WiFi.localIP().toString():WiFi.softAPIP().toString();
  if(ip!=dashLastIp){
    tft.fillRect(34,213,160,20,TFT_BG);
    tftText(36,216,2,TFT_TEXT,TFT_BG,ip);
    dashLastIp=ip;
  }

  String touchText=touchCalValid?"OK":"CAL";
  if(touchText!=dashLastTouch){
    tft.fillRect(273,213,39,20,TFT_BG);
    tftText(274,216,2,touchCalValid?TFT_OK:TFT_WARN,TFT_BG,touchText);
    dashLastTouch=touchText;
  }
}

static void drawCalHeader(const String &title){
  tft.fillScreen(TFT_BG);
  tft.fillRoundRect(3,3,314,29,5,TFT_BLUE);
  tftText(10,9,2,TFT_TEXT,TFT_BLUE,title);
  tftButton(252,5,63,25,"BACK");
}

static void drawCalHome(){
  drawCalHeader("CALIBRATION");

  tftText(8,39,2,TFT_MUTED,TFT_BG,"MODE");
  tftButton(8,55,145,34,"2-POINT",calibrationMode==CAL_GS232_2POINT);
  tftButton(167,55,145,34,"MULTI",calibrationMode==CAL_MULTI_POINT);

  tftText(8,96,2,TFT_MUTED,TFT_BG,"AZ RANGE");
  tftButton(8,112,145,34,"P36",gsAzMode==36);
  tftButton(167,112,145,34,"P45",gsAzMode==45);

  tftButton(8,166,145,52,"AZ CAL");
  tftButton(167,166,145,52,"EL CAL");

  tftText(8,223,2,TFT_MUTED,TFT_BG,
          String("A")+String(adcAvgMv(PIN_AZ_ADC))+" E"+String(adcAvgMv(PIN_EL_ADC))+" mV");
}

static void drawCalAz(){
  drawCalHeader(String("AZ CAL ")+(gsAzMode==36?"P36":"P45"));
  int mv=adcAvgMv(PIN_AZ_ADC);
  tftText(8,40,2,TFT_AZ,TFT_BG,String("ADC ")+String(mv)+" mV");

  if(calibrationMode==CAL_GS232_2POINT){
    tftButton(8,70,145,48,"O = 0",false,activeAzOSet());
    tftButton(167,70,145,48,gsAzMode==36?"F = 360":"F = 450",false,activeAzFSet());
    tftText(8,130,2,TFT_MUTED,TFT_BG,
            String("O:")+(activeAzOSet()?String(activeAz2Zero()):"---"));
    tftText(167,130,2,TFT_MUTED,TFT_BG,
            String("F:")+(activeAzFSet()?String(activeAz2Full()):"---"));
    tftButton(8,166,304,45,"RESET AZ");
  }else{
    const int vals36[5]={0,90,180,270,360};
    const int vals45[6]={0,90,180,270,360,450};
    int n=gsAzMode==36?5:6;
    for(int i=0;i<n;i++){
      int row=i/3, col=i%3;
      int x=8+col*104, y=68+row*58;
      int deg=gsAzMode==36?vals36[i]:vals45[i];
      int val=gsAzMode==36?az36CalMv[i]:az45CalMv[i];
      tftButton(x,y,96,44,String(deg),false,val>=0);
    }
    tftButton(8,190,304,38,"RESET AZ");
  }
}

static void drawCalEl(){
  drawCalHeader("EL CAL");
  int mv=adcAvgMv(PIN_EL_ADC);
  tftText(8,40,2,TFT_EL,TFT_BG,String("ADC ")+String(mv)+" mV");

  if(calibrationMode==CAL_GS232_2POINT){
    tftButton(8,70,145,48,"O2 = 0",false,elOSet);
    tftButton(167,70,145,48,"F2 = 180",false,elFSet);
    tftText(8,130,2,TFT_MUTED,TFT_BG,String("O2:")+(elOSet?String(cal.el0):"---"));
    tftText(167,130,2,TFT_MUTED,TFT_BG,String("F2:")+(elFSet?String(cal.elMax):"---"));
    tftButton(8,166,304,45,"RESET EL");
  }else{
    const int vals[5]={0,45,90,135,180};
    for(int i=0;i<5;i++){
      int row=i/3, col=i%3;
      int x=8+col*104, y=68+row*58;
      tftButton(x,y,96,44,String(vals[i]),false,elCalMv[i]>=0);
    }
    tftButton(8,190,304,38,"RESET EL");
  }
}


static void drawTouchCalibration(){
  tft.fillScreen(TFT_BG);
  tft.fillRoundRect(3,3,314,29,5,TFT_BLUE);
  tftText(10,9,2,TFT_TEXT,TFT_BLUE,"TOUCH CALIBRATION");

  if(touchCalStep>=4){
    tftText(42,64,2,touchCalValid?TFT_OK:TFT_STOP,TFT_BG,
            touchCalValid?"CALIBRATION SAVED":"CALIBRATION FAILED");
    if(touchCalValid){
      tftText(12,90,2,TFT_MUTED,TFT_BG,
              String("X ")+String(touchX0Raw)+"-"+String(touchX1Raw));
      tftText(12,108,2,TFT_MUTED,TFT_BG,
              String("Y ")+String(touchY0Raw)+"-"+String(touchY1Raw));
      tftText(12,126,2,TFT_MUTED,TFT_BG,
              String("SWAP ")+(touchSwapXY?"YES":"NO"));
    }else{
      tftText(12,104,2,TFT_WARN,TFT_BG,"PRESS FIRMLY");
    }
    tftButton(65,155,190,48,touchCalValid?"BACK":"RETRY");
    return;
  }

  static const char* names[4]={"TOP LEFT","TOP RIGHT","BOTTOM RIGHT","BOTTOM LEFT"};
  static const int px[4]={20,299,299,20};
  static const int py[4]={20,20,219,219};

  tftText(16,39,2,TFT_MUTED,TFT_BG,"PRESS CROSS");
  tftText(16,57,2,TFT_TEXT,TFT_BG,String(touchCalStep+1)+"/4  "+String(names[touchCalStep]));

  int x=px[touchCalStep], y=py[touchCalStep];
  tft.drawCircle(x,y,10,TFT_WARN);
  tft.drawFastHLine(x-15,y,31,TFT_WARN);
  tft.drawFastVLine(x,y-15,31,TFT_WARN);
}

static void startTouchCalibration(){
  touchCalStep=0;
  touchWaitRelease=true;
  for(int i=0;i<4;i++){ touchCalRawX[i]=0; touchCalRawY[i]=0; }
  tftPage=TFT_PAGE_TOUCH_CAL;
  tftNeedsRedraw=true;
  drawTftPage();
}

static bool readTouchRawAverage(int &rx,int &ry){
  if(!touch.touched()) return false;
  long sx=0, sy=0;
  int n=0;
  for(int i=0;i<12;i++){
    if(touch.touched()){
      TS_Point p=touch.getPoint();
      if(p.z==0 || p.z>=150){
        sx+=p.x; sy+=p.y; n++;
      }
    }
    delay(3);
  }
  if(n<5) return false;
  rx=(int)(sx/n);
  ry=(int)(sy/n);
  return true;
}

static void finishTouchCalibration(){
  long horizDX=labs(touchCalRawX[1]-touchCalRawX[0]) + labs(touchCalRawX[2]-touchCalRawX[3]);
  long horizDY=labs(touchCalRawY[1]-touchCalRawY[0]) + labs(touchCalRawY[2]-touchCalRawY[3]);

  touchSwapXY=(horizDY>horizDX);

  int sx0 = touchSwapXY ? touchCalRawY[0] : touchCalRawX[0];
  int sx1 = touchSwapXY ? touchCalRawY[1] : touchCalRawX[1];
  int sx2 = touchSwapXY ? touchCalRawY[2] : touchCalRawX[2];
  int sx3 = touchSwapXY ? touchCalRawY[3] : touchCalRawX[3];

  int sy0 = touchSwapXY ? touchCalRawX[0] : touchCalRawY[0];
  int sy1 = touchSwapXY ? touchCalRawX[1] : touchCalRawY[1];
  int sy2 = touchSwapXY ? touchCalRawX[2] : touchCalRawY[2];
  int sy3 = touchSwapXY ? touchCalRawX[3] : touchCalRawY[3];

  touchX0Raw=(sx0+sx3)/2;
  touchX1Raw=(sx1+sx2)/2;
  touchY0Raw=(sy0+sy1)/2;
  touchY1Raw=(sy2+sy3)/2;

  touchCalValid=(abs(touchX1Raw-touchX0Raw)>500 &&
                 abs(touchY1Raw-touchY0Raw)>500);
  saveTouchCalibration();
}

static void serviceTouchCalibration(){
  if(tftPage!=TFT_PAGE_TOUCH_CAL) return;

  if(touchCalStep>=4){
    if(!touch.touched()){ touchWaitRelease=false; return; }
    if(touchWaitRelease) return;

    int rx,ry;
    if(!readTouchRawAverage(rx,ry)) return;

    if(!touchCalValid){
      // On failed calibration, accept the lower half of the panel in RAW form
      // as RETRY so recovery never depends on a bad coordinate map.
      startTouchCalibration();
      return;
    }

    int x,y;
    if(readTouchXY(x,y) && hit(x,y,45,145,230,68)){
      tftPage=TFT_PAGE_DASH;
      tftNeedsRedraw=true;
      drawTftPage();
      touchWaitRelease=true;
    }
    return;
  }

  if(!touch.touched()){
    touchWaitRelease=false;
    return;
  }
  if(touchWaitRelease) return;

  int rx,ry;
  if(!readTouchRawAverage(rx,ry)) return;

  touchCalRawX[touchCalStep]=rx;
  touchCalRawY[touchCalStep]=ry;
  touchCalStep++;
  touchWaitRelease=true;

  if(touchCalStep>=4) finishTouchCalibration();
  tftNeedsRedraw=true;
  drawTftPage();
}


static void showHeaderImage(){
  // Full-screen image, but controller logic keeps running normally.
  tft.fillScreen(TFT_BG);
  tft.drawRGBBitmap(0,0,catImage,CAT_IMAGE_W,CAT_IMAGE_H);
  imagePageStartedMs=millis();
  tftPage=TFT_PAGE_IMAGE;
  tftNeedsRedraw=false;
}

static void serviceHeaderImage(){
  if(tftPage!=TFT_PAGE_IMAGE) return;
  if(millis()-imagePageStartedMs>=IMAGE_PAGE_DURATION_MS){
    tftPage=TFT_PAGE_DASH;
    tftNeedsRedraw=true;
    drawTftPage();
  }
}

static void drawTftPage(){
  if(tftPage==TFT_PAGE_DASH) drawDashboardStatic();
  else if(tftPage==TFT_PAGE_CAL_HOME) drawCalHome();
  else if(tftPage==TFT_PAGE_CAL_AZ) drawCalAz();
  else if(tftPage==TFT_PAGE_CAL_EL) drawCalEl();
  else if(tftPage==TFT_PAGE_TOUCH_CAL) drawTouchCalibration();
  else if(tftPage==TFT_PAGE_IMAGE) return;
  tftNeedsRedraw=false;
}

static bool hit(int x,int y,int bx,int by,int bw,int bh){
  return x>=bx && x<bx+bw && y>=by && y<by+bh;
}

static bool readTouchXY(int &x,int &y){
  if(!touch.touched()) return false;
  TS_Point p=touch.getPoint();

  // Ignore extremely weak/noisy contacts.
  if(p.z>0 && p.z<150) return false;

  int rawX=touchSwapXY ? p.y : p.x;
  int rawY=touchSwapXY ? p.x : p.y;

  long sx,sy;
  if(touchCalValid){
    sx=20L+(long)(rawX-touchX0Raw)*(299-20)/(long)(touchX1Raw-touchX0Raw);
    sy=20L+(long)(rawY-touchY0Raw)*(219-20)/(long)(touchY1Raw-touchY0Raw);
  }else{
    sx=map((long)p.y,TOUCH_RAW_Y_MIN,TOUCH_RAW_Y_MAX,0,319);
    sy=map((long)p.x,TOUCH_RAW_X_MIN,TOUCH_RAW_X_MAX,239,0);
  }

  x=constrain((int)sx,0,319);
  y=constrain((int)sy,0,239);
  return true;
}

static void resetTouchCalAxis(bool az){
  if(calibrationMode==CAL_GS232_2POINT){
    if(az){
      if(gsAzMode==36){
        cal.az36_0=0; cal.az36Max=3100;
        az36OSet=false; az36FSet=false; az36LinearValid=false;
      }else{
        cal.az45_0=0; cal.az45Max=3100;
        az45OSet=false; az45FSet=false; az45LinearValid=false;
      }
    }else{
      cal.el0=0; cal.elMax=3100;
      elOSet=false; elFSet=false; elLinearValid=false;
    }
  }else{
    if(az){
      if(gsAzMode==36) for(uint8_t i=0;i<AZ36_CAL_N;i++) az36CalMv[i]=-1;
      else for(uint8_t i=0;i<AZ45_CAL_N;i++) az45CalMv[i]=-1;
    }else{
      for(uint8_t i=0;i<EL_CAL_N;i++) elCalMv[i]=-1;
    }
    refreshMultiCalValidity();
  }
  saveCal();
  tftNeedsRedraw=true;
}

static void captureTouchMultiAz(uint8_t idx){
  int mv=adcAvgMv(PIN_AZ_ADC);
  if(gsAzMode==36){
    if(idx<AZ36_CAL_N) az36CalMv[idx]=mv;
  }else{
    if(idx<AZ45_CAL_N) az45CalMv[idx]=mv;
  }
  refreshMultiCalValidity();
  saveCal();
  tftNeedsRedraw=true;
}

static void captureTouchMultiEl(uint8_t idx){
  if(idx>=EL_CAL_N) return;
  elCalMv[idx]=adcAvgMv(PIN_EL_ADC);
  refreshMultiCalValidity();
  saveCal();
  tftNeedsRedraw=true;
}


static void serviceTouchRecoveryHold(){
  if(tftPage!=TFT_PAGE_DASH) {
    touchHoldStartMs=0;
    touchHoldArmed=false;
    return;
  }

  if(touch.touched()){
    if(!touchHoldArmed){
      touchHoldArmed=true;
      touchHoldStartMs=millis();
    }else if(millis()-touchHoldStartMs>=2500){
      touchHoldArmed=false;
      touchHoldStartMs=0;
      startTouchCalibration();
    }
  }else{
    touchHoldArmed=false;
    touchHoldStartMs=0;
  }
}

static void serviceTouch(){
  if(tftPage==TFT_PAGE_IMAGE) return;
  if(tftPage==TFT_PAGE_TOUCH_CAL){ serviceTouchCalibration(); return; }

  // Raw long-press recovery does not depend on touch calibration.
  serviceTouchRecoveryHold();
  if(tftPage==TFT_PAGE_TOUCH_CAL) return;

  int x,y;
  if(!readTouchXY(x,y)) return;
  if(millis()-lastTouchMs<260) return;
  lastTouchMs=millis();

  // BACK exists on every calibration page.
  if(tftPage!=TFT_PAGE_DASH && hit(x,y,235,0,85,40)){
    if(tftPage==TFT_PAGE_CAL_HOME) tftPage=TFT_PAGE_DASH;
    else tftPage=TFT_PAGE_CAL_HOME;
    tftNeedsRedraw=true;
    drawTftPage();
    return;
  }

  if(tftPage==TFT_PAGE_DASH){
    // Tap the top title bar to show the cat image for 3 seconds.
    if(hit(x,y,0,0,320,40)){
      showHeaderImage();
    }else if(hit(x,y,0,174,125,36)){
      tftPage=TFT_PAGE_CAL_HOME;
      tftNeedsRedraw=true;
      drawTftPage();
    }else if(hit(x,y,198,204,122,36)){
      startTouchCalibration();
    }
    return;
  }

  if(tftPage==TFT_PAGE_CAL_HOME){
    if(hit(x,y,8,55,145,34)){
      setCalibrationMode(CAL_GS232_2POINT);
      saveCal();
      tftNeedsRedraw=true;
    }else if(hit(x,y,167,55,145,34)){
      setCalibrationMode(CAL_MULTI_POINT);
      saveCal();
      tftNeedsRedraw=true;
    }else if(hit(x,y,8,112,145,34)){
      setGsAzMode(36);
      saveCal();
      tftNeedsRedraw=true;
    }else if(hit(x,y,167,112,145,34)){
      setGsAzMode(45);
      saveCal();
      tftNeedsRedraw=true;
    }else if(hit(x,y,8,166,145,52)){
      tftPage=TFT_PAGE_CAL_AZ; tftNeedsRedraw=true;
    }else if(hit(x,y,167,166,145,52)){
      tftPage=TFT_PAGE_CAL_EL; tftNeedsRedraw=true;
    }
    if(tftNeedsRedraw) drawTftPage();
    return;
  }

  if(tftPage==TFT_PAGE_CAL_AZ){
    if(calibrationMode==CAL_GS232_2POINT){
      if(hit(x,y,8,70,145,48)) captureGs232Cal("O");
      else if(hit(x,y,167,70,145,48)) captureGs232Cal("F");
      else if(hit(x,y,4,158,312,62)) resetTouchCalAxis(true);
    }else{
      int n=gsAzMode==36?5:6;
      for(int i=0;i<n;i++){
        int row=i/3, col=i%3;
        if(hit(x,y,8+col*104,68+row*58,96,44)){
          captureTouchMultiAz((uint8_t)i);
          break;
        }
      }
      if(hit(x,y,4,184,312,52)) resetTouchCalAxis(true);
    }
    tftNeedsRedraw=true;
    drawTftPage();
    return;
  }

  if(tftPage==TFT_PAGE_CAL_EL){
    if(calibrationMode==CAL_GS232_2POINT){
      if(hit(x,y,8,70,145,48)) captureGs232Cal("O2");
      else if(hit(x,y,167,70,145,48)) captureGs232Cal("F2");
      else if(hit(x,y,4,158,312,62)) resetTouchCalAxis(false);
    }else{
      for(int i=0;i<5;i++){
        int row=i/3, col=i%3;
        if(hit(x,y,8+col*104,68+row*58,96,44)){
          captureTouchMultiEl((uint8_t)i);
          break;
        }
      }
      if(hit(x,y,4,184,312,52)) resetTouchCalAxis(false);
    }
    tftNeedsRedraw=true;
    drawTftPage();
  }
}

static void initLocalDisplay(){
  pinMode(PIN_TFT_CS,OUTPUT);
  pinMode(PIN_TOUCH_CS,OUTPUT);
  digitalWrite(PIN_TFT_CS,HIGH);
  digitalWrite(PIN_TOUCH_CS,HIGH);

  SPI.begin(PIN_TFT_SCK,PIN_TFT_MISO,PIN_TFT_MOSI,PIN_TFT_CS);
  tft.begin();
  tft.setRotation(1);

  touch.begin(SPI);
  touch.setRotation(1);

  tft.fillScreen(TFT_BG);
  tftText(23,62,2,TFT_TEXT,TFT_BG,"G-5500 ESP32-S3");
  tftText(40,92,2,TFT_TARGET,TFT_BG,"TFT + TOUCH");
  tftText(72,122,2,TFT_OK,TFT_BG,"UA1CFM");
  tftText(82,154,2,TFT_WARN,TFT_BG,"FW v1.6.23");
  delay(1200);

  tftPage=TFT_PAGE_DASH;
  tftNeedsRedraw=true;
}

static void updateLocalDisplay(){
  if(tftPage==TFT_PAGE_IMAGE){
    serviceHeaderImage();
    return;
  }
  if(tftPage==TFT_PAGE_DASH) updateDashboard();
  else if(tftNeedsRedraw) drawTftPage();
}

void setup(){
  bootResetReason = esp_reset_reason();
  bootResetReasonText = resetReasonToText(bootResetReason);

  // Preload output latches LOW before enabling output mode to reduce relay glitches at boot.
  digitalWrite(PIN_LEFT,LOW);
  digitalWrite(PIN_RIGHT,LOW);
  digitalWrite(PIN_UP,LOW);
  digitalWrite(PIN_DOWN,LOW);
  pinMode(PIN_LEFT,OUTPUT);
  pinMode(PIN_RIGHT,OUTPUT);
  pinMode(PIN_UP,OUTPUT);
  pinMode(PIN_DOWN,OUTPUT);
  allStop();
  rgbSet(0,0,0);
  // UART0 USB: PSTROTATOR GS-232 working channel
  Serial0.begin(9600);

  // HC-05 Bluetooth Classic on UART1
  pinMode(PIN_HC05_STATE,INPUT);
  HC05Serial.begin(HC05_BAUD, SERIAL_8N1, PIN_HC05_RX, PIN_HC05_TX);

  // Native USB Serial/JTAG: diagnostics/programming
#if ARDUINO_USB_MODE
  HWCDCSerial.begin();
#endif

  analogReadResolution(12);analogSetPinAttenuation(PIN_AZ_ADC,ADC_11db);analogSetPinAttenuation(PIN_EL_ADC,ADC_11db);
  serialCmd.reserve(80);
  hc05Cmd.reserve(80);
  initLocalDisplay();
  loadPrefs();
  loadTouchCalibration();

  // IMPORTANT: before the first successful touch calibration there is no
  // trustworthy screen-coordinate mapping, so do not require the user to hit
  // a mapped button. Start the raw 4-point wizard automatically.
  if(!touchCalValid){
    startTouchCalibration();
  }else{
    tftPage=TFT_PAGE_DASH;
    tftNeedsRedraw=true;
    drawTftPage();
  }
  initRssiHistory();

  dbgPrintln("");
  dbgPrintln("######## FIRMWARE v1.6.23 TFT TOUCH CAL + HC-05 ########");
  dbgPrintln("[DIAG] Web console: /diaglog, RAM ring buffer 160 lines");
  dbgPrint("[RESET REASON] "); dbgPrintln(bootResetReasonText);
  dbgPrint("[RESET CODE] "); dbgPrintln(String((int)bootResetReason));
  dbgPrintln("[V1559] USB debug alive");
  dbgPrintln("[V1559] internal BLE disabled; external HC-05 enabled");
  startWifi();

  bool timeOk=syncClockFromNtp();
  addResetHistoryEntry();
  dbgPrint("[TIME SYNC] "); dbgPrintln(timeOk ? "NTP OK" : "NO NTP - history entry has no exact time");

  dbgPrintln("");
  dbgPrintln("=== G5500 ESP32-S3 v1.6.23 TFT TOUCH CAL ===");
  dbgPrintln("Copyright (c) UA1CFM");
  dbgPrintln("UART0 USB: PSTROTATOR GS-232, 9600 baud (COM number assigned by Windows)");
  dbgPrintln("Native USB Serial/JTAG: diagnostics + programming (COM number assigned by Windows)");
  dbgPrintln("TCP 4533: Look4Sat/Hamlib + GS-232B full command set");
  dbgPrintln("R4UAB DDE Client TCP: decimal comma accepted in Hamlib P az el command");
  dbgPrintln("BUGFIX: Serial/Web manual or target commands cancel active GS-232 timed sequence");
  dbgPrintln("AP password: configurable from Web UI; empty = open, 8..63 chars = WPA2");
  dbgPrintln("RSSI history: 1h/1d/1w/1m/1y, year saved daily");
  dbgPrintln("Reset history: last 20 boots stored in NVS with NTP UTC time when available");
  dbgPrintln("Target control: LVBTrack-style; STOP halts motion, next target starts again");
  dbgPrintln("Manual control: cancels current axis tracking; next target starts automatically");
  dbgPrintln("Calibration: GS232 2-point + multi-point; separate AZ P36/P45 tables, shared EL");
  dbgPrintln("ADC averaging: 25 samples, matching LVBTrack smoothing depth");
  dbgPrintln("Manual axis behavior: AZ manual cancels AZ track only; EL manual cancels EL track only");
  dbgPrintln("Boot outputs: LOW preloaded before pinMode(OUTPUT)");
  dbgPrintln("Web calibration: explicit GS232 2-POINT / MULTI-POINT mode selector");
  dbgPrintln("Web calibration menu: collapsible, default closed");
  dbgPrintln("Calibration buttons: saved points highlighted in green");
  dbgPrintln("Calibration data stored separately; separate reset for each mode and axis");
  dbgPrintln("TFT SPI: MOSI GPIO10, SCK GPIO11, MISO GPIO2, TFT_CS GPIO12, DC GPIO13, RST GPIO14");
  dbgPrintln("XPT2046: TOUCH_CS GPIO8, TOUCH_IRQ GPIO9; shared SPI bus");
    dbgPrintln("Rotator movement: LVBTrack.c reference algorithm, no added hysteresis or hold logic");
  dbgPrintln("Web main cards: current / target shown inline for AZ and EL");
  dbgPrintln("TFT: ILI9341 320x240 dashboard + XPT2046 touch calibration UI");
  dbgPrintln("TFT calibration: 2-POINT / MULTI-POINT, P36/P45, AZ/EL capture + reset");
  dbgPrint("[WiFi AP]  ");
  dbgPrintln(WiFi.softAPIP().toString());
  if(WiFi.status()==WL_CONNECTED){
    dbgPrint("[WiFi LAN] ");
    dbgPrintln(WiFi.localIP().toString());
  }else{
    dbgPrintln("[WiFi LAN] not connected");
  }
  dbgPrintln("==============================");

  lastHeartbeatMs=millis();
}
void loop(){
  updatePosition();
  handlePstRotatorSerial();   // UART0 USB / PSTROTATOR / GS-232
  handleHc05Serial();         // HC-05 Bluetooth Classic / GS-232B
  handleRotctldTcp();         // Wi-Fi TCP 4533 / Look4Sat / Hamlib rotctld
  web.handleClient();
  serviceWifiReconnect();
  serviceRssiHistory();
  serviceGsSequence();
  controlRotator();
  serviceTouch();
  updateLocalDisplay();
  updateRgb();
  delay(5);
}
