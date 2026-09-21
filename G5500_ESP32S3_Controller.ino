/*
  G5500 ESP32-S3 Controller v1.5.49 NO BLE
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
#include <LiquidCrystal.h>
static const int PIN_AZ_ADC=4, PIN_EL_ADC=5;
static const int PIN_LCD_RS=8, PIN_LCD_E=9, PIN_LCD_D4=10, PIN_LCD_D5=11, PIN_LCD_D6=12, PIN_LCD_D7=13;
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
static const char* AP_PASS="g5500setup";

LiquidCrystal lcd(PIN_LCD_RS,PIN_LCD_E,PIN_LCD_D4,PIN_LCD_D5,PIN_LCD_D6,PIN_LCD_D7);
Preferences prefs;
WebServer web(80);
WiFiServer tcpServer(TCP_PORT);
WiFiClient rotctldClient;
String rotctldLine;

struct Calibration { int az0=0, azMax=4095, el0=0, elMax=4095; bool valid=false; } cal;
static const uint8_t AZ_CAL_N=6, EL_CAL_N=5;
static const float azCalDeg[AZ_CAL_N]={0,90,180,270,360,450};
static const float elCalDeg[EL_CAL_N]={0,45,90,135,180};
int azCalAdc[AZ_CAL_N]={-1,-1,-1,-1,-1,-1};
int elCalAdc[EL_CAL_N]={-1,-1,-1,-1,-1};
bool azMultiValid=false, elMultiValid=false;
bool azLinearValid=false, elLinearValid=false;
String wifiSsid,wifiPass;
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
bool gsSouthCenter=false;   // Z toggle; meaningful in 360-degree mode
uint8_t gsSpeed=4;          // X1..X4 accepted; relay hardware has fixed speed

static const int8_t RSSI_EMPTY = 127;
int8_t rssiHour[60], rssiDay[288], rssiWeek[336], rssiMonth[360], rssiYear[365];
uint16_t rssiHourPos=0,rssiHourCount=0,rssiDayPos=0,rssiDayCount=0;
uint16_t rssiWeekPos=0,rssiWeekCount=0,rssiMonthPos=0,rssiMonthCount=0,rssiYearPos=0,rssiYearCount=0;
int32_t acc1m=0,acc5m=0,acc30m=0,acc2h=0,acc24h=0;
uint16_t cnt1m=0,cnt5m=0,cnt30m=0,cnt2h=0,cnt24h=0;
uint32_t rssiLastSampleMs=0,rssiT1m=0,rssiT5m=0,rssiT30m=0,rssiT2h=0,rssiT24h=0;
static const uint32_t RSSI_SAMPLE_MS=10000UL;

uint32_t lastTargetCommandMs=0,lastLcdMs=0;

String serialCmd;
enum RgbState : uint8_t { RGB_IDLE, RGB_RX, RGB_TX, RGB_HEARTBEAT };
RgbState rgbState = RGB_IDLE;
uint32_t rgbUntilMs = 0;
uint32_t lastHeartbeatMs = 0;
bool pendingTxFlash = false;

static float clampf(float v,float lo,float hi){ return v<lo?lo:(v>hi?hi:v); }
static int adcAvg(int pin){ uint32_t s=0; for(int i=0;i<25;i++){s+=analogRead(pin);delayMicroseconds(150);} return s/25; }
static int adcAvgMv(int pin){ uint32_t s=0; for(int i=0;i<25;i++){s+=analogReadMilliVolts(pin);delayMicroseconds(150);} return s/25; }

// ADC diagnostic only. This DOES NOT change the position calculation.
// analogRead() = raw 12-bit code.
// analogReadMilliVolts() = calibrated millivolts from Arduino-ESP32 ADC calibration.
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
static void refreshMultiCalValidity(){
  azMultiValid=calPointsMonotonic(azCalAdc,AZ_CAL_N);
  elMultiValid=calPointsMonotonic(elCalAdc,EL_CAL_N);
}



static void dbgPrint(const String &s){
#if ARDUINO_USB_MODE
  if(HWCDCSerial) HWCDCSerial.print(s);
#endif
}
static void dbgPrintln(const String &s){
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
static void processGs232Command(String cmd);

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
  Serial0.print(s);
  dbgPstTx(s);
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

  if(cmd=="R"){ moveDir('R'); return; }
  if(cmd=="L"){ moveDir('L'); return; }
  if(cmd=="U"){ moveDir('U'); return; }
  if(cmd=="D"){ moveDir('D'); return; }
  if(cmd=="A"){ stopAzimuth(); return; }
  if(cmd=="E"){ stopElevation(); return; }
  if(cmd=="S"){ allStop(); return; }

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
    float az,el;
    if(sscanf(cmd.c_str()+1,"%f %f",&az,&el)==2){
      setReferenceTarget(clampf(az,0,450),clampf(el,0,180),true,true);
      lastTargetCommandMs=millis(); lastTargetRxMs=millis();
    }
    return;
  }

  // Azimuth-only GS-232 command: Maaa
  if(cmd[0]=='M' && cmd.length()>1){
    float az=cmd.substring(1).toFloat();
    setReferenceTarget(clampf(az,0,450),currentEl,true,false);
    lastTargetCommandMs=millis(); lastTargetRxMs=millis();
    return;
  }

  // P36/P45 are accepted for compatibility. This controller keeps its
  // physical AZ calibration at 0..450 degrees.
  if(cmd=="P36" || cmd=="P45") return;

  // Basic help response. Useful when testing manually in a serial terminal.
  if(cmd=="H" || cmd=="H2" || cmd=="H3"){
    sendReply("GS-232B: R L A U D E S C B C2 Maaa Waaa eee P36 P45\r\n");
    return;
  }
}
static void handlePstRotatorSerial(){
  while(Serial0.available()){
    char c=(char)Serial0.read();
    if(c=='\r' || c=='\n'){
      if(serialCmd.length()){
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


static void updatePosition(){
  // Working measurement path in v1.5.49:
  // factory-calibrated ESP32-S3 millivolts -> our mechanical calibration -> angle.
  int az=adcAvgMv(PIN_AZ_ADC), el=adcAvgMv(PIN_EL_ADC);

  if(azMultiValid) currentAz=mapPiecewise(az,azCalAdc,azCalDeg,AZ_CAL_N);
  else if(azLinearValid) currentAz=mapCal(az,cal.az0,cal.azMax,450.0f);
  else currentAz=clampf(az*450.0f/3100.0f,0,450);

  if(elMultiValid) currentEl=mapPiecewise(el,elCalAdc,elCalDeg,EL_CAL_N);
  else if(elLinearValid) currentEl=mapCal(el,cal.el0,cal.elMax,180.0f);
  else currentEl=clampf(el*180.0f/3100.0f,0,180);
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

  // v1.5.49 calibration format:
  // 0/1 = legacy RAW ADC calibration (do not reuse as mV)
  // 2   = calibrated millivolts from analogReadMilliVolts()
  uint8_t calFmt=prefs.getUChar("calfmt",0);

  if(calFmt==2){
    cal.az0=prefs.getInt("az0mv",0); cal.azMax=prefs.getInt("azmaxmv",3100);
    cal.el0=prefs.getInt("el0mv",0); cal.elMax=prefs.getInt("elmaxmv",3100);

    azLinearValid=prefs.getBool("azlinmv",false);
    elLinearValid=prefs.getBool("ellinmv",false);
    cal.valid=(azLinearValid && elLinearValid);

    azCalAdc[0]=prefs.getInt("azm0",cal.az0);
    azCalAdc[1]=prefs.getInt("azm90",-1);
    azCalAdc[2]=prefs.getInt("azm180",-1);
    azCalAdc[3]=prefs.getInt("azm270",-1);
    azCalAdc[4]=prefs.getInt("azm360",-1);
    azCalAdc[5]=prefs.getInt("azm450",cal.azMax);

    elCalAdc[0]=prefs.getInt("elm0",cal.el0);
    elCalAdc[1]=prefs.getInt("elm45",-1);
    elCalAdc[2]=prefs.getInt("elm90",-1);
    elCalAdc[3]=prefs.getInt("elm135",-1);
    elCalAdc[4]=prefs.getInt("elm180",cal.elMax);
  }else{
    // Deliberately invalidate legacy RAW calibration after migration.
    // User performs one new calibration in calibrated millivolts.
    cal.az0=0; cal.azMax=3100;
    cal.el0=0; cal.elMax=3100;
    cal.valid=false;
    azLinearValid=false; elLinearValid=false;
    for(uint8_t i=0;i<AZ_CAL_N;i++) azCalAdc[i]=-1;
    for(uint8_t i=0;i<EL_CAL_N;i++) elCalAdc[i]=-1;
  }
  prefs.end();
  refreshMultiCalValidity();
}

static void saveCal(){
  prefs.begin("g5500",false);
  prefs.putUChar("calfmt",2);

  prefs.putInt("az0mv",cal.az0); prefs.putInt("azmaxmv",cal.azMax);
  prefs.putInt("el0mv",cal.el0); prefs.putInt("elmaxmv",cal.elMax);
  prefs.putBool("azlinmv",azLinearValid);
  prefs.putBool("ellinmv",elLinearValid);

  prefs.putInt("azm0",azCalAdc[0]); prefs.putInt("azm90",azCalAdc[1]);
  prefs.putInt("azm180",azCalAdc[2]); prefs.putInt("azm270",azCalAdc[3]);
  prefs.putInt("azm360",azCalAdc[4]); prefs.putInt("azm450",azCalAdc[5]);

  prefs.putInt("elm0",elCalAdc[0]); prefs.putInt("elm45",elCalAdc[1]);
  prefs.putInt("elm90",elCalAdc[2]); prefs.putInt("elm135",elCalAdc[3]);
  prefs.putInt("elm180",elCalAdc[4]);

  prefs.end();
  cal.valid=(azLinearValid && elLinearValid);
  refreshMultiCalValidity();
}

static void saveWifi(String s,String p){
  prefs.begin("g5500",false);prefs.putString("ssid",s);prefs.putString("pass",p);prefs.end();
}

static const char WEB_PAGE[] PROGMEM = R"HTML(
<!doctype html><html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>G5500 Controller v1.5.49 - UA1CFM</title>
<style>
:root{--bg:#081018;--p:#111d28;--p2:#162635;--t:#f4f7fa;--m:#96a9b7;--a:#24d18d;--s:#e74c3c;--b:#2b4557}
*{box-sizing:border-box}body{margin:0;background:linear-gradient(#081018,#0b1721);color:var(--t);font-family:Arial,sans-serif}
.wrap{max-width:980px;margin:auto;padding:16px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:14px}
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
@media(max-width:700px){.grid{grid-template-columns:1fr}.big{font-size:46px}.targetbig{font-size:30px}.sep{font-size:26px}}
</style></head><body><div class="wrap">
<div style="font-size:13px;color:#96a9b7;margin:2px 0 6px">G5500 Controller v1.5.49 &copy; UA1CFM</div>
<h1>YAESU G-5500 / ESP32-S3</h1>
<div class="grid">
<div class="card"><div class="label">Азимут — текущее / задание</div><div><span id="az" class="big">--.-</span><span class="sep"> / </span><span id="azt" class="targetbig">---</span><span class="unit">°</span></div><div class="bar"><div id="azbar" class="fill"></div></div><div>0° <span style="float:right">450°</span></div></div>
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
<div class="field"><label>AZ 0..450°</label><input id="taz" type="number" step="0.1" min="0" max="450"></div>
<div class="field"><label>EL 0..180°</label><input id="tel" type="number" step="0.1" min="0" max="180"></div>
<button class="btn good" onclick="go()">GO</button></div></div>
<div class="card">
<button id="caltoggle" class="cal-toggle" onclick="toggleCalMenu()">Многоточечная калибровка <span class="arrow">▼</span></button>
<div id="calbody" class="cal-body">
<div class="calhead"><div style="font-weight:700">AZ</div><div class="callive"><span id="calaz">--.-</span><span>°</span></div></div>
<div class="row" style="margin-top:8px">
<button id="b_az0" class="btn warn" onclick="cal('az0')">AZ 0°</button><button id="b_az90" class="btn warn" onclick="cal('az90')">AZ 90°</button>
<button id="b_az180" class="btn warn" onclick="cal('az180')">AZ 180°</button><button id="b_az270" class="btn warn" onclick="cal('az270')">AZ 270°</button>
<button id="b_az360" class="btn warn" onclick="cal('az360')">AZ 360°</button><button id="b_az450" class="btn warn" onclick="cal('az450')">AZ 450°</button></div>
<div class="kv" style="margin-top:10px">
<div>AZ0 mV: <b id="azp0">-</b></div><div>AZ90 mV: <b id="azp90">-</b></div>
<div>AZ180 mV: <b id="azp180">-</b></div><div>AZ270 mV: <b id="azp270">-</b></div>
<div>AZ360 mV: <b id="azp360">-</b></div><div>AZ450 mV: <b id="azp450">-</b></div></div>
<div class="row" style="margin-top:10px"><button class="btn calreset" onclick="resetCal('az')">СБРОСИТЬ КАЛИБРОВКУ AZ</button></div>
<div class="calhead" style="margin-top:14px"><div style="font-weight:700">EL</div><div class="callive"><span id="calel">--.-</span><span>°</span></div></div>
<div class="row" style="margin-top:8px">
<button id="b_el0" class="btn warn" onclick="cal('el0')">EL 0°</button><button id="b_el45" class="btn warn" onclick="cal('el45')">EL 45°</button>
<button id="b_el90" class="btn warn" onclick="cal('el90')">EL 90°</button><button id="b_el135" class="btn warn" onclick="cal('el135')">EL 135°</button>
<button id="b_el180" class="btn warn" onclick="cal('el180')">EL 180°</button></div>
<div class="kv" style="margin-top:10px">
<div>EL0 mV: <b id="elp0">-</b></div><div>EL45 mV: <b id="elp45">-</b></div>
<div>EL90 mV: <b id="elp90">-</b></div><div>EL135 mV: <b id="elp135">-</b></div>
<div>EL180 mV: <b id="elp180">-</b></div><div>Mode: <b id="calmode">-</b></div></div>
<div class="row" style="margin-top:10px"><button class="btn calreset" onclick="resetCal('el')">СБРОСИТЬ КАЛИБРОВКУ EL</button></div>
</div></div>
<div class="card"><div class="label">История RSSI домашнего Wi-Fi</div>
<div class="row" style="margin-top:10px"><button class="btn rbtn active" data-r="hour">1 HOUR</button><button class="btn rbtn" data-r="day">1 DAY</button><button class="btn rbtn" data-r="week">1 WEEK</button><button class="btn rbtn" data-r="month">1 MONTH</button><button class="btn rbtn" data-r="year">1 YEAR</button></div>
<canvas id="rchart" width="900" height="260"></canvas>
<div class="kv" style="margin-top:10px"><div>MIN: <b id="rmin">-</b></div><div>MAX: <b id="rmax">-</b></div><div>AVG: <b id="ravg">-</b></div><div>Points: <b id="rcount">0</b></div></div></div>
<div class="card"><div class="label">Подключение</div><div class="kv" style="margin-top:10px">
<div>Home IP: <b id="ip">-</b></div><div>AP IP: <b>192.168.4.1</b></div><div>TCP rotctld: <b>4533</b></div><div>Mode: <b id="mode">-</b></div><div>Home Wi-Fi RSSI: <b id="rssi">-</b></div><div>Signal: <b id="sig">-</b></div></div>
<div class="field" style="margin-top:10px"><label>Wi‑Fi SSID</label><input id="ssid"></div>
<div class="field" style="margin-top:8px"><label>Wi‑Fi password</label><input id="pass" type="password"></div>
<button class="btn good" style="margin-top:10px" onclick="wifi()">Save Wi‑Fi + reboot</button><a href="/adc" style="display:inline-block;margin:10px 0 0 10px;color:#24d18d;font-weight:700;text-decoration:none">ADC TEST →</a><a href="/resetdiag" style="display:inline-block;margin:10px 0 0 14px;color:#24d18d;font-weight:700;text-decoration:none">RESET DIAG →</a></div>
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
async function resetCal(axis){
  const name=axis==='az'?'АЗИМУТА':'ЭЛЕВАЦИИ';
  if(!confirm('Сбросить все точки калибровки '+name+'?')) return;
  await q('/api/calreset?axis='+axis);
  setTimeout(upd,150);
}
function markCalBtn(id, ok){
  const b=document.getElementById(id);
  if(!b) return;
  b.classList.toggle('done', !!ok);
  b.classList.toggle('warn', !ok);
}
async function wifi(){if(!ssid.value)return;if(confirm('Сохранить Wi‑Fi и перезагрузить контроллер?'))await q('/api/wifi?ssid='+encodeURIComponent(ssid.value)+'&pass='+encodeURIComponent(pass.value))}
function pct(v,m){v=Math.max(0,Math.min(m,v));return(v/m*100).toFixed(1)+'%'}
async function upd(){try{let j=await (await fetch('/api/status',{cache:'no-store'})).json();
az.textContent=Number(j.az).toFixed(1);el.textContent=Number(j.el).toFixed(1);
azt.textContent=j.targetValid?Number(j.targetAz).toFixed(1):'---';
elt.textContent=j.targetValid?Number(j.targetEl).toFixed(1):'---';
calaz.textContent=Number(j.az).toFixed(1);calel.textContent=Number(j.el).toFixed(1);
azbar.style.width=pct(j.az,450);elbar.style.width=pct(j.el,180);
motion.textContent=j.motion;
caltxt.textContent=(j.azMulti&&j.elMulti)?'Multi-point calibration (mV) OK':(j.calibrated?'Linear calibration (mV) OK':'Not calibrated');caldot.className='dot '+(((j.azMulti&&j.elMulti)||j.calibrated)?'on':'');
azp0.textContent=j.azp0;azp90.textContent=j.azp90;azp180.textContent=j.azp180;azp270.textContent=j.azp270;azp360.textContent=j.azp360;azp450.textContent=j.azp450;
elp0.textContent=j.elp0;elp45.textContent=j.elp45;elp90.textContent=j.elp90;elp135.textContent=j.elp135;elp180.textContent=j.elp180;
calmode.textContent=(j.azMulti?'AZ multi':(j.azLinear?'AZ linear':'AZ none'))+' / '+(j.elMulti?'EL multi':(j.elLinear?'EL linear':'EL none'));
markCalBtn('b_az0', j.azp0>=0); markCalBtn('b_az90', j.azp90>=0); markCalBtn('b_az180', j.azp180>=0);
markCalBtn('b_az270', j.azp270>=0); markCalBtn('b_az360', j.azp360>=0); markCalBtn('b_az450', j.azp450>=0);
markCalBtn('b_el0', j.elp0>=0); markCalBtn('b_el45', j.elp45>=0); markCalBtn('b_el90', j.elp90>=0);
markCalBtn('b_el135', j.elp135>=0); markCalBtn('b_el180', j.elp180>=0);
ip.textContent=j.ip;mode.textContent=j.wifiMode;
rssi.textContent=(j.rssi===null?'-':j.rssi+' dBm');
sig.textContent=j.signal||'-';
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
<title>G5500 ADC TEST v1.5.49</title>
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
</style></head><body><div class="wrap">
<div class="muted" style="font-size:13px">G5500 Controller v1.5.49 &copy; UA1CFM</div>
<h1>ADC TEST — ESP32-S3</h1>
<div><a href="/">← Вернуться к управлению G5500</a></div>

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
<title>G5500 RESET DIAG v1.5.49</title>
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
</style></head><body><div class="wrap">
<div class="muted" style="font-size:13px">G5500 Controller v1.5.49 &copy; UA1CFM</div>
<h1>RESET DIAG</h1>
<div><a href="/">← Вернуться к управлению G5500</a> &nbsp;&nbsp; <a href="/adc">ADC TEST →</a></div>

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


static void setupWeb(){
  web.on("/",HTTP_GET,[]{web.send_P(200,"text/html; charset=utf-8",WEB_PAGE);});
  web.on("/resetdiag",HTTP_GET,[]{web.send_P(200,"text/html; charset=utf-8",RESET_DIAG_PAGE);});
  web.on("/adc",HTTP_GET,[]{web.send_P(200,"text/html; charset=utf-8",ADC_TEST_PAGE);});
  web.on("/api/adcdiag",HTTP_GET,[]{
    int azRaw=0,azMv=0,elRaw=0,elMv=0;
    adcDiagRead(PIN_AZ_ADC,azRaw,azMv);
    adcDiagRead(PIN_EL_ADC,elRaw,elMv);
    String j="{\"azRaw\":"+String(azRaw)+",\"azMv\":"+String(azMv)+",\"elRaw\":"+String(elRaw)+",\"elMv\":"+String(elMv)+"}";
    web.send(200,"application/json",j);
  });
  web.on("/api/move",HTTP_GET,[]{String d=web.arg("d");moveDir(d.length()?d[0]:'S');web.send(200,"text/plain","OK");});
  web.on("/api/target",HTTP_GET,[]{
    setReferenceTarget(clampf(web.arg("az").toFloat(),0,450),clampf(web.arg("el").toFloat(),0,180),true,true);
    lastTargetCommandMs=millis(); lastTargetRxMs=millis();
    web.send(200,"text/plain","OK");
  });
  web.on("/api/cal",HTTP_GET,[]{
    String p=web.arg("p");int a=adcAvgMv(PIN_AZ_ADC),e=adcAvgMv(PIN_EL_ADC);
    if(p=="az0"){azCalAdc[0]=a;cal.az0=a;} else if(p=="az90")azCalAdc[1]=a; else if(p=="az180")azCalAdc[2]=a;
    else if(p=="az270")azCalAdc[3]=a; else if(p=="az360")azCalAdc[4]=a; else if(p=="az450"){azCalAdc[5]=a;cal.azMax=a;}
    else if(p=="el0"){elCalAdc[0]=e;cal.el0=e;} else if(p=="el45")elCalAdc[1]=e; else if(p=="el90")elCalAdc[2]=e;
    else if(p=="el135")elCalAdc[3]=e; else if(p=="el180"){elCalAdc[4]=e;cal.elMax=e;}
    azLinearValid=(azCalAdc[0]>=0 && azCalAdc[AZ_CAL_N-1]>=0 && abs(cal.azMax-cal.az0)>100);
    elLinearValid=(elCalAdc[0]>=0 && elCalAdc[EL_CAL_N-1]>=0 && abs(cal.elMax-cal.el0)>100);
    cal.valid=(azLinearValid && elLinearValid);
    saveCal();
    web.send(200,"text/plain","OK");
  });
  web.on("/api/calreset",HTTP_GET,[]{
    String axis=web.arg("axis");
    axis.toLowerCase();

    if(axis=="az"){
      for(uint8_t i=0;i<AZ_CAL_N;i++) azCalAdc[i]=-1;
      cal.az0=0;
      cal.azMax=3100;
      azMultiValid=false;
      azLinearValid=false;
    }else if(axis=="el"){
      for(uint8_t i=0;i<EL_CAL_N;i++) elCalAdc[i]=-1;
      cal.el0=0;
      cal.elMax=3100;
      elMultiValid=false;
      elLinearValid=false;
    }else{
      web.send(400,"text/plain","BAD AXIS");
      return;
    }

    cal.valid=(azLinearValid && elLinearValid);
    saveCal();
    web.send(200,"text/plain","RESET OK");
  });
  web.on("/api/wifi",HTTP_GET,[]{saveWifi(web.arg("ssid"),web.arg("pass"));web.send(200,"text/plain","Saved");delay(500);ESP.restart();});
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
    String j="{\"az\":"+String(currentAz,2)+",\"el\":"+String(currentEl,2)+",\"targetAz\":"+String(targetAz,2)+",\"targetEl\":"+String(targetEl,2)+",\"targetValid\":"+(targetValid?"true":"false")+",\"targetAgeMs\":"+String(targetAge)+",\"calibrated\":"+(cal.valid?"true":"false")+",\"azLinear\":"+(azLinearValid?"true":"false")+",\"elLinear\":"+(elLinearValid?"true":"false")+",\"azMulti\":"+(azMultiValid?"true":"false")+",\"elMulti\":"+(elMultiValid?"true":"false")+",\"motion\":\""+motion()+"\",\"azp0\":"+String(azCalAdc[0])+",\"azp90\":"+String(azCalAdc[1])+",\"azp180\":"+String(azCalAdc[2])+",\"azp270\":"+String(azCalAdc[3])+",\"azp360\":"+String(azCalAdc[4])+",\"azp450\":"+String(azCalAdc[5])+",\"elp0\":"+String(elCalAdc[0])+",\"elp45\":"+String(elCalAdc[1])+",\"elp90\":"+String(elCalAdc[2])+",\"elp135\":"+String(elCalAdc[3])+",\"elp180\":"+String(elCalAdc[4])+",\"ip\":\""+ip+"\",\"apIp\":\""+apIp+"\",\"staIp\":\""+staIp+"\",\"wifiMode\":\""+mode+"\",\"ssid\":\""+wifiSsid+"\",\"rssi\":"+rssiJson+",\"signal\":\""+signal+"\",\"resetReason\":\""+bootResetReasonText+"\"}";
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
  float az=clampf(gsSeqAz[0],0,gsAzMode==36?360.0f:450.0f);
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
    float az=clampf(gsSeqAz[gsSeqIndex],0,gsAzMode==36?360.0f:450.0f);
    float el=gsSeqHasEl?clampf(gsSeqEl[gsSeqIndex],0,180):currentEl;
    setReferenceTarget(az,el,true,gsSeqHasEl);
    gsSeqNextMs=millis()+(uint32_t)gsSeqIntervalSec*1000UL;
  }
}

static void rotctldSend(const String &s){
  if(rotctldClient && rotctldClient.connected()){
    rotctldClient.print(s);
  }
#if ARDUINO_USB_MODE
  dbgPrint("[TCP TX] ");
  String t=s;
  t.replace("\r","");
  t.replace("\n"," | ");
  dbgPrintln(t);
#endif
  flashTx();
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
      setReferenceTarget(clampf(gsSeqAz[0],0,gsAzMode==36?360.0f:450.0f),currentEl,true,false);
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
      setReferenceTarget(clampf(gsSeqAz[0],0,gsAzMode==36?360.0f:450.0f),clampf(gsSeqEl[0],0,180),true,true);
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

  if(upper=="P36"){ gsAzMode=36; gsSouthCenter=false; return; }
  if(upper=="P45"){ gsAzMode=45; gsSouthCenter=false; return; }
  if(upper=="Z"){
    if(gsAzMode==36) gsSouthCenter=!gsSouthCenter;
    return;
  }

  // X1..X4 are accepted for software compatibility. The G-5500 relay
  // outputs in this controller are ON/OFF, so physical speed is unchanged.
  if(upper.length()==2 && upper[0]=='X' && upper[1]>='1' && upper[1]<='4'){
    gsSpeed=(uint8_t)(upper[1]-'0');
    return;
  }

  // Calibration commands are recognized but deliberately do not alter our
  // stored web calibration. This avoids accidental calibration from a client.
  if(upper=="O" || upper=="O2" || upper=="F" || upper=="F2"){
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
    String s="P45 P36 Z MODE=";
    s += (gsAzMode==45 ? "450" : "360");
    s += (gsSouthCenter ? " S-CENTER\r\n" : " N-CENTER\r\n");
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
      setReferenceTarget(clampf(az,0,450),clampf(el,0,180),true,true);
      lastTargetCommandMs=millis(); lastTargetRxMs=millis();

#if ARDUINO_USB_MODE
      dbgPrint("[LOOK4SAT TARGET] AZ=");
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
    rotctldSend("G5500 ESP32-S3 v1.5.49 UA1CFM Look4Sat bridge\n");
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

  bool apOk = WiFi.softAP(AP_SSID);   // open AP for validation
  delay(300);

#if ARDUINO_USB_MODE
  dbgPrintln("[V1549] AP-only start");
  dbgPrint("[V1549] AP start = ");
  dbgPrintln(apOk ? "OK" : "FAILED");
  dbgPrint("[V1549] AP IP = ");
  dbgPrintln(WiFi.softAPIP().toString());
#endif

  // Try home Wi-Fi once, only if credentials are stored.
  if(wifiSsid.length()){
#if ARDUINO_USB_MODE
    dbgPrint("[V1549] one STA attempt: ");
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
      dbgPrint("[V1549] LAN connected = ");
      dbgPrintln(WiFi.localIP().toString());
      dbgPrint("[V1549] AP remains = ");
      dbgPrintln(WiFi.softAPIP().toString());
#endif
    }else{
      // Home network not available: completely stop STA and restore AP-only.
      WiFi.disconnect(true, false);
      delay(150);
      WiFi.mode(WIFI_AP);
      delay(150);

      WiFi.softAPConfig(apIP,apGW,apMask);
      WiFi.softAP(AP_SSID);
      delay(250);

#if ARDUINO_USB_MODE
      dbgPrintln("[V1549] LAN unavailable -> AP ONLY");
      dbgPrint("[V1549] AP restored = ");
      dbgPrintln(WiFi.softAPIP().toString());
#endif
    }
  }else{
#if ARDUINO_USB_MODE
    dbgPrintln("[V1549] no saved LAN credentials -> AP ONLY");
#endif
  }

  tcpServer.begin();
  setupWeb();
}
static void updateLcd(){
  if(millis()-lastLcdMs<250)return;lastLcdMs=millis();
  // LCD: integer current position, then commanded target in parentheses.
  // Example:
  // AZ 14 (20)
  // EL 5 (12)
  int azNow=constrain((int)lroundf(currentAz),0,450);
  int elNow=constrain((int)lroundf(currentEl),0,180);
  int azCmd=constrain((int)lroundf(targetAz),0,450);
  int elCmd=constrain((int)lroundf(targetEl),0,180);

  char line1[17], line2[17];
  if(targetValid){
    snprintf(line1,sizeof(line1),"AZ %d (%d)",azNow,azCmd);
    snprintf(line2,sizeof(line2),"EL %d (%d)",elNow,elCmd);
  }else{
    snprintf(line1,sizeof(line1),"AZ %d (---)",azNow);
    snprintf(line2,sizeof(line2),"EL %d (---)",elNow);
  }

  lcd.setCursor(0,0);
  lcd.print(line1);
  for(int i=strlen(line1);i<16;i++) lcd.print(' ');

  lcd.setCursor(0,1);
  lcd.print(line2);
  for(int i=strlen(line2);i<16;i++) lcd.print(' ');
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
  // UART0 USB: PSTROTATOR GS-232 working channel (Windows COM number may vary)
  Serial0.begin(9600);

  // Native USB Serial/JTAG: diagnostics/programming (Windows COM number may vary)
#if ARDUINO_USB_MODE
  HWCDCSerial.begin();
#endif

  analogReadResolution(12);analogSetPinAttenuation(PIN_AZ_ADC,ADC_11db);analogSetPinAttenuation(PIN_EL_ADC,ADC_11db);
  serialCmd.reserve(80);
  lcd.begin(16,2);
  lcd.setCursor(0,0);
  lcd.print("G5500 ESP32-S3");
  lcd.setCursor(0,1);
  lcd.print("(c)UA1CFM 1.5.49");
  loadPrefs();
  initRssiHistory();

  dbgPrintln("");
  dbgPrintln("######## FIRMWARE v1.5.49 NO BLE ########");
  dbgPrint("[RESET REASON] "); dbgPrintln(bootResetReasonText);
  dbgPrint("[RESET CODE] "); dbgPrintln(String((int)bootResetReason));
  dbgPrintln("[V1549] USB debug alive");
  dbgPrintln("[V1549] BLE disabled - Flash/RAM saving");
startWifi();

  bool timeOk=syncClockFromNtp();
  addResetHistoryEntry();
  dbgPrint("[TIME SYNC] "); dbgPrintln(timeOk ? "NTP OK" : "NO NTP - history entry has no exact time");

  dbgPrintln("");
  dbgPrintln("=== G5500 ESP32-S3 v1.5.49 ===");
  dbgPrintln("Copyright (c) UA1CFM");
  dbgPrintln("UART0 USB: PSTROTATOR GS-232, 9600 baud (COM number assigned by Windows)");
  dbgPrintln("Native USB Serial/JTAG: diagnostics + programming (COM number assigned by Windows)");
  dbgPrintln("TCP 4533: Look4Sat/Hamlib + GS-232B full command set");
  dbgPrintln("R4UAB DDE Client TCP: decimal comma accepted in Hamlib P az el command");
  dbgPrintln("RSSI history: 1h/1d/1w/1m/1y, year saved daily");
  dbgPrintln("Reset history: last 20 boots stored in NVS with NTP UTC time when available");
  dbgPrintln("Target control: LVBTrack-style; STOP halts motion, next target starts again");
  dbgPrintln("Manual control: cancels current axis tracking; next target starts automatically");
  dbgPrintln("Calibration: AZ 6-point / EL 5-point piecewise linear");
  dbgPrintln("ADC averaging: 25 samples, matching LVBTrack smoothing depth");
  dbgPrintln("Manual axis behavior: AZ manual cancels AZ track only; EL manual cancels EL track only");
  dbgPrintln("Boot outputs: LOW preloaded before pinMode(OUTPUT)");
  dbgPrintln("Web calibration: live AZ/EL duplicated beside calibration buttons");
  dbgPrintln("Web calibration menu: collapsible, default closed");
  dbgPrintln("Calibration buttons: saved points highlighted in green");
  dbgPrintln("Calibration reset: separate AZ / EL reset buttons");
  dbgPrintln("Rotator movement: LVBTrack.c reference algorithm, no added hysteresis or hold logic");
  dbgPrintln("Web main cards: current / target shown inline for AZ and EL");
  dbgPrintln("LCD: integer AZ/EL with commanded target in parentheses");
  dbgPrintln("LCD startup: (c)UA1CFM 1.5.49 shown on second line");
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
  handleRotctldTcp();         // Wi-Fi TCP 4533 / Look4Sat / Hamlib rotctld
  web.handleClient();
  serviceRssiHistory();
  serviceGsSequence();
  controlRotator();
  updateLcd();
  updateRgb();
  delay(5);
}
