#include <HardwareSerial.h>
#include <Preferences.h>

// Dart1 Firmware - IRcamera Protocol Prototype
// Based on IRcamera Detailed Command Set.
// Preserves original button mapping and behavior.
#define VERBOSE  1

#define NAV_LEFT_PIN 0
#define NAV_RIGHT_PIN 1
#define THUMB_BUTTON 2
#define EXTRA_BUTTON_PIN 6
#define NAV_MID_PIN 7
#define LED_BLUE 8
#define SCREEN_BRIGHT_PLUS   9
#define SCREEN_BRIGHT_MINUS  10
#define RX_PIN 20
#define TX_PIN 21

#define LED_ON LOW
#define LED_OFF HIGH

#define MAX_PALLETS 6

/*****************Data Structures**************************/

// Structure to store extracted Section A Runtime Status fields
typedef struct {
    uint32_t uptime_ms;           // Protocol runtime after startup (ms)
    int16_t  chip_temp;           // Chip temperature (sampled cache)
    uint16_t reserved0;           // Reserved
    uint16_t img_w;               // Sensor input image width
    uint16_t img_h;               // Sensor input image height
    uint16_t fps_x10;             // Framerate x10 (e.g., 500 = 50.0 fps)
    uint16_t vo_w;                // Current video output width[cite: 1]
    uint16_t vo_h;                // Current video output height[cite: 1]
    uint8_t  run_isp;             // ISP main pipeline running status[cite: 1]
    uint8_t  auto_cut_en;         // Automatic shutter enable flag[cite: 1]
    uint8_t  run_pause;           // Application processing paused status[cite: 1]
    uint8_t  freeze_en;          // Current image freeze status[cite: 1]
    uint8_t  hmirr_en;            // Horizontal mirror switch[cite: 1]
    uint8_t  vmirr_en;            // Vertical mirror switch[cite: 1]
    uint8_t  sffc_alg;            // Current SFFC algorithm mode[cite: 1]
    uint8_t  fix_snc_en;          // Fixed SNC switch[cite: 1]
    uint8_t  cvbs_en;             // CVBS output enable[cite: 1]
    uint8_t  mem_en;              // Memory / USB output enable[cite: 1]
    uint8_t  mipi_en;             // MIPI output enable[cite: 1]
    uint8_t  reserved1;           // Reserved[cite: 1]
    uint16_t cut_width;           // Crop window width[cite: 1]
    uint16_t cut_height;          // Crop window height[cite: 1]
    uint16_t cut_start_h;         // Crop window horizontal start[cite: 1]
    uint16_t cut_start_v;         // Crop window vertical start[cite: 1]
    uint32_t fault_bitmap;        // Fault bitmap[cite: 1]
    uint32_t event_seq;           // Reserved field[cite: 1]
} runtime_status_t;

struct ISPParams {
  uint8_t enable=1,luma=50,contrast=50,nr=0,enhance=0,agc=0;
} isp;

/**********Global Variables******************/

HardwareSerial &cam = Serial1;
Preferences preferences;

int brightness=100;
int screenBrightness = 5;
const int screenBrightnessMin = 0;
const int screenBrightnessMax = 10;
int contrastIdx=3;
int contrastSteps[8]={13,25,38,50,63,75,88,100};
uint8_t pseudocolor=6;
uint16_t zoomX10=10;
bool autoCalEnabled=false;
runtime_status_t status;

unsigned long thumbDownTime=0, navMidDownTime=0,lastBtnPressTime=0,lastNavMoveTime=0,lastExtraPressTime=0,thumbActionCooldown=0;
bool thumbWasInteracted=false;

/*************Funtion Prototypes************/
uint16_t crc16_modbus(const uint8_t *, uint32_t);
void printHex(const char*, const uint8_t*, size_t);
void sendFrame(uint8_t rw,uint8_t cmd,uint8_t sub,const uint8_t *data,uint8_t dlen);
void ph(uint8_t);
void clearRx();
void readResponse(uint32_t);
void sendRawFrame(uint8_t *, size_t, bool);
void manualShutter();
void logicalOOC();
void autoShutter(bool);
void saveParams();
void writeISP(uint8_t);
void setBrightness(uint8_t);
void setContrast(uint8_t);
void enablePseudoColor();
void setPseudocolor(uint8_t);
void setZoom(uint16_t);
void setScreenBrightness(int);
void setMiscStartupCommands(void);
void runtimeStatusQuery(void);
void parseStatus(uint8_t *);
/***************Functions*******************/

uint16_t crc16_modbus(const uint8_t *buf, uint32_t len){
 uint16_t crc=0xFFFF;
 for(uint32_t i=0;i<len;i++){
  crc^=buf[i];
  for(int j=0;j<8;j++) crc=(crc&1)?((crc>>1)^0xA001):(crc>>1);
 }
 return crc;
}

void printHex(const char* label, const uint8_t* data, size_t len)
{
Serial.print(label);

  for (size_t i = 0; i < len; i++)
  {
    if (data[i] < 16)
      Serial.print("0");

      Serial.print(data[i], HEX);
      Serial.print(" ");
  }

  Serial.println();
}

void sendFrame(uint8_t rw,uint8_t cmd,uint8_t sub,const uint8_t *data,uint8_t dlen){
 uint8_t buf[300];
 buf[0]=0x55; buf[1]=0xAA; buf[2]=dlen+5; buf[3]=rw; buf[4]=cmd; buf[5]=sub;
 for(int i=0;i<dlen;i++) buf[6+i]=data[i];
 uint16_t crc=crc16_modbus(buf,6+dlen);
 buf[6+dlen]=(crc>>8)&0xFF;
 buf[7+dlen]=crc&0xFF;
 cam.write(buf,8+dlen);
 if(VERBOSE) printHex("TX: ",buf,8+dlen);
}

void ph(uint8_t b) {
  if (b < 0x10) Serial.print('0');
  Serial.print(b, HEX);
}

void clearRx() {
  while (cam.available()) cam.read();
}

void readResponse(uint32_t timeoutMs = 450) {
  uint32_t start = millis(), last = start;
  bool got = false;
  uint8_t rxBuffer[100];
  uint16_t rxBuffIndex = 0;

  Serial.print("RX: ");
  while (millis() - start < timeoutMs) {
    while (cam.available()) {
      uint8_t b = cam.read();
      got = true;
      last = millis();
      ph(b);
      Serial.print(' ');
      if (rxBuffIndex < 100)
        rxBuffer[rxBuffIndex++] = b;
    }
    if (got && millis() - last > 50) break;
    delay(1);
  }
  if (!got) Serial.print("<no data>");
  Serial.println();

  if (rxBuffer[4] == 0xE0 && rxBuffer[5] == 0)
  {
    parseStatus(rxBuffer);
  }
}

void sendRawFrame(uint8_t *frame, size_t len, bool print = true) {
  uint16_t crc = crc16_modbus(frame, len - 2);
  frame[len - 2] = crc >> 8;
  frame[len - 1] = crc & 0xFF;

  clearRx();

  if (print) {
    Serial.print("[TX] ");
    for (size_t i = 0; i < len; i++) {
      ph(frame[i]);
      if (i + 1 < len) Serial.print(' ');
    }
    Serial.println();
  }

  cam.write(frame, len);
  cam.flush();

  if (print) readResponse();
}

void manualShutter(){ sendFrame(0x00,0x40,0x00,nullptr,0); }
void logicalOOC(){ sendFrame(0x00,0x10,0x02,nullptr,0); }
void autoShutter(bool en){ sendFrame(0x00,0x40,en?0x03:0x04,nullptr,0); }
void saveParams(){ sendFrame(0x00,0x40,0xFF,nullptr,0); }

void writeISP(uint8_t mask){
 uint8_t d[8]={0,mask,isp.enable,isp.luma,isp.contrast,isp.nr,isp.enhance,isp.agc};
 sendFrame(0x00,0xE3,0x01,d,8);
}
void setBrightness(uint8_t v){ isp.luma=v; brightness=v; writeISP(0x02); }
void setContrast(uint8_t v){ isp.contrast=v; writeISP(0x04); }

void enablePseudoColor() {
  // Confirmed command used during testing.
  const uint8_t raw[] = {
    0x01,0x01,0x80,0x00,
    0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x1F,0x00,0x00,0x00,0x00,0x45,0x20
  };

  clearRx();
  Serial.println("[CORE] Pseudo-Color activated");
  cam.write(raw, sizeof(raw));
  cam.flush();
  readResponse();
}

void setPseudocolor(uint8_t slot) {
  if (slot > MAX_PALLETS) return;

  uint8_t f[12] = {
    0x55,0xAA,0x09,0x00,0xE2,0x04,
    0x00,0x00,0x00,slot,
    0x00,0x00
  };

  Serial.print("[PALETTE] ");
  Serial.println(slot);
  sendRawFrame(f, sizeof(f));
}

void setZoom(uint16_t ratioX10){
 uint8_t d[14]={0};
 d[0]=0; // center anchor
 d[1]=1; //horizontal mirror
 d[4]=(ratioX10>>8)&0xFF;
 d[5]=ratioX10&0xFF;
 sendFrame(0x00,0xE1,0x03,d,14);
}

//Change microdisplay brightness by pulsing the brightness lines
void setScreenBrightness(int delta){
 screenBrightness = constrain(screenBrightness + delta, screenBrightnessMin, screenBrightnessMax);
 if (delta > 0){
  digitalWrite(SCREEN_BRIGHT_PLUS, LOW);
  delay(100);
  digitalWrite(SCREEN_BRIGHT_PLUS, HIGH);
  Serial.print("Screen Brightness +");
 }else{
  digitalWrite(SCREEN_BRIGHT_MINUS, LOW);
  delay(100);
  digitalWrite(SCREEN_BRIGHT_MINUS, HIGH);
  Serial.print("Screen Brightness -");
 }
}

void setMiscStartupCommands(void)
{
  //Restore default 50 Hz
  uint8_t f[12] = {
    0x55, 0xAA, 0x09, 0x00, 0xE6, 0x05, 0x00, 0x00, 0x00, 0x00, 0x9D, 0xFF
  };
  sendRawFrame(f, sizeof(f));
  delay(150);
  //Set full-frame 640×512, horizontal mirroring
  uint8_t g[18] = {
    0x55, 0xAA, 0x0F, 0x00, 0xE1, 0x00, 0x02, 0x80, 0x02, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x01, 0x00, 0x97, 0x5B
  };
  sendRawFrame(g, sizeof(g));
  delay(150);
}

void runtimeStatusQuery(void)
{
  Serial.println("Status: ");
  uint8_t f[8] = {
    0x55, 0xAA, 0x05, 0x01, 0xE0, 0x00, 0xCA, 0x0C
  };
  sendRawFrame(f, sizeof(f));
}

void parseStatus(uint8_t * pRxBuff)
{
  status.cut_width = *(uint16_t*)(pRxBuff + 30);
  status.cut_height = *(uint16_t*)(pRxBuff + 32);
  status.cut_start_h = *(uint16_t*)(pRxBuff + 34);
  status.cut_start_v = *(uint16_t*)(pRxBuff + 36);
  if (VERBOSE)
  {
    Serial.println("Cut width: ");
    Serial.println(status.cut_width);
    Serial.println("Cut height: ");
    Serial.println(status.cut_height);
    Serial.println("Cut start h: ");
    Serial.println(status.cut_start_h);
    Serial.println("Cut start v: ");
    Serial.println(status.cut_start_v);
  }
}

void setup(){
 Serial.begin(115200);
 cam.begin(115200,SERIAL_8N1,RX_PIN,TX_PIN);
 preferences.begin("dart1",false);
 autoCalEnabled=preferences.getBool("autocal",false);
 pinMode(NAV_LEFT_PIN,INPUT_PULLUP);
 pinMode(NAV_RIGHT_PIN,INPUT_PULLUP);
 pinMode(NAV_MID_PIN,INPUT_PULLUP);
 pinMode(THUMB_BUTTON,INPUT_PULLUP);
 pinMode(EXTRA_BUTTON_PIN,INPUT_PULLUP);
 pinMode(LED_BLUE,OUTPUT);
 pinMode(SCREEN_BRIGHT_PLUS, OUTPUT);
 pinMode(SCREEN_BRIGHT_MINUS, OUTPUT);
 digitalWrite(SCREEN_BRIGHT_PLUS, HIGH);
 digitalWrite(SCREEN_BRIGHT_MINUS, HIGH);
 digitalWrite(LED_BLUE,LED_OFF);
 delay(5000);
//  setMiscStartupCommands();
//  autoShutter(autoCalEnabled);
//  delay(150);
//  enablePseudoColor();
//  delay(150);
//  setContrast(contrastSteps[contrastIdx]);
//  delay(150);
//  setBrightness(brightness);
//  delay(150);
//  setZoom(zoomX10);
//  delay(150);
 runtimeStatusQuery();
 if(VERBOSE) Serial.println("Camera online.");
}

void loop(){
 unsigned long now=millis();
 bool thumb=digitalRead(THUMB_BUTTON)==LOW;
 bool mid=digitalRead(NAV_MID_PIN)==LOW;
 bool extra=digitalRead(EXTRA_BUTTON_PIN)==LOW;

//capture left and right navigation
 int delta=0;
 if(now-lastNavMoveTime>200){
   if(digitalRead(NAV_LEFT_PIN)==LOW){delta=-1; lastNavMoveTime=now;}
   else if(digitalRead(NAV_RIGHT_PIN)==LOW){delta=1; lastNavMoveTime=now;}
 }

//we're navigating left and right
 if(delta){
  //and the thumb button is held (change brightness)
   if(thumb){
     thumbWasInteracted=true;
     brightness=constrain(brightness+delta*2,0,100);
     setBrightness(brightness);
     if(VERBOSE) Serial.printf("Set brightness: %d\n", brightness);
     //or else it's not held (change zoom)
   } else if (extra) {
     setScreenBrightness(delta);
   }
   else {
     zoomX10=constrain((int)zoomX10+delta,10,70);
     setZoom(zoomX10);
     if(VERBOSE) Serial.printf("Set zoom: %d\n", zoomX10);
   }
 } 
 //Extra button press by itself: toggle auto calibration
 else if(extra && now-lastExtraPressTime>350){
   autoCalEnabled=!autoCalEnabled;
   autoShutter(autoCalEnabled);
   delay(150);
   saveParams();
   preferences.putBool("autocal",autoCalEnabled);
   lastExtraPressTime=now;
 }

//middle navigation press
 if(mid){
   if(navMidDownTime==0) navMidDownTime=now;
   //thumb is held and middle nav held >3s: logical correction 
   if(thumb && now-navMidDownTime>3000){
     logicalOOC();
     if(VERBOSE) Serial.println("Manual OOC");
     navMidDownTime=0;
     thumbWasInteracted=true;
   }
 } else if(navMidDownTime){
   if(now-navMidDownTime<2000 && now-lastBtnPressTime>450){
    //thumb is held and middle nav is pressed: change contrast
     if(thumb){
       thumbWasInteracted=true;
       contrastIdx=(contrastIdx+1)%8;
       setContrast(contrastSteps[contrastIdx]);
       if(VERBOSE) Serial.printf("Set contrast: %d\n", contrastIdx);
       //middle nav is pressed with nothing else: change pseudocolor pallet
     } else {
       pseudocolor=(pseudocolor+1)%MAX_PALLETS;
       setPseudocolor(pseudocolor);
       if(VERBOSE) Serial.printf("Set pseudocolor: %d\n", pseudocolor);
     }
   }
   lastBtnPressTime=now;
   navMidDownTime=0;
 }

 if(thumb){
   if(thumbDownTime==0 && now-thumbActionCooldown>150){
     thumbDownTime=now; thumbWasInteracted=false;
   }
   //regular thumb press: manual shutter once
 } else if(thumbDownTime){
   unsigned long hold=now-thumbDownTime;
   if(hold>30 && hold<500 && !thumbWasInteracted) {
    manualShutter();
    if(VERBOSE) Serial.printf("Trigger manual shutter\n");
   }
   thumbDownTime=0;
   thumbActionCooldown=now;
 }
}
