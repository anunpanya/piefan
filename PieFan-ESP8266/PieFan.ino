/*  NETPIE ESP8266 basic sample                            */
/*  More information visit : https://netpie.io             */

#include <ESP8266WiFi.h>
#include <MicroGear.h>

#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>

#define APPID   "APPID"
#define KEY     "KEY"
#define SECRET  "SECRET"
#define ALIAS   "piefan"

WiFiClient client;

#define SPEEDPIN D1 //ขาควบคุมความเร็วมอเตอร์
#define LED1PIN D5  //speed no. 1
#define LED2PIN D6  //speed no. 2
#define LED3PIN D7  //speed no. 3
#define LEDONLINEPIN D8 //สถานะเชื่อมต่อ netpie
#define BUTTONPIN D2 //ปุ่มกดเปลี่ยนสถานะ

int state = 0; //สถานะของการทำงานปัจจุบัน
int stateOld = 0;  //สถานะของการทำงานก่อนหน้า
long timeout = 1000; //กำหนด timeout ของการกดปุ่ม
long last_change_time = 0; //เก็บเวลาสำหรับเช็ค timeout
int MoterSpeed[] = { 0, 511, 767, 1023 }; //ค่าสัญญาณ analog 0-1023
int lock = 1; //lock คำสั่ง เมื่อมีการอัพเดทสถานะการทำงานของอุปกรณ์ เพื่อสั่งงานอุปกรณ์

MicroGear microgear(client);

void sendSpeed() { 
  if (state!=stateOld) {
    if (state == 1) {
      digitalWrite(LED1PIN, HIGH);
      digitalWrite(LED2PIN, LOW);
      digitalWrite(LED3PIN, LOW);
      Serial.println("SP1");
    } else if (state == 2) {
      digitalWrite(LED1PIN, LOW);
      digitalWrite(LED2PIN, HIGH);
      digitalWrite(LED3PIN, LOW);
      Serial.println("SP2");
    } else if (state == 3) {
      digitalWrite(LED1PIN, LOW);
      digitalWrite(LED2PIN, LOW);
      digitalWrite(LED3PIN, HIGH);
      Serial.println("SP3");
    } else if (state == 0) {
      digitalWrite(LED1PIN, LOW);
      digitalWrite(LED2PIN, LOW);
      digitalWrite(LED3PIN, LOW);
      Serial.println("SP0");
    }
  
    if(state>=0 && state<=3){
      stateOld=state;
      analogWrite(SPEEDPIN, MoterSpeed[state]); //กำหนดแรงขับมอเตอร์
      microgear.publish("/piefan/state",state);
    }
  }
  lock=0; //unlock
}

void onButtonPressed() {
  int difference = millis()-last_change_time;
  if(difference > timeout || last_change_time == 0){
    state++;
    last_change_time = millis();
  }
  if(state>=4) state = 0;
  lock=1; //lock
}

void onMsghandler(char *topic, uint8_t* msg, unsigned int msglen) {
  char *m = (char *)msg;
  m[msglen] = '\0';
  int sp = atoi(m);
  
  Serial.print("SPEED : ");  
  if(sp>=0 && sp<=3){
    state = sp;
    lock = 1;
    Serial.println(m);
  }
}

void onConnected(char *attribute, uint8_t* msg, unsigned int msglen) {
  Serial.println("Connected to NETPIE...");
  digitalWrite(LEDONLINEPIN, HIGH); //เปลี่ยนสถานะ led เป็นออนไลน์
}

void setup() {
  
  Serial.begin(115200);
  Serial.println("Starting...");

  //กำหนดโหมดของ pin ต่างๆ
  pinMode(BUTTONPIN, INPUT_PULLUP);
  pinMode(SPEEDPIN, OUTPUT);
  pinMode(LED1PIN, OUTPUT);
  pinMode(LED2PIN, OUTPUT);
  pinMode(LED3PIN, OUTPUT);
  pinMode(LEDONLINEPIN, OUTPUT);

  attachInterrupt( digitalPinToInterrupt(BUTTONPIN), onButtonPressed, RISING ); //ฟังก์ชั่นควบคุมการทำงานมอเตอร์
    
  WiFiManager wifiManager;
  wifiManager.setTimeout(180);

  //wifiManager.resetSettings();

  if (!wifiManager.autoConnect("PieFan")) {
    Serial.println("Failed to connect and hit timeout");
    delay(3000);
    ESP.reset();
    delay(5000);
  }
  
  microgear.on(MESSAGE, onMsghandler);
  microgear.on(CONNECTED, onConnected);
  
  microgear.init(KEY, SECRET, ALIAS);
  microgear.connect(APPID); //เชื่อมต่อ netpie
}

void loop() {
  if (microgear.connected()) {
    microgear.loop();
    if(lock) sendSpeed(); //กำหนดความเร็วมอเตอร์ และตอบกลับสถานะของอุปกรณ์ผ่าน netpie
  }
  else {
    Serial.println("connection lost, reconnect...");
    microgear.connect(APPID);
    if(digitalRead(LEDONLINEPIN)==HIGH)
      digitalWrite(LEDONLINEPIN, LOW); //เปลี่ยนสถานะ led เป็นออฟไลน์
  }
}

