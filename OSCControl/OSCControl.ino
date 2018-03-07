/*---------------------------------------------------------------------------------------------
  Open Sound Control (OSC) library for the ESP8266
  Example for receiving open sound control (OSC) bundles on the ESP8266
  Send integers '0' or '1' to the address "/led" to turn on/off the built-in LED of the esp8266.
  This example code is in the public domain.
--------------------------------------------------------------------------------------------- */
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCData.h>

#include <SLIPEncodedSerial.h>
SLIPEncodedSerial SLIPSerial(Serial1);

#include <stdio.h> // for function sprintf

extern "C" { 
#include <user_interface.h>
}

char ssid[] = "SpirolaserPulsado";
char pass[] = "qualquerbosta";

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;
IPAddress outIp(192, 168, 1, 100);
const unsigned int outPort = 9000;          // remote port (not needed for receive)
const unsigned int localPort = 8000;        // local port to listen for UDP packets (here's where we send the packets)

OSCErrorCode error;

//Outputs
uint16_t _m1 = 0;
uint16_t _m2 = 0;
uint16_t _m3 = 0;
uint16_t _m4 = 0;
uint16_t _dt = 0;
uint16_t _ps = 0;
uint16_t _st = 0;

uint16_t rawSt = 0;
uint16_t mPushDelta = 10;
int16_t microSt = 0;

int8_t enableCounter = 0;

#define ENABLEPIN D3
#define LEDPIN D0

void setup() {
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);

  pinMode(ENABLEPIN, OUTPUT);
  digitalWrite(ENABLEPIN, LOW);

  Serial.begin(9600);
  SLIPSerial.begin(9600);

  // Connect to WiFi network
  Serial.println();
  Serial.print("Starting hotspot");
  
  WiFi.mode(WIFI_AP);
  WiFi.hostname(ssid);

  boolean result = WiFi.softAP(ssid, pass);
  if(result == true) {
    Serial.println("Hotspot Ready");
  } else {
    Serial.println("Hotspot Failed!");
  }

  IPAddress ap_ip(192, 168, 1, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(ap_ip, ap_ip, subnet);
  
  Serial.println("IP address: ");
  Serial.println(ap_ip);

  Serial.println("Starting UDP");
  Udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(Udp.localPort());
}

uint8_t currentClients = 0;
void loop() {
  uint8_t clients = wifi_softap_get_station_num();
  if (clients != currentClients) {
    currentClients = clients;
    Serial.println("Clients Connected: " + String((int)currentClients));
  }
  
  if (currentClients == 0) {
    enableCounter--;
    if (enableCounter <= 0) {
      digitalWrite(ENABLEPIN, LOW);
      enableCounter = 0;
    }    
  }

  OSCMessage msg;
  uint8_t size = Udp.parsePacket();

  if (size > 0) {
    digitalWrite(LEDPIN, HIGH);
    
    while (size--) {
      msg.fill(Udp.read());
    }
    
    if (!msg.hasError()) {
      msg.dispatch("/spirolaser/enable", enable);
      msg.dispatch("/spirolaser/delta", delta);
      
      msg.dispatch("/spirolaser/m1", m1);
      msg.dispatch("/spirolaser/m1up", m1up);
      msg.dispatch("/spirolaser/m1down", m1down);
      
      msg.dispatch("/spirolaser/m2", m2);
      msg.dispatch("/spirolaser/m2up", m2up);
      msg.dispatch("/spirolaser/m2down", m2down);

      msg.dispatch("/spirolaser/m3", m3);
      msg.dispatch("/spirolaser/m3up", m3up);
      msg.dispatch("/spirolaser/m3down", m3down);

      msg.dispatch("/spirolaser/m4", m4);
      msg.dispatch("/spirolaser/m4up", m4up);
      msg.dispatch("/spirolaser/m4down", m4down);
      
      msg.dispatch("/spirolaser/duty", duty);
      
      msg.dispatch("/spirolaser/ps", ps);
      msg.dispatch("/spirolaser/psup", psup);
      msg.dispatch("/spirolaser/psdown", psdown);
      
      msg.dispatch("/spirolaser/steps", steps);
      msg.dispatch("/spirolaser/stepsup", stepsup);
      msg.dispatch("/spirolaser/stepsdown", stepsdown);
      
      msg.dispatch("/spirolaser/msteps", msteps);
      msg.dispatch("/spirolaser/mstepsup", mstepsup);
      msg.dispatch("/spirolaser/mstepsdown", mstepsdown);

      //Serial.print("no error");
    } else {
      error = msg.getError();
//      Serial.print("error: ");
//      Serial.println(error);
    }
  } else {
    int ledStatus = digitalRead(LEDPIN);
    if (ledStatus > 0) {
      digitalWrite(LEDPIN, LOW);
      //Serial.print("no buffer");
    }
  }

  sendOsc();
  printParam(); 
}

void sendOsc() {
  OSCMessage m("/m");
  m.add(_m1);
  m.add(_m2);
  m.add(_m3);
  m.add(_m4);

  OSCMessage l("/l");
  l.add(_dt);
  l.add(_ps);
  l.add(_st);

  SLIPSerial.beginPacket();
  m.send(SLIPSerial);
  SLIPSerial.endPacket();
  delay(10);

  SLIPSerial.beginPacket();
  l.send(SLIPSerial);
  SLIPSerial.endPacket();
  delay(10);
}

void printParam() {
  char mirrors[20];
  sprintf(mirrors, "%04d:%04d:%04d:%04d", _m1, _m2, _m3, _m4);
  Serial.print("(");
  Serial.print(mirrors);
  Serial.print(") - ");

  char laser[15];
  sprintf(laser, "%04d:%04d:%04d", _dt, _ps, _st);
  Serial.print("[");
  Serial.print(laser);
  Serial.print("]");
  
  Serial.println("");
}

//functions
void returnMessage(const char *_address, float value) {
  OSCMessage returnMsg(_address);
  returnMsg.add(value);

  Udp.beginPacket(outIp, outPort);
  returnMsg.send(Udp); 
  Udp.endPacket();
}

void enable(OSCMessage &msg) { 
  if (msg.getFloat(0) > 0) {
    enableCounter = 50;
    digitalWrite(ENABLEPIN, HIGH);
  } else {
    digitalWrite(ENABLEPIN, LOW); 
  }
}

void delta(OSCMessage &msg) { 
  mPushDelta = (int)msg.getFloat(0);
}

void m1(OSCMessage &msg) { 
  _m1 = (int)msg.getFloat(0);
}
void m1up(OSCMessage &msg) { 
  if (_m1+mPushDelta < 1024) {
    _m1 += mPushDelta;
  }
  returnMessage("/spirolaser/m1", (float)_m1);
}
void m1down(OSCMessage &msg) { 
  if (_m1-mPushDelta >= 0) {
    _m1 -= mPushDelta;
  }
  returnMessage("/spirolaser/m1", (float)_m1);
}

void m2(OSCMessage &msg) { 
  _m2 = (int)msg.getFloat(0);
}
void m2up(OSCMessage &msg) { 
  if (_m2+mPushDelta < 1024) {
    _m2 += mPushDelta;
  }
  returnMessage("/spirolaser/m2", (float)_m2);
}
void m2down(OSCMessage &msg) { 
  if (_m2-mPushDelta >= 0) {
    _m2 -= mPushDelta;
  }
  returnMessage("/spirolaser/m2", (float)_m2);
}

void m3(OSCMessage &msg) { 
  _m3 = (int)msg.getFloat(0);
}
void m3up(OSCMessage &msg) { 
  if (_m3+mPushDelta < 1024) {
    _m3 += mPushDelta;
  }
  returnMessage("/spirolaser/m3", (float)_m3);
}
void m3down(OSCMessage &msg) { 
  if (_m3-mPushDelta >= 0) {
    _m3 -= mPushDelta;
  }
  returnMessage("/spirolaser/m3", (float)_m3);
}

void m4(OSCMessage &msg) { 
  _m4 = (int)msg.getFloat(0);
}
void m4up(OSCMessage &msg) { 
  if (_m4+mPushDelta < 1024) {
    _m4 += mPushDelta;
  }
  returnMessage("/spirolaser/m4", (float)_m4);
}
void m4down(OSCMessage &msg) { 
  if (_m4-mPushDelta >= 0) {
    _m4 -= mPushDelta;
  }
  returnMessage("/spirolaser/m4", (float)_m4);
}

void duty(OSCMessage &msg) { 
  _dt = (int)msg.getFloat(0);
}

void ps(OSCMessage &msg) { 
  _ps = (int)msg.getFloat(0);
}
void psup(OSCMessage &msg) { 
  if (_ps < 15) {
    _ps++;
  }

  returnMessage("/spirolaser/ps", (float)_ps);
}
void psdown(OSCMessage &msg) { 
  if (_ps > 0) {
    _ps--;
  }
  
  returnMessage("/spirolaser/ps", (float)_ps);
}

void steps(OSCMessage &msg) { 
  rawSt = (int)msg.getFloat(0);
  
  int newSt = rawSt + microSt;
  _st = newSt >= 0 ? newSt : 0;
}
void stepsup(OSCMessage &msg) { 
  rawSt += mPushDelta;
  
  int newSt = rawSt + microSt;
  _st = newSt >= 0 ? newSt : 0;
  
  returnMessage("/spirolaser/steps", (float)_st);
}
void stepsdown(OSCMessage &msg) { 
  rawSt -= mPushDelta;
  
  int newSt = rawSt + microSt;
  _st = newSt >= 0 ? newSt : 0;
  
  returnMessage("/spirolaser/steps", (float)_st);
}

void msteps(OSCMessage &msg) { 
  microSt = (int)msg.getFloat(0);

  int newSt = rawSt + microSt;
  _st = newSt >= 0 ? newSt : 0;
}
void mstepsup(OSCMessage &msg) { 
  microSt++;

  int newSt = rawSt + microSt;
  _st = newSt >= 0 ? newSt : 0;
  
  returnMessage("/spirolaser/msteps", (float)microSt);
}
void mstepsdown(OSCMessage &msg) { 
  microSt--;

  int newSt = rawSt + microSt;
  _st = newSt >= 0 ? newSt : 0;

  returnMessage("/spirolaser/msteps", (float)microSt);
}
