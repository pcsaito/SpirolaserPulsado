#include <ResponsiveAnalogRead.h>

#include <SLIPEncodedSerial.h>
SLIPEncodedSerial SLIPSerial(Serial);

#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCData.h>

///DEFINES
#define pulseLaserDutyInputPin 2
#define pulseLaserStepsInputPin 4
#define pulseLaserPrescalerInputPin 3

#define oscEnablePin 4

#define mirrorInputPinCount 4
#define mirrorSoftPWMShift 2 

#define analogMax 1024

///VARIABLES
bool constantDutyMode = true;
bool automaticMode = true;
bool oscMode = false;

uint16_t softPWMCounter = 0;
uint16_t prescalerMultiplier = 1;

int mirrorInputPins[mirrorInputPinCount] = {0, 1, 5, 6};
uint16_t mirrorZeroValue[mirrorInputPinCount] = {200, 150, 200, 200};
uint16_t mirrorMaxValue[mirrorInputPinCount] = {900, 1024, 1023, 700};

uint16_t mirrorOscValues[mirrorInputPinCount] = {0, 0, 0, 0};
uint16_t laserOscValues[3] = {0, 0, 0};

#define analogSmooth 0.5

ResponsiveAnalogRead _m1(0, true, analogSmooth);
ResponsiveAnalogRead _m2(0, true, analogSmooth);
ResponsiveAnalogRead _m3(0, true, analogSmooth);
ResponsiveAnalogRead _m4(0, true, analogSmooth);
ResponsiveAnalogRead _dt(0, true, analogSmooth);
ResponsiveAnalogRead _ps(0, true, 0.1);
ResponsiveAnalogRead _st(0, true, analogSmooth);

/// SETUP
void setup() {
//  Serial.begin(9600);
SLIPSerial.begin(9600);

  Serial.println("Spirolaser booting...");

//Act as gnd for laser modulation signal
  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);

  setupLaserPWM();

  setupMirrorsPWM();

  //setupRandom();
  //setupSpecialModes();
}

void setupLaserPWM() {
  //Timer 1 16bit
    DDRB |= _BV(PB1) | _BV(PB2);                      /* set pins as outputs */
    TCCR1A = _BV(COM1A1) | _BV(COM1B1)  | _BV(WGM11); /* non-inverting mode 14: fast PWM, TOP=ICR1 */
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);     /* no prescaling */
    ICR1 = 0xffff;                                    /* TOP counter value */
}

void setupMirrorsPWM() {
  //Timer 0 & 2  max hz
  pinMode(3, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  digitalWrite(3, LOW);
  digitalWrite(11, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  
  //-----------------------------
  //Timer 0 with no prescaler
  TCCR0B = _BV(CS00);

  //Timer 2 in Fast PWM with no prescaler
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = (TCCR2B & 0b11111000) | 0x01;
}

void setupRandom() {
  uint16_t seed = analogRead(0);
  seed += analogRead(1);
  seed += analogRead(2);
  seed += analogRead(3);
  seed += analogRead(4);
  seed += analogRead(5);
  seed += analogRead(6);
  seed += analogRead(7);
  randomSeed(seed);
}

void setupSpecialModes() {
  uint16_t loLevel = 24;
  uint16_t hiLevel = analogMax - loLevel;

  /// Manual Mode
  uint16_t m1 = analogRead(0);
  uint16_t m2 = analogRead(1);
  uint16_t m3 = analogRead(5);
  uint16_t m4 = analogRead(6);
  if ((m1 >= hiLevel) && (m3 >= hiLevel)) {
    if ((m2 <= loLevel) && (m4 <= loLevel)) {
      automaticMode = false;
    }
  }

  ///Static Mode
  uint16_t l1 = analogRead(2);
  uint16_t l2 = analogRead(3);
  uint16_t l3 = analogRead(4);
  if ((l1 >= hiLevel) && (l3 >= hiLevel)) {
    if (l2 <= loLevel) {
      constantDutyMode = false;
    }
  }
}

// RUNTIME
void loop() {  
  //Apply laser prescaler
  applyLaserPrescaler();
  
  //Apply laser duty
  applyLaserDuty();   
  
  softPWMLoop();

  checkOSCMode();
  if (oscMode) {
    readSerialInputs();
    //printOscInputs();
  }

  //printAnalogInputs();
}

void applyLaserDuty() {
  uint16_t duty;
  if (oscMode) { 
    duty = laserOscValues[0];
  } else {
    duty = 256;//analogRead(pulseLaserDutyInputPin);
  }

  uint16_t steps = pulseLaserSteps(duty) * prescalerMultiplier;

  if (!oscMode) {
    while (TCNT1 > steps) { 
      TCNT1 = TCNT1 << 1; 
    }
  }

  _st.update(steps);
  ICR1 = _st.getValue();

  if (constantDutyMode) {
    uint16_t mapDuty = map(duty, 0, analogMax, 0, steps);
    _dt.update(mapDuty);
    
    OCR1B = _dt.getValue();
  } else {
    _dt.update(duty * prescalerMultiplier);

    OCR1B = _dt.getValue();
  }
}


uint16_t pulseLaserSteps(uint16_t duty) {
  uint16_t steps;
  if (oscMode) { 
    steps = laserOscValues[2];
  } else {
    steps = analogRead(pulseLaserDutyInputPin);//pulseLaserStepsInputPin);
  }
  steps = steps < 16 ? 16 : steps;
  
  if (constantDutyMode) {
    return steps;
  } else {
    return steps + duty;
  }
}

int applyLaserPrescaler() {
  int prescaler;
  if (oscMode) { 
    prescaler = laserOscValues[1];
  } else {
    prescaler = map(analogRead(pulseLaserPrescalerInputPin), 0, analogMax, 0, 15);;
  }

  _ps.update(prescaler);
 
  switch (prescaler) {
    case 0:
      prescalerMultiplier = 1;
      TCCR1B = (TCCR1B & 0b11111000) | 0x01; 
      break;
    case 1:
      prescalerMultiplier = 2;
      TCCR1B = (TCCR1B & 0b11111000) | 0x01; 
      break;
    case 2:
      prescalerMultiplier = 4;
      TCCR1B = (TCCR1B & 0b11111000) | 0x01; 
      break;
    case 3:
      prescalerMultiplier = 1;
      TCCR1B = (TCCR1B & 0b11111000) | 0x02; 
      break;
    case 4:
      prescalerMultiplier = 2;
      TCCR1B = (TCCR1B & 0b11111000) | 0x02; 
      break;
    case 5:
      prescalerMultiplier = 4;
      TCCR1B = (TCCR1B & 0b11111000) | 0x02; 
      break;
    case 6:
      prescalerMultiplier = 1;
      TCCR1B = (TCCR1B & 0b11111000) | 0x03; 
      break;
    case 7:
      prescalerMultiplier = 2;
      TCCR1B = (TCCR1B & 0b11111000) | 0x03; 
      break;      
    case 8:
      prescalerMultiplier = 1;
      TCCR1B = (TCCR1B & 0b11111000) | 0x04; 
      break;
    case 9:
      prescalerMultiplier = 2;
      TCCR1B = (TCCR1B & 0b11111000) | 0x04; 
      break;    
      case 10:
      prescalerMultiplier = 4;
      TCCR1B = (TCCR1B & 0b11111000) | 0x04; 
      break;    
      case 11:
      prescalerMultiplier = 8;
      TCCR1B = (TCCR1B & 0b11111000) | 0x04; 
      break;   
      case 12:
      prescalerMultiplier = 16;
      TCCR1B = (TCCR1B & 0b11111000) | 0x04; 
      break;    
      case 13:
      prescalerMultiplier = 32;
      TCCR1B = (TCCR1B & 0b11111000) | 0x04; 
      break;   
      case 14:
      prescalerMultiplier = 64;
      TCCR1B = (TCCR1B & 0b11111000) | 0x04; 
      break;                  
  }
  Serial.print(prescaler);
  Serial.print(" - ");
  Serial.println(prescalerMultiplier);
}

///HELPERS
void checkOSCMode() {
  oscMode = (digitalRead(oscEnablePin) == 1);
}

void sleep(unsigned long milis) {
  delay(milis * 64);
}

unsigned long timeStamp() {
  return millis() * 64;
}

uint16_t mirrorRead(int pin) { ////// TODO
  uint16_t read;
  if (oscMode) {
    read = mirrorOscValues[pin];
  } else {
    read = analogRead(mirrorInputPins[pin]);
  }
  
  return map(read, 0, analogMax, mirrorZeroValue[pin], mirrorMaxValue[pin]);  
}

///SOFT PWM
struct SoftPWM {    
 int hardOutput;
 int softOutput;
};

struct SoftPWM splitComponents(uint16_t input) {
  int shift = mirrorSoftPWMShift;

  int hardOutput = (input >> shift) ;
  int softOutput = input - (hardOutput << shift);

  struct SoftPWM retVal = {hardOutput, softOutput};
  return retVal;
}

void analogWriteLow(int pin, int value) {
  switch (pin) {
    case 0:
      analogWrite(11, value); 
      break;
    case 1:
      analogWrite(6, value); 
      break;
    case 2:
      analogWrite(3, value);       
      break;
    case 3:
      analogWrite(5, value);       
      break;
  }
}

void analogWriteHigh(int pin, int value) {
  analogWriteLow(pin, value+1);
}

void softPWMLoop() {  
  int shiftSteps = 1 << mirrorSoftPWMShift;
  if (softPWMCounter % shiftSteps == 0) {
    softPWMCounter = 0;
  }

  uint16_t reads[4];
  struct SoftPWM mirrorInputs[4];
  for (int pin = 0; pin < mirrorInputPinCount; pin++) {
    uint16_t rawRead = mirrorRead(pin);
    mirrorInputs[pin] = splitComponents(rawRead);
    reads[pin] = rawRead;
  }

  uint16_t rawRead = mirrorRead(0);
  _m1.update(rawRead);
  mirrorInputs[0] = splitComponents(_m1.getValue());
  reads[0] = _m1.getValue();

  rawRead = mirrorRead(1);
  _m2.update(rawRead);
  mirrorInputs[1] = splitComponents(_m2.getValue());
  reads[1] = _m2.getValue();

  rawRead = mirrorRead(2);
  _m3.update(rawRead);
  mirrorInputs[2] = splitComponents(_m3.getValue());
  reads[2] = _m3.getValue();

  rawRead = mirrorRead(3);
  _m4.update(rawRead);
  mirrorInputs[3] = splitComponents(_m4.getValue());
  reads[3] = _m4.getValue();

  //printMirrorInputs(mirrorInputs);

  for (int pin = 0; pin < mirrorInputPinCount; pin++) {
    int cycle = 0;
 
    switch (softPWMCounter) {
    case 0:
      setSoftPWMCycle(mirrorInputs, pin, 0);
      break;
    case 4:
      setSoftPWMCycle(mirrorInputs, pin, 0);
      setSoftPWMCycle(mirrorInputs, pin, 1);
      break; 
    case 2:
      setSoftPWMCycle(mirrorInputs, pin, 0);
      setSoftPWMCycle(mirrorInputs, pin, 1);
      setSoftPWMCycle(mirrorInputs, pin, 2);
      break;
    case 6:
      setSoftPWMCycle(mirrorInputs, pin, 0);
      setSoftPWMCycle(mirrorInputs, pin, 1);
      setSoftPWMCycle(mirrorInputs, pin, 2);
      setSoftPWMCycle(mirrorInputs, pin, 3);
      break;
    case 1:
      setSoftPWMCycle(mirrorInputs, pin, 0);
      setSoftPWMCycle(mirrorInputs, pin, 1);
      setSoftPWMCycle(mirrorInputs, pin, 2);
      setSoftPWMCycle(mirrorInputs, pin, 3);
      setSoftPWMCycle(mirrorInputs, pin, 4);
      break;
    case 5:
      setSoftPWMCycle(mirrorInputs, pin, 0);
      setSoftPWMCycle(mirrorInputs, pin, 1);
      setSoftPWMCycle(mirrorInputs, pin, 2);
      setSoftPWMCycle(mirrorInputs, pin, 3);
      setSoftPWMCycle(mirrorInputs, pin, 4);
      setSoftPWMCycle(mirrorInputs, pin, 5);
      break;     
    case 3:
      setSoftPWMCycle(mirrorInputs, pin, 0);
      setSoftPWMCycle(mirrorInputs, pin, 1);
      setSoftPWMCycle(mirrorInputs, pin, 2);
      setSoftPWMCycle(mirrorInputs, pin, 3);
      setSoftPWMCycle(mirrorInputs, pin, 4);
      setSoftPWMCycle(mirrorInputs, pin, 5);
      setSoftPWMCycle(mirrorInputs, pin, 6);
      break;
    case 7:
      setSoftPWMCycle(mirrorInputs, pin, 0);
      setSoftPWMCycle(mirrorInputs, pin, 1);
      setSoftPWMCycle(mirrorInputs, pin, 2);
      setSoftPWMCycle(mirrorInputs, pin, 3);
      setSoftPWMCycle(mirrorInputs, pin, 4);
      setSoftPWMCycle(mirrorInputs, pin, 5);
      setSoftPWMCycle(mirrorInputs, pin, 6);
      setSoftPWMCycle(mirrorInputs, pin, 7);
      break;
    }
  }

  softPWMCounter++;
}

void setSoftPWMCycle(SoftPWM mirrorInputs[4], int pin, int cycle) {
     if (mirrorInputs[pin].softOutput >= cycle && mirrorInputs[pin].softOutput < (cycle + 1) ) { 
      analogWriteHigh(pin, mirrorInputs[pin].hardOutput); 
    } else { 
      analogWriteLow(pin, mirrorInputs[pin].hardOutput); 
    }
}


///DEBUG
void printMirrorInputs(struct SoftPWM mirrorInputs[4]) {
  Serial.print("1: ");
  Serial.print(mirrorInputs[0].hardOutput);
  Serial.print(" - ");
  Serial.print(mirrorInputs[0].softOutput);
  Serial.print(" | 2: ");
  Serial.print(mirrorInputs[1].hardOutput);
  Serial.print(" - ");
  Serial.print(mirrorInputs[1].softOutput);
  Serial.print(" | 3: ");
  Serial.print(mirrorInputs[2].hardOutput);
  Serial.print(" - ");
  Serial.print(mirrorInputs[2].softOutput);
  Serial.print(" | 4: ");
  Serial.print(mirrorInputs[3].hardOutput);
  Serial.print(" - ");
  Serial.print(mirrorInputs[3].softOutput);
  Serial.println("");
}

void printAnalogInputs() {
  Serial.print("1: ");
  Serial.print(_m1.getValue());
  Serial.print(" | 2: ");
  Serial.print(_m2.getValue());
  Serial.print(" | 3: ");
  Serial.print(_m3.getValue());
  Serial.print(" | 4: ");
  Serial.print(_m4.getValue());
  Serial.print(" | dt: ");
  Serial.print(_dt.getValue());
  Serial.print(" | ps: ");
  Serial.print(_ps.getValue());
  Serial.print(" | st: ");
  Serial.print(_st.getValue());
  Serial.println("");
}

void printOscInputs() {
  Serial.print("1: ");
  Serial.print(mirrorOscValues[0]);
  Serial.print(" | 2: ");
  Serial.print(mirrorOscValues[1]);
  Serial.print(" | 3: ");
  Serial.print(mirrorOscValues[2]);
  Serial.print(" | 4: ");
  Serial.print(mirrorOscValues[3]);
  Serial.print(" | dt: ");
  Serial.print(laserOscValues[0]);
  Serial.print(" | ps: ");
  Serial.print(laserOscValues[1]);
  Serial.print(" | st: ");
  Serial.print(laserOscValues[2]);
  Serial.println("");
}

//OSC
void readSerialInputs() {
  OSCMessage msg;
  while(!SLIPSerial.endofPacket()){
    int size = SLIPSerial.available();
    if (size > 0){
      //fill the msg with all of the available bytes
      while(size--){
        msg.fill(SLIPSerial.read());
      }
    }
  }
  
 if (!msg.hasError()) {
    msg.dispatch("/m", m);      
    msg.dispatch("/l", l);
  } else {
    OSCErrorCode error = msg.getError();
    Serial.print("error: ");
    Serial.println(error);
  }
}

void m(OSCMessage &msg) { 
  mirrorOscValues[0] = (uint16_t)msg.getInt(0);
  mirrorOscValues[1] = (uint16_t)msg.getInt(1);
  mirrorOscValues[2] = (uint16_t)msg.getInt(2);
  mirrorOscValues[3] = (uint16_t)msg.getInt(3);
}
void l(OSCMessage &msg) { 
  laserOscValues[0] = (uint16_t)msg.getInt(0);
  laserOscValues[1] = (uint16_t)msg.getInt(1);
  laserOscValues[2] = (uint16_t)msg.getInt(2);
}
