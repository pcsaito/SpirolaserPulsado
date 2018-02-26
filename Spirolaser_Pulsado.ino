///DEFINES
#define pulseLaserOutputPin 10
#define pulseLaserOutputTopPin 9

#define pulseLaserDutyInputPin 2
#define pulseLaserStepsInputPin 4
#define pulseLaserPrescalerInputPin 3

#define mirrorInputPinCount 4

#define mirrorSoftPWMShift 2 

#define analogMax 1024

///VARIABLES
bool constantDutyMode = true;
bool automaticMode = true;

int softPWMCounter = 0;
int prescalerMultiplier = 1;
int mirrorInputPins[mirrorInputPinCount] = {0, 1, 5, 6};
int mirrorZeroValue[mirrorInputPinCount] = {100, 100, 100, 100};
int mirrorMaxValue[mirrorInputPinCount] = {800, 800, 1023, 600};

unsigned long timeStamp() {
  return millis() * 64;
}

/// SETUP
void setup() {
  Serial.begin(9600);
  
  setupLaserPWM();
  setupMirrorsPWM();
  //setupRandom();
  //setupSpecialModes();
  
  pinMode(pulseLaserOutputPin, OUTPUT);
  pinMode(pulseLaserOutputTopPin, OUTPUT);
}

void setupLaserPWM() {
  //Timer 1 16bit
    DDRB |= _BV(PB1) | _BV(PB2);        /* set pins as outputs */
    TCCR1A = _BV(COM1A1) | _BV(COM1B1)  /* non-inverting PWM */
        | _BV(WGM11);                   /* mode 14: fast PWM, TOP=ICR1 */
    TCCR1B = _BV(WGM13) | _BV(WGM12)
        | _BV(CS10);                    /* no prescaling */
    ICR1 = 0xffff;                      /* TOP counter value */
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
  int seed = analogRead(0);
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
  int loLevel = 24;
  int hiLevel = analogMax - loLevel;

  /// Manual Mode
  int m1 = analogRead(0);
  int m2 = analogRead(1);
  int m3 = analogRead(5);
  int m4 = analogRead(6);
  if ((m1 >= hiLevel) && (m3 >= hiLevel)) {
    if ((m2 <= loLevel) && (m4 <= loLevel)) {
      automaticMode = false;
    }
  }

  ///Static Mode
  int l1 = analogRead(2);
  int l2 = analogRead(3);
  int l3 = analogRead(4);
  if ((l1 >= hiLevel) && (l3 >= hiLevel)) {
    if (l2 <= loLevel) {
      constantDutyMode = false;
    }
  }
}

// RUNTIME
void loop() {
  //Apply laser duty
  applyLaserDuty();   
  
  //Apply laser prescaler
  applyLaserPrescaler();
  
  softPWMLoop();
}

void applyLaserDuty() {
  int duty = analogRead(pulseLaserDutyInputPin);
  int steps = pulseLaserSteps(duty);
  ICR1 = steps * prescalerMultiplier;  

  if (constantDutyMode) {
    int mapDuty = map(duty, 0, analogMax, 0, steps);
    analogWrite16(10, mapDuty);
  } else {
    analogWrite16(10, (duty * prescalerMultiplier));
  }
}


int pulseLaserSteps(int duty) {
  int steps = analogRead(pulseLaserStepsInputPin);
  
  if (constantDutyMode) {
    return steps;
  } else {
    return steps + duty;
  }
}

int applyLaserPrescaler() {
  int prescaler = analogRead(pulseLaserPrescalerInputPin);
  int mapPrescaler = map(prescaler, 0, analogMax, 0, 12);
  
  switch (mapPrescaler) {
    case 0:
      prescalerMultiplier = 1;
      TCCR1B = (TCCR1B & 0b11111000) | 0x01; 
      break;
    case 1:
      prescalerMultiplier = 2;
      TCCR1B = (TCCR1B & 0b11111000) | 0x01; 
    case 2:
      prescalerMultiplier = 4;
      TCCR1B = (TCCR1B & 0b11111000) | 0x01; 
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
      prescalerMultiplier = 1;
      TCCR1B = (TCCR1B & 0b11111000) | 0x05; 
      break;  
    case 11:
      prescalerMultiplier = 2;
      TCCR1B = (TCCR1B & 0b11111000) | 0x05; 
      break;                        
  }
  
  applyLaserDuty();
}

///HELPERS
void analogWrite16(uint8_t pin, uint16_t val)
{
  switch (pin) {
    case  9: OCR1A = val; break;
    case 10: OCR1B = val; break;
  }
}

void sleep(unsigned long milis) {
  delay(milis * 64);
}

float counter = 0;
int smoothedAnalogRead(int pin) { ////// TODO
  int read = analogRead(mirrorInputPins[pin]);
  if (pin == 99) {
    read = read - (float)counter;
    counter = counter + 0.02;
    Serial.print(counter);
  }
  return map(read, 0, 1024, mirrorZeroValue[pin], mirrorMaxValue[pin]);
}

///SOFT PWM
struct SoftPWM {    
 int hardOutput;
 int softOutput;
};

struct SoftPWM splitComponents(int input) {
  int shift = mirrorSoftPWMShift;

  int hardOutput = (input >> shift) ;
  int softOutput = input - (hardOutput << shift);

  struct SoftPWM retVal = {hardOutput, softOutput};
  return retVal;
}

void analogWriteLow(int pin, int value) {
  switch (pin) {
    case 0:
      analogWrite(6, value); 
      break;
    case 1:
      analogWrite(11, value); 
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

void softPWMLoop2() {
    int reads[4];
    for (int pin = 0; pin < mirrorInputPinCount; pin++) {
      int rawRead = smoothedAnalogRead(mirrorInputPins[pin]);
        analogWriteLow(pin, rawRead);
        reads[pin] = rawRead;
    }
//      Serial.print("1: ");
//  Serial.print(reads[0]);
//  Serial.print(" | 2: ");
//  Serial.print(reads[1]);
//  Serial.print(" | 3: ");
//  Serial.print(reads[2]);
//  Serial.print(" | 4: ");
//  Serial.print(reads[3]);
//  Serial.println("");

     return;
}

void softPWMLoop() {  
  int shiftSteps = 1 << mirrorSoftPWMShift;
  if (softPWMCounter % shiftSteps == 0) {
    softPWMCounter = 0;
  }

  int reads[4];
  struct SoftPWM mirrorInputs[4];
  for (int pin = 0; pin < mirrorInputPinCount; pin++) {
    int rawRead = smoothedAnalogRead(pin);
    mirrorInputs[pin] = splitComponents(rawRead);
    reads[pin] = rawRead;
  }

//  Serial.print("1: ");
//  Serial.print(mirrorInputs[0].hardOutput);
//  Serial.print(" - ");
//  Serial.print(mirrorInputs[0].softOutput);
//  Serial.print(" | 2: ");
//  Serial.print(mirrorInputs[1].hardOutput);
//  Serial.print(" - ");
//  Serial.print(mirrorInputs[1].softOutput);
//  Serial.print(" | 3: ");
//  Serial.print(mirrorInputs[2].hardOutput);
//  Serial.print(" - ");
//  Serial.print(mirrorInputs[2].softOutput);
//  Serial.print(" | 4: ");
//  Serial.print(mirrorInputs[3].hardOutput);
//  Serial.print(" - ");
//  Serial.print(mirrorInputs[3].softOutput);
//  Serial.println("");

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
  sleep(5);
}

void setSoftPWMCycle(SoftPWM mirrorInputs[4], int pin, int cycle) {
     if (mirrorInputs[pin].softOutput >= cycle && mirrorInputs[pin].softOutput < (cycle + 1) ) { 
      analogWriteHigh(pin, mirrorInputs[pin].hardOutput); 
    } else { 
      analogWriteLow(pin, mirrorInputs[pin].hardOutput); 
    }
}

