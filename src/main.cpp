#include <SPI.h>
#include <Control_Surface.h>
#include <driver/gpio.h>
#include <esp32-hal-spi.h>
#include <EEPROM.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN 1
#define NUM_LEDS 12
#define B0 7
#define B1 15
#define B2 16
#define B3 17
#define BUTTON 2
#define BUTTON2 42
#define BUTTON3 41
#define vel_off 127
#define note_on 1
#define note_off 0


bool is_good = true;

bool is_pressed = false;
bool modeChanged = false;
bool octave_changed = false;
bool octave_changed2 = false;
const int number_of_keys = 40;
bool can_be_off[number_of_keys];
int high[number_of_keys];
int low[number_of_keys];
float adc_val[number_of_keys];

int MIDI_OFFSET = 64;

const float high_level = 6;
const float low_level = 2;
const float top = 10;
const float bottom = 0;
const int EEPROM_SIZE = 1024;
const int EEPROM_CALIBRATION_FLAG_ADDR = 0;
const int EEPROM_HIGH_START_ADDR = 1;
const int EEPROM_LOW_START_ADDR = EEPROM_HIGH_START_ADDR + number_of_keys * sizeof(int);

bool calibrationMode = false;
unsigned long buttonPressTime = 0;
bool buttonHeld = false; 
bool buttonHeld2 = false;
bool buttonHeld3 = false;

unsigned long start[number_of_keys];
unsigned long finish[number_of_keys];
static int mode = 1;// 1 = line


unsigned long calibrationStartTime = 0; // Czas rozpoczęcia trybu kalibracji
bool calibrationHighPhase = true;      // Czy trwa faza ustawiania high[i]

USBMIDI_Interface midi;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

void noteOn(byte pitch, byte velocity) {
  midi.sendNoteOn(pitch, velocity);
}

void noteOff(byte pitch, byte velocity) {
  midi.sendNoteOff(pitch, velocity);
}

static const int spiClk = 4000000;


SPIClass *spi = NULL;

void activatePins(uint8_t binaryValue) {
    if (binaryValue & B1000) {
        REG_WRITE(GPIO_OUT_W1TS_REG, (1 << B3));  // Set B3 high
    } else {
        REG_WRITE(GPIO_OUT_W1TC_REG, (1 << B3));  // Set B3 low
    }
    if (binaryValue & B0100) {
        REG_WRITE(GPIO_OUT_W1TS_REG, (1 << B2));  // Set B2 high
    } else {
        REG_WRITE(GPIO_OUT_W1TC_REG, (1 << B2));  // Set B2 low
    }
    if (binaryValue & B0010) {
        REG_WRITE(GPIO_OUT_W1TS_REG, (1 << B1));  // Set B1 high
    } else {
        REG_WRITE(GPIO_OUT_W1TC_REG, (1 << B1));  // Set B1 low
    }
    if (binaryValue & B0001) {
        REG_WRITE(GPIO_OUT_W1TS_REG, (1 << B0));  // Set B0 high
    } else {
        REG_WRITE(GPIO_OUT_W1TC_REG, (1 << B0));  // Set B0 low
    }
}

void deactivatePins() {
    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << B3) | (1 << B2) | (1 << B1) | (1 << B0));
}

void setupDMA() {
    spi->setFrequency(spiClk);
    spi->setHwCs(false);
}

uint16_t readADC(byte channel, int CS) {
    byte commandout = 0x80 + (channel << 4);
    uint16_t result = 0;
    
    spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    activatePins(CS);

    byte txBuffer[3] = {0x01, commandout, 0x00};
    byte rxBuffer[3] = {0};

    spi->transferBytes(txBuffer, rxBuffer, 3);

    deactivatePins();

    spi->endTransaction();

    result = ((rxBuffer[1] & 0x03) << 8) + rxBuffer[2];
    return result;
}

void displayNoteVelocity(int noteNumber, byte velocity) {
    char* notes[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};

    int index = noteNumber % 12;
    
    Serial.print("Note: ");
    Serial.print(notes[index]);
    Serial.print(", Velocity: ");
    Serial.println(velocity);
}

// double scale_value(double x, double p1[], double p2[], double zakres) {
//     double p = p1[0] * (zakres - (p2[0] - p1[0])) / zakres; // Definiujemy p
//     double q = -((p2[1] - p1[1]) / (pow(p2[0] - p, 2)) * pow(p1[0] - p, 2) + p1[1] - p1[1]) / 2;

//     return (p2[1] - (p1[1] + q)) / (pow(p2[0] - p, 2)) * pow(x - p, 2) + q + p1[1];
// }

// inline double scale_value(double x, double p1, double p2, double p3, double p4) {
//     const double zakres = 1024.0;
//     double delta_p = p3 - p1;
//     double delta_x = x - p1;
//     double p = p1 * (zakres - delta_p) / zakres *0.8;
//     double delta_p_sq = delta_p * delta_p;
//     double q = -((p4 - p2) / delta_p_sq * (p1 - p) * (p1 - p) + p2 - p2) / 2;

//     return (p4 - (p2 + q)) / delta_p_sq * delta_x * delta_x + q + p2;
// }

inline int inv_scale_value(double y, double p1, double p2, double p3, double p4) {
    const double zakres = 1024.0;
    double delta_p = p3 - p1;
    double p = p1 * (zakres - delta_p) / zakres;
    delta_p = p3 - p;
    double delta_p_sq = delta_p * delta_p;
    double q = -((p4 - p2) / delta_p_sq * (p1 - p) * (p1 - p));
    double sqrt_term = sqrt((y - p2 - q) * delta_p_sq / (p4 - (p2 + q)));

    return static_cast<int>(p + sqrt_term);
}


double scale_value2(double x, double p1, double p2, double p3, double p4){
  return (p4 - p2)/(p3 - p1)*(x - p1) + p2 - 1;
}


int calculateVelocity(float period) {
    if (period <= 5.25) {
      return 127;
    }
    if (period >= 97.19){
      return 0;
    }

    return 127 * (-log((period - 5.25) / 97.19)) * 0.12;
}

int calculateVelocity2(int time_us) { 
  // Przekształcenie x / 1000 na x >> SHIFT_DIV 
  int32_t scaled_x = time_us >> 10; 
  int32_t y = (1010 / (scaled_x +3.3f)) - 10; 
  if (y < 0) { 
    y = 0; 
  } else if (y > 127) { 
    y = 127; 
  }
return y; 
}


void handleButtonPressOctave() {
    bool currentButtonState2 = digitalRead(BUTTON2) == LOW;
    bool currentButtonState3 = digitalRead(BUTTON3) == LOW;

    if (currentButtonState2) {
      if (!buttonHeld2) {
            buttonHeld2 = true;
        }
        if(buttonHeld2 && !calibrationMode && !octave_changed){
          if(MIDI_OFFSET >= 29+12){
            MIDI_OFFSET = MIDI_OFFSET - 12;
            Serial.print("Przesunięcie klawiszy: ");
            Serial.print(MIDI_OFFSET);
            Serial.println();
            strip.setPixelColor(7, strip.Color(255,255,255));
            strip.setPixelColor(8, strip.Color(255,255,255));
            strip.show();
            delay(100);
            switch(mode){
              case 0:
                setAllLeds(strip.Color(0, 255, 0));
                strip.show();
              break;
              case 1:
                setAllLeds(strip.Color(0, 0, 255));
              strip.show();
              break;
            }
          }
          octave_changed = true;
        }
    }else{
      buttonHeld2 = false;
      octave_changed = false;
    }
    if (currentButtonState3) {
      if (!buttonHeld3) {
            buttonHeld3 = true;
        }
        if(buttonHeld3 && !calibrationMode && !octave_changed2){
          if(MIDI_OFFSET < 88+21 - 4 - number_of_keys){
            MIDI_OFFSET = MIDI_OFFSET + 12;
            Serial.print("Przesunięcie klawiszy: ");
            Serial.print(MIDI_OFFSET);
            Serial.println();
            strip.setPixelColor(10, strip.Color(255,255,255));
            strip.setPixelColor(11, strip.Color(255,255,255));
            strip.show();
            delay(100);
            switch(mode){
              case 0:
                setAllLeds(strip.Color(0, 255, 0));
                strip.show();
              break;
              case 1:
                setAllLeds(strip.Color(0, 0, 255));
              strip.show();
              break;
            }
          }
          octave_changed2=true;
        }
    }else{
      buttonHeld3 = false;
      octave_changed2 = false;
    }
    
}

void handleButtonPress() {
    bool currentButtonState = digitalRead(BUTTON) == LOW;

    if (currentButtonState) {
        if (!buttonHeld) {
            buttonPressTime = millis();
            buttonHeld = true;
        } else if (millis() - buttonPressTime >= 3000 && !calibrationMode) {
            delay(100);
            startCalibration();
            calibrationStartTime = millis();
        } else if(calibrationMode && (millis() - buttonPressTime < 100)){
          delay(100);
          completeCalibration();
        }
        if(buttonHeld && !calibrationMode && !modeChanged){
          mode = mode + 1;
          if(mode >= 2){
            mode = 0;
          }
          Serial.print("Tryb numer: ");
          Serial.println(mode);
          switch(mode){
            case 0:
              setAllLeds(strip.Color(0, 255, 0));
              strip.show();
            break;
            case 1:
              setAllLeds(strip.Color(0, 0, 255));
            strip.show();
            break;
          }
          

          modeChanged = true;
        }

    }else{
      buttonHeld = false;
      modeChanged = false;
    }
}

void updateCalibrationValues() {
    if(!calibrationMode){
    return;
    }

    for (int i = 0; i < number_of_keys; i++) {
        adc_val[i] = 1024 - readADC(i % 8, i / 8);
        // Serial.print(adc_val[i]);

        if (adc_val[i] > 1014 || adc_val[i] < 10) {
          continue;
        }
        if (calibrationHighPhase) {
            if (high[i] < adc_val[i]) {
                high[i] = adc_val[i];
            }
        }
        if (low[i] > adc_val[i]) {
            low[i] = adc_val[i];
        }
    }
    for (int i = 0; i < number_of_keys; i++) {
        Serial.print(adc_val[i]);
        
    }
    Serial.println();
      if (calibrationHighPhase && millis() - calibrationStartTime >= 1000) {
          calibrationHighPhase = false;
          setAllLeds(strip.Color(255, 0, 255));
          strip.show();
          Serial.println("High calibration phase completed");
      }
      
}

void startCalibration() {
    if(!is_good){
      Serial.println("CANNOT CALIBRATE, DEVICE IS NOT CONNECTED PROPERLY");
      return;
    }

    Serial.println("Re-calibration started");
    Serial1.println("r");
    calibrationMode = true;

    // Resetowanie danych kalibracji
    for (int i = 0; i < number_of_keys; i++) {
        high[i] = 0;
        low[i] = 1000;
    }

    calibrationStartTime = millis();
    calibrationHighPhase = true;

    EEPROM.write(EEPROM_CALIBRATION_FLAG_ADDR, 0);
    EEPROM.commit();
    delay(1000);
}

void completeCalibration() {
    Serial.println("Calibration completed");
    Serial1.println("k");
    calibrationMode = false;
    calibrationStartTime = 0;
    for (int i=0; i<number_of_keys; i++){
      int oldHigh = high[i];
      high[i] = inv_scale_value(high_level, low[i], bottom, high[i], top);
      low[i] = inv_scale_value(low_level, low[i], bottom, oldHigh, top);
    }

    // Zapis danych kalibracji do EEPROM
    for (int i = 0; i < number_of_keys; i++) {
        EEPROM.put(EEPROM_HIGH_START_ADDR + i * sizeof(int), high[i]);
        delay(20);
        EEPROM.put(EEPROM_LOW_START_ADDR + i * sizeof(int), low[i]);
        delay(20);
    }

    EEPROM.write(EEPROM_CALIBRATION_FLAG_ADDR, 1);
    EEPROM.commit();
    delay(1000);
    setAllLeds(strip.Color(0, 0, 255));
    strip.show();

void processKey(int key, double adcValue) {
    adcValue = 1024 - adcValue;
    // if(adcValue>=1014 || adcValue<=10){
    //   setAllLeds(strip.Color(255, 0, 0));  
    //   strip.show();        // Wyłączenie wszystkich LED na początku
    //   Serial.println("DEVICE IS NOT CONNECTED PROPERLY");
    //   delay(1000);
    //   is_good = false;
    //   return;
    // }else{
    //   if(!is_good){
    //     is_good = true;
    //     Serial.println("DEVICE IS CONNECTED SUCCESFULLY");
    //     switch(mode){
    //         case 0:
    //           setAllLeds(strip.Color(0, 255, 0));  
    //           strip.show();        // Wyłączenie wszystkich LED na początku
    //         break;
    //         case 1:
    //           setAllLeds(strip.Color(0, 0, 255));  
    //           strip.show();        // Wyłączenie wszystkich LED na początku
    //         break;
    //       }
    //   }
      
      if (adcValue < high[key]) {
          if (start[key] == 0) {
              start[key] = micros();
          }
          else if(adcValue < low[key] && finish[key] == 0 && start[key]!=0) {
              finish[key] = micros();
              unsigned long period = (finish[key] - start[key]);// * 0.001f;

              if (!can_be_off[key]) {
                  byte velocity = calculateVelocity2(period);
                  noteOn(key + MIDI_OFFSET, velocity);
                  midi.update();
                  displayNoteVelocity(key + MIDI_OFFSET, velocity);
                  Serial.println(period);
                  can_be_off[key] = true;
              }
          }
      } else if (adcValue > high[key] && (finish[key] != 0 || start[key] != 0)) {
          if (can_be_off[key]) {
              can_be_off[key] = false;
              noteOff(key + MIDI_OFFSET, 64);
              midi.update();
          }
          start[key] = 0;
          finish[key] = 0;
      }
    
    
}

void updateKeys() {
    for (int i = 0; i < number_of_keys; i++) {
        double adcValue = readADC(i % 8, i / 8);
        processKey(i, adcValue);
    }
}

void setAllLeds(uint32_t color) {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, color);
  }
}


void setup() {
  midi.begin();
  neopixelWrite(RGB_BUILTIN, 0, 0, 0);
  strip.begin();
  setAllLeds(strip.Color(100, 0, 0));  
  strip.setBrightness(64);
  strip.show();

  spi = new SPIClass(SPI);

  spi->begin(6, 5, 4);

  setupDMA();

  Serial.begin(1000000);
  delay(1000);
  Serial.print("Dziala");

  Serial1.begin(9600, SERIAL_8N1, 40, 39);

  pinMode(B0, OUTPUT);
  pinMode(B1, OUTPUT);
  pinMode(B2, OUTPUT);
  pinMode(B3, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);

  for (int i = 0; i < number_of_keys; i++) {
        can_be_off[i] = false;
        high[i] = 0;
        low[i] = 1000;
        adc_val[i] = 0;
        start[i] = 0;
        finish[i] = 0;
  }

  EEPROM.begin(EEPROM_SIZE);
  delay(100);
  calibrationMode = EEPROM.read(EEPROM_CALIBRATION_FLAG_ADDR) != 1;
  if (!calibrationMode) {
    setAllLeds(strip.Color(0, 0, 255));  
    strip.show();
    for (int i = 0; i < number_of_keys; i++) {
      EEPROM.get(EEPROM_HIGH_START_ADDR + i * sizeof(int), high[i]);
      delay(10);
      EEPROM.get(EEPROM_LOW_START_ADDR + i * sizeof(int), low[i]);
      delay(10);
      Serial.print(high[i]);
      Serial.print(" ");
      Serial.println(low[i]);

      if (high[i] == 0xFFFF || low[i] == 0xFFFF) {
        high[i] = 0;
        low[i] = 1000;
      }
    }
  } else {
    
    for (int i = 0; i < number_of_keys; i++) {
      high[i] = 0;
      low[i] = 1000;
    }
  }
}

unsigned long lastFastSampleTime = 0;
unsigned long lastSlowSampleTime = 0;
unsigned long lastTime = 0;
int iterator = 0;
void loop() {

    // iterator = iterator + 1;
    // if(iterator>=100000)
    // {
    //   double diff = millis() - lastTime;
    //   // Serial.println(100000.0f/(diff/1000));
    //   iterator = 0;
    //   lastTime = millis();
    // }


    if (micros() - lastFastSampleTime >= 1000) {
      // iterator = iterator + 1;
      // if(iterator>=10000)
      // {
      //   double diff = millis() - lastTime;
      //   Serial.println(10000.0f/(diff/1000));
      //   iterator = 0;
      //   lastTime = millis();
      // }
      lastFastSampleTime = micros();
      if (!calibrationMode) {
          updateKeys();
      }
      updateCalibrationValues();
    }
    if (millis() - lastSlowSampleTime >= 50) {
      lastSlowSampleTime = millis();
      handleButtonPress();
      handleButtonPressOctave();
        
      if (Serial1.available() > 0) {
          byte receivedData = Serial1.read();

        if (receivedData == 0x00 && is_pressed == false) {
            midi.sendControlChange(64, 127);
            midi.update();
            Serial.println("Prawy pedal: ON");
            is_pressed = true;
        } else if (receivedData == 0x01 && is_pressed == true) {
            midi.sendControlChange(64, 0);
            midi.update();
            Serial.println("Prawy pedal: OFF");
            is_pressed = false;
        }
        if (receivedData == 0x02) {
            midi.sendControlChange(67, 127);
            midi.update();
            Serial.println("Lewy pedal: ON");
        } else if (receivedData == 0x03) {
            midi.sendControlChange(67, 0);
            midi.update();
            Serial.println("Lewy pedal: OFF");
        }
      }
    }
}

