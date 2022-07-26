#include <LiquidCrystal.h>
#include <Servo.h>

//Servo motor
Servo myservo;
int servo = 6;

//Sound sensors and theirs associated LEDs
int led1 = 5;
int led2 = 4;
int sound_d1 = 2;
int sound_d2 = 3;

unsigned long deltaS = 0;
bool firstS1 = false;
bool firstS2 = false;
bool calculatingOrigin = false;
unsigned long firstSignal = 0;
unsigned long timeToWaitAnotherCapture = 500000;

//Assyncronous timers that set the time each LED stays on
unsigned long startT1 = 0;
unsigned long deltaT1 = 3000;
unsigned long startT2 = 0;
unsigned long deltaT2 = 3000;

//LCD display
const int rs = 13, en = 12, d4 = 11, d5 = 10, d6 = 9, d7 = 8;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
byte smiley[8] = {
  B00000,
  B10001,
  B00000,
  B00000,
  B10001,
  B01110,
  B00000,
};

byte note[8] = {
  B00010,
  B00011,
  B00010,
  B00010,
  B01110,
  B11110,
  B01100,
};
byte note2[8] = {
  B00001,
  B00111,
  B01101,
  B01001,
  B01011,
  B11011,
  B11000,
};

void setup(){
  //Servo configuration
  myservo.attach(servo);

  //Sound sensors configuration
  Serial.begin(9600);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(sound_d1, INPUT);  
  pinMode(sound_d2, INPUT);  

  //Assyncronous timers for the LEDs activation
  startT1 = millis();
  startT2 = millis();

  //LCD writing and configuration
  lcd.createChar(0, note);
  lcd.createChar(1, note2);
  lcd.begin(16, 2);
  lcd.print("      Som  ");
  lcd.setCursor(0, 1);
  lcd.print(" ");
  lcd.write(byte(1));
  lcd.print(" Imaginario ");
  lcd.write(byte(0));
}

void loop(){
  myservo.detach();

  //Read sound signals
  int val_digital1 = digitalRead(sound_d1);
  int val_digital2 = digitalRead(sound_d2);
  //Serial.println(val_digital2);

  if(!firstS1) {
    if(val_digital1 == HIGH) {
      if(!calculatingOrigin && (micros() - (firstSignal + deltaS) > timeToWaitAnotherCapture)){
        firstSignal = micros();
        firstS1 = true;
        calculatingOrigin = true;
      }
      else if(firstS2) {
        deltaS = micros() - firstSignal;
        calculatingOrigin = false;
        firstS1 = false;
        firstS2 = false;
        Serial.println(deltaS);
      }
      
      startT1 = millis();
      digitalWrite(led1, HIGH);
    }
    else if(millis() > (startT1 + deltaT1)){
      digitalWrite(led1, LOW);
    }
  }

  if(!firstS2){
    if(val_digital2 == HIGH){
      if(!calculatingOrigin && (micros() - (firstSignal + deltaS) > timeToWaitAnotherCapture)){
        firstSignal = micros();
        calculatingOrigin = true;
        firstS2 = true;
      }
      else if(firstS1) {
        deltaS = micros() - firstSignal;
        calculatingOrigin = false;
        firstS2 = false;
        firstS1 = false;
        Serial.println(deltaS);
      }
      
      startT2 = millis();
      digitalWrite (led2, HIGH);
    }
    else if(millis() > (startT2 + deltaT2)){
      digitalWrite (led2, LOW);
    }
  }

  

  myservo.attach(servo);
//  for (int pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
//    myservo.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(30);                       // waits 15ms for the servo to reach the position
//  }
//  for (int pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
//    myservo.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(30);                       // waits 15ms for the servo to reach the position
//  }

   //calculatingOrigin = (millis() - firstSignal) < timeToWaitAnotherCapture;
   //Serial.println(calculatingOrigin);

  
}
