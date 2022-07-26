#include <LiquidCrystal.h>
#include <Servo.h>

//Servo motor
Servo myservo;
int servo = 6;

//Button
int button = 7;

//Potentiometer
int potentiometer = A5;


//Sound sensors and theirs associated LEDs
int led1 = 5;
int led2 = 4;
int rightSensorPin = 2;
int leftSensorPin = 3;

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

byte delta[8] = {
  B00000,
  B00100,
  B00100,
  B01010,
  B01010,
  B10001,
  B11111,
};


void setup(){
  //Button configuration
  pinMode(button, INPUT);

  //Sound sensors configuration
  Serial.begin(9600);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(rightSensorPin, INPUT);  
  pinMode(leftSensorPin, INPUT);  

  //Assyncronous timers for the LEDs activation
  startT1 = millis();
  startT2 = millis();

  //LCD writing and configuration
  lcd.createChar(0, note);
  lcd.createChar(1, note2);
  lcd.createChar(2, delta);
  lcd.begin(16, 2);
  lcd.print("      Som  ");
  lcd.setCursor(0, 1);
  lcd.print(" ");
  lcd.write(byte(1));
  lcd.print(" Imaginario ");
  lcd.write(byte(0));
}

void printDelta(unsigned long deltaS, String firstSide) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("first: " + firstSide);
  lcd.setCursor(0, 1);
  lcd.write(byte(2));
  lcd.print("=");
  lcd.print(deltaS);
  lcd.print(",");
  lcd.setCursor(12, 1);
  lcd.print("E-9s");
}

void loop(){
  

  //Read sound signals
  int rightSensorVal = digitalRead(rightSensorPin);
  int leftSensorVal = digitalRead(leftSensorPin);

  if(!firstS1) {
    if(rightSensorVal == HIGH) {
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
        printDelta(deltaS, "left");
        deltaS = 12;
        unsigned long servoAngle = acos(((deltaS / 1000000L) * 343L) / 0.168L);
        //Convert angle from radians to degrees
        servoAngle = servoAngle * 57296L / 1000L;
        Serial.println(deltaS / 1000000L);
        Serial.println(servoAngle);
        myservo.attach(servo);
        //Convert angle from radians to degrees
        myservo.write(servoAngle);
        delay(200);
        myservo.detach();
      }
      
      startT1 = millis();
      digitalWrite(led1, HIGH);
    }
    else if(millis() > (startT1 + deltaT1)){
      digitalWrite(led1, LOW);
    }
  }

  if(!firstS2){
    if(leftSensorVal == HIGH){
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
        printDelta(deltaS, "right");
        unsigned long servoAngle = acos(((deltaS / 1000000L) * 343L) / 0.168L);
        //Convert angle from radians to degrees
        servoAngle = servoAngle * 57296L / 1000L;
        Serial.println(servoAngle);
        myservo.attach(servo);
        myservo.write(180L - servoAngle);
        delay(200);
        myservo.detach();
      }
      
      startT2 = millis();
      digitalWrite (led2, HIGH);
    }
    else if(millis() > (startT2 + deltaT2)){
      digitalWrite (led2, LOW);
    }
  }
}
