#include <LiquidCrystal.h>
#include <Servo.h>
//#include <SoftwareSerial.h>


//Bluetooth
//SoftwareSerial mySerial(A3, A4); // RX, TX
//String command = "";


//Servo motor
Servo myservo;
int servo = 6;


//Button
int button = 7;
bool buttonPressed = false;
unsigned long buttonTimePressed = 0;
unsigned long buttonAction2Time = 1000;
bool gameRunning = false;
int gameMode = 0;
bool startMode1 = false;
String imaginaryFirstSide;
unsigned long imaginaryDeltaS;


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
unsigned long firstSignal = 0; //microseconds
unsigned long timeToWaitAnotherCapture = 500000; //microseconds


//Assyncronous timers that set the time each LED stays on
unsigned long startT1 = 0;
unsigned long deltaT1 = 1000;
unsigned long startT2 = 0;
unsigned long deltaT2 = 1000;


//LCD display
const int rs = 13, en = 12, d4 = 11, d5 = 10, d6 = 9, d7 = 8;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
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
byte arrow[8] = {
  B00000,
  B00000,
  B01000,
  B11111,
  B01000,
  B00000,
  B00000,
};

void drawMode2(unsigned long deltaS, String firstSide) {
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
void drawSplashScreen() {
  lcd.clear();
  lcd.print("      Som  ");
  lcd.setCursor(0, 1);
  lcd.print(" ");
  lcd.write(byte(1));
  lcd.print(" Imaginario ");
  lcd.write(byte(0));
}
void drawMenu(int option) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("mode 1");
  if(option == 1) {
    lcd.write(byte(3));
  }
  lcd.setCursor(0, 1);
  lcd.print("mode 2");
  if(option == 2) {
    lcd.write(byte(3));
  }
}


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
  lcd.createChar(3, arrow);
  lcd.begin(16, 2);
  drawSplashScreen();
}



int showMenu() {
  int buttonVal = digitalRead(button);
  
  while(buttonVal == HIGH){
    buttonVal = digitalRead(button);
  }
  
  int option = 1;
  drawMenu(option);
  
  while(true) {
    int buttonVal = digitalRead(button);

    //First moment that the button is pressed
    if(buttonVal == HIGH && !buttonPressed) {
      buttonPressed = true;
      buttonTimePressed = millis();
    }
    /*If the button is pressed for greater or equal than the time defined by buttonAction2Time, 
    then the option selected is toggled*/
    else if(buttonVal == HIGH && buttonPressed && (millis() - buttonTimePressed) >= buttonAction2Time) {
      option = (option == 1)? 2 : 1;
      buttonPressed = false;
      buttonTimePressed = 0;
      drawMenu(option);
      delay(2000);
    }
    //If the button was pressed for less than buttonAction2Time, then the current option is selected
    else if(buttonVal == LOW && buttonPressed) {
      //drawSplashScreen();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("     mode ");
      lcd.print(option);
      startMode1 = (option == 1);
      return option;
    }
    else if(buttonVal == LOW) {
      buttonPressed = false;
      buttonTimePressed = 0;  
    }
  }
}

void mode2() {
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
        drawMode2(deltaS, "left");
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
        drawMode2(deltaS, "right");
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

void mode1() {
  if(startMode1) {
    imaginaryFirstSide = (random(0, 2) == 0)? "left" : "right";
    imaginaryDeltaS = random(0, 401);
    drawMode2(imaginaryDeltaS, imaginaryFirstSide);
    delay(2000);
    startMode1 = false;
  }

  int buttonVal = digitalRead(button);
  if(buttonVal == HIGH) {
    int potentiometerVal = analogRead(potentiometer);
//    unsigned long imaginaryAngle = acos(((imaginaryDeltaS / 1000000L) * 343L) / 0.168L);
//    //Convert angle from radians to degrees
//    imaginaryAngle = imaginaryAngle * 57296L / 1000L;
//    if(imaginaryFirstSide == right) 
      Serial.println(potentiometerVal);
      Serial.println(imaginaryFirstSide);
    if((potentiometerVal < 430) && (imaginaryFirstSide == "right")){
      lcd.clear();
      lcd.print("Correct!!!");
      delay(3000);
    }
    else if((potentiometerVal > 430) && (imaginaryFirstSide == "left")){
      lcd.clear();
      lcd.print("Correct!!!");
      delay(3000);
    }
    else {
      lcd.clear();
      lcd.print("Try again!");
      delay(3000);
    }

      
  }
}


void loop(){
  if(gameMode == 2) {
    mode2();
  }
  else if(gameMode == 1) {
    mode1();
  }
  
  int buttonVal = digitalRead(button);
  if(buttonVal == HIGH && !gameRunning) {
    gameMode = showMenu();
    gameRunning = true;
  }
  
}
