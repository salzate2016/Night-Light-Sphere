
//#include <DCMotorBot.h>
#include <SoftwareSerial.h>
#include <BlynkSimpleSerialBLE.h>

#define BLYNK_PRINT Serial
#define LDR 0
#define PIR 9
#define LED 12


SoftwareSerial SwSerial(10, 11); // RX, TX
    
int forward=0;
int backward=0;
int left=0;
int right=0;

int pirState;
int ldrValue;
int option=1;
int button=0;
int security=0;
int follow=0;
const int Lin = A6, Rin = 13, Lout = A7, Rout = 6;//Sets up the ultrasonic sensor input and output
long Rduration, Lduration, Rcms, Lcms; // Duration of ultrasonic pulse and resulting distance in cms
int threshold = 10; 

boolean debug = true; // Starts the serial monitor
 
byte speed = 255;  // change this (0-255) to control the speed of the motors

// connect motor controller pins to Arduino digital pins
// motor one
int enA = 3;
int in1 = 2;
int in2 = 4;
// motor two
int enB = 5;
int in3 = 7;
int in4 = 8;


char auth[] = "7fe5b78d36ad4378836654a74d136a6a";

SoftwareSerial SerialBLE(10, 11); // RX, TX

BLYNK_WRITE(V1) {
  button=param.asInt();
}

BLYNK_WRITE(V0) {
  option=param.asInt();
}

BLYNK_WRITE(V2) {
  security=param.asInt();
}
BLYNK_WRITE(V3) {
  forward=param.asInt();
}

BLYNK_WRITE(V4) {
  backward=param.asInt();
}

BLYNK_WRITE(V5) {
  left=param.asInt();
}

BLYNK_WRITE(V6) {
  right=param.asInt();
}

BLYNK_WRITE(V7) {
  follow=param.asInt();
}

//DCMotorBot bot;

void setup() {

 // set all the motor control pins to outputs
 pinMode(enA, OUTPUT);
 pinMode(enB, OUTPUT);
 pinMode(in1, OUTPUT);
 pinMode(in2, OUTPUT);
 pinMode(in3, OUTPUT);
 pinMode(in4, OUTPUT);

  pinMode(LED, OUTPUT);
  pinMode(PIR, INPUT);
  pinMode(LDR, INPUT);
  digitalWrite(LED, LOW);

  pinMode(Rout, OUTPUT); // sets pins for ultrasonic sensors
  pinMode(Rin, INPUT);
  pinMode(Lout, OUTPUT);
  pinMode(Lin, INPUT);

    // initialize bot pins
  //bot.setEnablePins(3, 5);
  //bot.setControlPins(2, 4, 7, 8);

   
     // Debug console
  Serial.begin(9600);

  SerialBLE.begin(9600);
  Blynk.begin(SerialBLE, auth);

  Serial.println("Waiting for connections...");
}

void loop() {
    Blynk.run();
    if(follow)
    {
        digitalWrite(Rout, LOW);  // Pulses for the right sensor. 
        digitalWrite(Rout, HIGH);
        delayMicroseconds(2);
        digitalWrite(Rout, LOW);
      
        Rduration = pulseIn(Rin, HIGH);
      
        digitalWrite(Lout, LOW);  // Pulses for the left sensor
        digitalWrite(Lout, HIGH);
        delayMicroseconds(2);
        digitalWrite(Lout, LOW);
      
        Lduration = pulseIn(Lin, HIGH);
      
        Rcms = microsecondsToCms(Rduration); 
        Lcms = microsecondsToCms(Lduration);
        
        if (debug)
        {
          Serial.print("Left: ");
          Serial.print(Lcms);
          Serial.println(" cms");
          Serial.print("Right: ");
          Serial.print(Rcms);
          Serial.println(" cms");
        }
        
        followMe(); 
    }
    else
    {
      control();
    }
    if(option==1){standard();}
    if(option==2){smart(button);}
    securityMode();
}

void control(){
  if(forward||backward||left||right){
    if(forward){goForward();Serial.println("1");}
    if(backward){goBackward();Serial.println("2");}
    if(left){turnLeft();Serial.println("3");}
    if(right){turnRight();Serial.println("4");}
  }
  else{stop();}
  
}
    
void standard(){
    ldrValue = analogRead(LDR);
    //Serial.println(ldrValue);
    pirState = digitalRead(PIR);
    
    if (ldrValue <= 512 && pirState == HIGH) { // dark and detect movement
      digitalWrite(LED, HIGH);
      delay(6000);
    }
    else{digitalWrite(LED, LOW);}
}

void smart(int button){
  if(button)
  {
    digitalWrite(LED, HIGH);
  }
  else{
     digitalWrite(LED, LOW);
  }
}

void securityMode(){
  pirState = digitalRead(PIR);
  if(security){
    Blynk.virtualWrite(V8, pirState);
  }
}

void goForward(){
  analogWrite(enA, 255);
  analogWrite(enB, 255);
  delay(20);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void goBackward(){
  analogWrite(enA, 255);
  analogWrite(enB, 255);
  delay(20);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void turnLeft(){
  analogWrite(enA, 100);
  analogWrite(enB, 100);
  delay(20);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void turnRight(){
  analogWrite(enA, 100);
  analogWrite(enB, 100);
  delay(20);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void stop(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW); 

}

long microsecondsToCms(long microseconds)
{
  // // Converts pulse time to a distance in cms
  return microseconds / 2 / 29.1;
}

void followMe() 
{
  if (Lcms == threshold || Rcms == threshold) {
    stop();
    Serial.print(" Stopping!");
  }
  
  if (Lcms > threshold) {   
    turnLeft();
    Serial.print(" Moving left!");
  }
  if (Rcms > threshold) { 
      turnRight();
      Serial.print(" Moving right!");
    }
      if (Lcms > threshold && Lcms < 60) { // Moves forward if the car is within this range
        goForward();
        Serial.print(" Moving forward!");
      }
        if (Rcms > threshold && Rcms < 60) {
          goBackward;
          Serial.print(" Moving forward!");
        }
}
  // methods for driving the motors in each direction. Backward isn't used here, but could be.
