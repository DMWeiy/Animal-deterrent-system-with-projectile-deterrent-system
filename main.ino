#include <Wire.h>
#include <NewPing.h>

#include <SoftwareSerial.h>


SoftwareSerial A(0, 1);


// motor right
int R_ISl = 4;
int L_ISl = 4;
int R_ENl = 5;
int L_ENl = 5;
int R_PWMl = 11;
int L_PWMl = 10;

// motor left
int R_IS2 = 2;
int L_IS2 = 2;
int R_EN2 = 3;
int L_EN2 = 3;
int L_PWM2 = 6;
int R_PWM2 = 9;

//define ultrassonic sensor pin
#define trig_pin 12  //analog input 1
#define echo_pin 13
#define MAX_DISTANCE 500
int B_distance = 50;
int cm = 0;
NewPing sonar(trig_pin, echo_pin, MAX_DISTANCE);

//ir sensor
#define left_ir 8
#define right_ir 7
bool M_stop = false;

// function for movement
void move_Forward(){
  analogWrite(R_PWMl, 220); 
  analogWrite(L_PWMl, 0); 
  analogWrite(R_PWM2, 220); 
  analogWrite(L_PWM2, 0); 
  Serial.println("forward"); 
  delay(1000); 
}

void foraward_A_T(){
  analogWrite(R_PWMl, 200); 
  analogWrite(L_PWMl, 0); 
  analogWrite(R_PWM2, 200); 
  analogWrite(L_PWM2, 0); 
  Serial.println( "forward At"); 
  delay(1000); 
}

void move_Backward(){
  analogWrite(R_PWMl, 0); 
  analogWrite(L_PWMl, 220); 
  analogWrite(R_PWM2, 0); 
  analogWrite(L_PWM2, 220); 
  Serial.println("BAck"); 
  delay(1000); 
}
void turn_Right() {
  analogWrite(R_PWMl, 0); 
  analogWrite(L_PWMl, 90); 
  analogWrite(R_PWM2, 90); 
  analogWrite(L_PWM2, 0); 
  Serial.println("RIGHT"); 
  delay(1000); 
}
void turn_Right_A_T() {
  analogWrite(R_PWMl, 0); 
  analogWrite(L_PWMl, 90); 
  analogWrite(R_PWM2, 90); 
  analogWrite(L_PWM2, 0); 
  Serial.println("RIGHT AT"); 
  delay(1000); 
}
void turn_Left(){
  analogWrite(R_PWMl, 90); 
  analogWrite(L_PWMl, 0); 
  analogWrite(R_PWM2, 0); 
  analogWrite(L_PWM2, 90); 
  Serial.println("Left"); 
  delay(1000); 
}

void turn_Left_A_T(){
  analogWrite(R_PWMl, 90); 
  analogWrite(L_PWMl, 0);
  analogWrite(R_PWM2, 0); 
  analogWrite(L_PWM2, 90); 
  Serial.println("Left AT"); 
  delay(1000); 
}
void stop(){
  analogWrite(R_PWMl, 0); 
  analogWrite(L_PWMl, 0); 
  analogWrite(R_PWM2, 0); 
  analogWrite(L_PWM2, 0); 
  delay(1500); 
}

void setup() {
  //define as master
  Wire.begin();
  A.begin(4800);
  Serial.begin(9600);

  //setup for motor right
  pinMode(R_ISl, OUTPUT);
  pinMode(R_ENl, OUTPUT);
  pinMode(R_PWMl, OUTPUT);

  pinMode(L_ISl, OUTPUT);
  pinMode(L_ENl, OUTPUT);
  pinMode(L_PWMl, OUTPUT);
  digitalWrite(R_ISl, LOW);
  digitalWrite(L_ISl, LOW);
  digitalWrite(R_ENl, HIGH);
  digitalWrite(L_ENl, HIGH);
  //setup for motor left
  pinMode(R_IS2, OUTPUT);
  pinMode(R_EN2, OUTPUT);
  pinMode(R_PWM2, OUTPUT);
  pinMode(L_IS2, OUTPUT);
  pinMode(L_EN2, OUTPUT);
  pinMode(L_PWM2, OUTPUT);
  digitalWrite(R_IS2, LOW);
  digitalWrite(L_IS2, LOW);
  digitalWrite(R_EN2, HIGH);
  digitalWrite(L_EN2, HIGH);

  //ir sensor
  pinMode(left_ir, INPUT);
  pinMode(right_ir, INPUT);
}



void loop() {
  delay(20);
  cm = sonar.ping_cm();
  delay(40);
  Serial.println(cm);
  //recieveData Stop forward form slave arduino
  if (A.available() > 0 && M_stop == true) {
    char message = "";
    message = A.read();
    delay(10);
    if (message == 'n' || message == 'N') {
      M_stop = true;
      Serial.println("Stop");
      message = "";
    }
  }
  delay(50);
  if (M_stop == true) {
    Serial.println("Stop");
    stop();
    delay(1000);
  }

  while (M_stop == false) {
    delay(20);
    cm = sonar.ping_cm();
    Serial.print("First");
    Serial.println(cm);
    delay(20);
    if (A.available() > 0) {
      char message = "";
      message = A.read();
      if (message == 's' || message == 'S') {
        M_stop = true;
        Serial.println("Stop");
        break;
        message = "";
      }
    }
    //obstacle avoid
    if (cm < B_distance) {
      stop();
      if (digitalRead(left_ir)==LOW && digitalRead(right_ir)== HIGH) {
        Serial.println("Statement 1");
        cm = sonar.ping_cm();
        turn_Left();
        delay(1400);
        stop();
        cm = sonar.ping_cm();
        foraward_A_T();
        delay(1000);
        cm = sonar.ping_cm();
        turn_Right_A_T();
        delay(1400);
        stop();
        cm = sonar.ping_cm();
        delay(20);
        cm = sonar.ping_cm();
        delay(50);
        if (cm >= B_distance) {
          Serial.println("End");
          break;
        } else {
          continue;
        }
      }else if (digitalRead(left_ir) == HIGH && digitalRead(right_ir) == LOW ){
        Serial.println("Statement 2");
        cm = sonar.ping_cm(); 
        turn_Right(); 
        delay(1400); 
        stop(); 
        foraward_A_T(); 
        delay(1000); 
        cm = sonar.ping_cm(); 
        turn_Left_A_T(); 
        delay(1400); 
        cm = sonar.ping_cm(); 
        stop(); 
        delay(20); 
        cm = sonar.ping_cm(); 
        delay(50); 
        if (cm >= B_distance) { 
          break; 
        }
      }else if (digitalRead(left_ir) == LOW && digitalRead(right_ir) == LOW ){
        Serial.println("Statement 3");
        cm = sonar.ping_cm(); 
        turn_Right(); 
        delay(1400); 
        stop(); 
        foraward_A_T(); 
        delay(1000); 
        cm = sonar.ping_cm(); 
        turn_Left_A_T(); 
        delay(1400); 
        cm = sonar.ping_cm();   
        stop(); 
        delay(20); 
        cm = sonar.ping_cm(); 
        delay(50); 
        if (cm >= B_distance) {
          break;
        }
      }else if (cm >= B_distance) {
        move_Forward(); 
        delay(20); 
        cm = sonar.ping_cm(); 
        Serial.println(cm); 
        delay(50); 
      }else if (cm < B_distance && digitalRead(left_ir) == HIGH && digitalRead(right_ir) == HIGH) {
        turn_Right(); 
        delay(1400); 
      }
      delay(10);
    }
  }
  //end
}

