#include "BluetoothSerial.h"
#include <ESP32Servo.h>
Servo myservo;

String device_name = "ESP32-BT-Slave";

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;
int i = 0;
#define RXp2 16
#define TXp2 17
#define BUZZER_PIN 23  // Replace with your buzzer pin
#define SERVO_PIN 2

// Replace with your servo pin
int motorlPinl = 21;
int motor1Pin2 = 19;
int motor2Pinl = 18;
int motor2Pin2 = 5;
int pos = 0;
char U_input;
int attack_count = 0;  // Counter for "attack" command
bool buzzer_active = false;


void setup() {
  Serial.begin(115200);
  Serial2.begin(4800, SERIAL_8N1, RXp2, TXp2);
  SerialBT.begin("ESP32Salve");  //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  pinMode(BUZZER_PIN, OUTPUT);  // Configure buzzer pin as output // Attach servo to its pin
  myservo.setPeriodHertz(50);   // standard 50 hz servo
  myservo.attach(SERVO_PIN, 500, 2400);
  pinMode(motorlPinl, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pinl, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
}

void moveBackward() {
  digitalWrite(motorlPinl, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pinl, HIGH);
  digitalWrite(motor2Pin2, LOW);
}

void moveForward() {
  digitalWrite(motorlPinl, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pinl, LOW);
  digitalWrite(motor2Pin2, HIGH);
}


void loop() {
  delay(10);

  if (SerialBT.available() > 0) {
    U_input = SerialBT.read();
    delay(10);
    Serial.println(U_input);
    if (U_input == 'a') {
      attack_count++;
      Serial2.println("s");
      SerialBT.println("attack");
      delay(10);
      if (attack_count == 1) {
        digitalWrite(BUZZER_PIN, HIGH);
        buzzer_active = true;
        delay(10000);
      }
    } else if (attack_count== 3) {
      Serial2.println("s");
      //II Deactivate buzzer and activate servo on third "attack"
      digitalWrite(BUZZER_PIN, LOW);
      buzzer_active = false;
    }
    //Move servo to 90 degrees and back to 0
    for (pos = 0; pos <= 90; pos += 1) {  //goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      myservo.write(pos);
      //tell servo to go to position in variable 'pos' 
      delay(15); // waits 15ms for the servo to reach the position
      Serial.println(pos);
    }
    delay(2000);
    for (pos = 90; pos >= 0; pos -= 1) {  //goes from 180 degrees to 0 degrees
      myservo.write(pos);                 //tell servo to go to position in variable 'pos'
      delay(15);
      Serial.println(pos);  // waits 15ms for the servo to reach the position
    }
  } else if (U_input== 'n') {
    delay(10);
    Serial2.println("n");
    SerialBT.println("nothing");
  }
}
