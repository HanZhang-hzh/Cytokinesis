#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>
#include "arduino_secrets.h"

char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "test.mosquitto.org";
int port = 1883;
const char topic[] = "HansDistanceSensor";

const int motorPWM = 9;
const int motorA1 = 10;
const int motorA2 = 11;
const int motorB1 = 8;
const int motorB2 = 7;
const int motor2PWM = 6;
const int trigPin = 5;
const int echoPin = 4;
bool isMotorMoving = false;
unsigned long motorStartTime = 0;          // Variable to store the start time of motor movement
const unsigned long motorDuration = 5000;  // Motor movement duration in milliseconds (5 seconds)

const long interval = 50;
unsigned long previousMillis = 0;
int count = 0;

void setup() {
  Serial.begin(9600);
  pinMode(motorPWM, OUTPUT);
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  //SENSOR TO TFHE OTHER BOARD
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);


  // while (!Serial) {
  //   ;
  // }

  Serial.print("Attempting to connect to SSID: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("You're connected to the network");
  Serial.println();

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    while (1)
      ;
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

  mqttClient.onMessage(onMqttMessage);
  mqttClient.subscribe(topic);
}

void loop() {
  mqttClient.poll();
  distanceToB();

  //Check if the motor is currently moving and has exceeded the duration
  if (isMotorMoving && (millis() - motorStartTime >= motorDuration)) {
    stopMotor();
    Serial.println("Motor stopped after 5 seconds.");
    isMotorMoving = false;
  }
}

void onMqttMessage(int messageSize) {
  int mqttClientData = mqttClient.parseInt();
  move(mqttClientData);
}

void move(int value) {
  if (value < 400) {
    Serial.println("Starting motor.");
    moveMotor();
    isMotorMoving = true;
    motorStartTime = millis();  // Record the start time of motor movement
  } else {
    Serial.println("Stopping motor.");
    stopMotor();
    isMotorMoving = false;
  }
}

void moveMotor() {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  analogWrite(motorPWM, 100);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
  analogWrite(motor2PWM, 100);
  Serial.println("Move motor Called.");
}

void reverseMotor() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
  analogWrite(motorPWM, 100);
  analogWrite(motor2PWM, 100);
}

void stopMotor() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  analogWrite(motorPWM, 0);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
  analogWrite(motor2PWM, 0);
}

void distanceToB() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time a message was sent
    previousMillis = currentMillis;

    //record random value from A0, A1 and A2
    digitalWrite(trigPin, LOW);
    // delay 2 microseconds:
    delayMicroseconds(2);
    // take the trigger pin high:
    digitalWrite(trigPin, HIGH);
    // delay 10 microseconds:
    delayMicroseconds(10);
    // take the trigger pin low again to complete the pulse:
    digitalWrite(trigPin, LOW);

    // listen for a pulse on the echo pin:
    long duration = pulseIn(echoPin, HIGH);
    // calculate the distance in cm.
    //Sound travels approx.0.0343 microseconds per cm.,
    // and it's going to the target and back (hence the /2):
    int distance = (duration * 0.0343) / 2;
    Serial.print("Distance: ");
    Serial.println(distance);
    // a short delay between readings:
    delay(10);
    // int Rvalue = analogRead(A0);
    // int Rvalue2 = analogRead(A1);
    // int Rvalue3 = analogRead(A2);

    Serial.print("Sending distance to topic: ");
    Serial.println(topic);
    Serial.println(distance);

    // send message, the Print interface can be used to set the message contents
    mqttClient.beginMessage(topic);
    mqttClient.print(distance);
    mqttClient.endMessage();

    // mqttClient.beginMessage(topic2);
    // mqttClient.print(Rvalue2);
    // mqttClient.endMessage();

    // mqttClient.beginMessage(topic3);
    // mqttClient.print(Rvalue3);
    // mqttClient.endMessage();

    Serial.println();
  }
}