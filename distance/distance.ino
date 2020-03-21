#include <OneWire.h>
#include <DallasTemperature.h>
#define trigPin2 D5
#define echoPin2 D6
#define INTERVAL_MESSAGE1 10000
#define INTERVAL_MESSAGE2 20000
// Data wire is plugged into digital pin 2 on the Arduino
#define ONE_WIRE_BUS D2

long duration, distance, FrontSensor;

unsigned long time_1 = 0;
unsigned long time_2 = 0;
// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire(ONE_WIRE_BUS);

// Pass oneWire reference to DallasTemperature library
DallasTemperature sensors(&oneWire);


void print_time(unsigned long time_millis);

void setup()
{
  Serial.begin (9600);
  sensors.begin();
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

}

void loop() {

  SonarSensor(trigPin2, echoPin2);
  FrontSensor = distance;

  // Serial.print("Distance:");

  //  Serial.println(FrontSensor);
  // Send the command to get temperatures
  sensors.requestTemperatures();

  //print the temperature in Celsius
  //  Serial.print("Temperature: ");
  //  Serial.print(sensors.getTempCByIndex(0));
  //
  //  Serial.println("C");

  if (millis() >= time_1 + INTERVAL_MESSAGE1) {
    time_1 += INTERVAL_MESSAGE1;
    print_time(time_1);
    Serial.println("I'm message number one!");
    Serial.print("Distance:");
    Serial.println(FrontSensor);
  }

  if (millis() >= time_2 + INTERVAL_MESSAGE2) {
    time_2 += INTERVAL_MESSAGE2;
    print_time(time_2);
    Serial.println("Hello, I'm the second message.");
    Serial.print("Temperature: ");
    Serial.print(sensors.getTempCByIndex(0));

    Serial.println("C");
  }


}
void print_time(unsigned long time_millis) {
  Serial.print("Time: ");
  Serial.print(time_millis / 1000);
  Serial.print("s - ");
}

void SonarSensor(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) / 29.1;

}
