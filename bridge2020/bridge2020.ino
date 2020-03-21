
#include <Wire.h>
#include <SPI.h>
//#include <Adafruit_Sensor.h>

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

const char* ssid = "DataSoft_WiFi"; // Enter your WiFi name
const char* password =  "support123"; // Enter WiFi password
//const char* mqttServer = "182.163.112.219";
//const char* mqttServer = "broker.hivemq.com";

const char* mqttServer = "broker.datasoft-bd.com";
const char* mqttUser = "iotdatasoft";
const char* mqttPassword = "brokeriot2o2o";

const int mqttPort = 1883;
const char* mqttTopic = "bridge";

#include "DHT.h"

uint8_t DHTPIN = D3; 
#define DHTTYPE DHT11 

DHT dht(DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient client(espClient);

unsigned long delayTime;


//vibration pin declaration
int LED_Pin = 13;
int vibr_Pin = D0;


//sonar sensor pin declaration
int trigPin = D8;
int echoPin = D5;

// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;

// Select SDA and SCL pins for I2C communication
const uint8_t scl = D6;
const uint8_t sda = D7;

// sensitivity scale factor respective to full scale setting provided in datasheet
const uint16_t AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;

// MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG  =  0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;

int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;


int minVal = 265;
int maxVal = 402;


int fsrPin = A0;     // the FSR and 10K pulldown are connected to a0
int fsrReading;     // the analog reading from the FSR resistor divider
int fsrVoltage;     // the analog reading converted to voltage
unsigned long fsrResistance;  // The voltage converted to resistance, can be very big so make "long"
unsigned long fsrConductance;
long fsrForce;



void setup() {
  Serial.begin(9600);

  WiFi.begin(ssid, password);
  //
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");

  client.setServer(mqttServer, mqttPort);
  ////  client.setCallback(callback);
  //
  //
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");

    if (client.connect("ESP8266Client5544", mqttUser, mqttPassword)) {
      delay(250);

      Serial.println("connected");

    } else {

      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);

    }
  }


  Serial.println("-- Default Test --");
  delayTime = 1000;

  Serial.println();


  
  // dht.begin();
  Wire.begin(sda, scl);
  Wire.beginTransmission(MPU6050SlaveAddress);
  Wire.write(0x6B);
  Wire.write(0);
  MPU6050_Init();
  //vibration pin
  pinMode(LED_Pin, OUTPUT);
  pinMode(vibr_Pin, INPUT);
  //ultarsonic pin
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(DHTPIN,INPUT);
  Serial.println("DHTxx test!");
 dht.begin();

}

void loop() {

  long measurement = TP_init();
  double vibra = measurement * 10;
  delay(50);
  Serial.print("Vibration : ");
  Serial.println(measurement);
  if (measurement > 1000) {
    digitalWrite(LED_Pin, HIGH);
  }
  else {
    digitalWrite(LED_Pin, LOW);
  }


  long duration, distance;
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;


  Serial.print( "distance: ");
  Serial.println( distance * 10);
  //Serial.println("cm");

  //delay(2000);

  //accelerometer && gyroscope

  double Ax, Ay, Az, T, Gx, Gy, Gz;

  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);

  //divide each with their sensitivity scale factor
  Ax = (double)AccelX / AccelScaleFactor;
  Ay = (double)AccelY / AccelScaleFactor;
  Az = (double)AccelZ / AccelScaleFactor;
  T = (double)Temperature / 340 + 36.53; //temperature formula
  Gx = (double)GyroX / GyroScaleFactor;
  Gy = (double)GyroY / GyroScaleFactor;
  Gz = (double)GyroZ / GyroScaleFactor;

  Serial.print("X: "); Serial.print(Ax);
  Serial.print("| Y: "); Serial.print(Ay);
  Serial.print("| Z: "); Serial.println(Az);
  Serial.print("| Temp: "); Serial.println(T);
  Serial.print("| G_X: "); Serial.print(Gx);
  Serial.print("| G_Y: "); Serial.print(Gy);
  Serial.print("| G_Z: "); Serial.println(Gz);

  //pressure code

  fsrReading = analogRead(fsrPin);
  Serial.print("pressure reading = ");
  Serial.println(fsrReading);

  fsrVoltage = map(fsrReading, 0, 1023, 0, 5000);
  Serial.print("Voltage reading in mV = ");
  Serial.println(fsrVoltage);

  if (fsrVoltage == 0) {
    Serial.println("No pressure");
  } else {
    // The voltage = Vcc * R / (R + FSR) where R = 10K and Vcc = 5V
    // so FSR = ((Vcc - V) * R) / V
    fsrResistance = 5000 - fsrVoltage;     // fsrVoltage is in millivolts so 5V = 5000mV
    fsrResistance *= 10000;                // 10K resistor
    fsrResistance /= fsrVoltage;
    Serial.print("FSR resistance in ohms = ");
    Serial.println(fsrResistance);

    fsrConductance = 1000000;
    fsrConductance /= fsrResistance;
    Serial.print("Conductance in microMhos: ");
    Serial.println(fsrConductance);

    if (fsrConductance <= 1000) {
      fsrForce = fsrConductance / 80;
      Serial.print("Force in Newtons: ");
      Serial.println(fsrForce);
    } else {
      fsrForce = fsrConductance - 1000;
      fsrForce /= 30;
      Serial.print("Force in Newtons: ");
      Serial.println(fsrForce);
    }
  }




   delay(2000); // Wait a few seconds between measurements
  int h = dht.readHumidity();
   // Reading temperature or humidity takes about 250 milliseconds!
   int t = dht.readTemperature();
   // Read temperature as Celsius (the default)
  int f = dht.readTemperature(true);
  
   if (isnan(h) || isnan(t) || isnan(f)) {
      Serial.println("Failed to read from DHT sensor!");
      return;
   }
 
   Serial.print ("Humidity: ");
   Serial.print (h);
   Serial.print (" %\t");
   Serial.print ("Temperature: ");
   Serial.print (t);
   Serial.print (" *C ");


  String msg2 = "";
 // String msg3 = "";
 // String msg4 = "";
  // String msg4 = "";

  //msg2 = msg2 + "{\"AX\":" + Ax + "," + "\"AY\":" + Ay + "," + "\"AZ\":" + Az + "," + "\"X\":" + Ax + "," + "\"Y\":" + Ay + "," + "\"Z\":" + Az + "}";
  //msg3 = msg3 + "{\"GX\":" + Gy + "," + "\"GY\":" + Gy + "," + "\"GZ\":" + Gz + "," + "\"V\":" + vibra + "," + "\"d\":" + distance + "," + "\"p\":" + fsrReading + "}";
 // msg4 = msg4 + "{\"AP\":" + Pressure + "," + "\"Temp\":" + Temperature + "," + "\"Alt\":" + Altitude + "," + "\"Humi\":" + Humidity +  "}";
  //  msg2 = msg2 + "{\"AngleX\":" + Ax + "}";


  //msg2 = msg2 + "{\"AX\":" + Ax + "," + "\"AY\":" + Ay + "," + "\"GX\":" + Gx + "," + "\"GY\":" + Gy + "," +  "\"V\":" + vibra + "," + "\"d\":" + distance + "," + "\"p\":" + fsrReading + "}";
  
   msg2 = msg2 + "{\"AX\":" + Ax + "," + "\"AY\":" + Ay + "," + "\"GX\":" + Gx + "," + "\"GY\":" + Gy + "," +  "\"V\":" + vibra + "," + "\"d\":" + distance + "," + "\"p\":" + fsrReading + "," + "\"temp\":" + t + "," + "\"humi\":" + h + "}";
  Serial.print(msg2);
 // Serial.print(msg3);
 // Serial.print(msg4);
  if (!client.connected()) {
    reconnect();
     Serial.println("client not connected");
  }
  else {
    client.publish(mqttTopic, msg2.c_str() );
  delay(300000);
//delay(8000);
   // client.publish(mqttTopic, msg3.c_str() );
    //client.publish(mqttTopic, msg4.c_str() );
  }



  Serial.println("-------------------");

  delay(3000);


}

//function for vibration
long TP_init() {
  delay(10);
  long measurement = pulseIn (vibr_Pin, HIGH);
  return measurement;
}



//function for mpu6050
void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

// read all 14 register
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read() << 8) | Wire.read());
  AccelY = (((int16_t)Wire.read() << 8) | Wire.read());
  AccelZ = (((int16_t)Wire.read() << 8) | Wire.read());
  Temperature = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroX = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroY = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroZ = (((int16_t)Wire.read() << 8) | Wire.read());
}

//configure MPU6050
void MPU6050_Init() {
  delay(150);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    //        Serial.print("Attempting MQTT connection...");
    // Attempt to connect, just a name to identify the client
    if (client.connect("ESP8266Client5544", mqttUser, mqttPassword)) {
      Serial.println("connected");


    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(2000);
    }
  }
}
