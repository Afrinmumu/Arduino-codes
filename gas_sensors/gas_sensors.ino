
#include <Wire.h>
#include <SPI.h>

#include <ESP8266WiFi.h>
#include <PubSubClient.h>


#define anInput     A0                        //analog feed from MQ135
#define digTrigger   D2                        //digital feed from MQ135
#define co2Zero     55                        //calibrated CO2 0 level


////co///////

float RS_gas = 0;
float ratio = 0;
float sensorValue = 0;
float sensor_volt = 0;
float R0 = 21813.95;
 
#define MQ135 A0


//Gas Sensor Load Resistance (RL)

#define RL_MQ135 10

float A_MQ135_NOx = 34.69756084;  
float B_MQ135_NOx = -3.422829698;


WiFiClient espClient;
PubSubClient client(espClient);
float t=0;
char data = 0;
//const char* ssid = "DataSoft_WiFi"; // Enter your WiFi name
//const char* password =  "support123";
const char* ssid = "sad"; // Enter your WiFi name
const char* password =  "qqqq1234";
// Enter WiFi password
//const char* mqttServer = "182.163.112.219";
const char* mqttServer = "broker.hivemq.com";
const int mqttPort = 1883;
const char* mqttTopic = "hack";



void setup() {
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

    if (client.connect("ESP8266Client")) {

      Serial.println("connected");

    } else {

      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);

    }
  }
  pinMode(anInput,INPUT);                     //MQ135 analog feed set for input
  pinMode(digTrigger,INPUT);                  //MQ135 digital feed set for input
 // pinMode(led,OUTPUT);                        //led set for output
  Serial.begin(9600);                         //serial comms for debuging
  //display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  //begin display @ hex addy 0x3C
  //display.display();                          //show buffer
  //display.clearDisplay();                     //clear buffer
  
}
//---------------------------------------------------------------------------------------------------------------
//                                               MAIN LOOP
//---------------------------------------------------------------------------------------------------------------
void loop() {
  
int co2now[10];                               //int array for co2 readings
int co2raw = 0;                               //int for raw value of co2
int co2comp = 0;                              //int for compensated co2 
int co2ppm = 0;                               //int for calculated ppm
int zzz = 0;                                  //int for averaging
int grafX = 0;                                //int for x value of graph


  //display.clearDisplay();                     //clear display @ beginning of each loop

  for (int x = 0;x<10;x++){                   //samplpe co2 10x over 2 seconds
    co2now[x]=analogRead(A0);
    delay(200);
  }

for (int x = 0;x<10;x++){                     //add samples together
    zzz=zzz + co2now[x];
    
  }
  co2raw = zzz/10;                            //divide samples by 10
  co2comp = co2raw - co2Zero;                 //get compensated value
  co2ppm = map(co2comp,0,1023,400,5000);      //map value for atmospheric levels

  Serial.print("co2 in ppm");
  Serial.println( co2ppm);


   t = analogRead(A0);  // Read sensor value and stores in a variable t

  Serial.print("Airquality = ");

  Serial.println(t);

  if (t<=500)
   {
   //lcd.print("Fresh Air");
   Serial.print("Fresh Air ");
   
}
  else if( t>=500 && t<=1000 )
{
   //lcd.print("Poor Air");
   Serial.print("Poor Air");
  
}
  else if (t>=1000 )
{
  //lcd.print("Very Poor");
  Serial.print("Very Poor");
  
}


 float VRL_MQ135; 
  float Rs_MQ135; 
  float Ro_MQ135 = 20.1;
  float ratio_MQ135;

  VRL_MQ135 = analogRead(MQ135)*(5.0/1023.0); 
  Rs_MQ135 = ((5.0/VRL_MQ135)-1)*(RL_MQ135); 
  ratio_MQ135 = Rs_MQ135/Ro_MQ135;
  
  

  float ppm_NOx = A_MQ135_NOx * pow(ratio_MQ135, B_MQ135_NOx);


Serial.print("nox ");
Serial.println(ppm_NOx );


/////co /////

sensorValue = analogRead(A0);
   sensor_volt = sensorValue/1024*5.0;
   RS_gas = (5.0-sensor_volt)/sensor_volt;
   ratio = RS_gas/R0; //Replace R0 with the value found using the sketch above
   float x = 1538.46 * ratio;
   float co = pow(x,-1.709);
   Serial.print("PPM: ");
   Serial.println(co);
   delay(1000);
 String msg2 = "";
  


 msg2 = msg2 + "{\"co2\":" + co2ppm + "," + "\"AQ\":" + t + "," + "\"nox\":" + ppm_NOx  + "," + "\"CO\":" + co  + "}";
  Serial.print(msg2);
 // Serial.print(msg3);
 // Serial.print(msg4);
  if (!client.connected()) {
    reconnect();
  }
  else {
    client.publish(mqttTopic, msg2.c_str() );
    delay(10000);
   // client.publish(mqttTopic, msg3.c_str() );
    //client.publish(mqttTopic, msg4.c_str() );
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    //        Serial.print("Attempting MQTT connection...");
    // Attempt to connect, just a name to identify the client
    if (client.connect("lens_TOgJyxLGLoiKVp4qmr7BzWuq7uq")) {
      Serial.println("connected");


    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(50);
    }
  }
}
