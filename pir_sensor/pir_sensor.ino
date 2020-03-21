int Balarm = 9;   //Buzzer  alarm  connected to GPIO-14 or D5 of nodemcu
int PIRsensor = 8; //PIR sensor output connected to GPIO-5 or D1 of nodemcu 

void setup() {
  Serial.begin(9600);
  pinMode(PIRsensor, INPUT); // PIR sensor as input  
  pinMode(Balarm, OUTPUT);   // Buzzer alaram as output
  digitalWrite (Balarm, LOW);// Initially buzzer off
}

void loop(){
  int state = digitalRead(PIRsensor); //Continuously check the state of PIR sensor
  delay(500);                         //Check state of PIR after every half second
  
    if(state == HIGH){                
      digitalWrite (Balarm, HIGH); 
      Serial.println("helllo");//If intrusion detected ring the buzzer
                       
                       //Ring buzzer for 15 seconds 
      //Serial.println("Motion detected!");
    }
    else {
      digitalWrite (Balarm, LOW); 
      Serial.println("bye");//No intrusion Buzzer off
      //Serial.println("Motion absent!");
      }
}
