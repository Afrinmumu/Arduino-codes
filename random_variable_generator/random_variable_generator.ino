                        
                        
                                  //LINK:https://forum.arduino.cc/index.php?topic=286513.0

//                                                    RANDOM NUMBER GENERATOR

//
//long randNumber;
//
//void setup() {
//   Serial.begin(9600);
//   // if analog input pin 0 is unconnected, random analog
//   // noise will cause the call to randomSeed() to generate
//   // different seed numbers each time the sketch runs.
//   // randomSeed() will then shuffle the random function.
//   randomSeed(analogRead(0));
//}
//
//void loop() {
//   // print a random number from 0 to 299
//   Serial.print("random1=");
//   randNumber = random(300);
//   Serial.println(randNumber); // print a random number from 0to 299
//   Serial.print("random2=");
//   randNumber = random(10, 20);// print a random number from 10 to 19
//   Serial.println (randNumber);
//   delay(2000);
//}


//                                                    RANDOM STRING GENERATOR


const int len = 10;
char song[] = {'c', 'd','P', 'e', 'f','c', 'g', 'a', 'b', 'C','T','e','f','g','h','i','j','k','l','m','n','o'};
const byte songLength = sizeof(song) / sizeof(song[0]);
char notes[len+1];  //allow 1 extra for the NULL

void setup()
{
  Serial.begin(115200);
//  for (int n = 0; n < len ; n++)
//  {
//    notes[n] = song[random(0, songLength)];
//    notes[n + 1] = '\0';
//    Serial.println(notes);
//  }
}

void loop()
{
  for (int n = 0; n < len ; n++)
  {
    notes[n] = song[random(0, songLength)];
    notes[n + 1] = 'ESP';
    
    
  }
  Serial.println(notes);
  delay(3000);
}
