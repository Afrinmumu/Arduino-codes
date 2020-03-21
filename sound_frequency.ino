#include <FreqMeasure.h>

void setup() {
  Serial.begin(9600);
  FreqMeasure.begin(); //Measures on pin 8 by default 
 
}

double sum=0;
int count=0;
bool state = false; 
float frequency;
int continuity =0;

void loop() {
  if (FreqMeasure.available()) {
    // average several reading together
    sum = sum + FreqMeasure.read();
    count = count + 1;
    if (count > 100) {
      frequency = FreqMeasure.countToFrequency(sum / count);
      Serial.println(frequency); 
      sum = 0;
      count = 0;
    }
  }

}
