
#include "HX711.h"

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3F, 16, 2);

HX711 scale(D5, D6);

int rbutton = D4; // this button will be used to reset the scale to 0.
float  weight;
float calibration_factor = 855; // for me this vlaue works just perfect 419640

void setup() 
{
  Serial.begin(115200);
  pinMode(rbutton, INPUT_PULLUP); 
  scale.set_scale();
  scale.tare(); //Reset the scale to 0
  long zero_factor = scale.read_average(); //Get a baseline reading

  Wire.begin(D2, D1);
  lcd.begin(16,2);
  lcd.init();
  lcd.backlight();

  lcd.setCursor(6,0);
  lcd.print("IOT");
  lcd.setCursor(1,1);
  lcd.print("Weighing Scale");
  delay(3000);
  lcd.clear();

  {
  delay(1000);
  Serial.print(".");
  lcd.clear();
  }
  Serial.println("");

  lcd.clear();
  
  
  delay(2000);
}

void loop() 

{

  
  scale.set_scale(calibration_factor); //Adjust to this calibration factor

  weight = scale.get_units(5); 
if(weight >=0){
  lcd.setCursor(0, 0);
  lcd.print("Measured Weight");
  lcd.setCursor(0, 1);
  lcd.print(weight);
  lcd.print(" G  ");
  
  delay(2000);
  lcd.clear();
}
  Serial.print("Weight: ");
  Serial.print(weight);
  Serial.println(" G");
  Serial.println();
  

  if ( digitalRead(rbutton) == LOW)
{
  scale.set_scale();
  scale.tare(); //Reset the scale to 0
}

}
    
 
