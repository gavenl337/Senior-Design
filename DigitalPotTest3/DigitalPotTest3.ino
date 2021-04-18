#include <SPI.h>
//#include <String.h>
int analogPin = A0;
byte address = 0x00;
int CS= 10;  //pin number for clock
int val = 0;  // variable to store the value read
int i = 128; //starting value of 540
int target = 256; //target analog bit value
String dataString = "";
int tNum = 1;

void setup(){
  Serial.begin(9600);           //  setup serial
  pinMode (CS, OUTPUT);
  SPI.begin();
  digitalPotWrite(i);
  val = analogRead(analogPin);
}
void loop(){
  for(int measurement = 0; measurement <= 10; measurement++){
  val = analogRead(analogPin);
  i = targetCheck(val, target, i);
  dataString = createCSV(val,val,val,tNum);
  if (i > 128){
    i = 128;
  }
  if (i < 0){
    i = 0;
  }
  
  displayResults(measurement, i, val);
  digitalPotWrite(i);
  delay(1000);
  }
  
  
  Serial.println(dataString);
  
  Serial.print("Test ");
  Serial.print(tNum);
  Serial.println(" Concluded");
  tNum++;
  delay(3000);
}
