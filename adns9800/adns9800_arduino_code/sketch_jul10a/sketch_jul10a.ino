#include <SPI.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode (ncs, OUTPUT);
  SPI.begin();
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));

  digitalWrite(SS,HIGH);
  digitalWrite(SS,LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  

}
