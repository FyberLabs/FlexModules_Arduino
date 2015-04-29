#include <Wire.h>
#include <FyberLabs_PCA9548.h>

uint8_t cur;
FyberLabs_PCA9548 i2cswitch = FyberLabs_PCA9548();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
    while (!Serial);
  Serial.println("PCA9548 switch test");
  i2cswitch.begin();
  
  pinMode(13, OUTPUT);
}

void loop() {
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);              // wait for a second
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);              // wait for a second
  
  for (uint8_t i=1; i<9; i += 1){
    i2cswitch.onSwitchChannel(i);
    cur = __builtin_ffs(i2cswitch.readSwitchChannel());
    Serial.print("Current switch channel is ");
    Serial.print(cur);
    Serial.println(".");
    i2cswitch.offSwitchChannel(i);    
  }
}
