/*   
HC05 - Bluetooth AT-Command mode  
modified on 10 Feb 2019 
by Saeed Hosseini 
https://electropeak.com/learn/ 
*/ 
#include <SoftwareSerial.h> 
SoftwareSerial MyBlue(10, 11); // RX | TX 
int flag = 0; 
int LED = LED_BUILTIN; 
void setup() 
{   
 Serial.begin(9600); 
 MyBlue.begin(9600); 
 pinMode(LED, OUTPUT); 
 Serial.println("Ready to connect\nDefualt password is 1234 or 000"); 
} 
void loop() 
{ 
 if (MyBlue.available()) 
   flag = MyBlue.read(); 
 if (flag == '1') 
 { 
   digitalWrite(LED, HIGH); 
   MyBlue.write('1');
 } 
 else
 { 
   digitalWrite(LED, LOW); 
 } 
}  
//#include <SoftwareSerial.h>
//
//SoftwareSerial mySerial(10, 11); // RX, TX
//
//void setup() {
//
//Serial.begin(9600);
//
//pinMode(9,OUTPUT); digitalWrite(9,HIGH);
//
//Serial.println("Enter AT commands:");
//
//mySerial.begin(38400);
//
//}
//
//void loop()
//
//{
//
//if (mySerial.available())
//
//Serial.write(mySerial.read());
//
//if (Serial.available())
//
//mySerial.write(Serial.read());
//
//}
