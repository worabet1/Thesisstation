/*
  HC05 - Bluetooth AT-Command mode
  modified on 10 Feb 2019
  by Saeed Hosseini
  https://electropeak.com/learn/
*/
#include <SoftwareSerial.h>
#include <String.h>
SoftwareSerial MyBlue(10, 11); // RX | TX
int flag = 0;
int LED = LED_BUILTIN;
int voltage = 0;
char str[8];
char str2[8];
void setup()
{
  Serial.begin(9600);
  MyBlue.begin(9600);
  pinMode(LED, OUTPUT);
  Serial.println("Ready to connect\nDefualt password is 1234 or 000");
}
void loop()
{
  if (MyBlue.available()) {
    //    Serial.write(MyBlue.read());
    flag = MyBlue.read();
    Serial.println(flag);
    if (flag == '1')
    {
      Serial.println("Detected");
      MyBlue.write('1');

    }
    else if (flag == '2')
    {
      voltage = analogRead(A0) * 22.5 * 3.3 / 2.23 / 1024.0 * 1000;
      //Serial.println(voltage);
      sprintf(str, "");
      sprintf(str, "%d", voltage);
      sprintf(str2, "");
      Serial.print("strbeforeeverything = ");
      Serial.println(str2);
      Serial.print("strvoltage = ");
      Serial.println(str);
      for (int i = 0; i < 8 - strlen(str); i++) {
        sprintf(str2, "%s0", str2);
      }
      Serial.print("strafterfill 0 = ");
      Serial.println(str2);
      sprintf(str2, "%s%s", str2, str);
      Serial.print("strafterfill 0 and after = ");
      Serial.println(str2);
      Serial.println(strlen(str2));
      MyBlue.write(str2);
    }
  }
}
//#include <SoftwareSerial.h>
//
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
