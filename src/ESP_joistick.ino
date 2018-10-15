/*************************************************************
  Download latest Blynk library here:
    https://github.com/blynkkk/blynk-library/releases/latest

  Blynk is a platform with iOS and Android apps to control
  Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build graphic interfaces for all your
  projects by simply dragging and dropping widgets.

    Downloads, docs, tutorials: http://www.blynk.cc
    Sketch generator:           http://examples.blynk.cc
    Blynk community:            http://community.blynk.cc
    Follow us:                  http://www.fb.com/blynkapp
                                http://twitter.com/blynk_app

  Blynk library is licensed under MIT license
  This example code is in public domain.

 *************************************************************

  You can receive x and y coords for joystick movement within App.

  App project setup:
    Two Axis Joystick on V1 in MERGE output mode.
    MERGE mode means device will receive both x and y within 1 message
 *************************************************************/

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial


#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).


// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Lutsman_m";
char pass[] = "buratino";
char auth[] = "95e410e7a7f04af197e3822d585c99e3";
char server[] = "blynk-cloud.com";
int port = 80;

BLYNK_WRITE(V1) {
  int x = param[0].asInt();
  int y = param[1].asInt();

  // directions=x,y;
  Serial.print("directions=");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.println(";");
}

BLYNK_WRITE(V2) {
  int speedLvl = param.asInt();
  
  Serial.print("speed=");
  Serial.print(speedLvl);
  Serial.println(";");
}

BLYNK_WRITE(V3) {
  int speedToggler = param.asInt();

  Serial.print("speedToggler=");
  Serial.print(speedToggler);
  Serial.println(";");
}

void setup()
{
  // Debug console
  Serial.begin(9600);

  Blynk.begin(auth, ssid, pass, server, port);
  // You can also specify server:
  //Blynk.begin(auth, ssid, pass, "blynk-cloud.com", 80);
  //Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);
  Blynk.syncVirtual(V1);
  Blynk.syncVirtual(V2);
  Blynk.syncVirtual(V3);
}

void loop()
{
  Blynk.run();
}
