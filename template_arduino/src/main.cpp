#include <Arduino.h>

#include "SEGGER_RTT.h"

void log(const char* fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    SEGGER_RTT_vprintf(0,fmt,&va);
    va_end(va);
}

void setup()
{
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(USER_BTN, INPUT);

  Serial.begin(115200);
}

void loop()
{
  // put your main code here, to run repeatedly:
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  
  delay(100);

  Serial.println("hello MonkeyPi-STM32");

  log("lot print with segger rtt\r\n");

  if (digitalRead(USER_BTN) == 0)
  {
    Serial.println("btn clicked");
  }
}