#ifndef ERRORS_HPP
#define ERRORS_HPP
#include <Arduino.h>


// Implement warnings?
enum WarningCodes{
    CURRENT_SENSE_SATURATION,
    DRIVER_VOLTAGE_EXCEEDS_USER_LIMIT,

};


enum ErrorCodes
{
  CURRENT_SENSE_OVER_LIMIT,
  DRIVER_INIT_FAIL,
  CURRENT_SENSE_INIT_FAIL,
  CURRENT_SENSE_ALIGN_FAIL,

};

void print_errors(ErrorCodes code)
{
  switch (code) {

    case ErrorCodes::CURRENT_SENSE_OVER_LIMIT:

      Serial.println("Current sensor exceeds safe limit");
      Serial.flush();
    case ErrorCodes::CURRENT_SENSE_INIT_FAIL:
      Serial.println("Current sensors failed to initialize");
      Serial.flush();
    case ErrorCodes::DRIVER_INIT_FAIL:
      Serial.println("Gate driver failed to initialize");
      Serial.flush();
    case ErrorCodes::CURRENT_SENSE_ALIGN_FAIL:
      Serial.println("Current sensors failed to align to phases");
      Serial.flush();
    
      return;
  }

}

void handle_errors(ErrorCodes code)
{
  print_errors(code);
  delay(10);
  exit(0);
}

void timer_errors(TeensyTimerTool::errorCode err)  // print out error code, stop everything and do a fast blink
{
  Serial.printf("Timer Error %d\n", err);
  Serial.flush();
  delay(1000);
  exit(0);
}



#endif
