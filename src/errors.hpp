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

};

void print_errors(ErrorCodes code)
{
  switch (code) {

    case ErrorCodes::CURRENT_SENSE_OVER_LIMIT:

      Serial.println("Current sensor exceeds safe limit");
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


#endif
