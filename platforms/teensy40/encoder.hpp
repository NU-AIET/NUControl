#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>
#include <imxrt.h>
#include <SPI.h>
#include <wiring.h>
#include "encoder_interface.hpp"

// AbsoluteEncoder is a Teensy-side alias retained for compatibility.
// New code should use IAbsoluteEncoder.
using AbsoluteEncoder = IAbsoluteEncoder;

#endif
