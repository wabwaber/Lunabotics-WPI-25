/*
 * encoder.cpp
 * 
 *  Created on: Dec 11, 2023
 *      Author: Speeep
 */

#include "rawEncoder.h"
#include <Wire.h>

RawEncoder::RawEncoder(){}

void RawEncoder::init(int id, int multiplexerId) {
    encoderNumber = id;
    multiplexerNumber = multiplexerId;
    lastRawAngle = 0;
}

int RawEncoder::getRawAngle()
{
    int maxTries = 5; // Maximum number of tries before giving up
    int attempts = 0;

    while (attempts < maxTries) {
        // Multiplexer things
        if (multiplexerNumber == 0) {
            Wire.beginTransmission(0x70);
            Wire.write(1 << encoderNumber);
            if (Wire.endTransmission() != 0) {
                attempts++;
                continue; // Try again
            }

            Wire.beginTransmission(0x71);
            Wire.write(1 << 7);
            if (Wire.endTransmission() != 0) {
                attempts++;
                continue; // Try again
            }
        } else {
            Wire.beginTransmission(0x71);
            Wire.write(1 << encoderNumber);
            if (Wire.endTransmission() != 0) {
                attempts++;
                continue; // Try again
            }

            Wire.beginTransmission(0x70);
            Wire.write(1 << 7);
            if (Wire.endTransmission() != 0) {
                attempts++;
                continue; // Try again
            }
        }

        // Encoder things
        // 7:0 - low
        Wire.beginTransmission(0x36);
        Wire.write(0x0D);
        if (Wire.endTransmission() != 0) {
            attempts++;
            continue; // Try again
        }

        Wire.requestFrom(0x36, 1);
        if (Wire.available() == 0) {
            attempts++;
            continue; // Try again
        }
        int lowbyte = Wire.read();

        // 11:8 - high
        Wire.beginTransmission(0x36);
        Wire.write(0x0C);
        if (Wire.endTransmission() != 0) {
            attempts++;
            continue; // Try again
        }

        Wire.requestFrom(0x36, 1);
        if (Wire.available() == 0) {
            attempts++;
            continue; // Try again
        }
        int highbyte = Wire.read();
        highbyte <<= 8;

        int rawAngle = (highbyte | lowbyte) & 0x0FFF;
        if (rawAngle == 0) {
            attempts++;
            continue; // Try again
        } else {
            lastRawAngle = rawAngle;
            return rawAngle;
        }
    }

    // If all tries fail, return the last known good angle
    return lastRawAngle;
}
