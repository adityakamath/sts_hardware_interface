/**
 * @file SCServo.h
 * @brief Master include file for Feetech Serial Servo SDK (SMS/STS series only)
 *
 * @details This is the main header file for the SMS/STS servo interface.
 * Include this single file to access the SMS_STS class.
 *
 * **Supported Servo Series:**
 * - SMS_STS: SMS and STS series (3 operating modes: servo, wheel closed-loop, wheel open-loop)
 *
 * **Usage:**
 * @code
 * #include "SCServo.h"
 *
 * SMS_STS servo;
 * servo.begin(1000000, "/dev/ttyUSB0");
 * servo.InitMotor(1, 0, 1);  // ID 1, servo mode, enable torque
 * servo.WritePosEx(1, 2048, 1000, 50);  // Move to position 2048
 * @endcode
 *
 * @note This file only includes headers; link against libSCServo.a for implementations
 * @see SMS_STS.h for protocol documentation
 */

#ifndef _SCSERVO_H
#define _SCSERVO_H

#include "SMS_STS.h"
#endif
