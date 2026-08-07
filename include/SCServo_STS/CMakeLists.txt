cmake_minimum_required(VERSION 3.14)

project(SCServo
  VERSION 1.0.0
  LANGUAGES CXX
  DESCRIPTION "Feetech SMS/STS serial bus servo SDK for Linux")

set(SCSERVO_HEADERS
  include/scservo/INST.h
  include/scservo/SCS.h
  include/scservo/SCSerial.h
  include/scservo/SCServo.h
  include/scservo/SMS_STS.h
  include/scservo/ServoErrors.h
  include/scservo/ServoUtils.h
  include/scservo/SyncWriteBuffer.h
)

set(SCSERVO_SOURCES
  src/SMS_STS.cpp
  src/SCSerial.cpp
  src/SCS.cpp
)

add_library(SCServo STATIC ${SCSERVO_SOURCES} ${SCSERVO_HEADERS})

target_include_directories(SCServo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_include_directories(SCServo PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/scservo>
)

target_compile_features(SCServo PRIVATE cxx_std_17)

set_target_properties(SCServo PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)
