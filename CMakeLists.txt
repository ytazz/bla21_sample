cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

project(bla21_sample)

if(WIN32)
	add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
endif()

add_executable(bla21_sample
  main.cpp
  bla21.h
  bla21.cpp
  crc16.h
  float16.h
  serial.h
  serial.cpp
)
