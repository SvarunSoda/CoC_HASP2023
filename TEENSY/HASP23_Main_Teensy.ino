//// HASP 2023 TESTING CODE - TEENSY ////

// INCLUDES //

#include "Arduino.h"

// GLOBAL VARIABLES //

#define Serial_RP Serial1
//#define Serial_Downlink Serial2

const int PIN_MotorAStep = 3;
const int PIN_MotorADir = 2;
const int PIN_MotorBStep = 5;
const int PIN_MotorBDir = 4;

const int RPMessageNum = 3;
const int BaudRate = 9600;
const int MotorDelay = 1000;

// FORWARD DECLARATIONS //

void HASP23_RunMotorA(int steps, int dir);
void HASP23_RunMotorB(int steps, int dir);
bool HASP23_CheckIncomingData(int data[]);
int HASP23_CountInString(String str, char c);

// MAIN FUNCTIONS //

void setup() 
{
  Serial.begin(BaudRate);
  Serial_RP.begin(BaudRate);

  pinMode(PIN_MotorAStep, OUTPUT);
  pinMode(PIN_MotorADir, OUTPUT);
  pinMode(PIN_MotorBStep, OUTPUT);
  pinMode(PIN_MotorBDir, OUTPUT);

  Serial.println("Initialized!");
}

void loop() 
{
  int data[RPMessageNum];
  
  if (HASP23_CheckIncomingData(data))
  {
    int motorADir = 0;
    int motorBDir = 0;

    if (data[0] > 0)
      motorADir = 1;
    if (data[1] > 0)
      motorBDir = 1;

    HASP23_RunMotorA(abs(data[0]), motorADir);
    HASP23_RunMotorB(abs(data[1]), motorBDir);
  }
}

// MOTOR FUNCTIONS //

void HASP23_RunMotorA(int steps, int dir)
{
  digitalWrite(PIN_MotorADir, dir);

  for (int i = 0; i < steps; i++)
  {
    digitalWrite(PIN_MotorAStep, HIGH);
    delayMicroseconds(MotorDelay);
    digitalWrite(PIN_MotorAStep, LOW);
    delayMicroseconds(MotorDelay);
  }
}

void HASP23_RunMotorB(int steps, int dir)
{
  digitalWrite(PIN_MotorBDir, dir);

  for (int i = 0; i < steps; i++)
  {
    digitalWrite(PIN_MotorBStep, HIGH);
    delayMicroseconds(MotorDelay);
    digitalWrite(PIN_MotorBStep, LOW);
    delayMicroseconds(MotorDelay);
  }
}

// SERIAL FUNCTIONS //

bool HASP23_CheckIncomingData(int data[])
{
  if (Serial_RP.available() > 0)
  {
    String message = Serial_RP.readString();

    if ((message.indexOf("(") == -1) || (message.indexOf(")") == -1) || (HASP23_CountInString(message, ',') != (RPMessageNum - 1)))
    {
      Serial.println("ERROR: Invalid recieved RP message: \"" + message + "\"!");

      return false;
    }

    String wordBuff = "";
    int valuesRead = 0;

    for (int i = 0; i < message.length(); i++)
    {
      char currChar = message[i];

      if ((currChar == ' ') || (currChar == '\t') || (currChar == '('))
      {
        continue;
      }
      else if ((currChar == ',') || (currChar == ')'))
      {
        data[valuesRead] = wordBuff.toInt();
        wordBuff = "";
        valuesRead++;

        continue;
      }

      wordBuff += currChar;
    }

    Serial.print("Successfully parsed RP message: [");
    
    for (int i = 0; i < RPMessageNum; i++)
    {
      Serial.print(String(data[i]));

      if (i != (RPMessageNum - 1))
        Serial.print(", ");
    }

    Serial.println("].");

    return true;
  }

  return false;
}

// SUPPORT FUNCTIONS //

int HASP23_CountInString(String str, char c)
{
  int count = 0;

  for (int i = 0; i < str.length(); i++)
    if (str[i] == c)
      count++;

  return count;
}
