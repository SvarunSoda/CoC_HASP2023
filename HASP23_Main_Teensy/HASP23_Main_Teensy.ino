//// HASP 2023 TESTING CODE - TEENSY ////

// INCLUDES //

#include "Arduino.h"
#include <SD.h>

//#include "TeensyThreads.h"

// GLOBAL VARIABLES //

#define SerialRP Serial2
#define SerialDownlink Serial6

const int PIN_MotorAEna = 29;
const int PIN_MotorAStep = 28;
const int PIN_MotorADir = 27;
const int PIN_MotorBEna = 32;
const int PIN_MotorBStep = 31;
const int PIN_MotorBDir = 30;
const int PIN_Switch1 = 37;
const int PIN_Switch2 = 35;
const int PIN_Switch3 = 34;
const int PIN_Switch4 = 33;
const int PIN_Therm1 = 19;
const int PIN_Therm2 = 18;
const int PIN_Therm3 = 17;
const int PIN_Therm4 = 16;
const int PIN_Therm5 = 15;
const int PIN_Therm6 = 14;
const int PIN_SD = BUILTIN_SDCARD;

const int SearchIncV = 5;
const int MotorAFastStepDelay = 5000;
const int MotorBFastStepDelay = 5000;
const int MotorASlowStepDelay = 10000;
const int MotorBSlowStepDelay = 10000;
const int MotorALimitBackupSteps = 50;
const int MotorBLimitBackupSteps = 50;
const int SquarePatternIters = 5;
const int SquarePatternMotorAIncSteps = 4;
const int SquarePatternMotorBIncSteps = 4;
const float SquarePatternMoveDelay = 0.1;
const int MotorMaxSteps = 999999999;
const int ThermResistance = 10000;
const int TrackFailTimeout = 10;
const int BaudRateRP = 115200;
const int BaudRateDownlink = 4800;
const int RPMessageNum = 4;
const int RPMessageBuffLen = 50;
const String DataFileNamePrefix = "HASP23_Data_";
const int DataFileMaxLines = 1000000;
const int DataBuffLen = 100;
const int SwitchReadNum = 20;
const float LightLostTimeout = 3;
const float TrackDelay = 0.1;
const float MonitorThreadDelay = 0.1;
const float LoopDelay = 1;
const float DataSaveDelay = 1;
const float DownlinkDelay = 5;

// DO NOT CHANGE //

int RPValuesBuff[RPMessageNum];
int TelescopeStatus = 0;
int CurrSearchDirH = 0;
bool NeedsHoming = true;
File CurrDataFile;
bool SDOpen = false;
int CurrDataFileLines = DataFileMaxLines;
int DataFileNum = 0;

uint32_t dataSaveTimer = 0;
uint32_t downlinkTimer = 0;
uint32_t LightTimer = 0;

// MAIN FUNCTIONS //

void setup() 
{
  Serial.begin(9600);
  SerialRP.begin(BaudRateRP);
  SerialDownlink.begin(BaudRateDownlink);

  pinMode(PIN_MotorAEna, OUTPUT);
  pinMode(PIN_MotorAStep, OUTPUT);
  pinMode(PIN_MotorADir, OUTPUT);
  pinMode(PIN_MotorBEna, OUTPUT);
  pinMode(PIN_MotorBStep, OUTPUT);
  pinMode(PIN_MotorBDir, OUTPUT);
  
  pinMode(PIN_Switch1, INPUT_PULLUP);
  pinMode(PIN_Switch2, INPUT_PULLUP);
  pinMode(PIN_Switch3, INPUT_PULLUP);
  pinMode(PIN_Switch4, INPUT_PULLUP);

  pinMode(PIN_Therm1, INPUT);
  pinMode(PIN_Therm2, INPUT);
  pinMode(PIN_Therm3, INPUT);
  pinMode(PIN_Therm4, INPUT);
  pinMode(PIN_Therm5, INPUT);
  pinMode(PIN_Therm6, INPUT);

  digitalWrite(PIN_MotorAEna, LOW);
  digitalWrite(PIN_MotorBEna, LOW);

  for (int i = 0; i < RPMessageNum; i++)
    RPValuesBuff[i] = -666666;

  //threads.addThread(HASP23_MonitorThread);
  //std::thread th1(HASP23_MonitorThread);
  //th1.detach();

  Serial.println("================================= Telescope initialized. ===================================");
}

void loop() 
{
  /*int data[RPMessageNum];
  
  if (HASP23_CheckIncomingData(data))
  {
    int motorADir = 0;
    int motorBDir = 0;

    if (data[0] > 0)
      motorADir = 1;
    if (data[1] > 0)
      motorBDir = 1;

    HASP23_RunMotorA(abs(data[0]), motorADir, MotorDelay);
    HASP23_RunMotorB(abs(data[1]), motorBDir, MotorDelay);
  }*/

  /*HASP23_RunMotorA(400, 0, 1200, false);
  HASP23_RunMotorA(400, 1, 1200, false);
  delay(2000);
  
  HASP23_RunMotorB(400, 0, 1200, false);
  HASP23_RunMotorB(400, 1, 1200, false);
  delay(2000);*/

  //Serial.println(HASP23_ReadSwitch(PIN_Switch1));
  //Serial.println(HASP23_ReadSwitch(PIN_Switch2));

  //int status = HASP23_RunMotorB(99999, 1, 2000, true, true);
  //Serial.println(String(status));

  //HASP23_TelescopeSearch();
  //HASP23_TelescopeHome();

  //Serial.print("1: " + String(HASP23_ReadSwitch(PIN_Switch1)));
  //Serial.print(", 2: " + String(HASP23_ReadSwitch(PIN_Switch2)));
  //Serial.print(", 3: " + String(HASP23_ReadSwitch(PIN_Switch3)));
  //Serial.println(", 4: " + String(HASP23_ReadSwitch(PIN_Switch4)));

  /*Serial.println("Temp 1: " + String(HASP23_ReadTemp(PIN_Therm1)) + " C.");
  Serial.println("Temp 2: " + String(HASP23_ReadTemp(PIN_Therm2)) + " C.");
  Serial.println("Temp 3: " + String(HASP23_ReadTemp(PIN_Therm3)) + " C.");
  Serial.println("Temp 4: " + String(HASP23_ReadTemp(PIN_Therm4)) + " C.");
  Serial.println("Temp 5: " + String(HASP23_ReadTemp(PIN_Therm5)) + " C.");
  Serial.println("Temp 6: " + String(HASP23_ReadTemp(PIN_Therm6)) + " C.");*/

  if (NeedsHoming)
  {
    Serial.println("Telescope homeing...");
    HASP23_TelescopeHome();
    Serial.println("Telescope homed.");

    NeedsHoming = false;
  }

  Serial.println("Telescope searching...");
  bool found = HASP23_TelescopeSearch();

  if (found)
  {
    Serial.println("Telescope found light (" + String(RPValuesBuff[0]) + ", " + String(RPValuesBuff[1]) + ")! Tracking...");
    HASP23_TelescopeTrack();
    Serial.println("Telescope lost light.");

    NeedsHoming = false;
  }
  else
  {
    Serial.println("Telescope didn't find anything.");

    NeedsHoming = true;
  }

  delay(LoopDelay * 1000);
}

// TELESCOPE FUNCTIONS //

void HASP23_TelescopeTrack()
{
  //Serial.println("HASP23_TelescopeTrack");

  TelescopeStatus = 3;
  
  // If we passed a light patch, we roll back using slower square pattern...
  bool aimed;

  do
  {
    delay(TrackDelay * 1000);

    Serial.println(String(RPValuesBuff[0]) + "::" + String(RPValuesBuff[1]));

    HASP23_CheckIncomingData();
    aimed = (RPValuesBuff[0] != -666666) && (RPValuesBuff[1] != -666666);

    if (aimed)
    {
      Serial.println("Staying at light...");
    }
    else
    {
      Serial.println("Lost light, running square pattern...");

      //int dirH = 0;
      //int dirV = 0;
      int stepsH = SquarePatternMotorBIncSteps;
      int stepsV = SquarePatternMotorAIncSteps;
      int dirH = 0;
      int dirV = 0;
      int statusV;
      int statusH;

      for (int i = 0; i < SquarePatternIters; i++)
      {
        delay(SquarePatternMoveDelay * 1000);
        statusV = HASP23_RunMotorA(stepsV, dirV, MotorASlowStepDelay, true, true);

        if (statusV == 2)
        {
          aimed = true;

          break;
        }

        stepsV += SquarePatternMotorAIncSteps;
        stepsH += SquarePatternMotorBIncSteps;

        delay(SquarePatternMoveDelay * 1000);
        statusH = HASP23_RunMotorB(stepsH, dirH, MotorBSlowStepDelay, true, true);

        if (statusH == 2)
        {
          aimed = true;

          break;
        }

        stepsV += SquarePatternMotorAIncSteps;
        stepsH += SquarePatternMotorBIncSteps;
        dirV = !dirV;
        dirH = !dirH;
      }
    }
  }
  while (aimed);

  TelescopeStatus = 0;
}

bool HASP23_TelescopeSearch()
{
  //Serial.println("HASP23_TelescopeSearch");

  TelescopeStatus = 2;

  // Running wide search...
  bool found = false;

  do
  {
    if (HASP23_RunMotorB(MotorMaxSteps, CurrSearchDirH, MotorBFastStepDelay, true, true) == 2)
    {
      found = true;

      break;
    }
  
    CurrSearchDirH = !CurrSearchDirH;
  } 
  while (HASP23_RunMotorA(SearchIncV, 0, MotorAFastStepDelay, true, false) == 0);

  TelescopeStatus = 0;

  return found;
}

void HASP23_TelescopeHome()
{
  //Serial.println("HASP23_TelescopeHome");

  TelescopeStatus = 1;

  HASP23_RunMotorB(MotorMaxSteps, 1, MotorBFastStepDelay, true, false);
  HASP23_RunMotorA(MotorMaxSteps, 1, MotorAFastStepDelay, true, false);
  //HASP23_RunMotorA(100, 0, MotorAFastStepDelay, true, false);

  CurrSearchDirH = 0;
  TelescopeStatus = 0;
}

// THREAD FUNCTIONS //

void HASP23_MonitorThread()
{
    uint32_t startTime = millis();

    HASP23_CheckIncomingData();

    if ((dataSaveTimer / 1000) >= DataSaveDelay)
    {
      //HASP23_SaveDataSD();
      dataSaveTimer = 0;
    }
    if ((downlinkTimer / 1000) >= DownlinkDelay)
    {
      HASP23_SendDownlink();
      downlinkTimer = 0;
    }

    delay(MonitorThreadDelay * 1000);

    dataSaveTimer += millis() - startTime;
    downlinkTimer += millis() - startTime;
}

// MOTOR FUNCTIONS //

int HASP23_RunMotorA(int steps, int dir, int delay, bool limitInterrupt, bool targetInterrupt)
{
  //Serial.println("HASP23_RunMotorA");

  digitalWrite(PIN_MotorADir, dir);

  for (int i = 0; i < steps; i++)
  {
    HASP23_CheckIncomingData();

    if (limitInterrupt && (((dir == 0) && HASP23_ReadSwitch(PIN_Switch4)) || ((dir == 1) && HASP23_ReadSwitch(PIN_Switch3))))
    {
      HASP23_RunMotorA(MotorALimitBackupSteps, !dir, MotorAFastStepDelay, false, false);

      return 1;
    }
    if (targetInterrupt && ((RPValuesBuff[0] != -666666) && (RPValuesBuff[1] != -666666)))
      return 2;

    digitalWrite(PIN_MotorAStep, HIGH);
    delayMicroseconds(delay);
    digitalWrite(PIN_MotorAStep, LOW);
    delayMicroseconds(delay);
  }

  return 0;
}

int HASP23_RunMotorB(int steps, int dir, int delay, bool limitInterrupt, bool targetInterrupt)
{
  //Serial.println("HASP23_RunMotorB");

  digitalWrite(PIN_MotorBDir, dir);

  for (int i = 0; i < steps; i++)
  {
    HASP23_CheckIncomingData();

    if (limitInterrupt && (((dir == 0) && HASP23_ReadSwitch(PIN_Switch1)) || ((dir == 1) && HASP23_ReadSwitch(PIN_Switch2))))
    {
      HASP23_RunMotorB(MotorBLimitBackupSteps, !dir, MotorBFastStepDelay, false, false);

      return 1;
    }
    if (targetInterrupt && ((RPValuesBuff[0] != -666666) && (RPValuesBuff[1] != -666666)))
      return 2;

    digitalWrite(PIN_MotorBStep, HIGH);
    delayMicroseconds(delay);
    digitalWrite(PIN_MotorBStep, LOW);
    delayMicroseconds(delay);
  }

  return 0;
}

// SWITCH FUNCTIONS //

bool HASP23_ReadSwitch(int pin)
{
  //Serial.println("HASP23_ReadSwitch");

  int sum = 0;

  for (int i = 0; i < SwitchReadNum; i++)
    sum += !digitalRead(pin);

  return round(sum / SwitchReadNum);
}

// TEMPERATURE FUNCTIONS //

float HASP23_ReadTemp(int pin)
{
  int Vo;
  float R1 = ThermResistance;
  float logR2, R2, T;
  float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

  Vo = analogRead(pin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  T = T - 273.15;
  //T = (T * 9.0)/ 5.0 + 32.0; 

  return T;
}

// SD FUNCTIONS //

void HASP23_SaveDataSD()
{
  bool firstSave = false;
  char buff[DataBuffLen];

  if (!SDOpen)
  {
    firstSave = true;

    if (SD.begin(PIN_SD))
    {
      SDOpen = true;
      HASP23_GetDataFileNumSD();
    }
    else
    {
      Serial.println("ERROR: Failed to open SD!");

      return;
    }
  }

  if (CurrDataFileLines >= DataFileMaxLines)
  {
    if (!firstSave)
      CurrDataFile.close();

    String fileName = DataFileNamePrefix + String(++DataFileNum) + ".txt";
    int nameLen = fileName.length() + 1;
    fileName.toCharArray(buff, nameLen);
    buff[nameLen - 1] = '\0';

    CurrDataFile = SD.open(buff, FILE_WRITE);

    CurrDataFile.write("-------- HASP 2023 - College of the Canyons - DataFile #");
    String fileNumStr = String(DataFileNum);
    fileNumStr.toCharArray(buff, fileNumStr.length() + 1);
    CurrDataFile.write(buff);
    CurrDataFile.write(" --------\n---- () ----\n\n");

    CurrDataFileLines = 0;
  }

  CurrDataFile.write("DATA\n");
  CurrDataFileLines++;
}

int HASP23_GetDataFileNumSD()
{
  int maxNum = 0;
  File root = SD.open("/");

  while (true) 
  {
    File entry = root.openNextFile();

    if (!entry)
      break;

    if (!entry.isDirectory()) 
    {
      
    }

    entry.close();
  }

  return maxNum;
}

// DOWNLINK FUNCTIONS //

void HASP23_SendDownlink()
{
  //Serial.println("downlink sent!");
}

// SERIAL FUNCTIONS //

bool HASP23_CheckIncomingData()
{
  //Serial.println("HASP23_CheckIncomingData");

  Serial.println("a");

  if (SerialRP.available() > 0)
  {
    Serial.println("b");

    char messageBuff[RPMessageBuffLen + 1];
    int i = 0;

    for (i = 0; i < RPMessageBuffLen; i++)
    {
      char recievedChar = SerialRP.read();

      if (recievedChar == ';')
        break;

      messageBuff[i] = recievedChar;
    }

    Serial.println("c");

    messageBuff[i + 1] = '\0';
    String message = String(messageBuff);

    //Serial.println("RP message recieved: " + message);
    
    int tempData[RPMessageNum];
    String wordBuff = "";
    int parsed = 0;
    bool startOK = false;
    bool endOK = false;

    for (int i = 0; i < message.length(); i++)
    {
      char currChar = message[i];

      if ((currChar == ' ') || (currChar == '\t'))
      {
        continue;
      }
      else if (currChar == '(')
      {
        if (startOK)
        {
          startOK = false;

          break;
        }

        startOK = true;

        continue;
      }
      else if ((currChar == ',') || (currChar == ')'))
      {
        tempData[parsed++] = wordBuff.toInt();
        wordBuff = "";

        if (currChar == ')')
          endOK = true;

        continue;
      }

      wordBuff += currChar;
    }

    Serial.println("d");

    if (startOK && endOK && (parsed == RPMessageNum))
    {
      for (int i = 0; i < RPMessageNum; i++)
        RPValuesBuff[i] = tempData[i];

      return true;
    }
  }

  //Serial.println("No RP message recieved!");

  //for (int i = 0; i < RPMessageNum; i++)
  //  RPValuesBuff[i] = -666666;

  return false;
}
