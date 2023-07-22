//// HASP 2023 TESTING CODE - TEENSY ////

// INCLUDES //

#include "Arduino.h"
#include <SD.h>

#include "TeensyThreads.h"

// GLOBAL VARIABLES //

#define SerialRP Serial2
#define SerialDownlink Serial6

const int PIN_MotorAEna = 29;
const int PIN_MotorAStep = 28;
const int PIN_MotorADir = 27;
const int PIN_MotorBEna = 32;
const int PIN_MotorBStep = 31;
const int PIN_MotorBDir = 30;
const int PIN_Switch1 = 36;
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

const int SearchIncV = 20;
const int MotorAFastStepDelay = 1500;
const int MotorBFastStepDelay = 1500;
const int MotorASlowStepDelay = 7000;
const int MotorBSlowStepDelay = 7000;
const int MotorALimitBackupSteps = 50;
const int MotorBLimitBackupSteps = 50;
const int MaxImageRes[2] = {3096, 2080};
const int MotorATrackSteps = 4;
const int MotorBTrackSteps = 4;
const int MotorBTrackRollbackSteps = 2;
const int SquarePatternIters = 5;
const int SquarePatternMotorAIncSteps = 4;
const int SquarePatternMotorBIncSteps = 4;
const float SquarePatternMoveDelay = 0.1;
const int TrackFailTimeout = 10;
const int MotorMaxSteps = 999999999;
const int ThermResistance = 10000;
const int BaudRateRP = 115200;
const int BaudRateDownlink = 4800;
const int RPMessageNum = 4;
const int RPMessageBuffLen = 50;
const String DataFileNamePrefix = "HASP23_Data_";
const int DataFileMaxLines = 100000;
const int DataBuffLen = 200;
const int SwitchReadNum = 20;
const int LightLostTimeout = 3;
const float TrackDelay = 0.1;
const float LoopDelay = 1;
const float DownlinkDelay = 300;
const float SerialThreadDelay = 0.1;
const float DownlinkThreadDelay = 0.1;

// DO NOT CHANGE //

int RPValuesBuff[RPMessageNum];
int TelescopeStatus = 0;
int CurrSearchDirH = 0;
int MissedLightMsgs = 0;
bool NeedsHoming = true;
File CurrDataFile;
bool SDOpen = false;
bool SDFirstSave = false;
int CurrDataFileLines = DataFileMaxLines;
int DataFileNum = 0;
bool RestartRequired = false;
int ResetNum = 0;

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

  if (!SD.begin(PIN_SD))
  {
    Serial.println("ERROR: Unable to open SD card!");
  }
  else
  {
    SDOpen = true;
    HASP23_GetDataFileNumSD();
  }

  threads.addThread(HASP23_SerialThread);
  threads.addThread(HASP23_DownlinkThread);

  Serial.println("Telescope initialized.");
}

void loop() 
{
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
  int statusV;
  int statusH;

  do
  {
    delay(TrackDelay * 1000);

    //Serial.println(String(RPValuesBuff[0]) + "::" + String(RPValuesBuff[1]));

    //HASP23_CheckIncomingData();
    aimed = (RPValuesBuff[0] != -666666) && (RPValuesBuff[1] != -666666);

    if (aimed)
    {
      Serial.println("Tracking light...");

      int dirH = 0;
      int dirV = 0;

      if (RPValuesBuff[0] < 0)
        dirH = 1;
      if (RPValuesBuff[1] < 0)
        dirV = 1;

      HASP23_RunMotorB(MotorBTrackSteps, dirH, MotorBSlowStepDelay, true, false);
      HASP23_RunMotorA(MotorATrackSteps, dirV, MotorASlowStepDelay, true, false);
    }
    else
    {
      Serial.println("Lost light, running square pattern...");

      //int dirH = 0;
      //int dirV = 0;
      int iters = SquarePatternIters;
      int stepsH = SquarePatternMotorBIncSteps;
      int stepsV = SquarePatternMotorAIncSteps;
      int dirH = 0;
      int dirV = 0;

      statusH = HASP23_RunMotorB(MotorBTrackRollbackSteps, !CurrSearchDirH, MotorBSlowStepDelay, true, true);

      if (statusH == 2)
        iters = 0;

      for (int i = 0; i < iters; i++)
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

void HASP23_SerialThread()
{
  while (true)
  {
    HASP23_CheckIncomingData();
    delay(SerialThreadDelay * 1000);
  }
}

void HASP23_DownlinkThread()
{
  uint32_t downlinkTimer = 0;

  while (true)
  {
    uint32_t startTime = millis();

    if (HASP23_CheckReset())
    {
      Serial.println("Restarting...");
      _reboot_Teensyduino_();
    }

    if ((downlinkTimer / 1000) >= DownlinkDelay)
    {
      HASP23_SendDownlink();
      downlinkTimer = 0;
    }

    delay(DownlinkThreadDelay * 1000);

    downlinkTimer += millis() - startTime;
  }
}

// MOTOR FUNCTIONS //

int HASP23_RunMotorA(int steps, int dir, int delay, bool limitInterrupt, bool targetInterrupt)
{
  //Serial.println("HASP23_RunMotorA");

  digitalWrite(PIN_MotorADir, dir);

  for (int i = 0; i < steps; i++)
  {
    HASP23_SaveDataSD();

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
    HASP23_SaveDataSD();

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
  if (!SDOpen)
    return;

  String data = "";
  data += String(((float)millis()) / 1000);
  data += ", ";
  data += String(TelescopeStatus);
  data += ", ";
  data += String(HASP23_ReadTemp(PIN_Therm1));
  data += ", ";
  data += String(HASP23_ReadTemp(PIN_Therm2));
  data += ", ";
  data += String(HASP23_ReadTemp(PIN_Therm3));
  data += ", ";
  data += String(HASP23_ReadTemp(PIN_Therm4));
  data += ", ";
  data += String(HASP23_ReadTemp(PIN_Therm5));
  data += ", ";
  data += String(HASP23_ReadTemp(PIN_Therm6));

  HASP23_WriteFileSD(data);
}

void HASP23_WriteFileSD(String data)
{
  char buff[DataBuffLen];

  if (CurrDataFileLines >= DataFileMaxLines)
  {
    if (!SDFirstSave)
      CurrDataFile.close();
    else
      SDFirstSave = true;

    DataFileNum++;

    String fileName = DataFileNamePrefix + String(DataFileNum) + ".txt";
    fileName.toCharArray(buff, fileName.length() + 1);
    buff[fileName.length()] = '\0';

    if (SD.exists(buff) == 1)
        SD.remove(buff);

    CurrDataFile = SD.open(buff, FILE_WRITE);
    CurrDataFile.seek(EOF);

    CurrDataFile.write("-------- HASP 2023 - College of the Canyons - DataFile #");

    String fileNumStr = String(DataFileNum);
    fileNumStr.toCharArray(buff, fileNumStr.length() + 1);
    buff[fileNumStr.length()] = '\0';
    CurrDataFile.write(buff);

    CurrDataFile.write(" --------\n---- (Time (s), Telescope Status, Temp 1 (C), Temp 2 (C), Temp 3 (C), Temp 4 (C), Temp 5 (C), Temp 6 (C)) ----\n\n");

    CurrDataFileLines = 0;
  }

  data += "\n";
  data.toCharArray(buff, data.length() + 1);
  buff[data.length()] = '\0';
  CurrDataFile.write(buff);

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
      String fileName = entry.name();
      int prefixIdx = fileName.indexOf(DataFileNamePrefix);
      int extIdx = fileName.indexOf(".txt");

      if ((prefixIdx != -1) && (extIdx != -1))
      {
        int currNum = fileName.substring(prefixIdx + DataFileNamePrefix.length(), extIdx).toInt();

        if (currNum > maxNum)
          maxNum = currNum;
      }
    }

    entry.close();
  }

  return maxNum;
}

// SERIAL FUNCTIONS //

bool HASP23_CheckIncomingData()
{
  //Serial.println("HASP23_CheckIncomingData");

  //Serial.println("b");

  char messageBuff[RPMessageBuffLen + 1];
  int i = 0;

  for (i = 0; i < RPMessageBuffLen; i++)
  {
    int recievedByte = SerialRP.read();

    if (recievedByte == -1)
      break;

    char recievedChar = (char)(recievedByte);

    if (recievedChar == ';')
      break;

    messageBuff[i] = recievedChar;
  }

  //Serial.println("c");

  if (i == 0)
  {
    //Serial.println("No RP message recieved!");

    if (MissedLightMsgs >= LightLostTimeout)
    {
      for (int i = 0; i < RPMessageNum; i++)
        RPValuesBuff[i] = -666666;

      MissedLightMsgs = 0;
    }
    else
    {
      MissedLightMsgs++;
    }

    return false;
  }

  MissedLightMsgs = 0;
  HASP23_ClearRPSerialBuffer();

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

  //Serial.println("d");

  if (startOK && endOK && (parsed == RPMessageNum))
  {
    for (int i = 0; i < RPMessageNum; i++)
      RPValuesBuff[i] = tempData[i];

    return true;
  }

  return false;
}

void HASP23_ClearRPSerialBuffer()
{
  while (true)
    if (SerialRP.read() == -1)
      break;
}

void HASP23_SendDownlink()
{
  SerialDownlink.write('\t');
  SerialDownlink.write((uint8_t)HASP23_ReadTemp(PIN_Therm1));
  SerialDownlink.write((uint8_t)HASP23_ReadTemp(PIN_Therm2));
  SerialDownlink.write((uint8_t)HASP23_ReadTemp(PIN_Therm3));
  SerialDownlink.write((uint8_t)HASP23_ReadTemp(PIN_Therm4));
  SerialDownlink.write((uint8_t)HASP23_ReadTemp(PIN_Therm5));
  SerialDownlink.write((uint8_t)HASP23_ReadTemp(PIN_Therm6));
  SerialDownlink.write((uint8_t)HASP23_ReadTemp(PIN_Therm4));
  SerialDownlink.write((uint8_t)HASP23_ReadTemp(PIN_Therm5));
  SerialDownlink.write((uint8_t)HASP23_ReadTemp(PIN_Therm6));
  SerialDownlink.write('\n');
}

bool HASP23_CheckReset()
{
  char buff[2] = {SerialDownlink.read(), SerialDownlink.read()};
  
  if ((buff[0] == 'R') && (buff[1] == 'S'))
  {
    while (true)
      if (SerialDownlink.read() == -1)
        break;

    return true;
  }

  return false;
}
