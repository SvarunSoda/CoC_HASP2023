//// HASP 2023 TESTING CODE - TEENSY ////

// INCLUDES //

#include "Arduino.h"
#include <SD.h>

#include <TeensyThreads.h>

// GLOBAL VARIABLES //

#define SerialRP Serial2                          // Teensy serial channel for RP data line
#define SerialComms Serial6                       // Teensy serial channel for HASP comms (downlink & uplink)

const int PIN_MotorAEna = 29;                     // Teensy pin for vertical stepper motor enable
const int PIN_MotorAStep = 28;                    // Teensy pin for vertical stepper motor step
const int PIN_MotorADir = 27;                     // Teensy pin for vertical stepper motor direction
const int PIN_MotorBEna = 32;                     // Teensy pin for horizontal stepper motor enable
const int PIN_MotorBStep = 31;                    // Teensy pin for horizontal stepper motor step
const int PIN_MotorBDir = 30;                     // Teensy pin for horizontal stepper motor direction
const int PIN_Switch1 = 36;                       // Teensy pin for limit switch #1
const int PIN_Switch2 = 35;                       // Teensy pin for limit switch #2
const int PIN_Switch3 = 34;                       // Teensy pin for limit switch #3
const int PIN_Switch4 = 33;                       // Teensy pin for limit switch #4
const int PIN_Therm1 = 19;                        // Teensy pin for thermistor sensor #1
const int PIN_Therm2 = 18;                        // Teensy pin for thermistor sensor #2
const int PIN_Therm3 = 17;                        // Teensy pin for thermistor sensor #3
const int PIN_Therm4 = 16;                        // Teensy pin for thermistor sensor #4
const int PIN_Therm5 = 15;                        // Teensy pin for thermistor sensor #5
const int PIN_Therm6 = 14;                        // Teensy pin for thermistor sensor #6
const int PIN_SD = BUILTIN_SDCARD;                // Teensy pin for SD card

const int SearchIncV = 20;                        // Vertical motor step increment after each wide search sweep
const int MotorAFastStepDelay = 3000;             // Vertical motor fast step delay
const int MotorBFastStepDelay = 3000;             // Horizontal motor fast step delay
const int MotorASlowStepDelay = 8000;             // Vertical motor slow step delay
const int MotorBSlowStepDelay = 8000;             // Horizontal motor slow step delay
const int MotorMaxSteps = 999999999;              // Maximum steps for both stepper motors
const int MotorALimitBackupSteps = 50;            // Vertical motor rollback steps upon reaching a limit switch
const int MotorBLimitBackupSteps = 50;            // Horizontal motor rollback steps upon reaching a limit switch
const int MotorATrackSteps = 1;                   // Vertical motor track steps during each tracking iteration
const int MotorBTrackSteps = 1;                   // Horizontal motor track steps during each tracking iteration
const int MotorBTrackRollbackSteps = 2;           // Horizontal motor rollback steps after each overshot light detection
const int SquarePatternIters = 8;                 // Iterations of each performed square search pattern
const int SquarePatternMotorAIncSteps = 4;        // Vertical motor steps by which each square search pattern is incremented
const int SquarePatternMotorBIncSteps = 4;        // Horizontal motor steps by which each square search pattern is incremented
const float SquarePatternMoveDelay = 0.1;         // (sec) Delay after each square search pattern iteration
const int ThermResistance = 10000;                // Resistance of resistors adjacent to thermistor sensors
const int BaudRateRP = 115200;                    // Baud rate for RP data serial channel
const int BaudRateComms = 4800;                   // Baud rate for HASP comms serial channel
const int RPMessageNum = 4;                       // Number of values in each recieved RP serial data message
const int RPMessageBuffLen = 50;                  // Buffer length for each recieved RP serial data message
const String DataFileNamePrefix = "HASP23_Data_"; // Prefix for all data files saved to SD card
const int DataFileMaxLines = 100000;              // Maximum amount of lines each SD card data file will reach
const int DataBuffLen = 200;                      // Buffer length for writing to SD card data files
const int SwitchReadNum = 40;                     // Number of times each limit switch read will be averaged
const int LightLostTimeout = 3;                   // Number of RP message checks after which the detected light position will be reverted to -666666
const float LoopDelay = 1;                        // (sec) Delay after each main telescope loop iteration
const float TrackDelay = 0.1;                     // (sec) Delay before each telescope track loop iteration
const float DownlinkDelay = 300;                  // (sec) Delay between each downlink message being sent
                                                  //Change Downlink to 300 seconds
const float CommsThreadDelay = 1;                 // (sec) Delay after each downlink & uplink check

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
int NewlyRestarted = 1;
bool CanTrackMove = false;

// MAIN FUNCTIONS //

void setup() 
{
  Serial.begin(9600);
  SerialRP.begin(BaudRateRP);
  SerialComms.begin(BaudRateComms);

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

  threads.addThread(HASP23_CommsThread);

  Serial.println("=================== Telescope initialized. ===================");
}

void loop() 
{
  // Serial.print("1: " + String(HASP23_ReadSwitch(PIN_Switch1)));
  // Serial.print(", 2: " + String(HASP23_ReadSwitch(PIN_Switch2)));
  // Serial.print(", 3: " + String(HASP23_ReadSwitch(PIN_Switch3)));
  // Serial.println(", 4: " + String(HASP23_ReadSwitch(PIN_Switch4)));

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

    HASP23_CheckIncomingData();
    aimed = (RPValuesBuff[0] != -666666) && (RPValuesBuff[1] != -666666);

    if (aimed)
    {
      Serial.println("Tracking light...");

      if (CanTrackMove)
      {
        /*int dirH = 1;
        int dirV = 1;

        if (RPValuesBuff[0] < 0)
          dirH = 0;
        if (RPValuesBuff[1] < 0)
          dirV = 0;

        HASP23_RunMotorB(MotorBTrackSteps, dirH, MotorBSlowStepDelay, true, false);
        HASP23_RunMotorA(MotorATrackSteps, dirV, MotorASlowStepDelay, true, false);*/

        CanTrackMove = false;
      }
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

      /*statusH = HASP23_RunMotorB(MotorBTrackRollbackSteps, !CurrSearchDirH, MotorBSlowStepDelay, true, true);

      if (statusH == 2)
        iters = 0;*/

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
  //println("HASP23_TelescopeHome");

  TelescopeStatus = 1;

  HASP23_RunMotorB(MotorMaxSteps, 1, MotorBFastStepDelay, true, false);
  HASP23_RunMotorA(MotorMaxSteps, 1, MotorAFastStepDelay, true, false);
  //HASP23_RunMotorA(100, 0, MotorAFastStepDelay, true, false);

  CurrSearchDirH = 0;
  TelescopeStatus = 0;
}

// THREAD FUNCTIONS //

void HASP23_CommsThread()
{
  //Serial.println("HASP23_CommsThread");

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

    delay(CommsThreadDelay * 1000);

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
    HASP23_CheckIncomingData();
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
    HASP23_CheckIncomingData();
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
  //Serial.println("HASP23_ReadTemp");

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
  //Serial.println("HASP23_SaveDataSD");

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
  //Serial.println("HASP23_WriteFileSD");

  if (!SDOpen)
    return;

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
  //Serial.println("HASP23_GetDataFileNumSD");

  if (!SDOpen)
    return 0;

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

  //Serial.println("a");

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

  //Serial.println("b");

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
  
  //Serial.println("c");

  while (true)
    if (SerialRP.read() == -1)
      break;

  //Serial.println("d");

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

  //Serial.println("e");

  if (startOK && endOK && (parsed == RPMessageNum))
  {
    for (int i = 0; i < RPMessageNum; i++)
      RPValuesBuff[i] = tempData[i];

    CanTrackMove = true;

    return true;
  }

  //Serial.println("f");

  return false;
}

void HASP23_SendDownlink()
{
  //Serial.println("HASP23_SendDownlink");

  SerialComms.print('s');              //starting symbol
  SerialComms.print(HASP23_ReadTemp(PIN_Therm1)); //Science camera get rid of the castings, add write print add comma
  SerialComms.print(',');
  SerialComms.print(HASP23_ReadTemp(PIN_Therm2)); //PCB
  SerialComms.print(',');
  SerialComms.print(HASP23_ReadTemp(PIN_Therm3)); //Guide Camera
  SerialComms.print(',');
  SerialComms.print(HASP23_ReadTemp(PIN_Therm4)); //Doesn't work
  SerialComms.print(',');
  SerialComms.print(HASP23_ReadTemp(PIN_Therm5));  //Pi
  SerialComms.print(',');
  SerialComms.print(HASP23_ReadTemp(PIN_Therm6)); //Outside
  SerialComms.print(',');
  SerialComms.print(RPValuesBuff[3]); //Amount of images taken
  SerialComms.print(',');
  SerialComms.print(RPValuesBuff[2]); // Is camera seeing sun (i.e.)
  SerialComms.print(',');
  SerialComms.print(NewlyRestarted);
  SerialComms.print(',');
  SerialComms.println('e');          //ending symbol
  
  NewlyRestarted = 0;
}

bool HASP23_CheckReset()
{
  //Serial.println("HASP23_CheckReset");
  
  int buff[2] = {SerialComms.read(), SerialComms.read()};
  
  if ((buff[0] == 0x53) && (buff[1] == 0x7D))
  {
    while (true)
      if (SerialComms.read() == -1)
      
        break;
    return true;
  }

  return false;
}
