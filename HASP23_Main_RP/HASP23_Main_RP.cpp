//// HASP 2023 TESTING CODE - RP ////
// Runner command: sudo LD_LIBRARY_PATH=/home/hasp23/Desktop/HASP23_Code_RP/ASI /home/hasp23/Desktop/HASP23_Code_RP/Output/HASP23_Main_RP

// INCLUDES //

#include <iostream>
#include <fstream>
#include <string>
#include <limits>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <time.h>
#include <cstdlib>
#include <experimental/filesystem>

#include "ASI/ASICamera2.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>

#include <wiringPi.h>
#include <wiringSerial.h>

#include <thread>

using namespace std;
using namespace cv;
using std::experimental::filesystem::directory_iterator;

// GLOBAL VARIABLES //

const string ProgramPath = "/home/hasp23/Desktop/HASP23_Code_RP/Output/";
const int LoopRuns = numeric_limits<int>::max();	// How many times the science loop runs
const uchar LightThreshold = 10;						// (0 - 255) Level above which pixels are considered bright
const int LightPatchSizeThreshold = 0;					// Total size above which light patches are considered for movement
const int CameraGuideIdx = 1;							// Index of guide camera
const int CameraScienceIdx = 0;							// Index of science camera
const int CameraGuideResolution[2] = {3096, 2080};		// (H, V) Resolution of guide camera images (max: 3096, 2080)
const int CameraGuideFormat[2] = {ASI_IMG_RAW8, 1};		// (type, bin) Format of guide camera images
const int CameraScienceResolution[2] = {3840, 2160};	// (H, V) Resolution of science camera images (max: 3840, 2160)
const int CameraScienceFormat[2] = {ASI_IMG_RAW8, 1};	// (type, bin) Format of science camera images
const float ExposureTimeGuide = 2;						// (ms) Exposure time of guide camera
//const float ExposureTimeScience = 0.2;					// (ms) Otiginal exposure time of science camera
const float ExposureTimeScience = 1.2;
const int GainGuide = 200;
//const int GainScience = 150;								original gain
const int GainScience = 1;
const int TargetTolerance = 200;							// Pixel space around image center inside which the target is considered as close
const int CameraSnapTimeout = 20000;						//  Timeout iters of cameras taking images
const int MaxLightPatches = 100000;						// Maximum amount of light patches detectable in one image
const string ScienceImagePrefix = "img_science_";		// Filename prefix of all saved science camera images
const string ImagesExt = ".png";					// Filename extension of all saved science camera images
const int SerialBaud = 115200;							// Baud rate of serial line to Teensy
const float CaptureDelay = 0;							// (sec) Delay between each camera capture
const float GuideLoopDelay = 0;							// (sec) Delay after each guide loop iteration
const float ScienceLoopDelay = 60;						// (sec) Delay after each science loop iteration
const float SerialDelay = 0.1;
const float ThreadMonitorDelay = 1;						// (sec) Delay between each thread status print
const bool UsingCameras = true;							// true: Takes the images from the cameras | false: loads the test images
const bool SaveDebugImages = true;						// true: saves debug images | false: doesn't save debug images

// DO NOT CHANGE //

IplImage* CVImageMMData;
IplImage* CVImageMCData;
long ImageMMSize = CameraGuideResolution[0] * CameraGuideResolution[1] * (1 + (CameraGuideFormat[0] == ASI_IMG_RAW16));
long ImageMCSize = CameraScienceResolution[0] * CameraScienceResolution[1] * (1 + (CameraScienceFormat[0] == ASI_IMG_RAW16));
unsigned char* ImageMMBuff = new unsigned char[ImageMMSize];
unsigned char* ImageMCBuff = new unsigned char[ImageMCSize];
Mat ImageMMMat;
Mat ImageMCMat;
Mat imageMCFinalMat;
int CurrLightPos[2] = {-666666, -666666};
int CameraNum = 0;
int RPSerialChannel = 0;
int TargetsAcquired = 0;
int ImagesSnapped = 0;
int GuideThreadStatus = 0;
int ScienceThreadStatus = 0;
float GuideThreadTime = 0;
float ScienceThreadTime = 0;

// FORWARD DECLARATIONS //

class LightPatch
{
public:
	int lX;
	int lY;
	int hX;
	int hY;
	float sizeX;
	float sizeY;
	int posX;
	int posY;

	LightPatch()
	{
		lX = -1;
		lY = -1;
		hX = -1;
		hY = -1;
		sizeX = -1;
		sizeY = -1;
		posX = -1;
		posY = -1;
	}

	LightPatch(int valueLX, int valueLY, int valueHX, int valueHY)
	{
		lX = valueLX;
		lY = valueLY;
		hX = valueHX;
		hY = valueHY;
		sizeX = valueHX - valueLX;
		sizeY = valueHY - valueLY;
		posX = lX + (sizeX / 2);
		posY = lY + (sizeY / 2);
	}

	void CalculatePos()
	{
		sizeX = hX - lX;
		sizeY = hY - lY;
		posX = lX + (sizeX / 2);
		posY = lY + (sizeY / 2);
	}
};

void HASP23_ThreadGuide();
void HASP23_ThreadScience();
void HASP23_ThreadSerial();
LightPatch HASP23_CalculateTarget(Mat* imageMat, int imageHeight, int imageWidth);
int HASP23_GetSnappedImageCount(string filePrefix);
bool HASP23_ProcessGuide();
void HASP23_ProcessScience();
bool HASP23_CameraMMSnap(unsigned char* imageBuff, int imageSize);
bool HASP23_CameraMCSnap(unsigned char* imageBuff, int imageSize);
long HASP23_CameraGetTemp(int cameraIdx);
void HASP23_CameraClose(int cameraIdx);
bool HASP23_CameraInit(int cameraIdx, float exposureTime, int gain, int imageWidth, int imageHeight, int imageType, int imageBin);
void HASP23_SerialSend(int serialChannel, string message);
void HASP23_SerialClose(int serialChannel);
int HASP23_SerialInit();

// MAIN FUNCTION //

int main()
{
	cout << "---- PROGRAM STARTED ----\n\n";

	// Initializing serial channel

	RPSerialChannel = HASP23_SerialInit();

	// Initializing cameras

	CameraNum = ASIGetNumOfConnectedCameras();

	if ((UsingCameras) && (CameraNum <= 1))
	{
		cout << "ERROR: Less than 2 cameras connected!\n";
		
		return 1;
	}

	ImagesSnapped = HASP23_GetSnappedImageCount(ScienceImagePrefix);

	if (UsingCameras)
	{
		if (!HASP23_CameraInit(CameraGuideIdx, ExposureTimeGuide, GainGuide, CameraGuideResolution[0], CameraGuideResolution[1], CameraGuideFormat[0], CameraGuideFormat[1]))
		{
			cout << "ERROR: MM camera failed to initialize!\n";
			GuideThreadStatus = 2;

			return 1;
		}
		if (!HASP23_CameraInit(CameraScienceIdx, ExposureTimeScience, GainScience, CameraScienceResolution[0], CameraScienceResolution[1], CameraScienceFormat[0], CameraScienceFormat[1]))
		{
			cout << "RROR: MC camera failed to initialize!\n";
			ScienceThreadStatus = 2;

			return 1;
		}

		CVImageMMData = cvCreateImage(cvSize(CameraGuideResolution[0], CameraGuideResolution[1]), IPL_DEPTH_8U, 1);
		CVImageMCData = cvCreateImage(cvSize(CameraScienceResolution[0], CameraScienceResolution[1]), IPL_DEPTH_8U, 1);
	}

	// Running guide & science threads

	thread guideThread = thread(HASP23_ThreadGuide);
	thread scienceThread = thread(HASP23_ThreadScience);
	thread serialThread = thread(HASP23_ThreadSerial);

	do
	{
		cout << "\nTargets acquired: " + to_string(TargetsAcquired) + "\n";
		cout << "Guide thread status: " + to_string(GuideThreadStatus) + "\n";
		cout << "Guide thread time: " + to_string(GuideThreadTime) + " s\n";
		cout << "Science thread status: " + to_string(ScienceThreadStatus) + "\n";
		cout << "Science thread time: " + to_string(ScienceThreadTime) + " s\n";
		cout << "Images snapped: " + to_string(ImagesSnapped) + "\n";

		usleep(ThreadMonitorDelay * 1000000);
	} 
	while ((GuideThreadStatus == 0) || (ScienceThreadStatus == 0));

	guideThread.join();
	scienceThread.join();
	serialThread.join();

	// Closing program

	HASP23_SerialClose(RPSerialChannel);

	cout << "\n-- FINAL STATS --\n\tTargets acquired: " + to_string(TargetsAcquired) + "\n";
	cout << "\tGuide thread status: " + to_string(GuideThreadStatus) + "\n";
	cout << "\tGuide thread time: " + to_string(GuideThreadTime) + " s\n";
	cout << "\tScience thread status: " + to_string(ScienceThreadStatus) + "\n";
	cout << "\tScience thread time: " + to_string(ScienceThreadTime) + " s\n";
	cout << "\tImages snapped: " + to_string(ImagesSnapped) + "\n";

	cout << "\n---- PROGRAM FINISHED ----\n";

	return 0;
}

// THREAD FUNCTIONS //

void HASP23_ThreadGuide()
{
	//if (UsingCameras)
	//	cout << "\nMM sensor temperature: " + to_string(HASP23_CameraGetTemp(CameraGuideIdx)) + " C\n";

	// Main loop of capturing image 

	//cout << "\n-- MAIN LOOP STARTING --\n";

	bool tracking = false;
	bool prevTracking = false;

	while (ImagesSnapped < LoopRuns)
	{
		//cout << "\nStarting loop iteration " + to_string(ImagesSnapped + 1) + "...\n";
		auto startTime = chrono::high_resolution_clock::now();

		tracking = HASP23_ProcessGuide();

		if (tracking)
		{
			HASP23_ProcessScience();

			if (!prevTracking)
				TargetsAcquired++;
		}

		auto endTime = chrono::high_resolution_clock::now();
		GuideThreadTime = (float)((endTime - startTime) / chrono::milliseconds(1)) / 1000;
		//cout << "Finished loop iteration #" + to_string(ImagesSnapped + 1) + " (" + to_string(elapsedTime) + " s).\n";

		usleep(GuideLoopDelay * 1000000);
	}

	//cout << "\n-- MAIN LOOP FINISHED --\n";

	if (UsingCameras)
		HASP23_CameraClose(CameraGuideIdx);

	cvReleaseImage(&CVImageMMData);

	if (ImageMMBuff)
		delete[] ImageMMBuff;

	GuideThreadStatus = 1;
}

void HASP23_ThreadScience()
{
	//if (UsingCameras)
	//	cout << "\nMC sensor temperature: " + to_string(HASP23_CameraGetTemp(CameraScienceIdx)) + " C\n";

	// Main loop of capturing image 

	//cout << "\n-- MAIN LOOP STARTING --\n";

	while (ImagesSnapped < LoopRuns)
	{
		//cout << "\nStarting loop iteration " + to_string(ImagesSnapped + 1) + "...\n";
		auto startTime = chrono::high_resolution_clock::now();

		HASP23_ProcessScience();

		auto endTime = chrono::high_resolution_clock::now();
		ScienceThreadTime = (float)((endTime - startTime) / chrono::milliseconds(1)) / 1000;
		//cout << "Finished loop iteration #" + to_string(ImagesSnapped + 1) + " (" + to_string(elapsedTime) + " s).\n";

		usleep(ScienceLoopDelay * 1000000);
	}

	//cout << "\n-- MAIN LOOP FINISHED --\n";

	if (UsingCameras)
		HASP23_CameraClose(CameraScienceIdx);

	cvReleaseImage(&CVImageMCData);

	if (ImageMCBuff)
		delete[] ImageMCBuff;

	ScienceThreadStatus = 1;
}

void HASP23_ThreadSerial()
{
	while (GuideThreadStatus == 0)
	{
		//cout << "Sending data to Teensy...\n";

		string moveData = "";
		moveData += "(";
		moveData += to_string(CurrLightPos[0]);
		moveData += ", ";
		moveData += to_string(CurrLightPos[1]);
		moveData += ", ";
		moveData += to_string(TargetsAcquired);
		moveData += ", ";
		moveData += to_string(ImagesSnapped + 1);
		moveData += "); ";

		HASP23_SerialSend(RPSerialChannel, moveData);

		usleep(SerialDelay * 1000000);
	}
}

// IMAGE FUNCTIONS //

LightPatch HASP23_CalculateTarget(Mat* imageMat, const int imageHeight, const int imageWidth)
{
	LightPatch lightPatchBuff[MaxLightPatches];
	int lightPatchNum = 0;

	int** patchIndices = new int*[imageHeight];

	for (int y = 0; y < imageHeight; y++)
		patchIndices[y] = new int[imageWidth];

	bool waitingForUp = false;
	int waitingLX = -1;
	int largestPatchIdx = 0;
	bool loopBroke = false;

	// Searching for all light patches
	for (int y = 0; y < imageHeight; y++)
  	{
		for (int x = 0; x < imageWidth; x++)
		{
			if (loopBroke)
			{
				patchIndices[y][x] = -1;

				continue;
			}

			uchar currLightValue = imageMat->at<uchar>(y, x);
			int connectedPatchIdx = -1;
			bool connectedUp = false;
			bool connectedLeft = false;

			// If we're not at first row
			if (y > 0)
			{
				// If current pixel is bright
				if (currLightValue >= LightThreshold)
				{
					if (patchIndices[y - 1][x] != -1)
					{
						connectedPatchIdx = patchIndices[y - 1][x];
						connectedUp = true;
						waitingForUp = false;
					}
					else if ((x > 0) && (patchIndices[y][x - 1] != -1))
					{
						connectedPatchIdx = patchIndices[y][x - 1];
						connectedLeft = true;
						waitingForUp = false;
						waitingLX = -1;
					}
					else
					{
						if (!waitingForUp)
						{
							waitingLX = x;
							waitingForUp = true;
						}
					}

					if (connectedUp)
					{
						if (y > lightPatchBuff[connectedPatchIdx].hY)
						{
							lightPatchBuff[connectedPatchIdx].hY = y;
						}
						if ((waitingLX != -1) && (waitingLX < lightPatchBuff[connectedPatchIdx].lX))
						{
							lightPatchBuff[connectedPatchIdx].lX = waitingLX;

							for (int prevX = waitingLX; prevX < x; prevX++)
								patchIndices[y][prevX] = connectedPatchIdx;
						}

						waitingLX = -1;
					}
					else if (connectedLeft)
					{
						if (x < lightPatchBuff[connectedPatchIdx].lX)
							lightPatchBuff[connectedPatchIdx].lX = x;
						if (x > lightPatchBuff[connectedPatchIdx].hX)
							lightPatchBuff[connectedPatchIdx].hX = x;
					}
					else
					{ 
						if (!waitingForUp)
						{
							if (lightPatchNum >= MaxLightPatches)
							{
								cout << "ERROR: Light Patch Buffer exceeded maximum size!\n";
								patchIndices[y][x] = -1;
								loopBroke = true;

								continue;
							}
							else
							{
								lightPatchBuff[lightPatchNum] = LightPatch(x, y, x, y);
								connectedPatchIdx = lightPatchNum;

								lightPatchNum++;
							}
						}
					}
				}
				// Else if current pixel is not bright
				else
				{
					if (waitingForUp)
					{
						if (lightPatchNum >= MaxLightPatches)
						{
							cout << "ERROR: Light Patch Buffer exceeded maximum size!\n";
							patchIndices[y][x] = -1;
							loopBroke = true;

							continue;
						}
						else
						{
							lightPatchBuff[lightPatchNum] = LightPatch(waitingLX, y, x, y);
							
							for (int prevX = waitingLX; prevX < x; prevX++)
								patchIndices[y][prevX] = lightPatchNum;

							connectedPatchIdx = lightPatchNum;

							lightPatchNum++;
						}

						waitingForUp = false;
						waitingLX = -1;
					}
				}
			}
			// If we're at first row
			else
			{
				if (currLightValue >= LightThreshold)
				{
					if ((x > 0) && (patchIndices[y][x - 1] != -1))
					{
						connectedPatchIdx = patchIndices[y][x - 1];
						connectedLeft = true;
					}

					if (connectedLeft)
					{
						if (x < lightPatchBuff[connectedPatchIdx].lX)
							lightPatchBuff[connectedPatchIdx].lX = x;
						if (x > lightPatchBuff[connectedPatchIdx].hX)
							lightPatchBuff[connectedPatchIdx].hX = x;
					}
					else
					{
						if (lightPatchNum >= MaxLightPatches)
						{
							cout << "ERROR: Light Patch Buffer exceeded maximum size!\n";
							patchIndices[y][x] = -1;
							loopBroke = true;

							continue;
						}
						else
						{
							lightPatchBuff[lightPatchNum] = LightPatch(x, y, x, y);
							connectedPatchIdx = lightPatchNum;

							lightPatchNum++;
						}
					}
				}
			}

			patchIndices[y][x] = connectedPatchIdx;
		}


  	}

	// Finding largest light patch
	if (lightPatchNum > 0)
	{
		lightPatchBuff[0].CalculatePos();
		float largestSize = (lightPatchBuff[0].sizeX * 2) + (lightPatchBuff[0].sizeY * 2);

		for (int i = 1; i < lightPatchNum; i++)
		{
			lightPatchBuff[i].CalculatePos();
			float currSize = (lightPatchBuff[i].sizeX * 2) + (lightPatchBuff[i].sizeY * 2);

			if (currSize > largestSize)
			{
				largestPatchIdx = i;
				largestSize = currSize;
			}
		}
	}

	for (int y = 0; y < imageHeight; y++)
		delete[] patchIndices[y];

	delete[] patchIndices;

	if (lightPatchNum > 0)
	{
		//cout << to_string(lightPatchNum) + " light patches detected.\n";

		return lightPatchBuff[largestPatchIdx];
	}

	//cout << "WARNING: No light patches detected!\n";

	return LightPatch();
}

int HASP23_GetSnappedImageCount(string filePrefix)
{
	int maxNum = 0;

	for (const auto &entry: directory_iterator(ProgramPath + "images/"))
	{
		string fileName = entry.path().filename().string();
		fileName = fileName.substr(0, fileName.find_last_of("."));
		int filePrefixIdx = fileName.find(filePrefix);
		
		if (filePrefixIdx != string::npos)
		{
			int currNum = stoi(fileName.substr(filePrefixIdx + filePrefix.length(), fileName.length()));

			if (currNum > maxNum)
				maxNum = currNum;
		}
	}

	return maxNum;
}

// CAMERA FUNCTIONS //

bool HASP23_ProcessGuide()
{
	bool tracking = false;

	// Take MM image

	if ((UsingCameras && HASP23_CameraMMSnap(ImageMMBuff, ImageMMSize)) || !UsingCameras)
	{
		//cout << "Loading guide image...\n";
		if (UsingCameras)
		{
			unsigned char *pCv8bit = (unsigned char*)CVImageMMData->imageData;
			unsigned char *pImg8bit = (unsigned char*)ImageMMBuff;

			for (int y = 0; y < CameraGuideResolution[1]; y++)
			{
				memcpy(pCv8bit, pImg8bit, CameraGuideResolution[0]);
				pCv8bit += CameraGuideResolution[0];
				pImg8bit += CameraGuideResolution[0];
			}

			ImageMMMat = Mat(CameraGuideResolution[1], CameraGuideResolution[0], CameraGuideFormat[0], CVImageMMData->imageData);
		}
		else
		{
			ImageMMMat = imread("debug/test_guide" + ImagesExt, 0);
		}
		//imwrite(ProgramPath + "images/" + GuideImagePrefix + to_string(GuideImagesSnapped + 1) + GuideImageExt, ImageMMMat);

		//cout << "Calculating largest light patch..\n";

		LightPatch largestPatch = HASP23_CalculateTarget(&ImageMMMat, CameraGuideResolution[1], CameraGuideResolution[0]);
		int currPosX = CameraGuideResolution[0] / 2;
		int currPosY = CameraGuideResolution[1] / 2;

		int lightPosX = -666666;
		int lightPosY = -666666;

		// If there has been a detected light patch...
		if ((largestPatch.posX != -1) && (largestPatch.posY != -1))
		{
			// If the detected light patch size is greater than the size threshold...
			if ((largestPatch.sizeX * largestPatch.sizeY) >= LightPatchSizeThreshold)
			{
				lightPosX = 0;
				lightPosY = 0;

				// If we need to move (if we're outside the tolerance)...
				if (((largestPatch.posX < (currPosX - TargetTolerance)) || (largestPatch.posX > (currPosX + TargetTolerance))) ||
					((largestPatch.posY < (currPosY - TargetTolerance)) || (largestPatch.posY > (currPosY + TargetTolerance))))
				{
					lightPosX = round(currPosX - largestPatch.posX);
					lightPosY = round(currPosY - largestPatch.posY);

					tracking = true;
				}
			}
		}

		CurrLightPos[0] = lightPosX;
		CurrLightPos[1] = lightPosY;

		//cout << "Saving guide image...\n";

		if (SaveDebugImages && tracking)
		{
			for (int y = 0; y < CameraGuideResolution[1]; y++)
				for (int x = 0; x < CameraGuideResolution[0]; x++)
					if (x == largestPatch.posX)
						ImageMMMat.at<uchar>(y, x) = ~ImageMMMat.at<uchar>(y, x);
					else if (y == largestPatch.posY)
						ImageMMMat.at<uchar>(y, x) = ~ImageMMMat.at<uchar>(y, x);

			imwrite(ProgramPath + "debug/trg_" + to_string(ImagesSnapped + 1) + ImagesExt, ImageMMMat);
		}
	}

	return tracking;
}

void HASP23_ProcessScience()
{
	// Take MC image

	if ((UsingCameras && HASP23_CameraMCSnap(ImageMCBuff, ImageMCSize)) || !UsingCameras)
	{
		//cout << "Loading science image...\n";

		if (UsingCameras)
		{
			unsigned char *pCv8bit = (unsigned char*)CVImageMCData->imageData;
			unsigned char *pImg8bit = (unsigned char*)ImageMCBuff;

			for (int y = 0; y < CameraScienceResolution[1]; y++)
			{
				memcpy(pCv8bit, pImg8bit, CameraScienceResolution[0]);
				pCv8bit += CameraScienceResolution[0];
				pImg8bit += CameraScienceResolution[0];
			}

			ImageMCMat = Mat(CameraScienceResolution[1], CameraScienceResolution[0], CameraScienceFormat[0], CVImageMCData->imageData);
		}
		else
		{
			ImageMCMat = imread("debug/test_science" + ImagesExt, 0);
		}

		//cout << "Saving science image...\n";

		cvtColor(ImageMCMat, imageMCFinalMat, COLOR_BayerRG2RGB);

		imwrite(ProgramPath + "images/" + ScienceImagePrefix + to_string(ImagesSnapped + 1) + ImagesExt, imageMCFinalMat);
	}

	ImagesSnapped++;
}

bool HASP23_CameraMMSnap(unsigned char* imageBuff, int imageSize)
{
	ASIStartExposure(CameraGuideIdx, ASI_FALSE);

	usleep(CaptureDelay * 1000000);

	ASI_EXPOSURE_STATUS status = ASI_EXP_WORKING;
	int iters = 0;

	while (status == ASI_EXP_WORKING)
	{
		ASIGetExpStatus(CameraGuideIdx, &status);

		iters++;

		if (iters >= CameraSnapTimeout)
		{
			cout << "ERROR: MM camera snap timeout!\n";

			return false;
		}
	}

	if (status == ASI_EXP_SUCCESS)
	{
		ASIGetDataAfterExp(CameraGuideIdx, imageBuff, imageSize);

		return true;
	}

	cout << "ERROR: MM camera snap failure!\n";

	return false;
}

bool HASP23_CameraMCSnap(unsigned char* imageBuff, int imageSize)
{
	ASIStartExposure(CameraScienceIdx, ASI_FALSE);

	usleep(CaptureDelay * 1000000);

	ASI_EXPOSURE_STATUS status = ASI_EXP_WORKING;
	int iters = 0;

	while (status == ASI_EXP_WORKING)
	{
		ASIGetExpStatus(CameraScienceIdx, &status);

		iters++;

		if (iters >= CameraSnapTimeout)
		{
			cout << "ERROR: MC camera snap timeout!\n";

			return false;
		}
	}

	if (status == ASI_EXP_SUCCESS)
	{
		ASIGetDataAfterExp(CameraScienceIdx, imageBuff, imageSize);

		return true;
	}

	cout << "ERROR: MC camera snap failure!\n";

	return false;
}

long HASP23_CameraGetTemp(int cameraIdx)
{
	long temp;
	ASI_BOOL bAuto;
	ASIGetControlValue(cameraIdx, ASI_TEMPERATURE, &temp, &bAuto);

	return temp / 10;
}

void HASP23_CameraClose(int cameraIdx)
{
	ASIStopExposure(cameraIdx);
	ASICloseCamera(cameraIdx);
}

bool HASP23_CameraInit(int cameraIdx, float exposureTime, int gain, int imageWidth, int imageHeight, int imageType, int imageBin)
{
	ASI_CAMERA_INFO camInfo;
	
	string bayer[4] = {"RG", "BG", "GR", "GB"};
	char buf[128] = {0};

	if ((cameraIdx >= CameraNum) || (cameraIdx < 0))
	{
		cout << "ERROR: Invalid picked camera idx!\n";

		return false;
	}

	cout << "\nAccessing camera " + to_string(cameraIdx) + "...\n";
	
	ASIGetCameraProperty(&camInfo, cameraIdx);

	if (ASIOpenCamera(cameraIdx) != ASI_SUCCESS)
	{
		cout << "ERROR: Unable to access camera! Ensure you are the root user.\n";

		return false;
	}

	ASIInitCamera(cameraIdx);

	cout << "\nCamera \"" + string(camInfo.Name) + "\" information:\n";

	int imageWidthMax = camInfo.MaxWidth;
	int imageHeightMax =  camInfo.MaxHeight;

	cout << "\tMax Resolution: " + to_string(imageWidthMax) + " x " + to_string(imageHeightMax) + "\n";
	cout << "\tColor: ";

	if (camInfo.IsColorCam)
		cout << "Bayer pattern: " + bayer[camInfo.BayerPattern] + "\n";
	else
		cout << "Mono\n";
	
	int ctrlnum;
	ASIGetNumOfControls(cameraIdx, &ctrlnum);
	ASI_CONTROL_CAPS ctrlcap;

	cout << "\nAvailable controls:\n";

	for (int i = 0; i < ctrlnum; i++)
	{
		ASIGetControlCaps(cameraIdx, i, &ctrlcap);
			
		cout << "\t" + string(ctrlcap.Name) + "\n";
	}

	cout << "\n";
	cout << "Picked image format:\n\tSize: " + to_string(imageWidth) + " x " + to_string(imageHeight) + "\n\tBIN: 1\n\tImgType: raw8\n";

	ASISetROIFormat(cameraIdx, imageWidth, imageHeight, imageBin, (ASI_IMG_TYPE)(imageType));

	ASISetControlValue(cameraIdx, ASI_EXPOSURE, exposureTime * 1000, ASI_FALSE);
	ASISetControlValue(cameraIdx, ASI_GAIN, gain, ASI_FALSE);
	//ASISetControlValue(cameraIdx, ASI_GAMMA, 1, ASI_FALSE);
	//ASISetControlValue(cameraIdx, ASI_HIGH_SPEED_MODE, 1, ASI_FALSE);
	ASISetControlValue(cameraIdx, ASI_BANDWIDTHOVERLOAD, 40, ASI_FALSE);

	return true;
}

// SERIAL FUNCTIONS //

void HASP23_SerialSend(int serialChannel, string message)
{
	//serialPrintf(serialChannel, message.c_str() + "\n");
	serialPuts(serialChannel, message.c_str());

	cout << "Sent over serial channel " + to_string(serialChannel) + ": \"" + message + "\".\n";
}

void HASP23_SerialClose(int serialChannel)
{
	serialClose(serialChannel);
}

int HASP23_SerialInit()
{
	int serialChannel = serialOpen("/dev/ttyS0", SerialBaud);

	if (serialChannel < 0)
	{
		cout << "ERROR: Unable to open serial device! (" + string(strerror(errno)) + ")\n";

		return -1;
	}

	if (wiringPiSetup() == -1)
	{
		cout << "ERROR: Unable to start wiringPi! (" + string(strerror(errno)) + ")\n";
		
		return -1;
	}

	cout << "Successfully initialized serial channel " + to_string(serialChannel) + ".\n";

	return serialChannel;
}
