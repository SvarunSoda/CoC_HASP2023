//// HASP 2023 TESTING CODE - RP ////

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

#include "ASI/ASICamera2.h"
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>

#include <wiringPi.h>
#include <wiringSerial.h>

using namespace std;
using namespace cv;

// GLOBAL VARIABLES //

const uchar LightThreshold = 120;
const int LoopRuns = 30;
const int CameraIdx = 0;
//const int ImageWidth = 3096;
//const int ImageHeight = 2080;
const int ImageFormat = 0;
const int TargetTolerance = 5;
const int CameraSnapTimeout = 1000000000;
const int MaxLightPatches = 100;
const int SerialBaud = 9600;
const float ExposureTime = 10;
const float CaptureDelay = 0.1;
const float ImageDelay = 1;
const bool UsingCamera = true;
const bool SaveImages = true;
const bool DisplayImages = false;

int ImagesSnapped = 0;

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

LightPatch HASP23_CalculateTarget(Mat* imageMat, int imageHeight, int imageWidth);
void HASP23_CameraClose();
bool HASP23_CameraSnap(unsigned char* imageBuff, int imageSize);
long HASP23_CameraGetTemp();
bool HASP23_CameraInit(int* imageWidth, int* imageHeight, int* imageType, int* imageBin);
void HASP23_SerialSend(int serialChannel, string message);
void HASP23_SerialClose(int serialChannel);
int HASP23_SerialInit();

// MAIN FUNCTION //

int main()
{
	cout << "---- PROGRAM STARTED ----\n\n";

	// Initializing serial channel

	int serialChannel = HASP23_SerialInit();

	// Initializing Camera

	IplImage* cvImageData;
	int imageWidth;
	int imageHeight;
	int imageType;
	int imageBin;

	if (UsingCamera)
	{
		if (!HASP23_CameraInit(&imageWidth, &imageHeight, &imageType, &imageBin))
		{
			cout << "Exiting...\n";

			return 1;
		}

		if (imageType == ASI_IMG_RAW16)
			cvImageData = cvCreateImage(cvSize(imageWidth, imageHeight), IPL_DEPTH_16U, 1);
		else if (imageType == ASI_IMG_RGB24)
			cvImageData = cvCreateImage(cvSize(imageWidth, imageHeight), IPL_DEPTH_8U, 3);
		else
			cvImageData = cvCreateImage(cvSize(imageWidth, imageHeight), IPL_DEPTH_8U, 1);
	}
	else
	{
		imageWidth = 1280;
		imageHeight = 960;
		imageType = ASI_IMG_RAW8;
		imageBin = 1;
	}
	
	cvImageData = cvCreateImage(cvSize(imageWidth, imageHeight), IPL_DEPTH_8U, 1);

	if (UsingCamera)
		cout << "\nSensor temperature: " + to_string(HASP23_CameraGetTemp()) + " C\n";

	// Main loop of capturing image 

	long imageSize = imageWidth * imageHeight * (1 + (imageType == ASI_IMG_RAW16));
	long displaySize = imageWidth * imageHeight * (1 + (imageType == ASI_IMG_RAW16));
	unsigned char* imageBuff = new unsigned char[imageSize];
	int iterTarget = LoopRuns;

	if (LoopRuns < 0)
		iterTarget = numeric_limits<int>::max();

	if (DisplayImages)
		cvNamedWindow("preview", 1);

	cout << "\n-- MAIN LOOP STARTING --\n\n";

	while (ImagesSnapped < iterTarget)
	{
		cout << "Capturing image " + to_string(ImagesSnapped + 1) + "...\n";

		if ((UsingCamera && HASP23_CameraSnap(imageBuff, imageSize)) || !UsingCamera)
		{
			Mat imageMat;

			if (UsingCamera)
			{
				if (imageType == ASI_IMG_RAW16)
				{
					unsigned short *pCv16bit = (unsigned short*)(cvImageData->imageData);
					unsigned short *pImg16bit = (unsigned short*)imageBuff;

					for (int y = 0; y < imageHeight; y++)
					{
						memcpy(pCv16bit, pImg16bit, imageWidth * 2);
						pCv16bit += imageWidth;
						pImg16bit += imageWidth;
					}
				}
				else
				{
					unsigned char *pCv8bit = (unsigned char*)cvImageData->imageData;
					unsigned char *pImg8bit = (unsigned char*)imageBuff;

					for (int y = 0; y < imageHeight; y++)
					{
						memcpy(pCv8bit, pImg8bit, imageWidth);
						pCv8bit += imageWidth;
						pImg8bit += imageWidth;
					}
				}

				imageMat = Mat(imageHeight, imageWidth, imageType, cvImageData->imageData);
			}
			else
			{
				imageMat = Mat(imageHeight, imageWidth, CV_8U);
			}

			/*for (int y = 500; y < 550; y++)
				for (int x = 30; x < 40; x++)
					imageMat.at<uchar>(y, x) = 100;
			for (int y = 200; y < 250; y++)
				for (int x = 50; x < 70; x++)
					imageMat.at<uchar>(y, x) = 115;
			for (int y = 800; y < 850; y++)
				for (int x = 1000; x < 1050; x++)
					imageMat.at<uchar>(y, x) = 250;*/

			/*srand((unsigned) time(NULL));

			for (int i = 0; i < 20;  i++)
			{
				int centerPosX = rand() % (imageWidth - 0 + 1);
				int centerPosY = rand() % (imageHeight - 0 + 1);
				int radius = rand() % (8 - 2 + 1);

				for (int y = centerPosY - radius; y < centerPosY + radius; y++)
					for (int x = centerPosX - radius; x < centerPosX + radius; x++)
						if ((sqrt(pow(x - centerPosX, 2) + pow(y - centerPosY, 2)) < radius) && ((x >= 0) && (x < imageWidth) && (y >= 0) && (y < imageHeight)))
							imageMat.at<uchar>(y, x) = 255;
			}*/

			LightPatch largestPatch = HASP23_CalculateTarget(&imageMat, imageHeight, imageWidth);
			int currPosX = imageWidth / 2;
			int currPosY = imageHeight / 2;

			if ((largestPatch.posX != -1) && (largestPatch.posY != -1))
			{
				if (((largestPatch.posX < (currPosX - TargetTolerance)) || (largestPatch.posX > (currPosX + TargetTolerance))) ||
					((largestPatch.posY < (currPosY - TargetTolerance)) || (largestPatch.posY > (currPosY + TargetTolerance))))
				{
					int stepsX = round((currPosX - largestPatch.posX) / 20);
					int stepsY = round((currPosY - largestPatch.posY) / 20);

					string moveData = "";
					moveData += "(";
					moveData += to_string(stepsX);
					moveData += ", ";
					moveData += to_string(stepsY);
					moveData += ", ";
					moveData += to_string(ImagesSnapped + 1);
					moveData += ")";

					HASP23_SerialSend(serialChannel, moveData);
				}
			}

			if (SaveImages)
			{
				imwrite("debug/img_" + to_string(ImagesSnapped + 1) + ".jpg", imageMat);

				for (int y = 0; y < imageHeight; y++)
				{
					for (int x = 0; x < imageWidth; x++)
					{
						if (x == largestPatch.posX)
							imageMat.at<uchar>(y, x) = 255;
						else if (y == largestPatch.posY)
							imageMat.at<uchar>(y, x) = 255;
					}
				}

				imwrite("debug/trg_" + to_string(ImagesSnapped + 1) + ".jpg", imageMat);
			}
				
			//cvSaveImage("image_" + to_string(ImagesSnapped + 1) + ".jpg", cvImageData);

			if (DisplayImages)
				cvShowImage("preview", cvImageData);
		}

		ImagesSnapped++;

		usleep(ImageDelay * 1000000);
	}

	cout << "\n-- MAIN LOOP FINISHED --\n";

	if (DisplayImages)
		cvDestroyWindow("preview");

	if (UsingCamera)
		HASP23_CameraClose();

	cvReleaseImage(&cvImageData);

	if (imageBuff)
		delete[] imageBuff;

	HASP23_SerialClose(serialChannel);

	cout << "\n---- PROGRAM FINISHED ----\n";

	return 0;
}

// IMAGE FUNCTIONS //

LightPatch HASP23_CalculateTarget(Mat* imageMat, const int imageHeight, const int imageWidth)
{
	LightPatch lightPatchBuff[MaxLightPatches];
	int lightPatchNum = 0;

	//static int patchIndices[ImageHeight][ImageWidth];
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

	// Saving debug images
	if (SaveImages)
	{
		string trgData = "";

		for (int y = 0; y < imageHeight; y++)
		{
			for (int x = 0; x < imageWidth; x++)
			{
				int currPatchIdx = patchIndices[y][x];
				String currPatchIdxStr = to_string(currPatchIdx);

				if (currPatchIdx == -1)
					currPatchIdxStr = "l";

				for (int i = 0; i < (4 - currPatchIdxStr.length()); i++)
					currPatchIdxStr += " ";

				trgData += currPatchIdxStr;
			}

			trgData += "\n";
		}

		trgData += "\nX: " + to_string(lightPatchBuff[largestPatchIdx].posX) + "\nY: " + to_string(lightPatchBuff[largestPatchIdx].posY);

		ofstream trgFile("debug/trg_" + to_string(ImagesSnapped + 1) + ".txt");
		trgFile << trgData;
		trgFile.close();
	}

	for (int y = 0; y < imageHeight; y++)
		delete[] patchIndices[y];

	delete[] patchIndices;

	if (lightPatchNum > 0)
	{
		cout << to_string(lightPatchNum) + " light patches detected.\n";

		return lightPatchBuff[largestPatchIdx];
	}

	cout << "WARNING: No light patches detected!\n";

	return LightPatch();
}

// CAMERA FUNCTIONS //

bool HASP23_CameraSnap(unsigned char* imageBuff, int imageSize)
{
	ASIStartExposure(CameraIdx, ASI_FALSE);

	usleep(CaptureDelay * 1000000);

	ASI_EXPOSURE_STATUS status = ASI_EXP_WORKING;
	int iters = 0;

	while (status == ASI_EXP_WORKING)
	{
		ASIGetExpStatus(CameraIdx, &status);

		iters++;

		if (iters >= CameraSnapTimeout)
		{
			cout << "ERROR: Camera snap timeout!\n";

			return false;
		}
	}

	if (status == ASI_EXP_SUCCESS)
	{
		ASIGetDataAfterExp(CameraIdx, imageBuff, imageSize);

		return true;
	}

	cout << "ERROR: Camera snap failure!\n";

	return false;
}

long HASP23_CameraGetTemp()
{
	long temp;
	ASI_BOOL bAuto;
	ASIGetControlValue(CameraIdx, ASI_TEMPERATURE, &temp, &bAuto);

	return temp / 10;
}

void HASP23_CameraClose()
{
	ASIStopExposure(CameraIdx);
	ASICloseCamera(CameraIdx);
}

bool HASP23_CameraInit(int* imageWidth, int* imageHeight, int* imageType, int* imageBin)
{
	ASI_CAMERA_INFO camInfo;
	
	string bayer[4] = {"RG", "BG", "GR", "GB"};
	char buf[128] = {0};

	int cameraNum = ASIGetNumOfConnectedCameras();

	if (cameraNum <= 0)
	{
		cout << "ERROR: No camera connected!\n";
		
		return false;
	}

	cout << "Connected cameras:\n";

	for (int i = 0; i < cameraNum; i++)
	{
		ASIGetCameraProperty(&camInfo, i);

		cout << "\t" + to_string(i) + " - \"" + camInfo.Name + "\"\n";
	}

	if ((CameraIdx >= cameraNum) || (CameraIdx < 0))
	{
		cout << "ERROR: Invalid picked camera idx!\n";

		return false;
	}

	cout << "\nAccessing camera " + to_string(CameraIdx) + "...\n";
	
	ASIGetCameraProperty(&camInfo, 0);

	if (ASIOpenCamera(CameraIdx) != ASI_SUCCESS)
	{
		cout << "ERROR: Unable to access camera! Ensure you are the root user.\n";

		return false;
	}

	ASIInitCamera(CameraIdx);

	cout << "\nCamera \"" + string(camInfo.Name) + "\" information:\n";

	int imageWidthMax = camInfo.MaxWidth;
	int imageHeightMax =  camInfo.MaxHeight;

	cout << "\tResolution: " + to_string(imageWidthMax) + " x " + to_string(imageHeightMax) + "\n";

	cout << "\tColor: ";

	if (camInfo.IsColorCam)
		cout << "Bayer pattern: " + bayer[camInfo.BayerPattern] + "\n";
	else
		cout << "Mono\n";
	
	int ctrlnum;
	ASIGetNumOfControls(CameraIdx, &ctrlnum);
	ASI_CONTROL_CAPS ctrlcap;

	cout << "\nAvailable controls:\n";

	for (int i = 0; i < ctrlnum; i++)
	{
		ASIGetControlCaps(CameraIdx, i, &ctrlcap);
			
		cout << "\t" + string(ctrlcap.Name) + "\n";
	}

	cout << "\n";

	switch (ImageFormat)
	{
	case 0:
		cout << "Picked image format:\n\tSize: " + to_string(imageWidthMax) + " x " + to_string(imageHeightMax) + "\n\tBIN: 1\n\tImgType: raw8\n";

		ASISetROIFormat(CameraIdx, imageWidthMax, imageHeightMax, 1, ASI_IMG_RAW8);
		*imageWidth = imageWidthMax;
		*imageHeight = imageHeightMax;
		*imageBin = 1;
		*imageType = ASI_IMG_RAW8;

		break;

	case 1:
		cout << "Picked image format:\n\tSize: " + to_string(imageWidthMax) + " x " + to_string(imageHeightMax) + "\n\tBIN: 1\n\tImgType: raw16\n";

		ASISetROIFormat(CameraIdx, imageWidthMax, imageHeightMax, 1, ASI_IMG_RAW16);
		*imageWidth = imageWidthMax;
		*imageHeight = imageHeightMax;
		*imageBin = 1;
		*imageType = ASI_IMG_RAW16;

		break;

	case 2:
		cout << "Picked image format:\n\tSize: 1920 x 1080\n\tBIN: 1\n\tImgType: raw8\n";

		ASISetROIFormat(CameraIdx, 1920, 1080, 1, ASI_IMG_RAW8);
		*imageWidth = 1920;
		*imageHeight = 1080;
		*imageBin = 1;
		*imageType = ASI_IMG_RAW8;

		break;

	case 3:
		cout << "Picked image format:\n\tSize: 1920 x 1080\n\tBIN: 1\n\tImgType: raw16\n";

		ASISetROIFormat(CameraIdx, 1920, 1080, 1, ASI_IMG_RAW16);
		*imageWidth = 1920;
		*imageHeight = 1080;
		*imageBin = 1;
		*imageType = ASI_IMG_RAW16;

		break;

	case 4:
		cout << "Picked image format:\n\tSize: 320 x 240\n\tBIN: 2\n\tImgType: raw8\n";

		ASISetROIFormat(CameraIdx, 320, 240, 2, ASI_IMG_RAW8);
		*imageWidth = 320;
		*imageHeight = 240;
		*imageBin = 2;
		*imageType = ASI_IMG_RAW8;

		break;

	default:
		cout << "WARNING: Invalid picked image format! Using option 0...\n";

		ASISetROIFormat(CameraIdx, imageWidthMax, imageHeightMax, 1, ASI_IMG_RAW8);
		*imageWidth = imageWidthMax;
		*imageHeight = imageHeightMax;
		*imageBin = 1;
		*imageType = ASI_IMG_RAW8;

		break;
	}

	ASISetControlValue(CameraIdx, ASI_EXPOSURE, ExposureTime * 1000, ASI_FALSE);
	ASISetControlValue(CameraIdx, ASI_BANDWIDTHOVERLOAD, 40, ASI_FALSE);

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
