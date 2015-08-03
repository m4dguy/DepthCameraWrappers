#include "Kinect1Grabber.h"


Kinect1Grabber::Kinect1Grabber(bool init, DWORD sleep) :
sleepTime(sleep)
{
	sensor = NULL;

	if (init)
	{
		if (initialize())
			printf("Successfully connected to Kinect v1 device.\n");
		else
			printf("Unable to find Kinect v1 device.\n");
	}
}

Kinect1Grabber::~Kinect1Grabber()
{
	if (sensor)
		sensor->NuiShutdown();
}


bool Kinect1Grabber::initialize()
{
	IplImage* ovImage = NULL;

	HRESULT hr = -1;

	int numSensors;
	if (NuiGetSensorCount(&numSensors) < 0 || numSensors < 1)
		return false;

	if (NuiCreateSensorByIndex(0, &sensor) < 0)
		return false;

	hr = sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);
	if (FAILED(hr))
		return false;

	DWORD ch, cw;
	NuiImageResolutionToSize(NUI_IMAGE_RESOLUTION_640x480, ch, cw);
	IRWidth = depthWidth = colourWidth = (int)cw;
	IRHeight = depthHeight = colourHeight = (int)ch;

	bufferSizeColour = colourWidth * colourHeight * sizeof(RGBQUAD);
	bufferSizeDepth = depthWidth * depthHeight * sizeof(BYTE);

	sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480,    // image type, image resolution
		0, 2, NULL,		// Image stream flags, e.g. near mode		// Number of frames to buffer  // Event handle
		&colourStream);

	if (FAILED(hr))
		return false;

	hr = sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, NUI_IMAGE_RESOLUTION_640x480,
		0, 2, NULL,
		&depthStream);

	if (FAILED(hr))
		return false;

	return (SUCCEEDED(hr));
}


bool Kinect1Grabber::getColourImage(cv::Mat& dst)
{
	HRESULT hr = -1;
	NUI_IMAGE_FRAME imageFrame;

	// Get next image stream frame
	hr = sensor->NuiImageStreamGetNextFrame(colourStream, sleepTime, &imageFrame);
	

	if (FAILED(hr))
		return false;

	// Lock frame texture to allow for copy
	INuiFrameTexture* pTexture = imageFrame.pFrameTexture;
	NUI_LOCKED_RECT lockedRect;
	pTexture->LockRect(0, &lockedRect, NULL, 0);

	// Check if image is valid
	if (lockedRect.Pitch != 0)
	{
		// Copy image information into buffer so it doesn't get overwritten later
		BYTE* buffer = lockedRect.pBits;
		cv::Mat colourFrame(colourWidth, colourHeight, CV_8UC4, reinterpret_cast<void*>(buffer));
		cv::flip(colourFrame, dst, 1);
	}
	
	// Unlock and release
	pTexture->UnlockRect(0);
	hr = sensor->NuiImageStreamReleaseFrame(colourStream, &imageFrame);
	return SUCCEEDED(hr);
}


bool Kinect1Grabber::getDepthImage(cv::Mat& dst)
{
	HRESULT hr = -1;
	NUI_IMAGE_FRAME imageFrame;

	hr = sensor->NuiImageStreamGetNextFrame(depthStream, 0, &imageFrame);

	if(FAILED(hr))
		return false;

	// Lock frame texture to allow for copy
	INuiFrameTexture* texture = imageFrame.pFrameTexture;
	NUI_LOCKED_RECT lockedRect;
	texture->LockRect(0, &lockedRect, NULL, 0);

	// Check if image is valid
	if (lockedRect.Pitch != 0)
	{
		// Copy image information into buffer
		BYTE* buffer = lockedRect.pBits;
		cv::Mat depthFrame(depthWidth, depthHeight, CV_16U, reinterpret_cast<void*>(buffer));
		cv::flip(depthFrame, dst, 1);
	}

	// Unlock texture and release frame
	texture->UnlockRect(0);
	hr = sensor->NuiImageStreamReleaseFrame(depthStream, &imageFrame);
	return SUCCEEDED(hr);
}

bool Kinect1Grabber::getValidDepthMask(cv::Mat& dst)
{
	bool success = getDepthImage(dst);

	if (!success)
		return false;

	cv::threshold(dst, dst, 1, 255, CV_THRESH_BINARY);
	return true;
}

