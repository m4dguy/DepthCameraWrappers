#include "Kinect2Grabber.h"


Kinect2Grabber::Kinect2Grabber(bool init)
{
	if (init)
	{
		if (initialize())
			printf("Successfully connected to Kinect v2 device.\n");
		else
			printf("Unable to find Kinect v2 device.\n");
	}
}

Kinect2Grabber::~Kinect2Grabber()
{
	if (sensor)
	{
		sensor->Close();
		sensor->Release();
	}

	if (depthBuffer)
		delete depthBuffer;

	if (colourBuffer)
		delete colourBuffer;
}


bool Kinect2Grabber::initialize()
{
	IFrameDescription* description;
	GetDefaultKinectSensor(&sensor);

	if (!sensor)
		return false;
	
	sensor->Open();

	///depth
	depthFrameSource = NULL;
	sensor->get_DepthFrameSource(&depthFrameSource);
	depthFrameSource->OpenReader(&depthFrameReader);

	if (!depthFrameSource)
		return false;

	if (!depthFrameSource)
		return false;

	depthFrameSource->get_FrameDescription(&description);
	description->get_Width(&depthWidth);
	description->get_Height(&depthHeight);



	///colour
	colourFrameSource = NULL;
	sensor->get_ColorFrameSource(&colourFrameSource);
	colourFrameSource->OpenReader(&colourFrameReader);

	if (!colourFrameSource)
		return false;

	if (!colourFrameSource)
		return false;

	colourFrameSource->get_FrameDescription(&description);
	description->get_Width(&colourWidth);
	description->get_Height(&colourHeight);
	


	///IR
	IRFrameSource = NULL;
	sensor->get_InfraredFrameSource(&IRFrameSource);
	IRFrameSource->OpenReader(&IRFrameReader);

	if (!IRFrameSource)
		return false;

	if (!IRFrameSource)
		return false;

	IRFrameSource->get_FrameDescription(&description);
	description->get_Width(&IRWidth);
	description->get_Height(&IRHeight);

	bufferSizeDepth = depthWidth * depthHeight;
	bufferSizeColour = colourWidth * colourHeight;

	colourBuffer = new RGBQUAD[bufferSizeColour];
	depthBuffer = new UINT16[bufferSizeDepth];

	return true;
}


bool Kinect2Grabber::getColourImage(cv::Mat& dst)
{
	HRESULT hr = -1;
	hr = colourFrameReader->AcquireLatestFrame(&colourFrame);
	if (SUCCEEDED(hr))
	{
		hr = colourFrame->CopyConvertedFrameDataToArray(bufferSizeColour * sizeof(RGBQUAD), reinterpret_cast<BYTE*>(colourBuffer), ColorImageFormat_Bgra);
		if (SUCCEEDED(hr))
		{
			cv::Mat colourFrame(colourHeight, colourWidth, CV_8UC4, reinterpret_cast<void*>(colourBuffer));
			cv::flip(colourFrame, dst, 2);
			dst.resize(colourHeight, colourWidth);
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
	
	if (colourFrame)
	{
		colourFrame->Release();
		colourFrame = NULL;
	}

	return true;
}


bool Kinect2Grabber::getDepthImage(cv::Mat& dst) 
{
	HRESULT hr = -1;

	hr = depthFrameReader->AcquireLatestFrame(&depthFrame);
	if (SUCCEEDED(hr))
	{
		hr = depthFrame->CopyFrameDataToArray(bufferSizeDepth, depthBuffer);
		if (SUCCEEDED(hr))
		{
			dst = cv::Mat(depthHeight, depthWidth, CV_16U, depthBuffer);
			cv::flip(dst, dst, 2);
			dst.resize(depthHeight, depthWidth);
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}

	if (depthFrame) {
		depthFrame->Release();
		depthFrame = NULL;
	}

	return true;
}

bool Kinect2Grabber::getValidDepthMask(cv::Mat& dst)
{
	bool success = getDepthImage(dst);

	if (!success)
		return false;
	
	cv::threshold(dst, dst, 1, 255, CV_THRESH_BINARY);
	return true;
}


int Kinect2Grabber::getDepthImageWidth() const
{
	return depthWidth;
}


int Kinect2Grabber::getDepthImageHeight() const
{
	return depthHeight;
}


int Kinect2Grabber::getColourImageWidth() const
{
	return colourWidth;
}


int Kinect2Grabber::getColourImageHeight() const
{
	return colourHeight;
}
