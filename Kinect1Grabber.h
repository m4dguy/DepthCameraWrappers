#ifndef KINECT1GRABBER_H
#define KINECT1GRABBER_H

#include <opencv2/imgproc/imgproc.hpp>

#include <Windows.h>
#include <Ole2.h>

#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>

#include "Utils.h"


/**
* Wrapper for Kinect v1. Independent from OpenNI, only requires Kinect SDK.
* Very similar to Kinect2Grabber class. Based on a stripped down and adaptd KinectBridgeWithOpenCVBasics-D2D project.
* Make sure the Kinect Developer Toolkit browser is finds the camera before usage.
*
*/


class Kinect1Grabber
{
public:

	/**
	* Constructor. Calls initialize().
	* @param init if set to true, initialize() will be called in the constructor. If not, this step has to be done maunally.
	* @see initialize()
	*
	*/
	Kinect1Grabber(bool init=true, DWORD sleep=2);
	
	/**
	* Cleans up all pointers and closes all connections safely.
	*/
	~Kinect1Grabber();


	/**
	* Initializes all pointers and opens up a connection to the Kinect v1 device.
	*
	*/
	bool initialize();


	/**
	* Retrieves the colour image and copies it into the cv::Mat.
	* @param dst cv::Mat to copy the data into.
	*/
	bool getColourImage(cv::Mat& dst);

	/**
	* Retrieves the depth image and copies it into the cv::Mat.
	* @param dst cv::Mat to copy the data into.
	*/
	bool getDepthImage(cv::Mat& dst);
	
	/**
	* Retrieves the valid depth mask and copies it into the cv::Mat.
	* TODO: This is a stub! the mask will indicate that the whole image is valid!
	* @param dst cv::Mat to copy the data into.
	*/
	bool getValidDepthMask(cv::Mat& dst);
	
	/**
	* Retrieves the infrared image and copies it into the cv::Mat.
	* @param dst cv::Mat to copy the data into.
	*/
	bool getIRImage(cv::Mat& dst);


	/**
	* Get wdith of depth image.
	* @return depth image width.
	*/
	int getDepthImageWidth() const { return depthWidth; };

	/**
	* Get height of depth image.
	* @return depth image height.
	*/
	int getDepthImageHeight() const { return depthHeight; };

	/**
	* Get width of infrared image.
	* @return depth image width.
	*/
	int getIRImageWidth() const  { return IRWidth; };

	/**
	* Get height of infrared image.
	* @return depth image height.
	*/
	int getIRImageHeight() const { return IRHeight; };

	/**
	* Get width of colour image.
	* @return depth image width.
	*/
	int getColourImageWidth() const { return colourWidth; };

	/**
	* Get height of colour image.
	* @return depth image height.
	*/
	int getColourImageHeight() const { return colourHeight; };

protected:	
	DWORD sleepTime;

	INuiSensor* sensor;

	uint bufferSizeDepth, bufferSizeColour;

	int colourHeight, colourWidth;
	HANDLE colourStream;
	cv::Mat colourMat;

	
	int IRHeight, IRWidth;
	cv::Mat IRMat;

	int depthHeight, depthWidth;
	
	HANDLE depthStream;
	cv::Mat depthMat;
};
#endif