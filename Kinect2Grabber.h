#ifndef KINECT2GRABBER_H
#define KINECT2GRABBER_H

#include <opencv2/imgproc/imgproc.hpp>

#include <Kinect.h>

#include "Utils.h"

class Kinect2Grabber
{
public:

	/**
	* Constructor. Calls initialize().
	* @param init if set to true, initialize() will be called in the constructor. If not, this step has to be done maunally.
	* @see initialize()
	*
	*/
	Kinect2Grabber(bool init=true);
	
	/**
	* Cleans up all pointers and closes all connections safely.
	*/
	~Kinect2Grabber();


	/**
	* Initializes all pointers and opens up a connection to the Kinect v2 device.
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
	int getDepthImageWidth() const;

	/**
	* Get height of depth image.
	* @return depth image height.
	*/
	int getDepthImageHeight() const;

	/**
	* Get width of infrared image.
	* @return depth image width.
	*/
	int getIRImageWidth() const;

	/**
	* Get height of infrared image.
	* @return depth image height.
	*/
	int getIRImageHeight() const;

	/**
	* Get width of colour image.
	* @return depth image width.
	*/
	int getColourImageWidth() const;

	/**
	* Get height of colour image.
	* @return depth image height.
	*/
	int getColourImageHeight() const;

protected:	
	uint bufferSizeDepth, bufferSizeColour;
	int colourWidth, colourHeight;
	int depthWidth, depthHeight;
	int IRWidth, IRHeight;


	IKinectSensor* sensor;
	
	IDepthFrame* depthFrame;
	IDepthFrameReader* depthFrameReader;
	IDepthFrameSource* depthFrameSource;
	UINT16* depthBuffer;
	cv::Mat depthMat;

	IInfraredFrame* IRFrame;
	IInfraredFrameReader* IRFrameReader;
	IInfraredFrameSource* IRFrameSource;


	IColorFrame* colourFrame;
	IColorFrameReader* colourFrameReader;
	IColorFrameSource* colourFrameSource;
	RGBQUAD* colourBuffer;
	cv::Mat colourMat;
};
#endif