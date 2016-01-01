#ifndef KINECT2GRABBER_H
#define KINECT2GRABBER_H

#include <opencv2/imgproc/imgproc.hpp>

#include <Kinect.h>

#include "Utils.h"
#include "UtilsKinect.h"

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
	* Initializes all buffers and opens up a connection to the Kinect v2 device.
	*/
	bool initialize();

	/*
	* Get new image and point cloud data from the sensor.
	* @return true, if data retrieval successful.
	*/
	bool update();


	/**
	* Retrieves the colour image and copies it into the cv::Mat.
	* @param dst cv::Mat to copy the data into.
	*/
	void getColourImage(cv::Mat& dst);

	/**
	* Retrieves the depth image and copies it into the cv::Mat.
	* @param dst cv::Mat to copy the data into.
	*/
	void getDepthImage(cv::Mat& dst);
	
	/**
	* Retrieves the infrared image and copies it into the cv::Mat.
	* @param dst cv::Mat to copy the data into.
	*/
	void getIRImage(cv::Mat& dst);


	/**
	* Get wdith of depth image.
	* @return depth image width.
	*/
	int getDepthImageWidth() const { return _depthWidth; };

	/**
	* Get height of depth image.
	* @return depth image height.
	*/
	int getDepthImageHeight() const { return _depthHeight; };

	/**
	* Get width of infrared image.
	* @return depth image width.
	*/
	int getIRImageWidth() const { return _IRWidth; };

	/**
	* Get height of infrared image.
	* @return depth image height.
	*/
	int getIRImageHeight() const { return _IRHeight; };

	/**
	* Get width of colour image.
	* @return depth image width.
	*/
	int getColourImageWidth() const { return _colourWidth; };

	/**
	* Get height of colour image.
	* @return depth image height.
	*/
	int getColourImageHeight() const{ return _colourHeight; }

	/**
	* Transform a point in the colour image to its corresponding point in depth image.
	* @param colourCoord the colour image coordinate
	* @return the transformed coordinate
	*/
	cv::Point colourToDepthCoordinate(const cv::Point& colourCoord) const;

	/**
	* Transform a point in the depth image to its corresponding point in colour image.
	* @param depthCoord the depth image coordinate
	* @return the transformed coordinate
	*/
	cv::Point depthToColourCoordinate(const cv::Point& depthCoord) const;



protected:	
	/*
	* Open a connection to colourframereadaer and get image information.
	* Initialize necessary buffers and image dimensions.
	* @return true, if initialization successful.
	*/
	bool initializeColourFrameBuffer();

	/*
	* Open a connection to depthframereader and get image information.
	* Initialize necessary buffers and image dimensions.
	* @return true, if initialization successful.
	*/
	bool initializeDepthFrameBuffer();

	/*
	* Open a connection to infraredframereader and get image information.
	* Initialize necessary buffers and image dimensions.
	* @return true, if initialization successful.
	*/
	bool initializeIRFrameBuffer();

	/*
	* Create coordinate mapper and store coordinate table.
	* @return true, if initialization successful.
	*/
	bool initializeCoordinateMapper();

	/*
	* Update Mat object holding the information of the current colour frame.
	* @return true, if update successful
	* @see update()
	*/
	bool updateColourMat();

	/*
	* Update Mat object holding the information of the current depth frame.
	* @return true, if update successful
	* @see update()
	*/
	bool updateDepthMat();


	IKinectSensor* _sensor;
	IMultiSourceFrameReader* _multiSourceFrameReader;
	IMultiSourceFrame* _multiSourceFrame;

	IColorFrame* _colourFrame;
	RGBQUAD* _colourBuffer;
	cv::Mat _colourMat;

	IDepthFrame* _depthFrame;
	UINT16* _depthBuffer;
	cv::Mat _depthMat;

	ICoordinateMapper* _cmapper;
	ColorSpacePoint* _depth2ColourMap;
	DepthSpacePoint* _colour2DepthMap;

	uint _bufferSizeDepth, _bufferSizeColour, _bufferSizeIR;
	int _colourWidth, _colourHeight;
	int _depthWidth, _depthHeight;
	int _IRWidth, _IRHeight;

};
#endif