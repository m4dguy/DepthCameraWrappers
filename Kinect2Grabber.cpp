#include "Kinect2Grabber.h"


Kinect2Grabber::Kinect2Grabber(bool init)
{
	_sensor = NULL;
	_cmapper = NULL;
	_depthBuffer = NULL;
	_colourBuffer = NULL;
	_colour2DepthMap = NULL;
	_depth2ColourMap = NULL;

	_colourFrame = NULL;
	_depthFrame = NULL;
	_multiSourceFrameReader = NULL;
	_multiSourceFrame = NULL;

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
	if (_sensor)
	{
		_sensor->Close();
		_sensor->Release();
		_sensor = NULL;
	}

	UtilsKinect::safeRelease(_cmapper);
	
	if (_depth2ColourMap)
	{
		delete _depth2ColourMap;
		_depth2ColourMap = NULL;
	}

	if (_colour2DepthMap)
	{
		delete _colour2DepthMap;
		_colour2DepthMap = NULL;
	}

	if (_depthBuffer)
	{
		delete _depthBuffer;
		_depthBuffer = NULL;
	}

	if (_colourBuffer)
	{
		delete _colourBuffer;
		_colourBuffer = NULL;
	}
}


bool Kinect2Grabber::initialize()
{
	HRESULT hr = -1;
	bool success = true;
	GetDefaultKinectSensor(&_sensor);

	if (!_sensor)
		return false;
	
	_sensor->Open();

	hr = _sensor->OpenMultiSourceFrameReader(
		FrameSourceTypes::FrameSourceTypes_Color | FrameSourceTypes::FrameSourceTypes_Depth,
		&_multiSourceFrameReader);

	if (FAILED(hr))
		return false;

	success &= initializeColourFrameBuffer();
	success &= initializeDepthFrameBuffer();
	//success &= initializeIRFrameBuffer();
	success &= initializeCoordinateMapper();

	if (!success)
		return false;

	return true;
}


bool Kinect2Grabber::update()
{
	bool success = true;

	if (!_multiSourceFrameReader)
		return false;

	HRESULT hr = _multiSourceFrameReader->AcquireLatestFrame(&_multiSourceFrame);
	if (FAILED(hr))
		return false;


	success &= updateColourMat();
	success &= updateDepthMat();

	UtilsKinect::safeRelease(_multiSourceFrame);

	return success;
}


void Kinect2Grabber::getColourImage(cv::Mat& dst)
{
	_colourMat.copyTo(dst);
}


void Kinect2Grabber::getDepthImage(cv::Mat& dst) 
{
	_depthMat.copyTo(dst);
}


cv::Point Kinect2Grabber::colourToDepthCoordinate(const cv::Point& colourCoord) const
{
	ColorSpacePoint p = _depth2ColourMap[colourCoord.y*_colourWidth + colourCoord.x];

	cv::Point res;
	res.x = (int)floor(p.X + 0.5);
	res.y = (int)floor(p.Y + 0.5);

	return res;
}


cv::Point Kinect2Grabber::depthToColourCoordinate(const cv::Point& depthCoord) const
{
	DepthSpacePoint p = _colour2DepthMap[depthCoord.y*_depthWidth + depthCoord.x];

	cv::Point res;
	res.x = (int)floor(p.X + 0.5);
	res.y = (int)floor(p.Y + 0.5);

	return res;
}


bool Kinect2Grabber::initializeColourFrameBuffer()
{
	IFrameDescription* description;
	IColorFrameSource* colourFrameSource;
	IColorFrameReader* colourFrameReader;

	_sensor->get_ColorFrameSource(&colourFrameSource);
	colourFrameSource->OpenReader(&colourFrameReader);

	if (!colourFrameSource)
		return false;

	if (!colourFrameSource)
		return false;

	colourFrameSource->get_FrameDescription(&description);
	description->get_Width(&_colourWidth);
	description->get_Height(&_colourHeight);

	_bufferSizeColour = _colourWidth * _colourHeight;
	_colourBuffer = new RGBQUAD[_bufferSizeColour];

	return true;
}


bool Kinect2Grabber::updateColourMat()
{
	bool success = true;
	HRESULT hr = -1;

	IColorFrameReference* colorFrameReference = NULL;
	hr = _multiSourceFrame->get_ColorFrameReference(&colorFrameReference);
	
	if (FAILED(hr))
		return false;

	hr = colorFrameReference->AcquireFrame(&_colourFrame);
	if (SUCCEEDED(hr))
	{
		hr = _colourFrame->CopyConvertedFrameDataToArray(_bufferSizeColour * sizeof(RGBQUAD), reinterpret_cast<BYTE*>(_colourBuffer), ColorImageFormat_Bgra);
		if (SUCCEEDED(hr))
		{
			_colourMat = cv::Mat(_colourHeight, _colourWidth, CV_8UC4, reinterpret_cast<void*>(_colourBuffer));
			cv::flip(_colourMat, _colourMat, 2);
		}
		else
		{
			success = false;
		}
	}
	else
	{
		success = false;
	}

	UtilsKinect::safeRelease(_colourFrame);

	return success;
}


bool Kinect2Grabber::updateDepthMat()
{
	bool success = true;
	HRESULT hr = -1;

	IDepthFrameReference* depthFrameReference = NULL;
	hr = _multiSourceFrame->get_DepthFrameReference(&depthFrameReference);
	if (FAILED(hr))
		return false;

	hr = depthFrameReference->AcquireFrame(&_depthFrame);
	if (SUCCEEDED(hr))
	{
		hr = _depthFrame->CopyFrameDataToArray(_bufferSizeDepth, _depthBuffer);
		if (SUCCEEDED(hr))
		{
			_depthMat = cv::Mat(_depthHeight, _depthWidth, CV_16U, _depthBuffer);
			cv::flip(_depthMat, _depthMat, 2);
		}
		else
		{
			success = false;
		}
	}
	else
	{
		success = false;
	}

	UtilsKinect::safeRelease(_depthFrame);

	return success;
}


bool Kinect2Grabber::initializeDepthFrameBuffer()
{
	IFrameDescription* description;
	IDepthFrameReader* depthFrameReader;
	IDepthFrameSource* depthFrameSource;

	_sensor->get_DepthFrameSource(&depthFrameSource);
	depthFrameSource->OpenReader(&depthFrameReader);

	if (!depthFrameSource)
		return false;

	if (!depthFrameSource)
		return false;

	depthFrameSource->get_FrameDescription(&description);
	description->get_Width(&_depthWidth);
	description->get_Height(&_depthHeight);

	_bufferSizeDepth = _depthWidth * _depthHeight;
	_depthBuffer = new UINT16[_bufferSizeDepth];

	return true;
}


bool Kinect2Grabber::initializeIRFrameBuffer()
{
	IFrameDescription* description;
	IInfraredFrameReader* IRFrameReader;
	IInfraredFrameSource* IRFrameSource;

	_sensor->get_InfraredFrameSource(&IRFrameSource);
	IRFrameSource->OpenReader(&IRFrameReader);

	if (!IRFrameSource)
		return false;

	if (!IRFrameSource)
		return false;

	IRFrameSource->get_FrameDescription(&description);
	description->get_Width(&_IRWidth);
	description->get_Height(&_IRHeight);

	return true;
}


bool Kinect2Grabber::initializeCoordinateMapper()
{
	HRESULT hr = -1;
	hr = _sensor->get_CoordinateMapper(&_cmapper);

	if (FAILED(hr))
		return false;

	_depth2ColourMap = new ColorSpacePoint[_bufferSizeColour];
	_colour2DepthMap = new DepthSpacePoint[_bufferSizeDepth];
	
	hr = _cmapper->MapDepthFrameToColorSpace(_bufferSizeDepth, _depthBuffer, _bufferSizeColour, _depth2ColourMap);
	hr = _cmapper->MapColorFrameToDepthSpace(_bufferSizeDepth, _depthBuffer, _bufferSizeDepth, _colour2DepthMap);

	return SUCCEEDED(hr);
}
