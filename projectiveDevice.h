/*
 * projectiveDevice.h
 *
 *  Created on: Apr 28, 2016
 *      Author: liming
 */

#ifndef PROJECTIVEDEVICE_H_
#define PROJECTIVEDEVICE_H_

#include "opencv2/opencv.hpp"
using namespace std;

class DeviceResult
{
public:
	int calibrationFlags;                          /**< Flag parameter for openCV function >*/
	cv::Mat intrinsicMatrix;
	cv::Mat distorsionMatrix;
	cv::Mat R, T, E, F;
	vector< cv::Mat > rotations;                /**< No type specifying for this two matrix, or calibrateCamera will not work > */
	vector< cv::Mat > translations;
	vector<float> perViewErrors;
	float totalErr;
	float maxErr;

	DeviceResult();
	void calibrateDevice(const vector< vector<cv::Point3f> > &worldPoints3D, const vector< vector<cv::Point2f> > &imgPoints2D, cv::Size imgSize);
	void computeReprojectionErrors(const vector< vector<cv::Point3f> > &worldPoints3D,
			const vector< vector<cv::Point2f> > &imgPoints2D);

	void setAsStereoOrigin();
	void calibrateStereo(const DeviceResult& baseDevice,
			const vector< vector<cv::Point3f> > &worldPoints3D,
			const vector< vector<cv::Point2f> > &imgPoints2D_baseDevice,
			const vector< vector<cv::Point2f> > &imgPoints2D_thisDevice);
	void write(cv::FileStorage& fs) const;

};

#endif /* PROJECTIVEDEVICE_H_ */
