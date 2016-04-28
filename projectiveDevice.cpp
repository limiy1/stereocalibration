/*
 * projectiveDevice.cpp
 *
 *  Created on: Apr 28, 2016
 *      Author: liming
 */

#include "projectiveDevice.h"

DeviceResult::DeviceResult()
: maxErr(0),
  totalErr(0)
{
	calibrationFlags = CV_CALIB_FIX_K3 | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5 | CV_CALIB_FIX_K6;
	calibrationFlags = calibrationFlags | CV_CALIB_FIX_INTRINSIC;
}

void DeviceResult::calibrateDevice(const vector< vector<cv::Point3f> > &worldPoints3D, const vector< vector<cv::Point2f> > &imgPoints2D, cv::Size imgSize)
{
	intrinsicMatrix = cv::Mat::eye(3, 3, CV_64F);
	distorsionMatrix = cv::Mat::zeros(4, 1, CV_64F);

	/**
	 * according to samuel's method, the points coordinates are all in float,
	 * but just use float ( vector< vector<cv::Point3f> >  ) to do the calibration
	 * for calibration result (intrinsicMatrix, distorsionMatrix), they are float as well
	 */

	cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, DBL_EPSILON);
	cv::calibrateCamera(worldPoints3D,
			imgPoints2D,
			imgSize,
			intrinsicMatrix, distorsionMatrix,
			rotations, translations,
			calibrationFlags,
			criteria);

	cout<<intrinsicMatrix<<endl;
	cout<<distorsionMatrix<<endl;
}

void DeviceResult::computeReprojectionErrors(const vector< vector<cv::Point3f> > &worldPoints3D,
			const vector< vector<cv::Point2f> > &imgPoints2D)
{
	vector<cv::Point2f> imagePoints2;
	int i, totalPoints = 0;
	float sumErr = 0, err;
	maxErr = 0;

	perViewErrors.resize(worldPoints3D.size(), 0);

	for( i = 0; i < (int)worldPoints3D.size(); ++i )
	{
		projectPoints( cv::Mat(worldPoints3D[i]), rotations[i], translations[i], intrinsicMatrix,  // project
							 distorsionMatrix, imagePoints2);

        for (int j = 0; j < (int)worldPoints3D[i].size(); j++) {
        	float ptErr = cv::norm(imgPoints2D[i][j] - imagePoints2[j]);
        	float err2 = ptErr*ptErr;

        	perViewErrors[i] += err2;
        	sumErr += err2;
        	totalPoints++;

            if (ptErr > maxErr) {
            	maxErr = ptErr;
            }
        }

        perViewErrors[i] = std::sqrt(perViewErrors[i]/(int)worldPoints3D[i].size());
	}

	totalErr = std::sqrt(sumErr/totalPoints);              // calculate the arithmetical mean
}


void DeviceResult::calibrateStereo(const DeviceResult& baseDevice,
					const vector< vector<cv::Point3f> > &worldPoints3D,
					const vector< vector<cv::Point2f> > &imgPoints2D_baseDevice,
					const vector< vector<cv::Point2f> > &imgPoints2D_thisDevice)
{
	//Point2f - Point2f
	assert(worldPoints3D.size() == imgPoints2D_baseDevice.size());
	assert(worldPoints3D.size() == imgPoints2D_thisDevice.size());

	cout<<baseDevice.intrinsicMatrix<<endl;
	cout<<baseDevice.distorsionMatrix<<endl;
	cout<<baseDevice.R<<endl;
	cout<<baseDevice.T<<endl;
	cout<<baseDevice.E<<endl;
	cout<<baseDevice.F<<endl;
	cout<<intrinsicMatrix<<endl;
	cout<<distorsionMatrix<<endl;

    cv::stereoCalibrate(worldPoints3D, imgPoints2D_baseDevice, imgPoints2D_thisDevice,
    		baseDevice.intrinsicMatrix, baseDevice.distorsionMatrix, intrinsicMatrix, distorsionMatrix,
            cv::Size(0, 0), R, T, E, F, calibrationFlags,
            cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,100,1e-6));

}

void DeviceResult::setAsStereoOrigin()
{
	//Current Device is at origne
	R = cv::Mat::eye(3,3, CV_64F);
	T = cv::Mat::zeros(3,1,CV_64F);
	E = cv::Mat::zeros(3,3, CV_64F);
	F = cv::Mat::zeros(3,3, CV_64F);
}

void DeviceResult::write(cv::FileStorage& fs) const
{
	fs << "{";

    if( calibrationFlags != 0 )
    {
      char buf[1024];
      //vona        sprintf( buf, "flags: %s%s%s%s",
      sprintf( buf, "flags: %s%s%s%s%s%s%s%s%s%s%s%s%s%s%s",
    		calibrationFlags & CV_CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
    		calibrationFlags & CV_CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
    		calibrationFlags & CV_CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
    		calibrationFlags & CV_CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "",
    		calibrationFlags & CV_CALIB_FIX_FOCAL_LENGTH ? "+fix_focal_length" : "",
    		calibrationFlags & CV_CALIB_FIX_K1 ? "+fix_k1" : "",
    		calibrationFlags & CV_CALIB_FIX_K2 ? "+fix_k2" : "",
    		calibrationFlags & CV_CALIB_FIX_K3 ? "+fix_k3" : "",
    	    calibrationFlags & CV_CALIB_FIX_K4 ? "+fix_k4" : "",
    	    calibrationFlags & CV_CALIB_FIX_K5 ? "+fix_k5" : "",
    	    calibrationFlags & CV_CALIB_FIX_K6 ? "+fix_k6" : "",
    	    calibrationFlags & CV_CALIB_RATIONAL_MODEL ? "+rational_model" : "",
    		calibrationFlags & (CV_CALIB_FIX_K1|CV_CALIB_FIX_K2|CV_CALIB_FIX_K3) ? "+zero_radial_dist" : "",

    		calibrationFlags & CV_CALIB_FIX_INTRINSIC ? "+(stereo)fix_intrinsic" : "",
    		calibrationFlags & CV_CALIB_SAME_FOCAL_LENGTH ? "+(stereo)same_focal_length" : ""); //vona
      cvWriteComment( *fs, buf, 0 );
    }
    fs << "flags" << calibrationFlags;

	fs << "cameraMatrix" << intrinsicMatrix;
	fs << "distortionCoeffs" << distorsionMatrix;

	//Save external parameters
	cv::Mat bigmat((int)rotations.size(), 6, rotations[0].type());
    if( !rotations.empty() && !translations.empty() )
    {
        CV_Assert(rotations[0].type() == translations[0].type());
        for( int i = 0; i < (int)rotations.size(); i++ )
        {
            cv::Mat r = bigmat(cv::Range(i, i+1), cv::Range(0,3));
            cv::Mat t = bigmat(cv::Range(i, i+1), cv::Range(3,6));

            CV_Assert(rotations[i].rows == 3 && rotations[i].cols == 1);
            CV_Assert(translations[i].rows == 3 && translations[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rotations[i].t();
            t = translations[i].t();
        }
        cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
    }

	fs << "extrParams" << bigmat;
	fs << "reprojErrs" << cv::Mat(perViewErrors).t();
	fs << "avgReprojErr" << totalErr;
	fs << "maxReprojErr" << maxErr;
	fs << "R"<< R;
	fs << "T" << T;
	fs << "E" << E;
	fs << "F" << F;
	fs << "}";
}


