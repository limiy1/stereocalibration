/* This is sample from the OpenCV book. The copyright notice is below */

/* *************** License:**************************
   Modified from the orignal version:

   BOOK: It would be nice if you cited it:
   Learning OpenCV: Computer Vision with the OpenCV Library
     by Gary Bradski and Adrian Kaehler
     Published by O'Reilly Media, October 3, 2008

   AVAILABLE AT:
     http://www.amazon.com/Learning-OpenCV-Computer-Vision-Library/dp/0596516134
     Or: http://oreilly.com/catalog/9780596516130/
     ISBN-10: 0596516134 or: ISBN-13: 978-0596516130
   ************************************************** */

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "projectiveDevice.h"

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

using namespace cv;
using namespace std;

static int print_help()
{
    cout <<
            " Given a list of chessboard images, the number of corners (nx, ny)\n"
            " on the chessboards, and a flag: useCalibrated for \n"
            "   calibrated (0) or\n"
            "   uncalibrated \n"
            "     (1: use cvStereoCalibrate(), 2: compute fundamental\n"
            "         matrix separately) stereo. \n"
            " Calibrate the cameras and display the\n"
            " rectified results along with the computed disparity images.   \n" << endl;
    cout << "Usage:\n ./stereo_calib -w=<board_width default=9> -h=<board_height default=6> -s=<square_size default=1.0> <image list XML/YML file default=../data/stereo_calib.xml>\n" << endl;
    return 0;
}

//These write and read functions must be defined for the serialization in FileStorage to work
void write(cv::FileStorage& fs, const std::string&, const DeviceResult& dr)
{
    dr.write(fs);
}

static void
StereoCalib(const vector<string>& imagelist, Size boardSize, float squareSize, bool displayCorners = false, bool useCalibrated=true, bool showRectified=true)
{

    if( imagelist.size() % 2 != 0 )
    {
        cout << "Error: the image list contains odd (non-even) number of elements\n";
        return;
    }

    const int maxScale = 2;
    // ARRAY AND VECTOR STORAGE:

    vector<vector<Point2f> > allimgcorner[2];
    vector<Point2f> tmpcorners[2];
    cv::Size imageSize[2];

    vector<vector<Point3f> > objectPoints;

    int i, j, k, nimages = (int)imagelist.size()/2;


    for( i = j = 0; i < nimages; i++ )
    {
    	tmpcorners[0].clear();
    	tmpcorners[1].clear();
        for( k = 0; k < 2; k++ )
        {
            const string& filename = imagelist[i*2+k];
            Mat img = imread(filename, 0);
            if(img.empty())
            {
            	cout<<"Fail to read image " << filename<<endl;
            	break;
            }
            if( imageSize[k] == Size() )
                imageSize[k] = img.size();
            else if( img.size() != imageSize[k] )
            {
                cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
                break;
            }
            bool found = false;


            for( int scale = 1; scale <= maxScale; scale++ )
            {
                Mat timg;
                if( scale == 1 )
                    timg = img;
                else
                    resize(img, timg, Size(), scale, scale);
                found = findChessboardCorners(timg, boardSize, tmpcorners[k],
                    CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
                if( found )
                {
                    if( scale > 1 )
                    {
                        Mat cornersMat(tmpcorners[k]);
                        cornersMat *= 1./scale;
                    }
                    break;
                }
            }
            if( displayCorners )
            {
                cout << filename << endl;
                Mat cimg, cimg1;
                cvtColor(img, cimg, COLOR_GRAY2BGR);
                drawChessboardCorners(cimg, boardSize, tmpcorners[k], found);
                double sf = 640./MAX(img.rows, img.cols);
                resize(cimg, cimg1, Size(), sf, sf);
                imshow("corners", cimg1);
                char c = (char)waitKey(500);
                if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
                    exit(-1);
            }
            else
                putchar('.');
            if( !found )
                break;
            cornerSubPix(img, tmpcorners[k], Size(11,11), Size(-1,-1),
                         TermCriteria(TermCriteria::COUNT+TermCriteria::EPS,
                                      30, 0.01));
        }
        if( k == 2 )
        {
        	allimgcorner[0].push_back(tmpcorners[0]);
        	allimgcorner[1].push_back(tmpcorners[1]);
            j++;
        }
    }
    cout << j << " pairs have been successfully detected.\n";
    nimages = j;
    if( nimages < 2 )
    {
        cout << "Error: too little pairs to run the calibration\n";
        return;
    }

    //Now, we have all imaged corners
    //Create world points
    objectPoints.resize(nimages);

    for( i = 0; i < nimages; i++ )
    {
        for( j = 0; j < boardSize.height; j++ )
            for( k = 0; k < boardSize.width; k++ )
                objectPoints[i].push_back(Point3f(k*squareSize, j*squareSize, 0));
    }

    cout << "Running device calibration ...\n";
    DeviceResult device[2];
    vector<float> perViewErrors;
    float meanErr, maxErr;
    for (int k = 0 ; k < 2 ; k++)
    {
    	device[k].calibrateDevice(objectPoints, allimgcorner[k], imageSize[k]);
    	device[k].computeReprojectionErrors(objectPoints, allimgcorner[k]);
    	cout << "RMS = "<<device[k].totalErr << endl;
    }

    cout << "Running stereo calibration ...\n";
    //Use the 0 device as base device to do stereocalibration
    device[0].setAsStereoOrigin();
    device[1].calibrateStereo(device[0], objectPoints, allimgcorner[0], allimgcorner[1]);

    //Save result
    cv::FileStorage fs( "calibrationresult.yaml", cv::FileStorage::WRITE );
    fs << "Device  0" << device[0];
    fs << "Device  1" << device[1];

}


static bool readStringList( const string& filename, vector<string>& l )
{
    l.resize(0);
    FileStorage fs(filename, FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != FileNode::SEQ )
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        l.push_back((string)*it);
    return true;
}

int main(int argc, char** argv)
{
    Size boardSize;
    string imagelistfn;
    bool showRectified;
    cv::CommandLineParser parser(argc, argv, "{w|9|}{h|6|}{s|1.0|}{nr||}{help||}{@input|../data/stereo_calib.xml|}");
    if (parser.has("help"))
        return print_help();
    showRectified = !parser.has("nr");
    imagelistfn = parser.get<string>("@input");
    boardSize.width = parser.get<int>("w");
    boardSize.height = parser.get<int>("h");
    float squareSize = parser.get<float>("s");
    if (!parser.check())
    {
        parser.printErrors();
        return 1;
    }
    vector<string> imagelist;
    bool ok = readStringList(imagelistfn, imagelist);
    if(!ok || imagelist.empty())
    {
        cout << "can not open " << imagelistfn << " or the string list is empty" << endl;
        return print_help();
    }

    StereoCalib(imagelist, boardSize, squareSize, false, true, showRectified);
    return 0;
}
