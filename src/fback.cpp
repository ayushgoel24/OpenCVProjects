#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>

using namespace cv;
using namespace std;

static void help()
{
    cout <<
            "\nThis program demonstrates dense optical flow algorithm by Gunnar Farneback\n"
            "Mainly the function: calcOpticalFlowFarneback()\n"
            "Call:\n"
            "./fback\n"
            "This reads from video camera 0\n" << endl;
}
static void drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step,double, const Scalar& color)
{
    std::cout<<flow.channels() <<std::endl;
    for(int y = 0; y < cflowmap.rows; y += step)
        for(int x = 0; x < cflowmap.cols; x += step)
        {
            const Point2f& fxy = flow.at<Point2f>(y, x);
            line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)), color);
            circle(cflowmap, Point(x,y), 2, color, -1);
        }
}


static void drawMotionToColor(const Mat& flow,  cv::Mat &bgr)
{
    //extraxt x and y channels
    cv::Mat xy[2]; //X,Y
    cv::split(flow, xy);

    //calculate angle and magnitude
    cv::Mat magnitude, angle;
    cv::cartToPolar(xy[0], xy[1], magnitude, angle, true);

    //translate magnitude to range [0;1]
    double mag_max;
    cv::minMaxLoc(magnitude, 0, &mag_max);
    magnitude.convertTo(magnitude, -1, 1.0 / mag_max);

    //build hsv image
    cv::Mat _hsv[3], hsv;
    _hsv[0] = angle;
    _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
    _hsv[2] = magnitude;
    cv::merge(_hsv, 3, hsv);

    //convert to BGR and show
   /* cv::Mat bgr*/;//CV_32FC3 matrix
    cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);

}

//https://github.com/xingdi-eric-yuan/optical-flow
//https://github.com/davidstutz/flow-io-opencv
//https://docs.opencv.org/3.3.1/d7/d8b/tutorial_py_lucas_kanade.html
//https://people.csail.mit.edu/celiu/OpticalFlow/
//https://stackoverflow.com/questions/10161351/opencv-how-to-plot-velocity-vectors-as-arrows-in-using-single-static-image
int main(int argc, char** argv)
{
    cv::CommandLineParser parser(argc, argv, "{help h||}");
    if (parser.has("help"))
    {
        help();
        return 0;
    }

    VideoCapture vid;
    //vid.open("/home/behnam/home.mp4");
	vid.open(1);

//    VideoCapture cap(0);
//    help();
//    if( !cap.isOpened() )
//        return -1;

    Mat flow, cflow,bgr ,frame;
    UMat gray, prevgray, uflow;
    namedWindow("flow", 1);
    namedWindow("motiontoflow", 1);
	
	double pyr_scale=0.5;
	int levels= 3;
	int winsize=5;
	int iterations=3;
	int poly_n=5;
	double poly_sigma=1.2;

    for(;;)
    {
        //cap >> frame;
        vid >> frame;
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        if( !prevgray.empty() )
        {
            calcOpticalFlowFarneback(prevgray, gray, uflow, pyr_scale, levels, winsize, iterations, poly_n, poly_sigma, 0);
            cvtColor(prevgray, cflow, COLOR_GRAY2BGR);
            uflow.copyTo(flow);
            drawOptFlowMap(flow, cflow, 15, 1.5, Scalar(0, 255, 0));
            drawMotionToColor(flow,  bgr);
            imshow("flow", cflow);
            imshow("motiontoflow", bgr);
        }
        if(waitKey(30)>=0)
            break;
        std::swap(prevgray, gray);
    }
    return 0;
}


/*


Lucas-Kanade Optical Flow in OpenCV

import numpy as np
import cv2
cap = cv2.VideoCapture('slow.flv')
# params for ShiTomasi corner detection
feature_params = dict( maxCorners = 100,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )
# Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (15,15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
# Create some random colors
color = np.random.randint(0,255,(100,3))
# Take first frame and find corners in it
ret, old_frame = cap.read()
old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
p0 = cv2.goodFeaturesToTrack(old_gray, mask = None, **feature_params)
# Create a mask image for drawing purposes
mask = np.zeros_like(old_frame)
while(1):
    ret,frame = cap.read()
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # calculate optical flow
    p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
    # Select good points
    good_new = p1[st==1]
    good_old = p0[st==1]
    # draw the tracks
    for i,(new,old) in enumerate(zip(good_new,good_old)):
        a,b = new.ravel()
        c,d = old.ravel()
        mask = cv2.line(mask, (a,b),(c,d), color[i].tolist(), 2)
        frame = cv2.circle(frame,(a,b),5,color[i].tolist(),-1)
    img = cv2.add(frame,mask)
    cv2.imshow('frame',img)
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break
    # Now update the previous frame and previous points
    old_gray = frame_gray.copy()
    p0 = good_new.reshape(-1,1,2)
cv2.destroyAllWindows()
cap.release()




//Dense Optical Flow in OpenCV

import cv2
import numpy as np
cap = cv2.VideoCapture("/home/behnam/Desktop/opencv-3.4.0/samples/data/vtest.avi")
ret, frame1 = cap.read()
prvs = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
hsv = np.zeros_like(frame1)
hsv[...,1] = 255
while(1):
    ret, frame2 = cap.read()
    next = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)
    flow = cv2.calcOpticalFlowFarneback(prvs,next, None, 0.5, 3, 15, 3, 5, 1.2, 0)
    mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
    hsv[...,0] = ang*180/np.pi/2
    hsv[...,2] = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)
    bgr = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
    cv2.imshow('frame2',bgr)
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break
    elif k == ord('s'):
        cv2.imwrite('opticalfb.png',frame2)
        cv2.imwrite('opticalhsv.png',bgr)
    prvs = next
cap.release()
cv2.destroyAllWindows()

*/
