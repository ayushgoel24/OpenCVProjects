/*
 * HOG-KalmanFilter.cpp
 *
 *  Created on: Oct 24, 2012
 *      Author: behnam
 */

/*

	Mat statePre;           // predicted state (x'(k)):
							//    x(k)=A*x(k-1)+B*u(k)
	Mat statePost;          // corrected state (x(k)):
							//    x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
	Mat transitionMatrix;   // state transition matrix (A)
	Mat controlMatrix;      // control matrix (B)
							//   (it is not used if there is no control)
	Mat measurementMatrix;  // measurement matrix (H)
	Mat processNoiseCov;    // process noise covariance matrix (Q)
	Mat measurementNoiseCov;// measurement noise covariance matrix (R)
	Mat errorCovPre;        // priori error estimate covariance matrix (P'(k)):
							//    P'(k)=A*P(k-1)*At + Q)
	Mat gain;               // Kalman gain matrix (K(k)):
							//    K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)
	Mat errorCovPost;       // posteriori error estimate covariance matrix (P(k)):
							//    P(k)=(I-K(k)*H)*P'(k)
*/

/*
	The covariance of process noise and of measurement noise are
	set to reasonable but interesting values (you can play with these yourself), and we ini-
	tialize the posterior error covariance to the identity as well (this is required to guarantee
	the meaningfulness of the first iteration; it will subsequently be overwritten)
*/

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <fstream>

#include <iostream>
#include <stdio.h>
#include <vector>

using namespace cv;
using namespace std;

Point COGOfRect(Rect rect)
{
	Point COGOfRect;
	COGOfRect.x=rect.x+rect.width/2;
	COGOfRect.y=rect.y+rect.height/2;
	return COGOfRect;
}

void HOG(Mat img,vector<Rect> &found_filtered)
{


//    Mat img(src);
    vector<Rect> found;
    HOGDescriptor hog;
    hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
    hog.detectMultiScale(img, found, 0, Size(8,8), Size(32,32), 1.05, 2);

    size_t i, j;
	for (i=0; i<found.size(); i++)
	{
	  Rect r = found[i];
	  for (j=0; j<found.size(); j++)
		  if (j!=i && (r & found[j]) == r)
			  break;
	  if (j== found.size())
		  found_filtered.push_back(r);
	}
	return;

}

template<typename to, typename from>
to lexical_cast(from const &x)
{
	std::stringstream os;
	to ret;
	os << x;
	os >> ret;
	return ret;
}

cv::Mat ResizeImage(cv::Mat src ,float scale)
{
	cv::Mat dst;
	cv::Size new_size;
	new_size.height=src.rows/scale;
	new_size.width=src.cols/scale;
	cv::resize(src,dst,new_size,INTER_LINEAR);
	return dst;
}

double DistanceBetweenTwoPoint(Point a, Point  b)
{
    int abx = a.x-b.x;
    int aby = a.y-b.y;
    return sqrt(abx*abx + aby*aby);
}

bool PointisInsideRectangle(Point CenterPoint,Point TopLeftPoint,Point DownRightPoint )
{
	if(  (TopLeftPoint.x <= CenterPoint.x )&&  (TopLeftPoint.y <= CenterPoint.y )
		&& (CenterPoint.x <= DownRightPoint.x ) && (CenterPoint.y <= DownRightPoint.y )   )
	{
		return true;
	} 
	return false;	
}

int main (int argc, char * const argv[])
{
	int TotalNumberOfFrames=0;
	int FrameNumber=-1;


	Point BoundingBoxTopLeftPoint;
	Point BoundingBoxBottomRightPoint;

	std::string ImageFileNameToStore;

	VideoCapture webCam(0); // video source for webcam
	webCam.set(CV_CAP_PROP_FRAME_WIDTH,640);
	webCam.set(CV_CAP_PROP_FRAME_HEIGHT,480);
	float scale_down=1.0;
	Mat original_scale_camImage;
	Mat camImage;		// raw image from webcam

	int NumberOfDimensionOfStateSpace=4;
	int NumberOfDimensionOfMeasurement=2;
	int NumberOfDimensionOfControl=0;
	int NumberOfGamePlayers=2;

	int Radius;
	int Thickness;
	Radius=5;
	Thickness=2;


	// Define Colors
	Scalar yellow	= Scalar(0,255,255,0);
	Scalar blue		= Scalar(255,0,0,0);
	Scalar white	= Scalar(255,255,255,0);
	Scalar red		= Scalar(0,0,255,0);
	Scalar green	= Scalar(0,255,0,0);
	Scalar cyan		= Scalar(255,255,0,0);
	Scalar  ColorSet[]={yellow,white,red,green,blue,cyan};

	char code = (char)-1;
	float delta_t=0.5;

	Mat predictionFirstPlayer;
	Mat predictionSecondPlayer;
	Mat NewMeasurmentFromPredictionFirstPlayer(NumberOfDimensionOfMeasurement,1,CV_32F);
	Mat NewMeasurmentFromPredictionSecondPlayer(NumberOfDimensionOfMeasurement,1,CV_32F);

	Point predictPtFirstPlayer;
	Point predictPtSecondPlayer;
	Point Temp;

//	FirstPlayer
	KalmanFilter KFFirstPlayer(  NumberOfDimensionOfStateSpace, NumberOfDimensionOfMeasurement, NumberOfDimensionOfControl);
	Mat stateFirstPlayer(NumberOfDimensionOfStateSpace, 1,CV_32F); //state space is 4x1 [x, y, Vx, Vy]
	Mat processNoiseFirstPlayer(NumberOfDimensionOfStateSpace, 1, CV_32F);
	Mat measurementFirstPlayer(NumberOfDimensionOfMeasurement,1,CV_32F);//measurement is 2x2 [x y]
	measurementFirstPlayer.setTo(Scalar(0));

//	End of FirstPlayer

//	SecondPlayer
	KalmanFilter KFSecondPlayer(  NumberOfDimensionOfStateSpace, NumberOfDimensionOfMeasurement, NumberOfDimensionOfControl);
	Mat stateSecondPlayer(NumberOfDimensionOfStateSpace, 1,CV_32F); //state space is 4x1 [x, y, Vx, Vy]
	Mat processNoiseSecondPlayer(NumberOfDimensionOfStateSpace, 1, CV_32F);
	Mat measurementSecondPlayer(NumberOfDimensionOfMeasurement,1,CV_32F);//measurement is 2x2 [x y]
	measurementSecondPlayer.setTo(Scalar(0));

	Point PredictedPointSecondPlayer;
	Point CorrectedStatePointSecondPlayer;
	Point MeasuredFromHOGPointSecondPlayer ;
//	End of SecondPlayer



	std::vector<Rect>  Rect_Surrounding_Human;
	namedWindow("HOG Kalman");

	if(argc>1)
	{
		webCam.open(argv[1]);
		scale_down= lexical_cast<float>(argv[2]);
	}
//	We will wait until we detect the human for the first time, then we initialize,
//	Rect_Surrounding_Human.at(0) for FirstPlayer and
//	Rect_Surrounding_Human.at(1) for SecondPlayer.
/*	while(true)
	{
		webCam >> original_scale_camImage;
		camImage=ResizeImage(original_scale_camImage,scale_down);
		imshow( "HOG Kalman", camImage );
		Rect_Surrounding_Human.clear();
		HOG(camImage,Rect_Surrounding_Human);
		cout<<Rect_Surrounding_Human.size() <<endl;
		if(Rect_Surrounding_Human.size()==NumberOfGamePlayers)
		{
			break;
		}
	}*/

//	FirstPlayer
	Point COGOfDetectedHuman;
//	COGOfDetectedHuman= COGOfRect(Rect_Surrounding_Human.at(0));
    KFFirstPlayer.statePre.at<float>(0) = COGOfDetectedHuman.x;
	KFFirstPlayer.statePre.at<float>(1) = COGOfDetectedHuman.y;
	KFFirstPlayer.statePre.at<float>(2) = 0;
	KFFirstPlayer.statePre.at<float>(3) = 0;
	KFFirstPlayer.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,delta_t,0,   0,1,0,delta_t,  0,0,1,0,  0,0,0,1);
    setIdentity(KFFirstPlayer.measurementMatrix);
    setIdentity(KFFirstPlayer.processNoiseCov, Scalar::all(1e-4));
    setIdentity(KFFirstPlayer.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(KFFirstPlayer.errorCovPost, Scalar::all(.1));
//	End of FirstPlayer



//	SecondPlayer
    Point COGOfDetectedHumanSecondPlayer;
//    COGOfDetectedHumanSecondPlayer= COGOfRect(Rect_Surrounding_Human.at(1));

	KFSecondPlayer.statePre.at<float>(0) = COGOfDetectedHumanSecondPlayer.x;
	KFSecondPlayer.statePre.at<float>(1) = COGOfDetectedHumanSecondPlayer.y;
	KFSecondPlayer.statePre.at<float>(2) = 0;
	KFSecondPlayer.statePre.at<float>(3) = 0;
	KFSecondPlayer.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,delta_t,0,   0,1,0,delta_t,  0,0,1,0,  0,0,0,1);
	setIdentity(KFSecondPlayer.measurementMatrix);
	setIdentity(KFSecondPlayer.processNoiseCov, Scalar::all(1e-4));
	setIdentity(KFSecondPlayer.measurementNoiseCov, Scalar::all(1e-1));
	setIdentity(KFSecondPlayer.errorCovPost, Scalar::all(.1));
//	End of SecondPlayer

/*******************************************************************************************************************************/
	string line;
	ifstream COG_R ("vid/images/COG_R.txt");
	ifstream COG_Y ("vid/images/COG_Y.txt");
	Point COG_Color_Tracker_R;
	Point COG_Color_Tracker_Y;
	Point COG_DetectedByHOG_FirstPerson;
	Point COG_DetectedByHOG_SecondPerson;
	
	COG_DetectedByHOG_FirstPerson.x=-1;
	COG_DetectedByHOG_FirstPerson.y=-1;
	
	COG_DetectedByHOG_SecondPerson.x=-1;
	COG_DetectedByHOG_SecondPerson.y=-1;
	
	
// 	Red color is first player 1
// 	Yellow color is second player 2
	
	int NumberofFrame_R_FromTracker_InKalmanBoundingBox=0;
	int NumberofFrame_Y_FromTracker_InKalmanBoundingBox=0;
	int NumberofFrame_HOG_R_InKalmanBoundingBox=0;
	int NumberofFrame_HOG_Y_InKalmanBoundingBox=0;
	int NumberofFrame_HOG_R_Detected=0;
	int NumberofFrame_HOG_Y_Detected=0;
	
	int fontFace = FONT_HERSHEY_PLAIN;
	double fontScale = .6;
	int thickness = 1;
	
	while(true)
	{
		FrameNumber++;
		webCam >> original_scale_camImage;

		if(original_scale_camImage.rows==0)
		{
			break;
		}
		line.clear();
		getline (COG_R,line);
		COG_Color_Tracker_R.x= lexical_cast<int>(line);
		line.clear();
		getline (COG_R,line);
		COG_Color_Tracker_R.y= lexical_cast<int>(line);


		line.clear();
		getline (COG_Y,line);
		COG_Color_Tracker_Y.x= lexical_cast<int>(line);
		line.clear();
		getline (COG_Y,line);
		COG_Color_Tracker_Y.y= lexical_cast<int>(line);
		

		
		camImage=ResizeImage(original_scale_camImage,scale_down);
		Rect_Surrounding_Human.clear();
		HOG(camImage,Rect_Surrounding_Human);

		predictionFirstPlayer=KFFirstPlayer.predict();
		predictionSecondPlayer=KFSecondPlayer.predict();

		predictPtFirstPlayer.x=predictionFirstPlayer.at<float>(0,0);
		predictPtFirstPlayer.y=predictionFirstPlayer.at<float>(1,0);

		predictPtSecondPlayer.x=predictionSecondPlayer.at<float>(0,0);
		predictPtSecondPlayer.y=predictionSecondPlayer.at<float>(1,0);

		NewMeasurmentFromPredictionFirstPlayer.at<float>(0,0)=predictionFirstPlayer.at<float>(0,0);
		NewMeasurmentFromPredictionFirstPlayer.at<float>(1,0)=predictionFirstPlayer.at<float>(1,0);
		NewMeasurmentFromPredictionSecondPlayer.at<float>(0,0)=predictionSecondPlayer.at<float>(0,0);
		NewMeasurmentFromPredictionSecondPlayer.at<float>(1,0)=predictionSecondPlayer.at<float>(1,0);

		switch ( Rect_Surrounding_Human.size() )
		{
			case 0:// nothing has been detected by HOG so we have to use predeic as measurement for both person
				KFFirstPlayer.correct(NewMeasurmentFromPredictionFirstPlayer);
				KFSecondPlayer.correct(NewMeasurmentFromPredictionSecondPlayer);
				COG_DetectedByHOG_FirstPerson.x=-1;
				COG_DetectedByHOG_FirstPerson.y=-1;
				COG_DetectedByHOG_SecondPerson.x=-1;
				COG_DetectedByHOG_SecondPerson.y=-1;
				break;
			case 1: // we have find Rect_Surrounding_Human.at(0) belong to FirstPlayer or SecondPlayer

				COGOfDetectedHuman= COGOfRect(Rect_Surrounding_Human.at(0));
				if( DistanceBetweenTwoPoint(predictPtFirstPlayer, COGOfDetectedHuman) <	DistanceBetweenTwoPoint(predictPtSecondPlayer, COGOfDetectedHuman) )
				{
					//this means the detected person is FirstPerson
					measurementFirstPlayer.at<float>(0,0) = COGOfDetectedHuman.x;
					measurementFirstPlayer.at<float>(1,0) = COGOfDetectedHuman.y;
					KFFirstPlayer.correct(measurementFirstPlayer);
					KFSecondPlayer.correct(NewMeasurmentFromPredictionSecondPlayer);
					//circle(camImage,COGOfDetectedHuman ,Radius,Scalar(0,0,255), Thickness   );//red ==> measurement
					cv::putText(camImage,"1",COGOfDetectedHuman,fontFace,fontScale,Scalar(0,0,255), thickness, 8);
					if(FrameNumber>45) 
					{
						NumberofFrame_HOG_R_Detected++;
					}
						
					COG_DetectedByHOG_FirstPerson.x=COGOfDetectedHuman.x;
					COG_DetectedByHOG_FirstPerson.y=COGOfDetectedHuman.y;
					COG_DetectedByHOG_SecondPerson.x=-1;
					COG_DetectedByHOG_SecondPerson.y=-1;
				}
				else
				{
					//this means the detected person is SecondPerson
					
					measurementSecondPlayer.at<float>(0,0) = COGOfDetectedHuman.x;
					measurementSecondPlayer.at<float>(1,0) = COGOfDetectedHuman.y;
					KFSecondPlayer.correct(measurementSecondPlayer);
					KFFirstPlayer.correct(NewMeasurmentFromPredictionFirstPlayer);
					//circle(camImage,COGOfDetectedHuman ,Radius,Scalar(0,0,255), Thickness   );//red ==> measurement
					cv::putText(camImage,"2",COGOfDetectedHuman,fontFace,fontScale,Scalar(0,255,255), thickness, 8);
					if(FrameNumber>45) 
					{
						NumberofFrame_HOG_Y_Detected++;
					}
					COG_DetectedByHOG_FirstPerson.x=-1;
					COG_DetectedByHOG_FirstPerson.y=-1;
					COG_DetectedByHOG_SecondPerson.x=COGOfDetectedHuman.x;
					COG_DetectedByHOG_SecondPerson.y=COGOfDetectedHuman.y;
					
				}
				break;
			case 2:// we have find Rect_Surrounding_Human.at(0) belong to FirstPlayer or SecondPlayer and find Rect_Surrounding_Human.at(1) as well

				COGOfDetectedHuman= COGOfRect(Rect_Surrounding_Human.at(0));
				Temp= COGOfRect(Rect_Surrounding_Human.at(1));

				if( DistanceBetweenTwoPoint(predictPtFirstPlayer, COGOfDetectedHuman) <	DistanceBetweenTwoPoint(predictPtSecondPlayer, COGOfDetectedHuman) )
				{
					//this means the detected person at COGOfDetectedHuman is FirstPerson
					measurementFirstPlayer.at<float>(0,0) = COGOfDetectedHuman.x;
					measurementFirstPlayer.at<float>(1,0) = COGOfDetectedHuman.y;
					KFFirstPlayer.correct(measurementFirstPlayer);

					measurementSecondPlayer.at<float>(0,0) = Temp.x;
					measurementSecondPlayer.at<float>(1,0) = Temp.y;
					KFSecondPlayer.correct(measurementSecondPlayer);
					cv::putText(camImage,"1",COGOfDetectedHuman,fontFace,fontScale,Scalar(0,0,255), thickness, 8);
					cv::putText(camImage,"2",Temp,fontFace,fontScale,Scalar(0,255,255), thickness, 8);
					
					COG_DetectedByHOG_FirstPerson.x=COGOfDetectedHuman.x;
					COG_DetectedByHOG_FirstPerson.y=COGOfDetectedHuman.y;
					COG_DetectedByHOG_SecondPerson.x=Temp.x;
					COG_DetectedByHOG_SecondPerson.y=Temp.y;
				}
				else
				{
					//this means the detected person at COGOfDetectedHuman is SecondPlayer
					measurementFirstPlayer.at<float>(0,0) = Temp.x;
					measurementFirstPlayer.at<float>(1,0) = Temp.y;
					KFFirstPlayer.correct(measurementFirstPlayer);

					measurementSecondPlayer.at<float>(0,0) = COGOfDetectedHuman.x;
					measurementSecondPlayer.at<float>(1,0) = COGOfDetectedHuman.y;
					KFSecondPlayer.correct(measurementSecondPlayer);
					cv::putText(camImage,"1",Temp,fontFace,fontScale,Scalar(0,0,255), thickness, 8);
					cv::putText(camImage,"2",COGOfDetectedHuman,fontFace,fontScale,Scalar(0,255,255), thickness, 8);
					
					COG_DetectedByHOG_FirstPerson.x=Temp.x;
					COG_DetectedByHOG_FirstPerson.y=Temp.y;
					COG_DetectedByHOG_SecondPerson.x=COGOfDetectedHuman.x;
					COG_DetectedByHOG_SecondPerson.y=COGOfDetectedHuman.y;
				}
				if(FrameNumber>45) 
				{
					NumberofFrame_HOG_R_Detected++;
					NumberofFrame_HOG_Y_Detected++;
				}
// 				circle(camImage,COGOfDetectedHuman ,Radius,Scalar(0,0,255), Thickness   );//red ==> measurement
// 				circle(camImage,Temp ,Radius,Scalar(0,0,255), Thickness   );//red ==> measurement
				break;
		}

// 		first person is red 1
// 		second person is yellow 2
		BoundingBoxTopLeftPoint.x=predictPtFirstPlayer.x-20;
		BoundingBoxTopLeftPoint.y=predictPtFirstPlayer.y-40;
		BoundingBoxBottomRightPoint.x=predictPtFirstPlayer.x+20;
		BoundingBoxBottomRightPoint.y=predictPtFirstPlayer.y+40;
		rectangle(camImage, BoundingBoxTopLeftPoint, BoundingBoxBottomRightPoint,Scalar(0,0,255) , 1);//red ==> Predicted
		cv::putText(camImage,"R",COG_Color_Tracker_R,fontFace,fontScale,Scalar(0,0,255), thickness, 8);
		
		
		if(FrameNumber<45)
		{
			continue;
		}
		
		
		if(PointisInsideRectangle(COG_Color_Tracker_R,BoundingBoxTopLeftPoint, BoundingBoxBottomRightPoint ))
		{
			NumberofFrame_R_FromTracker_InKalmanBoundingBox++;
		}
		
		if(PointisInsideRectangle(COG_DetectedByHOG_FirstPerson ,BoundingBoxTopLeftPoint, BoundingBoxBottomRightPoint ))
		{
			NumberofFrame_HOG_R_InKalmanBoundingBox++;
		}

		BoundingBoxTopLeftPoint.x=predictPtSecondPlayer.x-20;
		BoundingBoxTopLeftPoint.y=predictPtSecondPlayer.y-40;
		BoundingBoxBottomRightPoint.x=predictPtSecondPlayer.x+20;
		BoundingBoxBottomRightPoint.y=predictPtSecondPlayer.y+40;
		rectangle(camImage, BoundingBoxTopLeftPoint, BoundingBoxBottomRightPoint, Scalar(0,255,255), 1);//yellow ==> Predicted
		cv::putText(camImage,"Y",COG_Color_Tracker_Y,fontFace,fontScale,Scalar(0,255,255), thickness, 8 );
		
		if(PointisInsideRectangle(COG_Color_Tracker_Y,BoundingBoxTopLeftPoint, BoundingBoxBottomRightPoint ))
		{
			NumberofFrame_Y_FromTracker_InKalmanBoundingBox++;
		}
		
		if(PointisInsideRectangle(COG_DetectedByHOG_SecondPerson ,BoundingBoxTopLeftPoint, BoundingBoxBottomRightPoint ))
		{
			NumberofFrame_HOG_Y_InKalmanBoundingBox++;
		}
		
//		circle(camImage,predictPtFirstPlayer ,Radius,Scalar(255,0,0), Thickness+2   );//blue ==> Predicted
//		circle(camImage,predictPtSecondPlayer ,Radius,Scalar(255,0,0), Thickness+2   );//blue ==> Predicted
//		these corcle are coming from color tracker:
//		circle(camImage,COG_Color_Tracker_R ,Radius-1,Scalar(0,0,255), Thickness+4   );//
//		circle(camImage,COG_Color_Tracker_Y,Radius-1,Scalar(0,255,255), Thickness+4   );//

		
		


		imshow( "HOG Kalman", camImage );
 		ImageFileNameToStore= "vid/images/"+ lexical_cast<std::string>(FrameNumber)+".jpg";
 		imwrite( ImageFileNameToStore,camImage );
		cout<<" FrameNumber: " <<FrameNumber<<endl;
		code = (char)waitKey(100);
		if( code > 0 )
			break;
	}
	cout<<"---------------------------------------- Report ----------------------------------------" <<endl;
	cout<<" NumberofFrame_HOG_R_Detected: "<<NumberofFrame_HOG_R_Detected <<endl;
	cout<<" NumberofFrame_HOG_Y_Detected "<<NumberofFrame_HOG_Y_Detected <<endl;
	cout<<" NumberofFrame_R_FromTracker_InKalmanBoundingBox "<<NumberofFrame_R_FromTracker_InKalmanBoundingBox <<endl;
	cout<<" NumberofFrame_Y_FromTracker_InKalmanBoundingBox "<<NumberofFrame_Y_FromTracker_InKalmanBoundingBox <<endl;	
	cout<<" NumberofFrame_HOG_R_InKalmanBoundingBox "<<NumberofFrame_HOG_R_InKalmanBoundingBox <<endl;
	cout<<" NumberofFrame_HOG_Y_InKalmanBoundingBox "<<NumberofFrame_HOG_Y_InKalmanBoundingBox <<endl;
    return 0;
}

