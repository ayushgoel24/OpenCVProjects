/*
 * Multi-Kalman.cpp
 *
 *  Created on: Jul 21, 2012
 *      Author: behnam
 */




/*
 * HOG-Kalman.cpp
 *
 *  Created on: Jul 19, 2012
 *      Author: behnam
 */
#include "cv.h"
#include "highgui.h"
#include <opencv2/opencv.hpp>
#include "omp.h"
#include <limits>

using namespace std;
using namespace cv;

CvPoint COGOfRect(Rect rect)
{
	return cvPoint(rect.x+rect.width/2, rect.y+rect.height/2  );
}

IplImage* ResizeImage(IplImage* Image,int k)
{
	CvSize size = cvSize(Image->width/k,Image->height/k );
	IplImage* Resized_Image=cvCreateImage(size,Image->depth,Image->nChannels);
	cvResize(Image,Resized_Image,CV_INTER_LINEAR);
	return Resized_Image;

}

IplImage* DetectHumanHOG(IplImage* frame,int ResizedDownTo,std::vector<Rect>&found_filtered)
{
	IplImage* Resize_Image= ResizeImage(frame,ResizedDownTo);
	HOGDescriptor hog;
	hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
	vector<Rect> found;
	hog.detectMultiScale( Resize_Image, found, 0, Size(8,8), Size(32,32), 1.05, 2);
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

	return Resize_Image;
}

double distanza(CvPoint a, CvPoint  b){
    int abx = a.x-b.x;
    int aby = a.y-b.y;

    return sqrt(abx*abx + aby*aby);
}

int FindIndexOfPlayerFromDetectedHumansList(Rect RectOfDetectedHumans,std::vector<CvPoint> ListOfPlayers)
{
	int IndexOfPlayer=-1;
	double MinimumDistance=numeric_limits<double>::max( );
	double Distance;
	for(int i=0;i<ListOfPlayers.size();i++)
	{
		Distance=distanza(COGOfRect(RectOfDetectedHumans) , ListOfPlayers.at(i) );
		if(Distance  < MinimumDistance)
		{
			IndexOfPlayer=i;
			MinimumDistance=Distance;
		}

	}
	return IndexOfPlayer;
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

void CreatAndInitializeKalmanFilter( CvKalman* kalman , CvMat* x_k , CvMat* z_k , CvMat* v_k , CvMat* y_k , double DELTA_T,
		std::vector<CvKalman*>  &ListOfKalmanFilters,
		std::vector<CvMat*>  &ListOfx_k,
		std::vector<CvMat*>  &ListOfz_k,
		std::vector<CvMat*>   &ListOfv_k,
		std::vector<const CvMat*>  &ListOfy_k)
{
	CvScalar NOISE_STD=cvScalar(2);			// AWGN standard deviation that afflicts the measurement vector z_k
	CvScalar NOISE_MEAN=cvScalar(0);			// AWGN mean that afflicts the measurement vector z_k

	// Set Up Kalman Filter Parameters
//	#define Q_COMMON 0.1							//System Noise Covariance 0.1
//	#define R_COMMON 0.05						//Measurement Nose Covariance 0.05

#define Q_COMMON 160*160*DELTA_T							//System Noise Covariance 0.1
#define R_COMMON 4.0						//Measurement Nose Covariance 0.05


	//	Initialize, create Kalman Filter object, window, random number  generator etc.


	int NumberofDimensionOfStateSpace=4;//X is NumberofDimensionOfStateSpace x 1
	int NumberofDimensionOfMeasurment=2 ;//Z is NumberofDimensionOfMeasurment x 1
	int NumberofDimensionOfContrlParam=0 ;//U is NumberofDimensionOfContrlParam x 1

	kalman = cvCreateKalman( NumberofDimensionOfStateSpace, NumberofDimensionOfMeasurment, NumberofDimensionOfContrlParam);
	// state is (x , y , Vx , Vy)
	x_k = cvCreateMat( NumberofDimensionOfStateSpace/2, 1, CV_32FC1 );		//   matrix with only position values
	z_k = cvCreateMat( NumberofDimensionOfMeasurment, 1, CV_32FC1 );		// measurements matrix
	v_k = cvCreateMat( NumberofDimensionOfMeasurment, 1, CV_32FC1 );		// measurements matrix noise



//	const CvMat* y_k;											// This variable recieves predicted state from kalman filter


	// Noise random matrix initialize
	CvRNG rng;
	cvRandArr(&rng,v_k,CV_RAND_NORMAL,NOISE_MEAN, NOISE_STD);


	// Transition matrix 'F' describes relationship between
	// model parameters at step k and at step k+1 (this is
	// the "dynamics" in our model.
	// x_k=F*x_k+B*u_k+w_k
	// our state space is:
	//[ x
	//  y
	//  Vx
	//  Vy ]
	//and  F is NumberofDimensionOfStateSpace*NumberofDimensionOfStateSpace==>4x4
	//


	// Transition matrix
	const float F[] =
	{1 , 0 , DELTA_T , 0 ,
	0 , 1 , 0 , DELTA_T ,
	0 , 0 , 1 , 0 ,
	0 , 0 , 0 , 1 };
	memcpy( kalman->transition_matrix->data.fl, F, sizeof(F));

	// Measurement Matrix
	const float H[] = {
	1, 0, 0, 0,
	0, 1, 0, 0};
	memcpy( kalman->measurement_matrix->data.fl, H, sizeof(H));

	// Post Error Covariance Matrix
//	const float P[] = {
//	1, 0, 0, 0,
//	0, 1, 0, 0,
//	0, 0, 1, 0,
//	0, 0, 0, 1};

	const float P[] = {
		R_COMMON, 0, 0, 0,
		0, R_COMMON, 0, 0,
		0, 0, 80*80, 0,
		0, 0, 0, 80*80};


	memcpy( kalman->error_cov_post->data.fl, P, sizeof(P));

	// Process Noise Covariance Q
	const float Q[] = {
	Q_COMMON, 0, 0, 0,
	0, Q_COMMON, 0, 0,
	0, 0, Q_COMMON, 0,
	0, 0, 0, Q_COMMON};
	memcpy( kalman->process_noise_cov->data.fl, Q, sizeof(Q));

	// Measuremetn Noise Covariance R
	const float R[] = {
	R_COMMON, 0,
	0, R_COMMON};
	memcpy( kalman->measurement_noise_cov->data.fl, R, sizeof(R));




	ListOfKalmanFilters.push_back(kalman) ;
	ListOfx_k.push_back(x_k);
	ListOfz_k.push_back(z_k);
	ListOfv_k.push_back(v_k);
	ListOfy_k.push_back(y_k);



//	cout<< kalman->transition_matrix->data.fl[0] <<endl;
//	cout<< kalman->transition_matrix->data.fl[1] <<endl;
//	cout<< kalman->transition_matrix->data.fl[2] <<endl;
//	cout<< kalman->transition_matrix->data.fl[3] <<endl;

}

IplImage * ConvertToBlackAndWhite(IplImage* Image )
{
	IplImage * BlackAndWhite=cvCreateImage(cvGetSize(Image),IPL_DEPTH_8U,1);
	cvCvtColor(Image, BlackAndWhite, CV_RGB2GRAY);
	return BlackAndWhite;
}

int main(int argc, char** argv)
{
	int NumberOfGamePlayers=2;

	std::vector<CvPoint > ListOfPlayers;
	std::vector<CvPoint > ListOfPlayers_Prev;
	std::vector<CvKalman*>  ListOfKalmanFilters;
	std::vector<CvMat*>  ListOfx_k;
	std::vector<CvMat*>  ListOfz_k;
	std::vector<CvMat*>   ListOfv_k;
	std::vector<const CvMat*>  ListOfy_k;
	std::vector<bool> PlayerHasBeenDetectedByHOG;

	// Define Colors
	CvScalar yellow	= cvScalar(0,255,255,0);
	CvScalar white		= cvScalar(255,255,255,0);
	CvScalar red		= cvScalar(0,0,255,0);
	CvScalar green		= cvScalar(0,255,0,0);
	CvScalar blue		= cvScalar(255,0,0,0);
	CvScalar cyan		= cvScalar(255,255,0,0);
	CvScalar  ColorSet[]={yellow,white,red,green,blue,cyan};



	cvNamedWindow( "PlayingMovie", CV_WINDOW_AUTOSIZE );
	std::string VideoPath=argv[1] ;
	CvCapture* capture = cvCreateFileCapture( VideoPath.c_str() );
//	CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
	std::string ImageName;
	double fps = cvGetCaptureProperty(capture,CV_CAP_PROP_FPS);
	const double DELTA_T = 1/fps;


	IplImage* frame;
	IplImage * ImageBlackAndWhite;
	int ResizedDownTo=6;
	std::vector<Rect> RectOfDetectedHumans;
	IplImage* Resize_Image;

	for(int i=0;i<NumberOfGamePlayers;i++)
	{
		CvMat* x_k;
		CvMat* z_k;
		CvMat* v_k;
		CvMat* y_k;
		CvKalman* KalmanFilter;

		CreatAndInitializeKalmanFilter(KalmanFilter,  x_k ,z_k,v_k, y_k,DELTA_T,
		ListOfKalmanFilters,
		ListOfx_k,
		ListOfz_k,
		ListOfv_k,
		ListOfy_k);
		PlayerHasBeenDetectedByHOG.push_back(false);
	}


	while(true)
	{
		frame = cvQueryFrame( capture );
		if( !frame ) break;



		RectOfDetectedHumans.clear();

		Resize_Image=DetectHumanHOG( frame,ResizedDownTo,RectOfDetectedHumans);
		cvShowImage( "PlayingMovie", Resize_Image );
		char c = cvWaitKey(1);
		if( c == 27 ) break;
		cout<<"RectOfDetectedHumans.size() " <<RectOfDetectedHumans.size() <<endl;
		if(RectOfDetectedHumans.size() == NumberOfGamePlayers)
		{
			break;
		}
	}
	CvPoint Player;
	for(int i=0;i<NumberOfGamePlayers;i++)
	{
		Player=COGOfRect( RectOfDetectedHumans[i] );
		ListOfPlayers.push_back(Player ) ;
		ListOfKalmanFilters[i]->state_post->data.fl[0]=Player.x;
		ListOfKalmanFilters[i]->state_post->data.fl[1]=Player.y;
		ListOfKalmanFilters[i]->state_post->data.fl[2]=0.0;
		ListOfKalmanFilters[i]->state_post->data.fl[3]=0.0;

	}

	RectOfDetectedHumans.clear();

	int nFrames = (int) cvGetCaptureProperty( capture , CV_CAP_PROP_FRAME_COUNT );
	int IndexOfPlayerDetectedByHOG;
	for(size_t i=0;i<nFrames ;i++)
	{
		frame = cvQueryFrame( capture );
		if( !frame ) break;
		Resize_Image=DetectHumanHOG( frame,ResizedDownTo,RectOfDetectedHumans);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////    Kalman and HOG    //////////////////////////////////////////////////////////////
///////////////////////////////////////////////////  Heart of the program  //////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		for(int j=0;j<RectOfDetectedHumans.size() ;j++)
		{
			IndexOfPlayerDetectedByHOG=FindIndexOfPlayerFromDetectedHumansList(RectOfDetectedHumans[j],ListOfPlayers);
			PlayerHasBeenDetectedByHOG[IndexOfPlayerDetectedByHOG]=true;
			ListOfPlayers[IndexOfPlayerDetectedByHOG] =COGOfRect(RectOfDetectedHumans[j]);
			cvCircle( Resize_Image, COGOfRect(RectOfDetectedHumans[j]), 15, ColorSet[j],2 );
		}

		for(int j=0;j<NumberOfGamePlayers;j++)
		{
			cout<<"ListOfKalmanFilters.at(j)->state_post->data.fl[0]: "<< ListOfKalmanFilters.at(j)->state_post->data.fl[0] <<endl;
			cout<<"ListOfKalmanFilters.at(j)->state_post->data.fl[1]: "<< ListOfKalmanFilters.at(j)->state_post->data.fl[1] <<endl;
			ListOfy_k[j]= cvKalmanPredict( ListOfKalmanFilters[j], 0 );
			if(!PlayerHasBeenDetectedByHOG[IndexOfPlayerDetectedByHOG])
			{
				//use prediction
//
//				Player=cvPoint(ListOfy_k[j]->data.fl[0] ,  ListOfy_k[j]->data.fl[1]);
//				ListOfPlayers[j]=Player;
//				ListOfz_k[j]->data.fl[0]=Player.x;
//				ListOfz_k[j]->data.fl[1]=Player.y;
			}
			else
			{
				//use measurement
				Player=ListOfPlayers[j];
				ListOfz_k[j]->data.fl[0]=Player.x;
				ListOfz_k[j]->data.fl[1]=Player.y;
				cout<< "ListOfz_k[j]->data.fl[0] "<<ListOfz_k[j]->data.fl[0]<<endl;
				cout<< "ListOfz_k[j]->data.fl[1] "<<ListOfz_k[j]->data.fl[1]<<endl;
				cvKalmanCorrect( ListOfKalmanFilters[j], ListOfz_k[j] );
			}
//			cvKalmanCorrect( ListOfKalmanFilters[j], ListOfz_k[j] );
			cvCircle( Resize_Image, cvPoint(ListOfKalmanFilters.at(j)->state_post->data.fl[0],ListOfKalmanFilters.at(j)->state_post->data.fl[1]), 5, cyan,2 );
		}
		RectOfDetectedHumans.clear();
		char c = cvWaitKey(1);
		if( c == 27 ) break;
		cvShowImage( "PlayingMovie", Resize_Image );
		ImageName= "kalman_images/"+lexical_cast<string>(i)+".jpg";
//		cvSaveImage( ImageName.c_str(),Resize_Image  );
		cvReleaseImage(&Resize_Image );
	}
	cvReleaseCapture( &capture );
	cvDestroyWindow( "PlayingMovie" );

return 0;
}
