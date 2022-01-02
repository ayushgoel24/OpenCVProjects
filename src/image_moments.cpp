//https://aishack.in/tutorials/image-moments/
//https://en.wikipedia.org/wiki/Image_moment

//void Moments(	CvMoments &moments,IplImage * img)
//{
///*	class Moments
//	{
//	public:
//	    Moments();
//	    Moments(double m00, double m10, double m01, double m20, double m11,
//	            double m02, double m30, double m21, double m12, double m03 );
//	    Moments( const CvMoments& moments );
//	    operator CvMoments() const;

//	    // spatial moments
//	    double  m00, m10, m01, m20, m11, m02, m30, m21, m12, m03;
//	    // central moments
//	    double  mu20, mu11, mu02, mu30, mu21, mu12, mu03;
//	    // central normalized moments
//	    double  nu20, nu11, nu02, nu30, nu21, nu12, nu03;

//		because:
//			m00=m00
//			mu01=0
//			mu10=0
//	}


//	Parameters:
//	array – Raster image (single-channel, 8-bit or floating-point 2D array) or an array (  or   ) of 2D points (Point or Point2f ).
//	binaryImage – If it is true, all non-zero image pixels are treated as 1’s. The parameter is used for images only.
//	moments – Output moments.

//	*/

///*
//	All of the moments—including spatial and central moments up to the 3rd order calculated and fills moment state structure.
//	void cvMoments(const CvArr* image,CvMoments* moments,	int	isBinary = 0 );

//	double  cvGetSpatialMoment( CvMoments* moments, int x_order, int y_order );//m00, m10, m01, m20, m11, m02, m30, m21, m12, m03

//	double cvGetCentralMoment( 	CvMoments* moments,	int x_order,int y_order );//mu20, mu11, mu02, mu30, mu21, mu12, mu03;

//	double cvGetNormalizedCentralMoment(CvMoments* moments,	int	x_order,int	y_order	);//u20, nu11, nu02, nu30, nu21, nu12, nu03;

//	void cvGetHuMoments(CvMoments* moments,	CvHuMoments* HuMoments);// h1, h2, h3,...
//*/
//}

//void Moments_Test(char ** argv)
//{
////	example of run:	images/gedeck1.jpg
//	IplImage* img=  cvLoadImage(argv[1],CV_LOAD_IMAGE_GRAYSCALE);
//	CvMoments moments;
//	cvMoments( img,&moments,1);
///*
//	All of the moments—including spatial and central moments up to the 3rd order calculated and fills moment state structure.
//	void cvMoments(const CvArr* image,CvMoments* moments,	int	isBinary = 0 );

//	double  cvGetSpatialMoment( CvMoments* moments, int x_order, int y_order );//m00, m10, m01, m20, m11, m02, m30, m21, m12, m03

//	double cvGetCentralMoment( 	CvMoments* moments,	int x_order,int y_order );//mu20, mu11, mu02, mu30, mu21, mu12, mu03;

//	double cvGetNormalizedCentralMoment(CvMoments* moments,	int	x_order,int	y_order	);//u20, nu11, nu02, nu30, nu21, nu12, nu03;

//	void cvGetHuMoments(CvMoments* moments,	CvHuMoments* HuMoments);// h1, h2, h3,...
//*/

//	double  m00, m10, m01, m20, m11, m02, m30, m21, m12, m03;
//	m00 = cvGetSpatialMoment(&moments,0,0);
//	m10 = cvGetSpatialMoment(&moments,1,0);
//	m01 = cvGetSpatialMoment(&moments,0,1);

////	http://opencv.willowgarage.com/wiki/cvBlobsLib#Download

////	cv::findContours()

//}

int main()
{
	return 0;
}
