/*
Corner detection
    Harris operator
    Shi and Tomasi
    Level curve curvature
    Hessian feature strength measures
    SUSAN
    FAST
*/



void goodFeaturesToTrack_Demo( int, void* )
{
  if( maxCorners < 1 ) { maxCorners = 1; }

  /// Parameters for Shi-Tomasi algorithm
  vector<Point2f> corners;
  double qualityLevel = 0.01;
  double minDistance = 10;
  int blockSize = 3;
  bool useHarrisDetector = false;
  double k = 0.04;

  /// Copy the source image
  Mat copy;
  copy = src_SubPix.clone();

  /// Apply corner detection
  goodFeaturesToTrack( src_gray_SubPix,
                       corners,
                       maxCorners,
                       qualityLevel,
                       minDistance,
                       Mat(),
                       blockSize,
                       useHarrisDetector,
                       k );


  /// Draw corners detected
  cout<<"** Number of corners detected: "<<corners.size()<<endl;
  int r = 4;
  for( int i = 0; i < corners.size(); i++ )
     { circle( copy, corners[i], r, Scalar(rng.uniform(0,255), rng.uniform(0,255),
                                                 rng.uniform(0,255)), -1, 8, 0 ); }

  /// Show what you got
  cv::namedWindow( source_window, cv::WINDOW_AUTOSIZE );
  imshow( source_window, copy );

  /// Set the neeed parameters to find the refined corners
  Size winSize = Size( 5, 5 );
  Size zeroZone = Size( -1, -1 );
//  TermCriteria criteria = TermCriteria( cv::TERMCRIT_EPS + cv::TERMCRIT_ITER, 40, 0.001 );
   TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::MAX_ITER|cv::TermCriteria::EPS, 40, 0.001);
  /// Calculate the refined corner locations
  cornerSubPix( src_gray_SubPix, corners, winSize, zeroZone, criteria );

  /// Write them down
  for( int i = 0; i < corners.size(); i++ )
     { cout<<" -- Refined Corner ["<<i<<"]  ("<<corners[i].x<<","<<corners[i].y<<")"<<endl; }
}

int main()
{
    return 0;
}
