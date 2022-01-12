#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>


#include <climits>
#include <opencv4/opencv2/videoio.hpp>
#include <opencv4/opencv2/opencv.hpp>

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

std::vector<std::string> split(const std::string &s, char delim)
{
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}




void Erosion( cv::Mat &src ,cv::Mat  &erosion_dst)
{
//  int erosion_type = 0;
//  if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
//  else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
//  else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

    int erosion_type = cv::MORPH_ELLIPSE;
    int erosion_size=1;

    cv::Mat element = getStructuringElement( erosion_type, cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ), cv::Point( erosion_size, erosion_size ) );
  /// Apply the erosion operation
  cv::erode( src, erosion_dst, element );
  //cv::imshow( "Erosion Demo", erosion_dst );
}


void Dilation( cv::Mat &src ,cv::Mat  &dilation_dst)
{
  int dilation_type = cv::MORPH_RECT;

  int dilation_size=3;

  cv::Mat element = cv::getStructuringElement( dilation_type, cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ), cv::Point( dilation_size, dilation_size ) );
  /// Apply the dilation operation
  cv::dilate( src, dilation_dst, element );
  //imshow( "Dilation Demo", dilation_dst );
}

int rectangle(cv::Mat image)
{
    cv::Mat input = cv::imread("truck.png");

//    // convert to grayscale (you could load as grayscale instead)
//    cv::Mat gray;
//    cv::cvtColor(input,gray, CV_BGR2GRAY);

//    // compute mask (you could use a simple threshold if the image is always as good as the one you provided)
//    cv::Mat mask;
//    cv::threshold(gray, mask, 0, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);

//    // find contours (if always so easy to segment as your image, you could just add the black/rect pixels to a vector)
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    //cv::findContours(mask,contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    cv::findContours(image,contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    /// Draw contours and find biggest contour (if there are other contours in the image, we assume the biggest one is the desired rect)
    // drawing here is only for demonstration!
    int biggestContourIdx = -1;
    float biggestContourArea = 0;
    cv::Mat drawing = cv::Mat::zeros( image.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
    {
        cv::Scalar color = cv::Scalar(0, 100, 0);
        drawContours( drawing, contours, i, color, 1, 8, hierarchy, 0, cv::Point() );

        float ctArea= cv::contourArea(contours[i]);
        if(ctArea > biggestContourArea)
        {
            biggestContourArea = ctArea;
            biggestContourIdx = i;
        }
    }

    // if no contour found
    if(biggestContourIdx < 0)
    {
        std::cout << "no contour found" << std::endl;
        return 1;
    }

    // compute the rotated bounding rect of the biggest contour! (this is the part that does what you want/need)
    cv::RotatedRect boundingBox = cv::minAreaRect(contours[biggestContourIdx]);
    // one thing to remark: this will compute the OUTER boundary box, so maybe you have to erode/dilate if you want something between the ragged lines



    // draw the rotated rect
    cv::Point2f corners[4];
    boundingBox.points(corners);
    cv::line(drawing, corners[0], corners[1], cv::Scalar(255,255,255));
    cv::line(drawing, corners[1], corners[2], cv::Scalar(255,255,255));
    cv::line(drawing, corners[2], corners[3], cv::Scalar(255,255,255));
    cv::line(drawing, corners[3], corners[0], cv::Scalar(255,255,255));

    // display
    cv::imshow("input", input);
    cv::imshow("drawing", drawing);
    cv::waitKey(0);

    cv::imwrite("rotatedRect.png",drawing);

    return 0;
}
void reading_file()
{


    std::vector<cv::Point2i> points;
    cv::Point2i point;

    int x_max, x_min, y_max, y_min;
    x_min=INT_MAX;
    x_max=INT_MIN;


    y_min=INT_MAX;
    y_max=INT_MIN;

    std::string line;
    char delim=' ';
    int x,y;
    std::ifstream myfile ("points.txt");
    //double scale=1000;


    int rows=3213+500;
    int columns=259+500 ;
    //CV_8UC CV_32FC1
    cv::Mat MatrixofRawData=cv::Mat::zeros(columns,rows,CV_8UC1);



    if (myfile.is_open())
    {
      while ( getline (myfile,line) )
      {
          std::vector<std::string> single_line_points= split(line, delim);


          x= int(10* atof(single_line_points.at(0).c_str() ) ) ;
          y= int(10* atof(single_line_points.at(1).c_str() ) ) ;


          x_max=std::max(x,x_max);
          y_max=std::max(y,y_max);

          x_min=std::min(x,x_min);
          y_min=std::min(y,y_min);


          point.x=x;
          point.y=y;
          points.push_back(point);

          MatrixofRawData.at<uchar>(y, x) = 225;



      }
      //std::cout<<"size of cloud " <<cloud->points.size() <<std::endl;

      std::cout<<"x_max,y_max,x_min,y_min :" << x_max << " , "<<y_max<< " , "<<x_min<<" , " <<y_min <<std::endl;

      myfile.close();
    }

    cv::Mat pointsMat = cv::Mat(points);
    std::cout<<pointsMat.channels()<<std::endl;



    Erosion( MatrixofRawData , MatrixofRawData);
    Dilation( MatrixofRawData , MatrixofRawData);


    cv::imwrite("modified_truck.png",MatrixofRawData);

    rectangle(MatrixofRawData);
    cv::waitKey(0);






}

cv::Mat & create_mat_by_ref()
{
    cv::Mat m=cv::Mat::ones(3,4,CV_64FC1);
    std::cout<<"m: "<<std::endl;
    std::cout<<m <<std::endl;
    return m;
}

cv::Mat create_mat()
{
    cv::Mat m=cv::Mat::ones(3,4,CV_64FC1);
    std::cout<<"m: "<<std::endl;
    std::cout<<m <<std::endl;
    return m;
}


void populate_mat(cv::Mat &m)
{
    //m=cv::Mat::ones(3,4,CV_64FC1);
    m.create(3,4,CV_64FC1);

}

void clone_mat(cv::Mat &mat)
{
    cv::Mat tmp=cv::Mat::ones(3,4, CV_64FC1);
    mat=tmp.clone();
    return;
}


void HSVColorFilter(cv::Mat &input_bgr_image, cv::Mat &output_hsv_image)
{

    cv::cvtColor(input_bgr_image, output_hsv_image, cv::COLOR_BGR2HSV);

//    std::vector<cv::Mat> h_s_v;
//    cv::Mat filtered_Hue, filtered_Sat, filtered_Val;
//    cv::split(output_hsv_image,h_s_v);
//    cv::threshold(h_s_v.at(0),filtered_Hue,0,179,cv::THRESH_BINARY);



    // Threshold the HSV image, keep only the red pixels
    cv::Mat lower_red_hue_range;
    cv::Mat upper_red_hue_range;
//    cv::inRange(output_hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), output_hsv_image);
    cv::inRange(output_hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
    cv::inRange(output_hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);

    //cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, output_hsv_image);
//    cv::bitwise_and(output_hsv_image
/*
Convert a color image from BGR to HSV using "cvConvert()" (or my better HSV conversion functions).
Split the HSV color image into its separate H, S and V components.
Use "cvThreshold()" to look for the pixels that are in the correct range of Hue, Saturation and Value (Brightness).
Use one of the blob libraries (listed below) to detect the blobs in the thresholded image so you can get the sizes and positions, etc, and you can track those blobs.

*/

}


void getVideoFromCam()
{
    cv::VideoCapture webCam(1); // open the default camera
    webCam.set(cv::CAP_PROP_FRAME_WIDTH ,640);
    webCam.set(cv::CAP_PROP_FRAME_HEIGHT,480);
    if(!webCam.isOpened())  // check if we succeeded
    return;

    cv::namedWindow("camera",1);
    for(;;)
    {
/*
In OpenCV, Hue varies between 0 to 179 , and a Saturation and Values vary between 0 and 255.

*/
        cv::Mat frame;
        webCam >> frame; // get a new frame from camera
        cv::Mat output_hsv_image;
        cv::Mat filtered_output_hsv_image;

        HSVColorFilter(frame, output_hsv_image);
/*
        cv::cvtColor(frame, output_hsv_image, CV_BGR2HSV);
*/



        std::cout<<output_hsv_image.rows<<std::endl;
        std::cout<<output_hsv_image.cols<<std::endl;
        std::cout<<output_hsv_image.channels()<<std::endl;

//        cv::inRange(output_hsv_image, cv::Scalar(0, 0, 0), cv::Scalar(180, 250, 250), filtered_output_hsv_image);

//        std::cout<<filtered_output_hsv_image.rows<<std::endl;
//        std::cout<<filtered_output_hsv_image.cols<<std::endl;
//        std::cout<<filtered_output_hsv_image.channels()<<std::endl;

        //cv::cvtColor(filtered_output_hsv_image, frame, CV_HSV2BGR);

        cv::imshow("camera", output_hsv_image);
        if(cv::waitKey(200) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return ;
}

void extract_frame_from_vid(int argc, char ** argv)
{

    cv::VideoCapture vid;
    vid.open(argv[1]);
    cv::namedWindow("playing a vidoe file",1);
    int i=0;
    for(;;)
    {
        cv::Mat frame;
        vid >> frame; // get a new frame from camera
        cv::imshow("playing a vidoe file", frame);
//        if(i % 4==0)
//            cv::imwrite("images/"+lexical_cast<std::string>(i) +".jpg",frame);
        if(cv::waitKey(30) >= 0) break;
        cv::imwrite("images/"+std::to_string(i) +".jpg",frame);
        i++;
    }
}
int main(int argc, char * argv[])
{
    //reading_file();
    //reading_file();
/*
    cv::Mat m1=cv::Mat::zeros(4,5,CV_32FC1);
    std::cout<<"m1: "<<std::endl;
    std::cout<<m1 <<std::endl;

    cv::Mat m2=cv::Mat::ones(3,3,CV_32FC1) ;
    std::cout<<"m2: "  <<std::endl;
    std::cout<<m2 <<std::endl;

    m2.copyTo(m1,cv::Mat::zeros(2,2,CV_32FC1));
    std::cout<<"m1: "<<std::endl;
    std::cout<<m1 <<std::endl;
*/
    /*
    cv::Mat m;
    populate_mat(m);
    std::cout<<"m: "<<std::endl;
    std::cout<<m <<std::endl;
*/

/*
    cv::Mat  m = create_mat();
    std::cout<<"m: "<<std::endl;
    std::cout<<m <<std::endl;


    cv::Mat  m_by_ref = create_mat_by_ref();
    std::cout<<"m_by_ref will be destroied after call, so an empty mat here: "<<std::endl;
    std::cout<<m_by_ref <<std::endl;

*/
    //getVideoFromCam();


//    strcpy("-w 7 -h 5 stereo_data/stereo_calib.xml",argv);


/*
    char* n_argv[] = { "sandbox", "-w", "7", "-h","5" ,"stereo_data/stereo_calib.xml"};

    int length = sizeof(n_argv)/sizeof(n_argv[0]);

    argc=length;
    argv = n_argv;



    std::cout<<argc <<std::endl;

    std::cout<<argv[0] <<std::endl;
    std::cout<<argv[1] <<std::endl;
    std::cout<<argv[2] <<std::endl;
    std::cout<<argv[3] <<std::endl;
    std::cout<<argv[4] <<std::endl;


*/

    //extract_frame_from_vid(argc,argv);

    return 0;
}

