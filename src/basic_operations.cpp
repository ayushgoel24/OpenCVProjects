#include <opencv4/opencv2/opencv.hpp>

void createMatrix()
{
/*
    CV_<bit_depth>(S|U|F)C<number_of_channels>
	elements type (uchar,short,int,float,double)
	CV_8UC1 means an 8-bit unsigned single channel
	CV_32FC3 means a 32-bit float matrix with three

    CV_32F is float!
    CV_64F is double!

              C1   C2    C3    C4
    CV_8U     0    8     16    24
    CV_8S     1    9     17    25
    CV_16U    2    10    18    26
    CV_16S    3    11    19    27
    CV_32S    4    12    20    28
    CV_32F    5    13    21    29
    CV_64F    6    14    22    30
*/
 
    int rows, cols;
    rows=600;
    cols=800;

    //1) 
    cv::Mat img1=cv::Mat::zeros(rows, cols ,CV_64FC1 )+0.5;

    std::cout<<"Total Number of Elements: " <<img1.total() <<std::endl;


    // According to the above table CV_64FC1=6
    std::cout<<"Matrix data type is: " <<img1.type() <<std::endl;
    
    //2) 
    cv::Mat dst = cv::Mat::zeros( img1.size(), img1.type() );

    //3) create matrix in several step;
    cv::Mat img3;
    img3.create(rows,cols,CV_32FC1);


    //4) cv::DataType<double>::type
    cv::Mat cameraMatrix1(3, 3, cv::DataType<double>::type);
    std::cout<<cameraMatrix1 <<std::endl;

    //5) cv::Mat_<double>(3,3)
    cv::Mat cameraMatrix2 = (cv::Mat_<double>(3,3) <<1,2,3,4,5,6);
    std::cout<<cameraMatrix2 <<std::endl;


    cv::namedWindow("window",cv::WINDOW_AUTOSIZE);
    cv::imshow("window",img1 );
    cv::waitKey(0);
}

void matrixVectorConversion()
{
    std::vector<double> values={1};

    
    cv::Mat MatrixofRawData;
    int NumberofRows=2;
    int NumberofCols=3;
    MatrixofRawData.create(NumberofRows,NumberofCols,CV_32FC1);
       
    std::vector<float> RowVector;
    RowVector.push_back(0.2);
    RowVector.push_back(1.2);
    RowVector.push_back(4.2);
    MatrixofRawData.push_back(RowVector);
    //MatrixofRawData.convertTo();
    cv::Mat MatFromVector(RowVector , true);
    std::cout<<"MatFromVector: "<< MatFromVector <<std::endl;   

    //createing image, merging, spliting channels

}

void matrixOperations()
{
   cv::Mat m1=cv::Mat::zeros(3,3,CV_64FC1);
   cv::Mat m2=cv::Mat::ones(3,3,CV_64FC1);


    std::cout<<"m1" <<std::endl;
    std::cout<<m1 <<std::endl;

    std::cout<<"m2"<<std::endl;
    std::cout<<m2 <<std::endl;


    std::cout << "(m2*2)" << std::endl;
    std::cout << (m2*2) << std::endl;

    std::cout << "m2*m1" << std::endl;
    std::cout << m2*m1 << std::endl;

    std::cout << "m1+m2.t()" << std::endl;
    std::cout << m1+m2.t() << std::endl;

    std::cout << "m1-m2.t()" << std::endl;
    std::cout << m1-m2.t() << std::endl;
}

void accessingMatrixElements()
{
    int rows, cols, i,j;
    rows=600;
    cols=800;
    i=rows/2;
    j=cols/2;

    cv::Point point=cv::Point(i,j);
    cv::Mat img1, img2, img3;

    //CV_8UC4 means unsinged char (8 bit 0-255) and 4 channel, so to access every pixel we use cv::Vec4b
    img1.create(rows,cols,CV_8UC4);

    //CV_64FC1 means double  and 4 channel, so to access every pixel we use cv::Vec4d
    img2.create(rows,cols,CV_64FC4);
    //cv::Vec4d;
    cv::Vec4d pixel_values_bgra =img2.at<cv::Vec4d>(point);
    double pixel_value_b =img2.at<cv::Vec4d>(point)[0];



    //CV_64FC1 means floar  and 1 channel, so to access every pixel we use cv::Vec4d
    img3.create(rows,cols,CV_32FC1);
    //float pixel=img3.at<float>(point);
    img3.at<float>(i,j)=0.5;
    float pixel=img3.at<float>(i,j);



    cv::namedWindow("window",cv::WINDOW_AUTOSIZE);
    cv::imshow("window",img3 );
    cv::waitKey(0);
}

void readWriteImage()
{
    std::string imageDir="../images/";
    std::string img1FileName="lena.jpg";
    std::string windowName="window";
    cv::Mat img1;
    img1= cv::imread(imageDir+img1FileName,cv::IMREAD_COLOR );

    //img1 type will be CV_8UC3
    std::cout<<img1.type()<<std::endl;

    //img1 type will be CV_8UC1
    img1= cv::imread(imageDir+img1FileName,cv::IMREAD_GRAYSCALE );
    std::cout<<img1.type()<<std::endl;

    /*
    cv::minMaxIdx finds the minimum and maximum element values and their positions, does not work with multi-channel arrays, use Mat::reshape first to reinterpret the array as single-channel. 
    Or you may extract the particular channel using either extractImageCOI , or mixChannels , or split .
    */
    std::vector<int> minIx(3),maxIx(3);

    double minValue, maxValue;
    cv::minMaxIdx(img1, &minValue, &maxValue, &minIx[0], &maxIx[0]);

    std::cout <<"min value: " <<minValue << " max value: " << maxValue << "min ix: [" << minIx[0] << " " << minIx[1] << " " << minIx[2] << "] max id [" << maxIx[0] << " " << maxIx[1] << " " << maxIx[2] << "]" << std::endl;


    //if you need to read it in double, you have to convert it:
    img1.convertTo(img1,CV_64FC1,1/255.0);
    std::cout<<img1.type()<<std::endl;
    cv::minMaxIdx(img1, &minValue, &maxValue, &minIx[0], &maxIx[0]);
    std::cout <<"min value: " <<minValue << " max value: " << maxValue << "min ix: [" << minIx[0] << " " << minIx[1] << " " << minIx[2] << "] max id [" << maxIx[0] << " " << maxIx[1] << " " << maxIx[2] << "]" << std::endl;


    cv::namedWindow(windowName,cv::WINDOW_AUTOSIZE);
    cv::imshow(windowName, img1 );
    cv::waitKey(0);

}

void matrixConversion()
{
    //convertTo
    //cv::cvtColor
}

void drawingFunsctionAndCoordinate()
{
    /*
    image's points in opencv has the follow index:
	(0,0) (1,0) (2,0) 3,0)
	(0,1) (1,1) (2,1) 3,1)
	(0,2) (1,2) (2,2) 3,2)

            X           (cols,0)
            -------------►
            |
          y |
            |
    (0,rows)▼           (cols,rows)

*/
    int blue, green, red;
    blue=255;
    green=255;
    red=255;


    cv::Mat img = cv::Mat::zeros(400,600, CV_32FC3);
    cv::namedWindow("WorkingwitDrawingcommands",cv::WINDOW_AUTOSIZE);
    
    // draw a box with red lines of width 1 between (0,100) and (200,200)
    cv::rectangle(img, cv::Point(0,100), cv::Point(200,200), cv::Scalar(blue,0,0), 1);

    // draw a circle at (300,300) with a radius of 20. Use green lines of width 1
    cv::circle(img, cv::Point( 300,100), 20, cv::Scalar(0,green,0), 1);

    //Draw a line segment:

    // draw a green line of width 1 between (100,100) and (200,200)
    cv::line(img, cv::Point(100,100), cv::Point(200,200), cv::Scalar(0,255,0), 1);

    //Draw a set of polylines:
    cv::Point  curve1[]={cv::Point(10,10),  cv::Point(10,100),  cv::Point(100,100),  cv::Point(100,10)};
    cv::Point  curve2[]={cv::Point(30,30),  cv::Point(30,130),  cv::Point(130,130),  cv::Point(130,30),  cv::Point(150,10)};
    const cv::Point* curveArr[2]={curve1, curve2};
    int      nCurvePts[2]={4,5};
    int      nCurves=2;
    int      isCurveClosed=1;
    int      lineWidth=1;
    cv::polylines(img,curveArr,nCurvePts,nCurves,isCurveClosed,cv::Scalar(0,255,255),lineWidth);

    //Draw a set of filled polygons:

    cv::fillPoly(img,curveArr,nCurvePts,nCurves,cv::Scalar(0,255,255));

    //Add text:

    double fontScale=1.0;
    cv::putText  (img,"My comment",cv::Point(200,400), cv::FONT_HERSHEY_SIMPLEX,fontScale ,cv::Scalar(255,255,0));
    //	Other possible fonts:
    //
    //	CV_FONT_HERSHEY_SIMPLEX, CV_FONT_HERSHEY_PLAIN,
    //	CV_FONT_HERSHEY_DUPLEX, CV_FONT_HERSHEY_COMPLEX,
    //	CV_FONT_HERSHEY_TRIPLEX, CV_FONT_HERSHEY_COMPLEX_SMALL,
    //	CV_FONT_HERSHEY_SCRIPT_SIMPLEX, CV_FONT_HERSHEY_SCRIPT_COMPLEX,


    cv::imshow("WorkingwitDrawingcommands",img);
    cv::waitKey(0);
}

void readingCameraMatrix()
{

    std::string camera_calibration_path="../data/front_webcam.yml";
    cv::FileStorage fs(camera_calibration_path,cv::FileStorage::READ);
    cv::Mat camera_matrix, distortion_coefficient;
    fs["camera_matrix"]>>camera_matrix;
    fs["distortion_coefficients"]>>distortion_coefficient;

    std::cout<<"Camera Matrix:" <<std::endl;

    std::cout<<"Fx: " <<camera_matrix.at<double>(0,0) <<std::endl;
    std::cout<<"Fy: " <<camera_matrix.at<double>(1,1) <<std::endl;

    std::cout<<"Cx: " <<camera_matrix.at<double>(0,2) <<std::endl;
    std::cout<<"Cy: " <<camera_matrix.at<double>(1,2) <<std::endl;
    std::cout<< "Distortion Coefficient:"<<std::endl;

    std::cout<<"K1: "<<distortion_coefficient.at<double>(0,0) <<std::endl;
    std::cout<<"K2: "<<distortion_coefficient.at<double>(0,1) <<std::endl;
    std::cout<<"P1: "<<distortion_coefficient.at<double>(0,2) <<std::endl;
    std::cout<<"P2: "<<distortion_coefficient.at<double>(0,3) <<std::endl;
    std::cout<<"K3: "<<distortion_coefficient.at<double>(0,4) <<std::endl;

}

void imageChannels()
{
// Mat img(5,5,CV_64FC3); // declare three channels image 
// Mat ch1, ch2, ch3; // declare three matrices 
// // "channels" is a vector of 3 Mat arrays:
// vector<Mat> channels(3);
// // split img:
// split(img, channels);
// // get the channels (follow BGR order in OpenCV)
// ch1 = channels[0];
// ch2 = channels[1];
// ch3 = channels[2]; 
// // modify channel// then merge

// merge(channels, img);



// Mat img, chans[3]; 
// img = imread(.....);  //make sure its loaded with an image

// //split the channels in order to manipulate them
// split(img, chans);

// //by default opencv put channels in BGR order , so in your situation you want to copy the first channel which is blue. Set green and red channels elements to zero.
// chans[1]=Mat::zeros(img.rows, img.cols, CV_8UC1); // green channel is set to 0
// chans[2]=Mat::zeros(img.rows, img.cols, CV_8UC1);// red channel is set to 0

// //then merge them back
// merge(chans, 3, img);

// //display 
// imshow("BLUE CHAN", img);
// cvWaitKey();
}

void displayingVideo()
{
    cv::VideoCapture vid;
    vid.open("VIDEO0005.mp4");
    cv::namedWindow("playing a vidoe file",1);

    for(;;)
    {
        cv::Mat frame;
        vid >> frame; // get a new frame from camera
        imshow("playing a vidoe file", frame);
        if(cv::waitKey(20) >= 0) break;
    }
    
}

int main(int argc, char** argv)
{
    //accessingPixel();
    //createMatrix();
    //readWriteImage();
    //matrixOperations();
    //readWriteImage();
    //matrixOperations();
    //drawingFunsctionAndCoordinate();
}




