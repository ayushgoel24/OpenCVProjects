#include <opencv2/opencv.hpp>


/*
OpenCV coordinate:

In OpenCV, cv::Point(x,y) represent (column,row)
x (or u) means column and y (or v) means row
 
                  Z
                ▲
               /
              /
             /1 2 3 4     x or u means column
            |------------ ⯈
           1|           
           2|       
           3|           
           4|           
            | y or v means row
            ⯆

mat.at<type>(row,column) or mat.at<type>(cv::Point(x,y))
to access the same point if x=column and y=row



0/0---column--->
 |
 |
row
 |
 |
 v

*/    


void coordinateSystemDisplay()
{

    int blue, green, red;
    blue=255;
    green=255;
    red=255;
    int rows, cols;

    rows=400;
    cols=600;

    cv::Mat img = cv::Mat::zeros(rows,cols, CV_8UC1);
    std::cout<<"rows:" <<img.rows  <<std::endl;
    std::cout<<"cols:" <<img.cols  <<std::endl;
    std::string windowTitle="OpenCV Coordinate System";
    cv::namedWindow(windowTitle,cv::WINDOW_AUTOSIZE);
    


    // draw a circle at (300,300) with a radius of 20. Use green lines of width 1
    cv::circle(img, cv::Point( 300,100), 20, cv::Scalar(blue,green,red), 1);


    cv::Vec3b color;
    color[0] = blue;
    color[1] = green;
    color[2] = red;

    img.at<cv::Vec3b>( cv::Point( 300,100))=cv::Vec3f(blue,green,red);


    cv::imshow(windowTitle,img);
    cv::waitKey(0);
}

int main()
{
    coordinateSystemDisplay();
}
