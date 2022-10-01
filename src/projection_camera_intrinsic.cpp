#include <opencv4/opencv2/opencv.hpp>
#include "csv.h"


void projection(std::string pathToPointFile="../data/points.csv")
{

/*
Points are in the following form (OpenCV coordinate):
 
                  Z
                ▲
               /
              /
             /1 2 3 4     X
            |------------ ⯈
           1|           
           2|       
           3|           
           4|           
            | Y
            ⯆

The parameters fx=f*mx  and fy=f*my  where mx=1/width and my=1/height  meaning size of 1 pixel in x and y

mx=1/width
my=1/height 

cx=Width/2;
cy=Height/2 ;

fx=f*mx

k=[fx  0  cx  
   0  fy  cy 
   0  0   1 ]

| u |  |fx*x +cx| 
| v	|= |fy*y +cy|

x or u=> column
z or v=> row
 
   		 	     _________________________________________
			  	|__|__|__|__|__|__|__|__|__|__|__|__|__|__|
		 	 	|__|__|__|__|__|__|__|__|__|__|__|__|__|__|
				|__|__|__|__|__|__|__|__|__|__|__|__|__|__|
				|__|__|__|__|__|__|__|__|__|__|__|__|__|__|
				|__|__|__|__|__|__|__|__|__|__|__|__|__|__|____________► x
				|__|__|__|__|__|__|__|__|__|__|__|__|__|__|
				|__|__|__|__|__|__|__|__|__|__|__|__|__|__|
				|__|__|__|__|__|__|__|__|__|__|__|__|__|__|
				|__|__|__|__|__|__|__|__|__|__|__|__|__|__|
				|__|__|__|__|__|__|__|__|__|__|__|__|__|__|
									 |	
			     					 |	
                                     ⯆ y

                Z
                ▲
               /
              /
             /1 2 3 4     X
            |------------ ⯈
           1|
           2|
           3|
           4|
            | Y
            ⯆

OpenCV Point(x,y) represent (column,row) 
0/0---column--->
 |
 |
row
 |
 |
 v

mat.at<type>(row,column) or mat.at<type>(cv::Point(x,y))
to access the same point if x=column and y=row

*/

//////////////////////////////////////////// camera intrinsic ///////////////////////////////////////////////////////

    int numberOfPixelInHeight,numberOfPixelInWidth;
    double heightOfSensor, widthOfSensor;
    double focalLength=0.2;
    double mx, my, cx, cy;

    numberOfPixelInHeight=480;
    numberOfPixelInWidth=640;
 
    heightOfSensor=10;
    widthOfSensor=10;

    mx=(numberOfPixelInWidth)/widthOfSensor;
    my=(numberOfPixelInHeight)/heightOfSensor ;

    cx=(numberOfPixelInWidth)/2;
    cy=(numberOfPixelInHeight)/2 ;

    cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) <<
    focalLength*mx,0,cx,
    0,focalLength*my,cy,
    0,0,1);

    std::cout<<"camera intrinsic:\n"<<cameraMatrix <<std::endl;

//////////////////////////////////////////// camera extrinsic ///////////////////////////////////////////////////////

    cv::Mat cameraRotation=cv::Mat::eye(3,3,CV_64FC1);
    cv::Mat cameraTranslation = (cv::Mat_<double>(1,3) << 0, 0, 0);

////////////////////////////////////////// 3D points from world  //////////////////////////////

 
    io::CSVReader<3> in(pathToPointFile);
    in.read_header(io::ignore_extra_column, "x", "y", "z");
    double x,y,z;
    std::vector<cv::Point3d> objectpoints;

    while(in.read_row(x,y,z))
    {
        objectpoints.push_back(cv::Point3d(x,y,z));
    }
          

    std::cout<< "points in 3d world:\n" <<std::endl;
    std::cout<< "                 Z"<<std::endl;
    std::cout<< "                ▲"<<std::endl;
    std::cout<< "               /"<<std::endl;
    std::cout<< "              /"<<std::endl;
    std::cout<< "             /1 2 3 4     X"<<std::endl;
    std::cout<< "            |------------ ⯈"<<std::endl;
    std::cout<< "           1|"<<std::endl;
    std::cout<< "           2|"<<std::endl;
    std::cout<< "           3|"<<std::endl;
    std::cout<< "           4|"<<std::endl;
    std::cout<< "            | Y"<<std::endl;
    std::cout<< "            ⯆"<<std::endl;


    for(const auto p:objectpoints)
        std::cout<<p <<std::endl;

    std::vector<cv::Point2d> projectedPoints;

//////////////////////////////////////////// projecting 3D points into camera ///////////////////////////////////////////////////////


    cv::Mat distortionCoefficient= (cv::Mat_<double>(5,1) <<0, 0, 0, 0, 0);
    cv::projectPoints(objectpoints, cameraRotation, cameraTranslation, cameraMatrix, distortionCoefficient, projectedPoints);
    std::cout<< "projected point in camera" <<std::endl;
    for(const auto p:projectedPoints)
        std::cout<<"row:" <<p.y <<"," <<"column:"<<p.x <<std::endl;


////////////////////////////////////////// 3D World Unit Vector  //////////////////////////////

    

    cv::Mat projectedPointsHomogenous;
    int cols= objectpoints.size();
    int rows = 3;
    
    projectedPointsHomogenous.create(rows,cols,CV_64FC1);
    for(int j=0;j< cols ;j++)
    {
        projectedPointsHomogenous.at<double>(0,j)=projectedPoints[j].x;
        projectedPointsHomogenous.at<double>(1,j)=projectedPoints[j].y;
        projectedPointsHomogenous.at<double>(2,j)=1;
    }

    
    cv::Mat rays =cameraMatrix.inv()*projectedPointsHomogenous; //put in world coordinates
    std::cout<< "camera rays" <<std::endl;
    std::cout<< rays<<std::endl;        

    std::cout<< "unit vector (normalized camera rays)" <<std::endl;
    rays *= 1/cv::norm(rays);
    std::cout<< rays<<std::endl;  

///////////////////////////////////////////////saving the image //////////////////////////////////////////
 
    double row, col;
 
    cv::Mat cameraImage=cv::Mat::zeros(numberOfPixelInHeight,numberOfPixelInWidth,CV_8UC1);
    cv::line(cameraImage,cv::Point2d(numberOfPixelInWidth/2,0),cv::Point2d(numberOfPixelInWidth/2,numberOfPixelInHeight),cv::Scalar(255,255,255));
    cv::line(cameraImage,cv::Point2d(0,numberOfPixelInHeight/2),cv::Point2d(numberOfPixelInWidth,numberOfPixelInHeight/2),cv::Scalar(255,255,255));
    for(std::size_t i=0;i<projectedPoints.size();i++)
    {
        col=int(projectedPoints.at(i).x);
        row=int(projectedPoints.at(i).y);
        std::cout<<row <<"," <<col  <<std::endl;
        cameraImage.at<char>(int(row),int(col))=char(255);
    }
    std::string fileName=std::string("image_")+std::to_string(focalLength)+ std::string("_.jpg");
    cv::imwrite(fileName,cameraImage);
}

int main(int argc, char * argv[])
{
    projection();
    return 0;
}