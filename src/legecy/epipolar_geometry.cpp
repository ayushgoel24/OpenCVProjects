


#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>

#include <iomanip> 




//http://docs.opencv.org/3.0-beta/doc/py_tutorials/py_calib3d/py_epipolar_geometry/py_epipolar_geometry.html

double max_dist;
double min_dist;


template <typename Type>
Type calcMedian(std::vector<Type> scores)
{
    Type median;
    size_t size = scores.size();

    std::sort(scores.begin(), scores.end());

    min_dist=scores.at(0);
    max_dist=scores.at(scores.size()-1);

    if (size  % 2 == 0)
    {
        median = (scores[size / 2 - 1] + scores[size / 2]) / 2;
    }
    else
    {
        median = scores[size / 2];
    }

    return median;
}

/*
How to run:
    ./epipolar_geometry ../images/stereo_vision/tsucuba_left.png ../images/stereo_vision/tsucuba_right.png 

*/

int main(int argc, char ** argv)
{
    
    char* n_argv[] = { "epipolar_geometry", "../images/stereo_vision/tsucuba_left.png", "../images/stereo_vision/tsucuba_right.png"};

    int length = sizeof(n_argv)/sizeof(n_argv[0]);

    argc=length;
    argv = n_argv;
    
    cv::Mat left_image= cv::imread(argv[1]);
    cv::Mat right_image= cv::imread(argv[2]);
    std::vector<cv::KeyPoint> left_image_sift_keypoints, right_image_sift_keypoints, left_image_good_sift_keypoints, right_image_good_sift_keypoints;

    cv::KeyPoint left_image_good_keypoint, right_image_good_keypoint;

    cv::SiftFeatureDetector sift_detector;
    sift_detector.detect(left_image,left_image_sift_keypoints );
    sift_detector.detect(right_image,right_image_sift_keypoints);


    cv::SiftDescriptorExtractor sift_descriptor_extractor;



    cv::Mat right_image_sift_descriptor, left_image_sift_descriptor;


    sift_descriptor_extractor.compute(left_image,left_image_sift_keypoints,left_image_sift_descriptor);
    sift_descriptor_extractor.compute(right_image,right_image_sift_keypoints,right_image_sift_descriptor);


    std::vector<cv::DMatch> matches;

//     cv::FlannBasedMatcher flann_based_matcher;
//     flann_based_matcher.match(left_image_sift_descriptor,left_image_sift_descriptor,matches);


    
    cv::BFMatcher matcher(cv::NORM_L1, true);
/* from the file SURF_Homography.cpp 
    matcher matchs from first param (descriptors_object) to second param (descriptors_scene)
    matcher.match( descriptors_object, descriptors_scene, matches );
    
    so here we match from left_image_sift_descriptor to the right_image_sift_descriptor
*/
    matcher.match( left_image_sift_descriptor, right_image_sift_descriptor, matches );
    


    std::vector<double> distances;



     for( std::size_t i = 0; i < matches.size(); i++ )
        distances.push_back(matches[i].distance);


    double median_distance=calcMedian<double>(distances);

    std::cout<<"-- number of matches : "<< matches.size() <<std::endl;

    std::cout<<"-- Max dist : "<< max_dist <<std::endl;
    std::cout<<"-- Min dist : "<< min_dist <<std::endl;
    std::cout<<"-- median_distance : "<< median_distance <<std::endl;

    
    
/*    */
    double k=4;

    std::vector <cv::DMatch> good_matches;
    for(std::size_t i=0;i<matches.size();i++)
    {
        if(matches.at(i).distance< median_distance/k )
        {
            good_matches.push_back(matches.at(i));
	    /*
		obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
		scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
		query is from left image
		train is from right
		becaus we did populate match like this:
		matcher.match( left_image_sift_descriptor, right_image_sift_descriptor, matches );
	    */
	    left_image_good_keypoint=left_image_sift_keypoints.at( matches.at(i).queryIdx);
        std::cout<<"matches.at("<<i<<").queryIdx: "<<matches.at(i).queryIdx << std::endl;
	    right_image_good_keypoint=right_image_sift_keypoints.at( matches.at(i).trainIdx) ;
        std::cout<<"matches.at("<<i<<").trainIdx: "<<matches.at(i).trainIdx<< std::endl;
	    /*
	     actually we don't need right/left_image_good_sift_keypoints cuz 
	     queryIdx and trainIdx points into index in right/left_image_sift_keypoints
	     */
            left_image_good_sift_keypoints.push_back(left_image_good_keypoint);
	    right_image_good_sift_keypoints.push_back(right_image_good_keypoint);
        }
    }

    
//This will draw good mtaches
 
/**/
    cv::Mat img_matches;
    cv::drawMatches( left_image, left_image_sift_keypoints, right_image, right_image_sift_keypoints,
                   good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                   std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    cv::imshow( "Good Matches", img_matches );

//This will draw all mtaches   
    
/*
    cv::Mat img_matches;
    cv::drawMatches( left_image, left_image_sift_keypoints, right_image, right_image_sift_keypoints,
                   matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                   std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    cv::imshow( "All Matches", img_matches );
*/

    

//This wil draw all keypoints
/*
    cv::drawKeypoints(left_image, left_image_sift_keypoints,left_image);
    cv::imshow("left image sift key points",left_image);


    cv::drawKeypoints(right_image, right_image_sift_keypoints,right_image);
    cv::imshow("right image sift key points",right_image);
*/    
    
//This will draw good keypoints    
/*    */   
    cv::drawKeypoints(left_image, left_image_good_sift_keypoints,left_image);
    cv::imshow("left image good sift key points",left_image);


    cv::drawKeypoints(right_image, right_image_good_sift_keypoints,right_image);
    cv::imshow("right image good sift key points",right_image);
 


    
    
    
    std::vector<cv::Point2f>left_imgpts,right_imgpts;
    for( unsigned int i = 0; i<good_matches.size(); i++ )
    {
      // queryIdx is the "left" image
      left_imgpts.push_back(left_image_sift_keypoints[good_matches[i].queryIdx].pt);
      // trainIdx is the "right" image
      right_imgpts.push_back(right_image_sift_keypoints[good_matches[i].trainIdx].pt);
    }
    
    cv::Mat F;
    F =  cv::findFundamentalMat  (left_imgpts, right_imgpts, cv::FM_RANSAC, 0.1, 0.99);

//    std::cout<<"Fundemental matrix using FM_RANSAC algorithm:" <<std::endl;
//    std::cout<< std::fixed;
//    std::cout<< std::setprecision(6)<< F<<std::endl;
//    std::cout<<"-------------------------------------------" <<std::endl;
    
    
    std::cout<<"Fundemental matrix using FM_8POINT algorithm:" <<std::endl;
    F =  cv::findFundamentalMat  (left_imgpts, right_imgpts, cv::FM_8POINT, 0,0);
//    std::cout<<F <<std::endl;
	
    std::vector<cv::Vec3f> lines1,lines2;
    cv::computeCorrespondEpilines(left_imgpts,1,F,lines1);
    cv::computeCorrespondEpilines(left_imgpts,2,F,lines2);
    
    
    cv::Mat image_out=right_image.clone();
    for (std::vector<cv::Vec3f>::const_iterator it = lines1.begin(); it!=lines1.end(); ++it)
    {
        // Draw the line between first and last column
        cv::Point begin=cv::Point(0,-(*it)[2]/(*it)[1]);
        cv::Point end= cv::Point(right_image.cols,-((*it)[2]+ (*it)[0]*(right_image.cols)/(*it)[1]) );
        cv::line(image_out, begin,end,cv::Scalar(255,255,255),1);
    }
    
    
    cv::imshow("Correspond Epilines from points in image",image_out);

    cv::waitKey(0);
//Hartley’s algorithm 
/*
Hartley’s algorithm attempts to fi nd homographies that map the epipoles to infi nity
while minimizing the computed disparities between the two stereo images; it does this
simply by matching points between two image pairs. 
 
*/    
    cv::Mat F_mat_from_hartley_algorithm (F.rows, F.cols, F.type());
    std::cout<<F_mat_from_hartley_algorithm <<std::endl;


    cv::Mat Hl(4,4, left_image.type());
    cv::Mat Hr(4,4, left_image.type());


    double threshold=100;
    cv::Size image_size = left_image.size();

    cv::Mat left_imgpts_mat=  cv::Mat(left_imgpts);
    cv::Mat right_imgpts_mat=   cv::Mat(right_imgpts);





//    for(std::size_t i=0;i<left_imgpts.size();i++)
//    {
//        cv::Point2f point=left_imgpts.at(i);
//        std::cout<< point.x << " "<<point.y <<std::endl;
//    }

//    std::cout<<"-------------------------------------------" <<std::endl;


//    for(std::size_t i=0;i<right_imgpts.size();i++)
//    {
//        cv::Point2f point=right_imgpts.at(i);
//        std::cout<< point.x << " "<<point.y <<std::endl;
//    }

    cv::stereoRectifyUncalibrated(left_imgpts, right_imgpts, F, image_size ,Hl, Hr,threshold);

    std::cout<<"-------------------------------------------" <<std::endl;
    std::cout<<"Hl:" <<std::endl;
    std::cout<<Hl <<std::endl;
    std::cout<<"Hr:" <<std::endl;
    std::cout<<Hr <<std::endl;

    
//Bouguet’s algorithm
//    cv::stereoRectify();
    
//cv::initUndistortRectifyMap();    
    return 0;

}

