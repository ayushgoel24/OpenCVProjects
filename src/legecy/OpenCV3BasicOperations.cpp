#define CERES_FOUND 
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/cvstd.hpp>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <algorithm>

//#include <opencv2/xfeatures2d.hpp>
//#include <opencv2/xfeatures2d/nonfree.hpp>
//#include <opencv2/sfm.hpp>

#include <Eigen/Geometry>

//https://docs.opencv.org/3.1.0/d0/d13/classcv_1_1Feature2D.html
namespace featuresdetectors {enum FEATURES_DETECTORS
{
AgastFeatureDetector,
AKAZE,
BRISK,
FastFeatureDetector,
GFTTDetector,
KAZE,
MSER,
ORB,
SimpleBlobDetector,
MSDDetector,
SIFT,
StarDetector,
SURF
};
}

namespace featuresdescriptor {enum FEATURES_DESCRIPTOR
{
AKAZE,
BRISK,
KAZE,
ORB,
SimpleBlobDetector,
BriefDescriptorExtractor,
DAISY,
FREAK,
LATCH,
LUCID,
MSDDetector,
SIFT,
StarDetector,
SURF
};
}

//void test()
//{
//    double max_dist = 0; double min_dist = 100;

//    //-- Quick calculation of max and min distances between keypoints
//    for( int i = 0; i < descriptors_object.rows; i++ )
//    {   double dist = matches[i].distance;
//        if( dist < min_dist ) min_dist = dist;
//        if( dist > max_dist ) max_dist = dist;
//    }

//    printf("-- Max dist : %f \n", max_dist );
//    printf("-- Min dist : %f \n", min_dist );

//    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
//    std::vector< cv::DMatch > good_matches;

//    for( int i = 0; i < descriptors_object.rows; i++ )
//    { if( matches[i].distance < 1.5*min_dist )
//        {
//            good_matches.push_back( matches[i]);
//        }
//    }
//}

int dumb(int argc, char ** argv)
{

    /**/


        unsigned int microseconds=1000000;
        cv::VideoCapture webCam(0); // open the default camera
        webCam.set(cv::CAP_PROP_FRAME_WIDTH,640);
        webCam.set(cv::CAP_PROP_FRAME_HEIGHT,480);
        webCam.set(cv::CAP_PROP_FPS, 1);
        //webCam.set(cv::CAP_PROP_MODE, CV_CAP_MODE_YUYV);










        if(!webCam.isOpened())  // check if we succeeded
    //    return;

        cv::namedWindow("camera",1);
        for(;;)
        {
    //        Mat frame;
    //        webCam >> frame; // get a new frame from camera
    //        imshow("camera", frame);
            if(cv::waitKey(200) >= 0) break;

    //        Mat img_1 = imread( argv[1], IMREAD_GRAYSCALE );
    //        Mat img_2 = imread( argv[2], IMREAD_GRAYSCALE );

            cv::Mat img_1, img_2;
            webCam >> img_1;



            usleep(microseconds);


            webCam >> img_2;









            std::vector<cv::Mat> points2d;


            if( !img_1.data || !img_2.data )
            { std::cout<< " --(!) Error reading images " << std::endl; return -1; }
            //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
            int minHessian = 400;
            cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create();


    //        cv::xfeatures2d::DAISY;


    //SURF
    //SIFT
    //ORB
    //BRISK
    //MSER
    //GFTT
    //Harris
    //Dense
    //SimpleBlob

    //FAST
    //STAR



    //        cv::xfeatures2d::

            cv::Ptr<cv::xfeatures2d::SIFT> SIFT_detector = cv::xfeatures2d::SIFT::create();
            SIFT_detector->defaultNorm();


//            Ptr<ORB> ORB_detector = ORB::create();
//            //ORB_detector->set

//            Ptr<BRISK> BRISK_detector = BRISK::create();



    //        SIFT::create();
            detector->setHessianThreshold(minHessian);
            std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
            cv::Mat descriptors_1, descriptors_2;
//            detector->detectAndCompute( img_1, cv::Mat(), keypoints_1, descriptors_1 );
//            detector->detectAndCompute( img_2, cv::Mat(), keypoints_2, descriptors_2 );



            SIFT_detector->detectAndCompute( img_1, cv::Mat(), keypoints_1, descriptors_1 );
            SIFT_detector->detectAndCompute( img_2, cv::Mat(), keypoints_2, descriptors_2 );


            //-- Step 2: Matching descriptor vectors using FLANN matcher
            cv::FlannBasedMatcher matcher;
            std::vector< cv::DMatch > matches;
            matcher.match( descriptors_1, descriptors_2, matches );
            double max_dist = 0; double min_dist = 100;
            //-- Quick calculation of max and min distances between keypoints
            for( int i = 0; i < descriptors_1.rows; i++ )
            { double dist = matches[i].distance;
              if( dist < min_dist ) min_dist = dist;
              if( dist > max_dist ) max_dist = dist;
            }
            printf("-- Max dist : %f \n", max_dist );
            printf("-- Min dist : %f \n", min_dist );
            //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
            //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
            //-- small)
            //-- PS.- radiusMatch can also be used here.
            std::vector< cv::DMatch > good_matches;
            for( int i = 0; i < descriptors_1.rows; i++ )
            { if( matches[i].distance <= std::max(2*min_dist, 0.02) )
              {
                    good_matches.push_back( matches[i]);
                }
            }

            int number_of_frames=2;
            int number_of_points=good_matches.size();
            int number_of_rows=2;
    //        std::cout <<"good_matches:"<<good_matches.size() <<std::endl;

            cv::Mat_<double> frame1(number_of_rows, number_of_points),frame2(number_of_rows, number_of_points);
            double  p_f1_x,
                    p_f1_y,
                    p_f2_x,
                    p_f2_y;
            for (std::size_t i=0;i<good_matches.size();i++)
            {
                p_f1_x=keypoints_1[good_matches.at(i).queryIdx].pt.x;
                p_f1_y=keypoints_1[good_matches.at(i).queryIdx].pt.y;

                frame1(0,i)=p_f1_x;
                frame1(1,i)=p_f1_y;
            }

            for (std::size_t i=0;i<good_matches.size();i++)
            {
                p_f2_x=keypoints_2[good_matches.at(i).trainIdx].pt.x;
                p_f2_y=keypoints_2[good_matches.at(i).trainIdx].pt.y;
                frame2(0,i)=p_f2_x;
                frame2(1,i)=p_f2_y;
            }
            points2d.clear();

            points2d.push_back(cv::Mat(frame1));
            points2d.push_back(cv::Mat(frame2));


            const double f  = std::atof(argv[1]),
                         cx = std::atof(argv[2]), cy = std::atof(argv[3]);
            cv::Matx33d K = cv::Matx33d( f, 0, cx,
                                 0, f, cy,
                                 0, 0,  1);
            bool is_projective = true;


            std::cout <<"points2d.size():"<<        points2d.size()  <<std::endl;

            std::vector<cv::Mat> Rs_est, ts_est, points3d_estimated;
            //cv::::reconstruct(points2d, Rs_est, ts_est, K, points3d_estimated, is_projective);
            std::cout<<"Rotation:" <<std::endl;
            std::cout<<Rs_est.size() <<std::endl;

            std::cout<<"Translation:" <<std::endl;
            std::cout<<ts_est.size() <<std::endl;




            //-- Draw only "good" matches
            cv::Mat img_matches;
            cv::drawMatches( img_1, keypoints_1, img_2, keypoints_2,
                         good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                         std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
            //-- Show detected matches
            cv::imshow( "Good Matches", img_matches );
            cv::imshow( "Diff", img_1-img_2 );
    //        for( int i = 0; i < (int)good_matches.size(); i++ )
    //        {
    //            printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx );
    //        }
            cv::waitKey(10);
    //        return 0;






        }



}


void featureDetection( cv::Mat &image_in, std::vector<cv::KeyPoint> &keypoints,cv::Mat &descriptors,featuresdetectors::FEATURES_DETECTORS featureDetector, featuresdescriptor::FEATURES_DESCRIPTOR featureDescriptor)
{
    cv::Ptr<cv::Feature2D> features_detector;
    cv::Ptr<cv::Feature2D> features_descriptor;

     switch(featureDetector)
    {
        case featuresdetectors::AgastFeatureDetector :
             features_detector = cv::AgastFeatureDetector::create();
            break;
        case featuresdetectors::AKAZE :
            features_detector = cv::AKAZE::create();
            break;
        case featuresdetectors::BRISK:
            features_detector = cv::BRISK::create();
            break;
        case featuresdetectors::FastFeatureDetector:
            features_detector = cv::FastFeatureDetector::create();
            break;
        case featuresdetectors::GFTTDetector:
            features_detector = cv::GFTTDetector::create();
            break;
        case featuresdetectors::KAZE:
            features_detector = cv::KAZE::create();
            break;
        case featuresdetectors::MSER:
            features_detector = cv::MSER::create();
            break;
        case featuresdetectors::ORB:
            features_detector = cv::ORB::create();
            break;
        case featuresdetectors::SimpleBlobDetector:
            features_detector = cv::SimpleBlobDetector::create();
            break;
        case featuresdetectors::MSDDetector:
            features_detector = cv::xfeatures2d::MSDDetector::create();
            break;
        case featuresdetectors::SIFT:
            features_detector = cv::xfeatures2d::SIFT::create();
            break;
        case featuresdetectors::StarDetector:
            features_detector = cv::xfeatures2d::StarDetector::create();
            break;
        case featuresdetectors::SURF:
            features_detector = cv::xfeatures2d::SURF::create();
            break;
    }

    switch(featureDescriptor)
    {
        case featuresdescriptor::AKAZE :
            features_descriptor = cv::AKAZE::create();
            break;
        case featuresdescriptor::BRISK:
            features_descriptor = cv::BRISK::create();
            break;
        case featuresdescriptor::KAZE:
            features_descriptor = cv::KAZE::create();
            break;
        case featuresdescriptor::ORB:
            features_descriptor = cv::ORB::create();
            break;
        case featuresdescriptor::BriefDescriptorExtractor:
            features_descriptor = cv::xfeatures2d::BriefDescriptorExtractor::create();
            break;
        case featuresdescriptor::DAISY:
            features_descriptor = cv::xfeatures2d::DAISY::create();
            break;
        case featuresdescriptor::FREAK:
            features_descriptor = cv::xfeatures2d::FREAK::create();
            break;
        case featuresdescriptor::LATCH:
            features_descriptor = cv::xfeatures2d::LATCH::create();
            break;
        case featuresdescriptor::LUCID:
            features_descriptor = cv::xfeatures2d::LUCID::create();
            break;
        case featuresdescriptor::MSDDetector:
            features_descriptor = cv::xfeatures2d::MSDDetector::create();
            break;
        case featuresdescriptor::SIFT:
            features_descriptor = cv::xfeatures2d::SIFT::create();
            break;
        case featuresdescriptor::StarDetector:
            features_descriptor = cv::xfeatures2d::StarDetector::create();
            break;
        case featuresdescriptor::SURF:
            features_descriptor = cv::xfeatures2d::SURF::create();
            break;
    }

    features_detector->detect(image_in,keypoints,cv::Mat());
    features_descriptor->compute(image_in,keypoints,descriptors);
}

void featureDetection_test(int argc , char ** argv )
{
    cv::Mat image_in;
    cv::Mat image_out;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    std::string file_path=argv[1];
    image_in=cv::imread( file_path,cv::IMREAD_COLOR);
    featureDetection(image_in,keypoints,descriptors,featuresdetectors::AgastFeatureDetector,featuresdescriptor::DAISY);
    cv::drawKeypoints(image_in,keypoints,image_out);
    std::string window_title="Feature";
    cv::imshow(window_title, image_out);
    cv::waitKey(0);
}

void featureMatching( cv::Mat &descriptors_object, cv::Mat descriptors_scene, std::vector< cv::DMatch >& matches)
{

    cv::BFMatcher bruteForceMatching;
    //bruteForceMatching.match();
    //bruteForceMatching.knnMatch();
    cv::FlannBasedMatcher matcher;

    //matcher.radiusMatch();


    matcher.match( descriptors_object, descriptors_scene, matches );


}

void refineMathes(std::vector< cv::DMatch > &matches,cv::Mat &object_descriptors ,std::vector< cv::DMatch > &good_matches)
{

    double max_dist = 0; double min_dist = 100;

    std::vector<double> vec;



    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < object_descriptors.rows; i++ )
    {   double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
        vec.push_back(dist);
    }

//    size_t midIndex = vec.size()/9;
    size_t midIndex = 40;
    std::nth_element(vec.begin(), vec.begin() + midIndex, vec.end());


//    printf("-- Max dist : %f \n", max_dist );
//    printf("-- Min dist : %f \n", min_dist );
//    std::cout<<"the 5the element is: "<<vec[midIndex] <<std::endl;

    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )


    for( int i = 0; i < object_descriptors.rows; i++ )
    {
        if( matches[i].distance < vec[midIndex] )
        {
            good_matches.push_back( matches[i]);
        }
    }
}

void featureMatching_Test(int argc, char ** argv)
{
    cv::Mat image_object,image_scene;
    std::vector<cv::KeyPoint> object_keypoints,scene_keypoints;
    cv::Mat object_descriptors, scene_descriptors;

    image_scene=cv::imread( argv[1],cv::IMREAD_COLOR);
    image_object=cv::imread( argv[2],cv::IMREAD_COLOR);

    featureDetection(image_object,object_keypoints,object_descriptors,featuresdetectors::SIFT,featuresdescriptor::SURF);
    featureDetection(image_scene,scene_keypoints,scene_descriptors,featuresdetectors::SIFT,featuresdescriptor::SURF);

    std::vector< cv::DMatch > matches;
    featureMatching(object_descriptors, scene_descriptors, matches);


//    std::cout<<"matches"<<matches.size() <<std::endl;
    std::vector< cv::DMatch > good_matches;

    refineMathes(matches,object_descriptors ,good_matches);

    cv::Mat img_matches;
    cv::drawMatches( image_object, object_keypoints, image_scene, scene_keypoints,
    good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
    std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    std::string window_title="matches";
    cv::imshow(window_title, img_matches);
    cv::waitKey(0);
    cv::imwrite("mathes.jpg",img_matches);


    std::vector<cv::Point2f> points1Raw; //Raw points from Keypoints
    std::vector<cv::Point2f> points1; //Undistorted points
    std::vector<cv::Point2f> points2Raw;
    std::vector<cv::Point2f> points2;
    for(int k=0; k<good_matches.size(); k++)
    {
        points1Raw.push_back(object_keypoints[good_matches[k].queryIdx].pt);
        points2Raw.push_back(scene_keypoints[good_matches[k].trainIdx].pt);
    }

//    cv::undistortPoints(points1Raw, points1, cameraMatrixm, distCoeffsm);
//    cv::undistortPoints(points2Raw, points2, cameraMatrixm, distCoeffsm);

    points1=points1Raw;
    points2=points2Raw;
    std::vector<uchar> states;


//https://stackoverflow.com/questions/25251676/opencv-findfundamentalmat-very-unstable-and-sensitive

//    cv::Mat f = cv::findFundamentalMat(points1, points2, cv::FM_RANSAC, 3, 0.99, states);

    cv::Mat f = cv::findFundamentalMat(points1, points2, cv::FM_LMEDS, 3, 0.99, states);

//    cv::Mat f = cv::findFundamentalMat(points1, points2, cv::FM_7POINT, 3, 0.99, states);

//    cv::Mat f = cv::findFundamentalMat(points1, points2, cv::FM_8POINT, 3, 0.99, states);
	
	

    std::cout<<"Fundamental Mat is: " <<f <<std::endl;

    double err=0;
    for(int k=0; k<good_matches.size(); k++)
    {
        cv::Mat p1(3, 1, CV_64F);
        p1.at<double>(0, 0) = points1[k].x;
        p1.at<double>(1, 0) = points1[k].y;
        p1.at<double>(2, 0) = 1;
        cv::Mat p2(1, 3, CV_64F);
        p2.at<double>(0, 0) = points2[k].x;
        p2.at<double>(0, 1) = points2[k].y;
        p2.at<double>(0, 2) = 1;

        cv::Mat res = cv::abs(p2 * f * p1); // f computed matrix

        if((bool)states[k]) //if match considered inlier (in my strange case all)
            err = err + res.at<double>(0, 0); //accumulate errors

    }

    std::cout<<"Total error is: " <<err <<std::endl;


    double focal=1.0;
    cv::Point2d pp=cv::Point2d(10, 10);
    double threshold=1;
    double prob=0.999;
    cv::LMEDS;
    cv::Mat E,R,t,mask;

    E= cv::findEssentialMat(points1, points2,focal,pp, cv::RANSAC,prob,threshold,mask);
    std::cout<<"E: " <<E<<std::endl;
    cv::recoverPose(E, points1, points2, R, t, focal, pp, mask);
    std::cout<<"R: " <<R<<std::endl;
    std::cout<<"t: " <<t<<std::endl;



}

void findFundamentalMatrix(cv::Mat &image1,cv::Mat &image2,cv::Mat &fundamentalMatrix,double &error)
{
    //1)featureDetection
    std::vector<cv::KeyPoint> image1_keypoints,image2_keypoints;
    cv::Mat image1_descriptors,image2_descriptors;

    featureDetection(  image1, image1_keypoints,image1_descriptors,featuresdetectors::SIFT,featuresdescriptor::SURF);
    featureDetection(  image2, image2_keypoints,image2_descriptors,featuresdetectors::SIFT,featuresdescriptor::SURF);

    //2)featureMatching
    std::vector< cv::DMatch > matches;

    featureMatching( image1_descriptors, image2_descriptors,  matches);


    //)refineMathes
    std::vector< cv::DMatch > good_matches;
    refineMathes(matches,image2_descriptors, good_matches);



    std::vector<cv::Point2f> points1Raw; //Raw points from Keypoints
    std::vector<cv::Point2f> points1; //Undistorted points
    std::vector<cv::Point2f> points2Raw;
    std::vector<cv::Point2f> points2;
    for(int k=0; k<good_matches.size(); k++)
    {
        points1Raw.push_back(image1_keypoints[good_matches[k].queryIdx].pt);
        points2Raw.push_back(image2_keypoints[good_matches[k].trainIdx].pt);
    }

    //4)undistortion
//    cv::undistortPoints(points1Raw, points1, cameraMatrixm, distCoeffsm);
//    cv::undistortPoints(points2Raw, points2, cameraMatrixm, distCoeffsm);

    points1=points1Raw;
    points2=points2Raw;
    std::vector<uchar> states;

    //5)findFundamentalMat
//https://stackoverflow.com/questions/25251676/opencv-findfundamentalmat-very-unstable-and-sensitive
//  cv::findFundamentalMat(points1, points2, cv::FM_RANSAC, 3, 0.99, states);
//  cv::findFundamentalMat(points1, points2, cv::FM_7POINT, 3, 0.99, states);
//  cv::findFundamentalMat(points1, points2, cv::FM_8POINT, 3, 0.99, states);
    fundamentalMatrix = cv::findFundamentalMat(points1, points2, cv::FM_LMEDS, 3, 0.99, states);
    for(int k=0; k<good_matches.size(); k++)
    {
        cv::Mat p1(3, 1, CV_64F);
        p1.at<double>(0, 0) = points1[k].x;
        p1.at<double>(1, 0) = points1[k].y;
        p1.at<double>(2, 0) = 1;
        cv::Mat p2(1, 3, CV_64F);
        p2.at<double>(0, 0) = points2[k].x;
        p2.at<double>(0, 1) = points2[k].y;
        p2.at<double>(0, 2) = 1;
        cv::Mat res = cv::abs(p2 * fundamentalMatrix * p1); // f computed matrix
        if((bool)states[k]) //if match considered inlier (in my strange case all)
            error = error + res.at<double>(0, 0); //accumulate errors
    }

}

void findFundamentalMatrix_test(int argc, char ** argv)
{
    cv::Mat image1=cv::imread( argv[1],cv::IMREAD_COLOR);
    cv::Mat image2=cv::imread( argv[2],cv::IMREAD_COLOR);
    cv::Mat fundamentalMatrix;
    double error;
    findFundamentalMatrix(image1,image2,fundamentalMatrix,error);
    std::cout<<"Fundamental Matrix is: " <<fundamentalMatrix <<std::endl;
    std::cout<<"error is: " <<error <<std::endl;
}

void findEssentialMatrix(cv::Mat &image1,cv::Mat &image2, cv::Mat &cameraMatrix, cv::Mat &EssentialMatrix,cv::Mat &rotation,cv::Mat &translation)
{
    //1)featureDetection
    std::vector<cv::KeyPoint> image1_keypoints,image2_keypoints;
    cv::Mat image1_descriptors,image2_descriptors;

    featureDetection(  image1, image1_keypoints,image1_descriptors,featuresdetectors::SIFT,featuresdescriptor::SURF);
    featureDetection(  image2, image2_keypoints,image2_descriptors,featuresdetectors::SIFT,featuresdescriptor::SURF);

    //2)featureMatching
    std::vector< cv::DMatch > matches;

    featureMatching( image1_descriptors, image2_descriptors,  matches);


    //)refineMathes
    std::vector< cv::DMatch > good_matches;
    refineMathes(matches,image2_descriptors, good_matches);



    std::vector<cv::Point2f> points1Raw; //Raw points from Keypoints
    std::vector<cv::Point2f> points1; //Undistorted points
    std::vector<cv::Point2f> points2Raw;
    std::vector<cv::Point2f> points2;
    for(int k=0; k<good_matches.size(); k++)
    {
        points1Raw.push_back(image1_keypoints[good_matches[k].queryIdx].pt);
        points2Raw.push_back(image2_keypoints[good_matches[k].trainIdx].pt);
    }

    //4)undistortion
//    cv::undistortPoints(points1Raw, points1, cameraMatrixm, distCoeffsm);
//    cv::undistortPoints(points2Raw, points2, cameraMatrixm, distCoeffsm);

    points1=points1Raw;
    points2=points2Raw;
    std::vector<uchar> states;


    double focal=1.0;
    cv::Point2d pp=cv::Point2d(10, 10);
    double threshold=1;
    double prob=0.999;
    cv::LMEDS;
    cv::Mat mask;

    EssentialMatrix= cv::findEssentialMat(points1, points2,focal,pp, cv::RANSAC,prob,threshold,mask);
    cv::recoverPose(EssentialMatrix, points1, points2, rotation, translation, focal, pp, mask);
}

void findEssentialMatrix_Test(int argc, char** argv)
{
    std::string camera_calibration_path="front_webcam.yml";
    cv::FileStorage fs(camera_calibration_path,cv::FileStorage::READ);
    cv::Mat camera_matrix, distortion_coefficient;
    fs["camera_matrix"]>>camera_matrix;
    fs["distortion_coefficients"]>>distortion_coefficient;


//    std::cout<<"R: " <<R<<std::endl;
//    std::cout<<"t: " <<t<<std::endl;
}

void test()
{
//    unsigned int microseconds=1000000;
    unsigned int microseconds=0;
    cv::VideoCapture webCam(1); // open the default camera
    webCam.set(CV_CAP_PROP_FRAME_WIDTH,640);
    webCam.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    webCam.set(CV_CAP_PROP_FPS, 30);


    if(!webCam.isOpened())  // check if we succeeded
        return;
//    cv::namedWindow("camera",1);
    for(;;)
    {

        try
        {


            if(cv::waitKey(200) >= 0) break;
            cv::Mat image1, image2;
            webCam >> image1;
            usleep(microseconds);
            webCam >> image2;

            std::vector<cv::KeyPoint> image1_keypoints,image2_keypoints;
            cv::Mat image1_descriptors, image2_descriptors;

            featureDetection(image1,image1_keypoints,image1_descriptors,featuresdetectors::SIFT,featuresdescriptor::DAISY);
            featureDetection(image2,image2_keypoints,image2_descriptors,featuresdetectors::SIFT,featuresdescriptor::DAISY);


//            std::cout<<"image1_keypoints.size()"<<image1_keypoints.size() <<std::endl;
//            std::cout<<"image2_keypoints.size()"<<image2_keypoints.size() <<std::endl;


            std::vector< cv::DMatch > matches;
            featureMatching(image1_descriptors, image2_descriptors, matches);


            if(matches.size()<20)
                continue;

        //    std::cout<<"matches"<<matches.size() <<std::endl;
            std::vector< cv::DMatch > good_matches;

    //        std::cout<<"matches.size()"<<matches.size() <<std::endl;

            refineMathes(matches,image1_descriptors ,good_matches);



            if(good_matches.size()<9)
                continue;
    //        std::cout<<"good_matches.size()"<<good_matches.size() <<std::endl;
    /**/
            cv::Mat img_matches;
            cv::drawMatches( image1, image1_keypoints, image2, image2_keypoints,
            good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
            std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );





            std::vector<cv::Point2f> points1Raw; //Raw points from Keypoints
            std::vector<cv::Point2f> points1; //Undistorted points
            std::vector<cv::Point2f> points2Raw;
            std::vector<cv::Point2f> points2;
            for(int k=0; k<good_matches.size(); k++)
            {
                points1Raw.push_back(image1_keypoints[good_matches[k].queryIdx].pt);
                points2Raw.push_back(image2_keypoints[good_matches[k].trainIdx].pt);
            }

            //4)undistortion
        //    cv::undistortPoints(points1Raw, points1, cameraMatrixm, distCoeffsm);
        //    cv::undistortPoints(points2Raw, points2, cameraMatrixm, distCoeffsm);

            points1=points1Raw;
            points2=points2Raw;
            std::vector<uchar> states;

            cv::Mat f = cv::findFundamentalMat(points1, points2, cv::FM_RANSAC, 1, 0.99, states);

            double err=0;
            for(int k=0; k<good_matches.size(); k++)
            {
                cv::Mat p1(3, 1, CV_64F);
                p1.at<double>(0, 0) = points1[k].x;
                p1.at<double>(1, 0) = points1[k].y;
                p1.at<double>(2, 0) = 1;
                cv::Mat p2(1, 3, CV_64F);
                p2.at<double>(0, 0) = points2[k].x;
                p2.at<double>(0, 1) = points2[k].y;
                p2.at<double>(0, 2) = 1;

                cv::Mat res = cv::abs(p2 * f * p1); // f computed matrix

                if((bool)states[k]) //if match considered inlier (in my strange case all)
                    err = err + res.at<double>(0, 0); //accumulate errors

            }

//            std::cout<<"Total error is: " <<err <<std::endl;


    ////        std::string camera_calibration_path="/home/behnam/workspace/OpenCVProjects/build/front_webcam.yml";
    ////        cv::FileStorage fs(camera_calibration_path,cv::FileStorage::READ);
    ////        cv::Mat camera_matrix, distortion_coefficient;
    ////        fs["camera_matrix"]>>camera_matrix;
    ////        fs["distortion_coefficients"]>>distortion_coefficient;


    ///*
    // camera_matrix:
    //    |fx   0   cx|
    //    |0   fy   cy|
    //    |0    0    1|

    //*/




            double cx,cy,focal;
            cv::Point2d pp;

    //        focal=camera_matrix.at<double>(1,1);


    //        cx=camera_matrix.at<double>(0,2);
    //        cy=camera_matrix.at<double>(1,2);
//            cv::Point2d pp=cv::Point2d(cx, cy);


            //normalize coordinates

            focal=1036.169926;
            cx=308.412977;
            cy=270.068003;
            pp=cv::Point2d(cx,cy);

            //(do not normalize coordinates)
//            cx=320;
//            cy=240;
//            focal=1;
            //http://answers.opencv.org/question/65788/undistortpoints-findessentialmat-recoverpose-what-is-the-relation-between-their-arguments/
            //http://answers.opencv.org/question/179981/does-recoverpose-return-up-to-scale-or-correct-translation/
            //https://docs.opencv.org/3.0-beta/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#decomposeessentialmat
            //https://stackoverflow.com/questions/23114047/opencv-camera-relative-pose-estimation
            //https://github.com/PacktPublishing/OpenCV3-Computer-Vision-Application-Programming-Cookbook-Third-Edition


            //By decomposing E, you can only get the direction of the translation, so the function returns unit

            double threshold=1;
            double prob=0.999;
            cv::LMEDS;
            cv::Mat mask,rotation,translation;

            cv::Mat EssentialMatrix= cv::findEssentialMat(points1, points2,focal,pp, cv::LMEDS,prob,threshold,mask);



//            cv::correctMatches(E, imgpts1, imgpts2, imgpts1, imgpts2)

//  The R and t are the rotation and translation (in camera 2's coordinates) to get to camera 1.


            cv::recoverPose(EssentialMatrix, points1, points2, rotation, translation, focal, pp, mask);






    //        Eigen::Quaternion<double> q =  yawAngle*pitchAngle *rollAngle;

            Eigen::Matrix3d rotationMatrix;
            rotationMatrix(0,0)=rotation.at<double>(0,0);
            rotationMatrix(0,1)=rotation.at<double>(0,1);
            rotationMatrix(0,2)=rotation.at<double>(0,2);
            rotationMatrix(1,0)=rotation.at<double>(1,0);
            rotationMatrix(1,1)=rotation.at<double>(1,1);
            rotationMatrix(1,2)=rotation.at<double>(1,2);
            rotationMatrix(2,0)=rotation.at<double>(2,0);
            rotationMatrix(2,1)=rotation.at<double>(2,1);
            rotationMatrix(2,2)=rotation.at<double>(2,2);



            std::cout.precision(3);
            std::cout.setf(std::ios_base::fixed);


//            std::cout<<rotationMatrix <<std::endl;

//            Eigen::Quaterniond quaternion_mat(rotationMatrix);
//            std::cout<<"quaternion_mat.x(): " <<quaternion_mat.x()<<std::endl;
//            std::cout<<"quaternion_mat.y(): " <<quaternion_mat.y()<<std::endl;
//            std::cout<<"quaternion_mat.z(): " <<quaternion_mat.z()<<std::endl;
//            std::cout<<"quaternion_mat.w(): " <<quaternion_mat.w()<<std::endl;
//            std::cout<<"--------------------------" <<std::endl;
//            std::cout<<"translation: " <<std::endl;
//            //cv::norm(translation, cv::NORM_L2, cv::Mat());

            double minVal;
            double maxVal=0;
            cv::Point minLoc;
            cv::Point maxLoc;
            double scale=100;

            cv::minMaxLoc(translation, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());
            translation=translation/(std::abs(maxVal)*scale);


            std::cout.precision(3);
            std::cout.setf(std::ios_base::fixed);

            std::cout<<translation.at<double>(0,0)<<std::endl;
//            std::cout<<translation.at<double>(1,0)<<std::endl;
//            std::cout<<translation.at<double>(2,0)<<std::endl;




//            std::cout<<"roll is: " <<atan2( rotationMatrix(2,1),rotationMatrix(2,2) ) <<std::endl;
//            std::cout<<"pitch: " <<atan2( -rotationMatrix(2,0), std::pow( rotationMatrix(2,1)*rotationMatrix(2,1) +rotationMatrix(2,2)*rotationMatrix(2,2) ,0.5  )  ) <<std::endl;
//            std::cout<<"yaw is: " <<atan2( rotationMatrix(1,0),rotationMatrix(0,0) ) <<std::endl;



//            std::cout<<"EssentialMatrix: "<<std::endl;
//            std::cout<<EssentialMatrix<<std::endl;
////            std::cout<<"f: "<<std::endl;
////            std::cout<<f<<std::endl;



            std::string window_title="matches";
            cv::imshow(window_title, img_matches);

        }
               catch (const std::exception& e)
               { /* */ }


////        cv::waitKey(0);



    }
}

int main(int argc , char ** argv)
{
    //dumb(argc, argv);
//    featureDetection_test(argc, argv);
//    featureMatching_Test(argc, argv);

    test();
}
