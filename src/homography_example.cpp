#include <opencv2/opencv.hpp>
//#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

/*
getPerspectiveTransform ->needs 4 points, based on SVD
findHomography -> N points, based on RANSAC and SVD


perspectiveTransform -> Performs the Homography matrix transformation on points
warpPerspective -> If you want to transform an image using perspective transformation, use warpPerspective()
*/
/**********************************************Finding homography Matrix from 4 Corresponding Points***********************************************/
void get_homographyMatrix()
{
    std::vector<cv::Point2f> obj;
    std::vector<cv::Point2f> scene;
    std::vector<cv::Point2f> obj_projection;

    cv::Point2f A,B,C,D,A_P,B_P,C_P,D_P ;
    A.x=0;
    A.y=0;

    B.x=150;
    B.y=0;

    C.x=150;
    C.y=150;

    D.x=0;
    D.y=150;

    obj.push_back(A);
    obj.push_back(B);
    obj.push_back(C);
    obj.push_back(D);


    A_P.x=100;
    A_P.y=100;

    B_P.x=200;
    B_P.y=80;

    C_P.x=220;
    C_P.y=80;

    D_P.x=100;
    D_P.y=200;

    scene.push_back(A_P);
    scene.push_back(B_P);
    scene.push_back(C_P);
    scene.push_back(D_P);

    cv::Mat homography_matrix= cv::getPerspectiveTransform(obj,scene);
    std::cout<<"Estimated Homography Matrix is:" <<std::endl;
    std::cout<< homography_matrix <<std::endl;

    perspectiveTransform( obj, obj_projection, homography_matrix);
    for(std::size_t i=0;i<obj_projection.size();i++)
    {
        std::cout<<obj_projection.at(i).x <<"," <<obj_projection.at(i).y<<std::endl;
    }
}


/**********************************************Finding homography Matrix using RANSAC***********************************************/

void findHomography_Test(int argc, char** argv)
{
//	The input arrays src_points and dst_points can be either
//		1)N-by-2(pixel coordinates) matrices
//		2)N-by-3 (homogeneous coordinates)	matrices
//	The final argument, homography, is just a 	3-by-3 matrix to


    if( argc != 3 )
    {
        printf(" Usage: ./main_Homography <img1> <img2>\n");
        return;
    }

    cv::Mat img_object = cv::imread( argv[1], cv::IMREAD_GRAYSCALE );
    cv::Mat img_scene = cv::imread( argv[2], cv::IMREAD_GRAYSCALE);

    if( !img_object.data || !img_scene.data )
    {
        printf(" --(!) Error reading images \n"); return ;
    }

    //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 400;

    cv::SurfFeatureDetector detector( minHessian );

    std::vector<cv::KeyPoint> keypoints_object, keypoints_scene;

    detector.detect( img_object, keypoints_object );
    detector.detect( img_scene, keypoints_scene );

    //-- Step 2: Calculate descriptors (feature vectors)
    cv::SurfDescriptorExtractor extractor;

    cv::Mat descriptors_object, descriptors_scene;

    extractor.compute( img_object, keypoints_object, descriptors_object );
    extractor.compute( img_scene, keypoints_scene, descriptors_scene );

    //-- Step 3: Matching descriptor vectors using FLANN matcher
    cv::FlannBasedMatcher matcher;
    std::vector< cv::DMatch > matches;
    matcher.match( descriptors_object, descriptors_scene, matches );

    double max_dist = 0; double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptors_object.rows; i++ )
    {   double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    printf("-- Max dist : %f \n", max_dist );
    printf("-- Min dist : %f \n", min_dist );

    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
    std::vector< cv::DMatch > good_matches;

    for( int i = 0; i < descriptors_object.rows; i++ )
    { if( matches[i].distance < 1.5*min_dist )
        {
            good_matches.push_back( matches[i]);
        }
    }

    cv::Mat img_matches;
    cv::drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
    good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
    std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );


    //-- Localize the object from img_1 in img_2
    std::vector<cv::Point2f> obj;
    std::vector<cv::Point2f> scene;

    for( size_t i = 0; i < good_matches.size(); i++ )
    {
    //-- Get the keypoints from the good matches
        obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }

    cv::Mat H = findHomography( obj, scene, CV_RANSAC );

    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<cv::Point2f> obj_corners(4);
    obj_corners[0] = cv::Point(0,0); obj_corners[1] = cv::Point( img_object.cols, 0 );
    obj_corners[2] = cv::Point( img_object.cols, img_object.rows ); obj_corners[3] = cv::Point( 0, img_object.rows );
    std::vector<cv::Point2f> scene_corners(4);

    perspectiveTransform( obj_corners, scene_corners, H);


    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    cv::Point2f offset( (float)img_object.cols, 0);
    cv::line( img_matches, scene_corners[0] + offset, scene_corners[1] + offset, cv::Scalar(0, 255, 0), 4 );
    cv::line( img_matches, scene_corners[1] + offset, scene_corners[2] + offset, cv::Scalar( 0, 255, 0), 4 );
    cv::line( img_matches, scene_corners[2] + offset, scene_corners[3] + offset, cv::Scalar( 0, 255, 0), 4 );
    cv::line( img_matches, scene_corners[3] + offset, scene_corners[0] + offset, cv::Scalar( 0, 255, 0), 4 );

    //-- Show detected matches
    imshow( "Good Matches & Object detection", img_matches );

    cv::waitKey(0);

    return ;
}




/**********************************************Applying homography perspective***********************************************/


// We need 4 corresponding 2D points(x,y) to calculate homography.
std::vector<cv::Point2f> left_image;      // Stores 4 points(x,y) of the logo image. Here the four points are 4 corners of image.
std::vector<cv::Point2f> right_image;    // stores 4 points that the user clicks(mouse left click) in the main image.

// Image containers for main and logo image
cv::Mat imageMain;
cv::Mat imageLogo;

// Function to add main image and transformed logo image and show final output.
// Icon image replaces the pixels of main image in this implementation.
void showFinal(cv::Mat src1,cv::Mat src2)
{

    cv::Mat gray,gray_inv,src1final,src2final;
    cv::cvtColor(src2,gray,CV_BGR2GRAY);
    threshold(gray,gray,0,255,CV_THRESH_BINARY);
    //adaptiveThreshold(gray,gray,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,5,4);
    bitwise_not ( gray, gray_inv );
    src1.copyTo(src1final,gray_inv);
    src2.copyTo(src2final,gray);
    cv::Mat finalImage = src1final+src2final;
    cv::namedWindow( "output", cv::WINDOW_AUTOSIZE );
    cv::imshow("output",finalImage);
    cvWaitKey(0);

}

// Here we get four points from the user with left mouse clicks.
// On 5th click we output the overlayed image.
void on_mouse( int e, int x, int y, int d, void *ptr )
{
    if (e == cv::EVENT_LBUTTONDOWN )
    {
        if(right_image.size() < 4 )
        {

            right_image.push_back(cv::Point2f(float(x),float(y)));
            std::cout << x << " "<< y <<std::endl;
        }
        else
        {
            std::cout << " Calculating Homography " <<std::endl;
            // Deactivate callback
            cv::setMouseCallback("Display window", NULL, NULL);
            // once we get 4 corresponding points in both images calculate homography matrix
            cv::Mat H = findHomography(  left_image,right_image,0 );
            cv::Mat logoWarped;
            // Warp the logo image to change its perspective
            cv::warpPerspective(imageLogo,logoWarped,H,imageMain.size() );
            showFinal(imageMain,logoWarped);

        }

    }
}


int homographyPerspective_Test( int argc, char** argv )
{
//  We need tow argumemts. "Main image" and "logo image"
    if( argc != 3)
    {
        std::cout <<" Usage: error" << std::endl;
        return -1;
    }


// Load images from arguments passed.
    imageMain = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    imageLogo = cv::imread(argv[2], CV_LOAD_IMAGE_COLOR);
// Push the 4 corners of the logo image as the 4 points for correspondence to calculate homography.
    left_image.push_back(cv::Point2f(float(0),float(0)));
    left_image.push_back(cv::Point2f(float(0),float(imageLogo.rows)));
    left_image.push_back(cv::Point2f(float(imageLogo.cols),float(imageLogo.rows)));
    left_image.push_back(cv::Point2f(float(imageLogo.cols),float(0)));



    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "Display window", imageMain );


    cv::setMouseCallback("Display window",on_mouse, NULL );


//  Press "Escape button" to exit
    while(1)
    {
        int key=cvWaitKey(10);
        if((char)key==(char)27) break;
    }


    return 0;
}


int main(int argc, char** argv)
{


/*
    How to run:
    ./main ../images/homography/homography1.JPG  ../images/homography/homography2.JPG
*/

//    findHomography_Test(argc, argv);


    /*
    How to run:
    ./main  ../images/homography/main.jpg ../images/homography/logo.jpg
    */
//    homographyPerspective_Test(  argc, argv );


    get_homographyMatrix();
}
