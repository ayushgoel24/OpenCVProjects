#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>


template<typename to, typename from>
to lexical_cast(from const &x)
{
    std::stringstream os;
    to ret;
    os << x;
    os >> ret;
    return ret;
}

static bool readStringList( const std::string& filepath, std::vector<std::string>& l )
{
    std::string path=filepath.substr(0,filepath.find_last_of("/"));
    l.resize(0);
    cv::FileStorage fs(filepath, cv::FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    cv::FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != cv::FileNode::SEQ )
        return false;
    cv::FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
    {
//        std::cout<<(std::string)*it <<std::endl;
//        std::cout<<filepath.substr(0,filepath.find_last_of("/") )<<std::endl;
        l.push_back(path+"/"+(std::string)*it);
    }

    return true;
}


/***************************************calibrating pair of cameras ******************************************/

void stereo_calibartion_from_camera_pair(int argc, char** argv)
{

    int numBoards = 0; //minimum 4
    int numCornersHor; //usually 9
    int numCornersVer; //usually 6
    float square_size; //in our case it is 0.025192
/*
    char file_name[256];

    printf("Enter number of corners along width (usually 9): ");
    scanf("%d", &numCornersHor);

    printf("Enter number of corners along height (usually 6): ");
    scanf("%d", &numCornersVer);

    printf("Enter number of boards(number of chess boards desired for calibration, minimum 4): ");
    scanf("%d", &numBoards);

    printf("square size im meter (0.25192): ");
    scanf("%f", &square_size);

    printf("file name to save calibration data (): ");
    scanf("%s", &file_name);

    printf("Press n to acquire next image");
*/
    numBoards = 10;
    numCornersHor=9;
    numCornersVer=6;
    square_size= 0.025192;
    std::string file_name="my_stereo";

    int numSquares = numCornersHor * numCornersVer;
    cv::Size board_sz = cv::Size(numCornersHor, numCornersVer);

    //we set it VideoCapture(1)
    cv::VideoCapture capture_left = cv::VideoCapture(0);
    cv::VideoCapture capture_right = cv::VideoCapture(1);
    std::vector<std::vector<cv::Point3f> > object_points;
    std::vector<std::vector<cv::Point2f> > image_points_left;
    std::vector<std::vector<cv::Point2f> > image_points_right;
    std::vector<cv::Mat> left_images, right_images;
    cv::Mat tmp_img_left,tmp_img_right;

    std::vector<cv::Point2f> corners_left,corners_right;
    int successes=0;
    cv::Mat image_left, image_right;
    cv::Mat gray_image_left,gray_image_right;

    capture_left >> image_left;
    capture_right >> image_right;
    std::vector<cv::Point3f> obj;
    for(int j=0;j<numSquares;j++)
    {
        obj.push_back(cv::Point3f(  (j%numCornersHor) *square_size,(j/numCornersHor *square_size ) ,0.0f));
//        std::cout<<(j%numCornersHor) *square_size<<std::endl;
//        std::cout<<(j/numCornersHor *square_size )<<std::endl;
//        std::cout<<"--------------------------------------"<<std::endl;
    }


    while(successes<numBoards)
    {
        cv::cvtColor(image_left, gray_image_left, CV_BGR2GRAY);
        cv::cvtColor(image_right, gray_image_right, CV_BGR2GRAY);
        bool found_left = cv::findChessboardCorners(image_left, board_sz, corners_left, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
        bool found_right = cv::findChessboardCorners(image_right, board_sz, corners_right, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
        if(found_left && found_right)
        {
            cv::cornerSubPix(gray_image_left, corners_left, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            cv::cornerSubPix(gray_image_right, corners_right, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

            tmp_img_left=image_left.clone();
            tmp_img_right=image_right.clone();

            drawChessboardCorners(tmp_img_left, board_sz, corners_left, found_left);
            drawChessboardCorners(tmp_img_right, board_sz, corners_right, found_right);
        }


        if(tmp_img_left.empty())
            imshow("left", image_left);
        else
            imshow("left", tmp_img_left);

        if(tmp_img_right.empty())
            imshow("right", image_right);
        else
            imshow("right", tmp_img_right);


        int key=cv::waitKey(1);
        if((char)key==(char)27)
            return ;

        if((char)key==(char)110 && found_left!=0  && found_right!=0 )
        {
            std::cout<<"Snap stored!"<<std::endl;
            left_images.push_back(image_left.clone());
            right_images.push_back(image_right.clone());
            image_points_left.push_back(corners_left);
            image_points_right.push_back(corners_right);
            object_points.push_back(obj);
            successes++;
            if(successes>=numBoards)
                break;
        }
        capture_right >> image_right;
        capture_left >> image_left;
        tmp_img_left.release();
        tmp_img_right.release();
    }


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//    cv::FileStorage fs(file_name+std::string(".yml"), FileStorage::WRITE);
//    fs<<"imagelist";
    std::string image_file_name;
    for(std::size_t i=0;i<left_images.size();i++)
    {
        image_file_name="left_image"+lexical_cast<std::string >(i)+std::string(".jpg");
        cv::imwrite(image_file_name, left_images.at(i) );

//        fs<<"\n";
//        fs<<"\"" +image_file_name+"\"";

        image_file_name="right_image"+lexical_cast<std::string >(i)+std::string(".jpg");
        cv::imwrite(image_file_name,right_images.at(i));
//        fs<<image_file_name;


    }
//    fs<<"imagelist";
//    fs.release();
    std::cout << "Writing list of files Done." << std::endl;


    cv::Mat cameraMatrix[2], distCoeffs[2];
    cameraMatrix[0] = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix[1] = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat R, T, E, F;




    double rms = cv::stereoCalibrate(object_points, image_points_left, image_points_right,
                    cameraMatrix[0], distCoeffs[0],
                    cameraMatrix[1], distCoeffs[1],
                    image_left.size(), R, T, E, F,
                    cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                    CV_CALIB_FIX_ASPECT_RATIO +
                    CV_CALIB_ZERO_TANGENT_DIST +
                    CV_CALIB_SAME_FOCAL_LENGTH +
                    CV_CALIB_RATIONAL_MODEL +
                    CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);
    std::cout << "done with RMS error=" << rms << std::endl;


    std::cout << "Translation between two cameras:" << std::endl;
    std::cout << T << std::endl;


    std::cout << "Rotation between two cameras:" << std::endl;
    std::cout << R << std::endl;


    std::cout << "Essential Matrix (maps pose of cameras together)" << std::endl;
    std::cout << E << std::endl;


    std::cout << "Fundemental Matrix (maps point on camera image plane to the other camera image plane )" << std::endl;
    std::cout << F << std::endl;
}

/***************************************Reading image from file ******************************************/

void stereo_calibration_from_file(int argc, char ** argv)
{
    std::vector<std::string> imagelist;
    //std::string path_to_image_list_file="stereo_calib.xml";
    std::string path_to_image_list_file=argv[1];
//    std::string path_to_image_list_file="../images/stereo_calibration/stereo_calib.xml";
    std::string path=path_to_image_list_file.substr(0,path_to_image_list_file.find_last_of("/"))+"/";

    readStringList(path_to_image_list_file, imagelist);
    float square_size;
    cv::Size boardSize;
    int height, width;
    height=6;
    width=9;
    boardSize = cv::Size(width, height);
    square_size=0.025;




    std::vector<std::vector<cv::Point2f> > left_imagePoints, right_imagePoints;
    std::vector<std::vector<cv::Point3f> > objectPoints;
    cv::Size imageSize;

    std::vector<std::string> good_left_imageList,good_right_imageList;


    std::vector<cv::Point3f> obj;
    int numSquares=width*height;
    for(int j=0;j<numSquares;j++)
    {
        obj.push_back(cv::Point3f(  (j%width) *square_size,(j/width *square_size ) ,0.0f));
//        std::cout<< (j%width) <<","<<(j/width)  <<std::endl;
    }

    bool found = false;
    std::string filename;
    std::vector<cv::Point2f> corners;
    cv::Mat image_gray ,img;



    for(std::size_t i = 0; i < imagelist.size() ; i++ )
    {
        corners.clear();
        filename=imagelist.at(i);
        img = cv::imread(imagelist.at(i));
        imageSize = img.size();

        found=cv::findChessboardCorners(img,boardSize,corners,CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
        if(found)
        {
            cv::cvtColor(img,image_gray,cv::COLOR_RGB2GRAY);
            cv::cornerSubPix(image_gray,corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));




            if(filename.find("left")!=std::string::npos)
            {
                std::cout<<"L" <<std::endl;
                left_imagePoints.push_back(corners);

                objectPoints.push_back(obj);

            }
            if(filename.find("right")!=std::string::npos)
            {
                std::cout<<"R" <<std::endl;
                right_imagePoints.push_back(corners);
            }
        }
        cv::drawChessboardCorners(image_gray,boardSize,corners,found);
        cv::imshow("corners", image_gray);
        char c = (char)cv::waitKey(500);
        if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
            exit(-1);
    }


    cv::Mat left_cameraMatrix, right_cameraMatrix, left_distCoeffs,right_distCoeffs;
    left_cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    right_cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat R, T, E, F;

    double rms = cv::stereoCalibrate(objectPoints, left_imagePoints, right_imagePoints,
                    left_cameraMatrix, left_distCoeffs,
                    right_cameraMatrix, right_distCoeffs,
                    imageSize, R, T, E, F,
                    cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                    CV_CALIB_FIX_ASPECT_RATIO +
                    CV_CALIB_ZERO_TANGENT_DIST +
                    CV_CALIB_SAME_FOCAL_LENGTH +
                    CV_CALIB_RATIONAL_MODEL +
                    CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);
    std::cout << "done with RMS error=" << rms << std::endl;



    std::cout << "Translation between two cameras:" << std::endl;
    std::cout << T << std::endl;


    std::cout << "Rotation between two cameras:" << std::endl;
    std::cout << R << std::endl;


    std::cout << "Essential Matrix:" << std::endl;
    std::cout << E << std::endl;


    std::cout << "Fundemental Matrix:" << std::endl;
    std::cout << F << std::endl;

    int number_of_left_image=left_imagePoints.size();
    std::vector<cv::Vec3f> left_lines, right_lines;


    double err = 0;
    int npoints = 0;


    for(int i=0;i<number_of_left_image;i++)
    {
        int number_of_corners=left_imagePoints.at(0).size();
        cv::Mat left_image_cornor_points=cv::Mat(left_imagePoints.at(i));

        cv::undistortPoints(left_image_cornor_points, left_image_cornor_points, left_cameraMatrix, left_distCoeffs, cv::Mat(), left_cameraMatrix);
/*
Every point in first image is a line in the second image, so here we
for every chessboard corner point we find the epipolar line in the right image
the lines are stored in right_lines  in the form of ax+by+cz=0 so

for example for the first corner point
right_lines[0][0] is a
right_lines[0][1] is b
right_lines[0][2] is c

These are cordinate of third chessboard corner in the sixth left image
left_imagePoints[5][2].x
left_imagePoints[5][2].y

so this should be zero:
    right_lines[0][0]*left_imagePoints[5][2].x
    +
    left_imagePoints[5][2].y*right_lines[0][1]
    +
    right_lines[0][2]

*/
        cv::computeCorrespondEpilines(left_image_cornor_points, 1, F, right_lines);



        cv::Mat right_image_cornor_points=cv::Mat(right_imagePoints.at(i));

        cv::undistortPoints(right_image_cornor_points, right_image_cornor_points, right_cameraMatrix, right_distCoeffs, cv::Mat(), right_cameraMatrix);
        cv::computeCorrespondEpilines(right_image_cornor_points, 2, F, left_lines);




        for( int j = 0; j < number_of_corners; j++ )
        {
            double errij = fabs(left_imagePoints[i][j].x*left_lines[j][0] +
                                left_imagePoints[i][j].y*left_lines[j][1] + left_lines[j][2]) +
                           fabs(right_imagePoints[i][j].x*right_lines[j][0] +
                                right_imagePoints[i][j].y*right_lines[j][1] + right_lines[j][2]);
            err += errij;
        }
        npoints += number_of_corners;

    }
    std::cout << "total reprojection err = " <<  err<< std::endl;
    std::cout << "average reprojection err = " <<  err/npoints << std::endl;




    // save intrinsic parameters
    cv::FileStorage fs(path+"intrinsics.yml", CV_STORAGE_WRITE);
    if( fs.isOpened() )
    {
        fs << "M1" << left_cameraMatrix << "D1" << left_distCoeffs <<
            "M2" << right_cameraMatrix << "D2" << right_distCoeffs;
        fs.release();
    }
    else
        std::cout << "Error: can not save the intrinsic parameters\n";

    cv::Mat R1, R2, P1, P2, Q;
    cv::Rect validRoi[2];

    cv::stereoRectify(left_cameraMatrix, left_distCoeffs,
                  right_cameraMatrix, right_distCoeffs,
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  cv::CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

    fs.open(path+"extrinsics.yml", CV_STORAGE_WRITE);
    if( fs.isOpened() )
    {
        fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
        fs.release();
    }
    else
        std::cout << "Error: can not save the extrinsic parameters\n";



    // OpenCV can handle left-right
    // or up-down camera arrangements
    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));



    cv::Mat left_rmap_x,left_rmap_y, right_rmap_x, right_rmap_y;
    /*
// IF BY CALIBRATED (BOUGUET'S METHOD)
    if( useCalibrated )
    {
        // we already computed everything
    }
// OR ELSE HARTLEY'S METHOD
    else
 // use intrinsic parameters of each camera, but
 // compute the rectification transformation directly
 // from the fundamental matrix
    {
        vector<Point2f> allimgpt[2];
        for( k = 0; k < 2; k++ )
        {
            for( i = 0; i < nimages; i++ )
                std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
        }
        F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
        Mat H1, H2;
        stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

        R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
        R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
        P1 = cameraMatrix[0];
        P2 = cameraMatrix[1];
    }
*/


    //Precompute maps for cv::remap()
    cv::initUndistortRectifyMap(left_cameraMatrix, left_distCoeffs, R1, P1, imageSize, CV_16SC2, left_rmap_x, left_rmap_y);
    cv::initUndistortRectifyMap(right_cameraMatrix, right_distCoeffs, R2, P2, imageSize, CV_16SC2, right_rmap_x, right_rmap_y);

    //cv::stereoRectifyUncalibrated()



    std::string rectified_file_path;

    for(std::size_t i=0;i<imagelist.size();i++)
    {
        filename=imagelist.at(i);
        cv::Mat img = cv::imread(filename, 0), rectified_img ;


        if(filename.find("left")!=std::string::npos)
        {
            std::cout<<"LL" <<std::endl;
            cv::remap(img, rectified_img, left_rmap_x, left_rmap_y, CV_INTER_LINEAR);



        }
        if(filename.find("right")!=std::string::npos)
        {
            std::cout<<"RR" <<std::endl;
            cv::remap(img, rectified_img, right_rmap_x, right_rmap_y, CV_INTER_LINEAR);
        }
        rectified_file_path=filename.insert(filename.find_last_of("/")+1, std::string("rect_") );
//        std::cout<<  <<std::endl;

        cv::imwrite(rectified_file_path,rectified_img);

    }



}


//undistort -> after camera calibration we can remove the radial and tangential distortion
//rectify -> rectifing images it is the process that as if the two images taken with parallel image plane that are align
//disparity
/*

stereo rectifi cation is the process of “correcting” the individual images so that they appear as if they had been taken by
two cameras with row-aligned image planes -> stereo solution reduce to search

 */
void image_rectification_test()
{

}



/*
Stereo calibration is the process of computing the geometrical relationship between the two cameras in space.
 */
void stereo_calibration()
{

}



//http://wiki.ros.org/stereo_image_proc/Tutorials/ChoosingGoodStereoParameters
//http://wiki.ros.org/stereo_image_proc#stereo_image_proc-1
//http://wiki.ros.org/stereo_image_proc#stereo_image_proc-1


//Bouguet’s algorithm

void stereoRectify_example()
{

}

//Hartley’s algorithm
void stereoRectifyUncalibrated_example()
{
//    cv::stereoRectifyUncalibrated() -> when we use chesbaord i.e
//    cv::initUndistortRectifyMap() -> when we only have correspon
}


void initUndistortRectifyMap_example()
{


}


//Pollefeys calibartion


int main(int argc, char ** argv)
{

    char* n_argv[] = { "stereo_calibration_example", "../images/stereo_calibration/stereo_calib.xml"};

    int length = sizeof(n_argv)/sizeof(n_argv[0]);

    argc=length;
    argv = n_argv;

    stereo_calibration_from_file( argc, argv);
}



