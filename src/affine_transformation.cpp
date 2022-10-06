#include <opencv2/opencv.hpp>

/*    
    The usual way to represent an Affine Transform is by using a  2x3 matrix:
	_      _    
       |a00 a01|
    A= |       |
       |a10 a11|
       |_     _|2x2
       
       _  _  
    B=|b00 | 
      |b10 |
      |_  _|
      
    
    M=[A B]  =|a00 a01 b00|
              |a10 a11 b10|
              
       
          |x|
        P=| |
          |y|    
          
          
          
        |x|
    T=A.| |+B=M.[x y 1]T
        |y|    

Questions:
A) we know X and T, what is M
B)We know M and X, To obtain T just T=M.X

https://docs.opencv.org/3.4/d4/d61/tutorial_warp_affine.html

*/      




void calculateAndApplyAffineTransformation(int argc, char **argv)
{
    std::string source_window = "Source image";
    std::string warp_window = "Warp";
    std::string warp_rotate_window = "Warp + Rotate";

    cv::Point2f srcTri[3];
    cv::Point2f dstTri[3];

    cv::Mat rot_mat( 2, 3, CV_32FC1 );
    cv::Mat warp_mat( 2, 3, CV_32FC1 );
    cv::Mat src, warp_dst, warp_rotate_dst;

    src = cv::imread( argv[1], cv::IMREAD_ANYCOLOR  );

    /// Set the dst image the same type and size as src
    warp_dst = cv::Mat::zeros( src.rows, src.cols, src.type() );

    /* 
    Set your 3 points to calculate the  Affine Transform
    The simplest way to define an affine transform is thus to set pts_src to three  corners in the source image 
    for example, the upper and lower left together with the upper right of the source image. 
    The mapping from the source to destination image is then entirely defined by specifying pts_dst, the locations to which
    these three points will be mapped in that destination image

    these 3 point define a triangle which  has corners like this:
                       
                           1                      
    0------1              /|
    |     /              / | 
    |    /              /  |
    |   /        ->  0 /   |
    |  /               \   |
    |2/                 \2 |
                        
    */
   
   
    srcTri[0] = cv::Point2f( 0,0 ); //src Top left
    srcTri[1] = cv::Point2f( src.cols - 1, 0 ); //src Top right
    srcTri[2] = cv::Point2f( 0, src.rows - 1 ); //src Bottom left offset


    /*
    since affine transform could be any combination of rotation, reflection, scaling, sheering, it doesn't preserve parallelism, 
    we can assign 3 point from source to 3 point in the destination

    */

    dstTri[0] = cv::Point2f( src.cols*0.0, src.rows*0.33 ); //dst Top left
    dstTri[1] =cv::Point2f( src.cols*0.85, src.rows*0.25 ); //dst Top right
    dstTri[2] =cv::Point2f( src.cols*0.15, src.rows*0.7 ); //dst Bottom left offset

    // Get the Affine Transform
    warp_mat = cv::getAffineTransform( srcTri, dstTri );

    // Apply the Affine Transform just found to the src image
    cv::warpAffine( src, warp_dst, warp_mat, warp_dst.size() );

    /// Show what you got
    cv::namedWindow( source_window, cv::WINDOW_AUTOSIZE );
    cv::imshow( source_window, src );

    cv::namedWindow( warp_window, cv::WINDOW_AUTOSIZE );
    cv::imshow( warp_window, warp_dst );


    cv::waitKey(0);

   return ;
    

    
}

void createAffineMatrixExample(int argc, char **argv)
{
    cv::Mat src, warp_dst, warp_rotate_dst;
    src = cv::imread( argv[1], cv::IMREAD_ANYCOLOR  );
    
    cv::Mat rot_mat( 2, 3, CV_32FC1 );
    cv::Mat warp_mat( 2, 3, CV_32FC1 );

    

    /// Set the dst image the same type and size as src
    warp_dst = cv::Mat::zeros( src.rows, src.cols, src.type() );

    /// Compute a rotation matrix with respect to the center of the image
    cv::Point center = cv::Point( src.cols/2, src.rows/2 );
    double angle = -50.0;
    double scale = 0.6;

    /// Get the rotation matrix with the specifications above
    rot_mat = cv::getRotationMatrix2D( center, angle, scale );

    std::cout<<"rot+scale matrix is: \n" <<rot_mat <<std::endl;

    /// Rotate the warped image
    cv::warpAffine( src, warp_dst,  rot_mat, warp_dst.size() );

    /// Show what you got
    std::string source_window = "Source image";
    std::string warp_window = "rot+scale";
    cv::namedWindow( source_window, cv::WINDOW_AUTOSIZE );
    cv::imshow( source_window, src );

    cv::namedWindow( warp_window, cv::WINDOW_AUTOSIZE );
    cv::imshow( warp_window, warp_dst );

    cv::waitKey(0);

   return ;
}

void estimat3DAffineion()
{
    //The function estimates an optimal 3D affine transformation between two 3D point sets using the RANSAC algorithm.
    //estimateAffine3D();
    return;
}

int main(int argc, char** argv)
{
    if( argc != 2)
    {
        std::cout <<" Usage: ./affine_transform <image> \n"<< std::endl;
        return -1;
    }
    calculateAndApplyAffineTransformation(argc, argv);
    createAffineMatrixExample(argc, argv);

    return 0;
}

