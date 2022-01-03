#include <opencv2/opencv.hpp>



//http://opencv.willowgarage.com/documentation/cpp/geometric_image_transformations.html
/*    
    We can use an Affine Transformation to express:

    1)Rotations (linear transformation)
    2)Translations (vector addition)
    3)Scale operations (linear transformation)
    
    
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
        X=| |
          |y|    
          
          
          
        |x|
    T=A.| |+B=M.[x y 1]T
        |y|    

Questions:
A) we know X and T, what is M
B)We know M and X, To obtain T just T=M.X
       
*/      




void getAffineTransformation(char **argv)
{

    //http://docs.opencv.org/doc/tutorials/imgproc/imgtrans/warp_affine/warp_affine.html


    /// Global variables
    std::string source_window = "Source image";
    std::string warp_window = "Warp";
    std::string warp_rotate_window = "Warp + Rotate";
    cv::Point2f srcTri[3];
    cv::Point2f dstTri[3];

    cv::Mat rot_mat( 2, 3, CV_32FC1 );
    cv::Mat warp_mat( 2, 3, CV_32FC1 );
    cv::Mat src, warp_dst, warp_rotate_dst;

    /// Load the image
    src = cv::imread( argv[1], 1 );

    /// Set the dst image the same type and size as src
    warp_dst = cv::Mat::zeros( src.rows, src.cols, src.type() );

    /// Set your 3 points to calculate the  Affine Transform
//    The simplest way to define an affine transform is thus to set pts_src to three  corners in the source image 
//    for example, the upper and lower left together with the upper right of the source image. 
//     The mapping from the source to destination image is then entirely defined by specifying pts_dst, the locations to which
//     these three points will be mapped in that destination image
   
//   these 3 point define a triangle which  has corners like this:
//   o----o
//   |   /
//   |  /
//   |o/
//    /
   
   
   
    srcTri[0] = cv::Point2f( 0,0 ); //src Top left
    srcTri[1] = cv::Point2f( src.cols - 1, 0 ); //src Top right
    srcTri[2] = cv::Point2f( 0, src.rows - 1 ); //src Bottom left offset

    dstTri[0] = cv::Point2f( src.cols*0.0, src.rows*0.33 ); //dst Top left
    dstTri[1] =cv::Point2f( src.cols*0.85, src.rows*0.25 ); //dst Top right
    dstTri[2] =cv::Point2f( src.cols*0.15, src.rows*0.7 ); //dst Bottom left offset

    /// Get the Affine Transform
    warp_mat = cv::getAffineTransform( srcTri, dstTri );

    /// Apply the Affine Transform just found to the src image
    cv::warpAffine( src, warp_dst, warp_mat, warp_dst.size() );

   
    //that was all for getting Affine Transform, in the following we just doing some examples of rotation and ...
    // Rotating the image after Warp

    /// Compute a rotation matrix with respect to the center of the image
    cv::Point center = cv::Point( warp_dst.cols/2, warp_dst.rows/2 );
    double angle = -50.0;
    double scale = 0.6;

    /// Get the rotation matrix with the specifications above
    rot_mat = cv::getRotationMatrix2D( center, angle, scale );

    /// Rotate the warped image
    cv::warpAffine( warp_dst, warp_rotate_dst, rot_mat, warp_dst.size() );

    /// Show what you got
    cv::namedWindow( source_window, cv::WINDOW_AUTOSIZE );
    cv::imshow( source_window, src );

    cv::namedWindow( warp_window, cv::WINDOW_AUTOSIZE );
    cv::imshow( warp_window, warp_dst );

    cv::namedWindow( warp_rotate_window, cv::WINDOW_AUTOSIZE );
    cv::imshow( warp_rotate_window, warp_rotate_dst );

    /// Wait until user exits the program
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

    if(argc == 1)
    {
        argv[1] = strdup("../images/lena.jpg");
    }
    getAffineTransformation(argv);
}

