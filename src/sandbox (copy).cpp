#include <opencv4/opencv2/opencv.hpp>


void calculateHitogram(char **argc)
{
    cv::Mat img=cv::imread(argc[1], cv::IMREAD_COLOR);
    std::vector<cv::Mat> channels;
    cv::split(img,channels);


    cv::Mat blueChannel =channels[0];
    cv::Mat greenChannel=channels[1];
    cv::Mat redChannel=channels[2];

    int numberOfBins=256;
    float range[]={0,256};
    const float * ranges[]={range};
    cv::Mat  blueHistogram, greenHistogram,redHistogram;

    cv::calcHist(&blueChannel,1,0,cv::Mat(),blueHistogram,1,&numberOfBins,ranges  );
    cv::calcHist(&greenChannel,1,0,cv::Mat(),greenHistogram,1,&numberOfBins,ranges  );
    cv::calcHist(&redChannel,1,0,cv::Mat(),redHistogram,1,&numberOfBins,ranges  );


    int hist_w = 512, hist_h = 400;
    int bin_w = cvRound( (double) hist_w/    numberOfBins );

    cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );


    double alpha = 0;
    double beta = histImage.rows;


    std::cout<< blueHistogram.depth() <<std::endl;
    std::cout<< blueHistogram.channels() <<std::endl;
    std::cout<< blueHistogram.dims <<std::endl;
    std::cout<< blueHistogram.rows <<std::endl;
    std::cout<< blueHistogram.cols <<std::endl;



    cv::normalize(blueHistogram, blueHistogram, alpha, beta, cv::NORM_MINMAX, -1, cv::Mat() );




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


    //cv::Point(x,y)

    cv::Point a,b;

    for( int i = 1; i <numberOfBins; i++ )
    {
        a=cv::Point( bin_w*(i-1), hist_h - cvRound(blueHistogram.at<float>(i-1)) );
        b=cv::Point( bin_w*(i), hist_h - cvRound(blueHistogram.at<float>(i)) );
        cv::line( histImage,a ,b , cv::Scalar( 255, 0, 0), 2, 8, 0  );


//        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(g_hist_normalized.at<float>(i-1)) ),
//              Point( bin_w*(i), hist_h - cvRound(g_hist_normalized.at<float>(i)) ),
//              Scalar( 0, 255, 0), 2, 8, 0  );
//        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(r_hist_normalized.at<float>(i-1)) ),
//              Point( bin_w*(i), hist_h - cvRound(r_hist_normalized.at<float>(i)) ),
//              Scalar( 0, 0, 255), 2, 8, 0  );
    }
    cv::imshow("histogram",histImage);
    cv::waitKey(0);
}

int main(int argc, char * argv[])
{
    if(argc == 1)
    {
        argv[1] = (char*)"../images/lena.jpg";
    }

    calculateHitogram(argv);
    return 0;
}

