#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main()
{
    Mat img_left = imread("eye.jpg");
    Mat img_right = imread("hand.jpg");

    Mat mask = 255 * Mat::ones(img_eye.rows, img_eye.cols, img_eye.depth());

    // Rect ROI(400, 950, img_eye.cols, img_eye.rows);
    // rectangle(img_hand, ROI, Scalar(0, 0, 255), 3, 4);
    // img_eye.copyTo(img_hand(ROI));//copy and paste

    // namedWindow("img_hand", CV_WINDOW_NORMAL);
    // imshow("img_hand", img_hand);
    // waitKey(0);

    Point p(580, 1050);
    Mat img_blend;
    seamlessClone(img_eye, img_hand, mask, p, img_blend, NORMAL_CLONE);
    namedWindow("img_blend", CV_WINDOW_NORMAL);
    imshow("img_blend", img_blend);
    waitKey(0);

    // imwrite("eye_in_hand.jpg", img_blend);
    return 0;
}