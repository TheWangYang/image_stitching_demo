#include <fstream>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/stitching.hpp"
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

//创建结构体
typedef struct
{
    Point2f left_top;
    Point2f left_bottom;
    Point2f right_top;
    Point2f right_bottom;
} four_corners_t;

//定义对象变量
four_corners_t corners;

//计算边角的函数
void CalcCorners(const Mat &H, const Mat &src)
{
    double v2[] = {0, 0, 1};          //左上角
    double v1[3];                     //变换后的坐标值
    Mat V2 = Mat(3, 1, CV_64FC1, v2); //列向量
    Mat V1 = Mat(3, 1, CV_64FC1, v1); //列向量

    V1 = H * V2;
    //左上角(0,0,1)
    cout << "V2: " << V2 << endl;
    cout << "V1: " << V1 << endl;
    corners.left_top.x = v1[0] / v1[2];
    corners.left_top.y = v1[1] / v1[2];

    //左下角(0,src.rows,1)
    v2[0] = 0;
    v2[1] = src.rows;
    v2[2] = 1;
    V2 = Mat(3, 1, CV_64FC1, v2); //列向量
    V1 = Mat(3, 1, CV_64FC1, v1); //列向量
    V1 = H * V2;
    corners.left_bottom.x = v1[0] / v1[2];
    corners.left_bottom.y = v1[1] / v1[2];

    //右上角(src.cols,0,1)
    v2[0] = src.cols;
    v2[1] = 0;
    v2[2] = 1;
    V2 = Mat(3, 1, CV_64FC1, v2); //列向量
    V1 = Mat(3, 1, CV_64FC1, v1); //列向量
    V1 = H * V2;
    corners.right_top.x = v1[0] / v1[2];
    corners.right_top.y = v1[1] / v1[2];

    //右下角(src.cols,src.rows,1)
    v2[0] = src.cols;
    v2[1] = src.rows;
    v2[2] = 1;
    V2 = Mat(3, 1, CV_64FC1, v2); //列向量
    V1 = Mat(3, 1, CV_64FC1, v1); //列向量
    V1 = H * V2;
    corners.right_bottom.x = v1[0] / v1[2];
    corners.right_bottom.y = v1[1] / v1[2];
}

vector<Mat> imgs; //保存拼接的原始图像向量

//导入所有原始拼接图像函数
void parseCmdArgs(int argc, char **argv);
//这里配置是否使用GPU
// bool use_gpu = true;
int main(int argc, char *argv[])
{
    //导入拼接图像
    parseCmdArgs(argc, argv);
    Mat pano;
    Ptr<Stitcher> stitcher = Stitcher::create();
    Stitcher::Status status = stitcher->stitch(imgs, pano); //拼接
    if (status != Stitcher::OK)                             //判断拼接是否成功
    {
        cout << "Can't stitch images, error code = " << int(status) << endl;
        return -1;
    }
    namedWindow("laplace stitching", 0);
    imshow("laplace stitching", pano);
    // imwrite("c:/WorkSpace/opencv/全景拼接.jpg", pano);
    waitKey();
    return 0;
}

//导入所有原始拼接图像函数
void parseCmdArgs(int argc, char **argv)
{
    // //在进行融合之前使用图像配准算法对左右两幅图像进行配准
    // Mat image01 = imread("../res/images/bgc1_right.png", 1); //右图
    // Mat image02 = imread("../res/images/bgc1_left.png", 1);  //左图
    // imshow("p2", image01);
    // imshow("p1", image02);

    // //灰度图转换
    // Mat image1, image2;
    // cvtColor(image01, image1, cv::COLOR_BGR2GRAY);
    // cvtColor(image02, image2, cv::COLOR_BGR2GRAY);

    // //创建detector存放到KeyPoints中
    // Ptr<SIFT> detector = SIFT::create(2000);
    // vector<KeyPoint> keyPoint1, keyPoint2;
    // detector->detect(image1, keyPoint1);
    // detector->detect(image2, keyPoint2);

    // Ptr<SiftDescriptorExtractor> descriptor = SiftDescriptorExtractor::create();
    // Mat imageDesc1, imageDesc2;
    // descriptor->compute(image1, keyPoint1, imageDesc1);
    // descriptor->compute(image2, keyPoint2, imageDesc2);

    // FlannBasedMatcher matcher;
    // vector<vector<DMatch>> matchePoints;
    // vector<DMatch> GoodMatchePoints;

    // vector<Mat> train_desc(1, imageDesc1);
    // matcher.add(train_desc);
    // matcher.train();

    // matcher.knnMatch(imageDesc2, matchePoints, 2);
    // cout << "total match points: " << matchePoints.size() << endl;

    // // Lowe's algorithm,获取优秀匹配点
    // for (int i = 0; i < matchePoints.size(); i++)
    // {
    //     if (matchePoints[i][0].distance < 0.4 * matchePoints[i][1].distance)
    //     {
    //         GoodMatchePoints.push_back(matchePoints[i][0]);
    //     }
    // }

    // Mat first_match;
    // drawMatches(image02, keyPoint2, image01, keyPoint1, GoodMatchePoints, first_match);
    // imshow("first_match ", first_match);

    // vector<Point2f> imagePoints1, imagePoints2;

    // for (int i = 0; i < GoodMatchePoints.size(); i++)
    // {
    //     imagePoints2.push_back(keyPoint2[GoodMatchePoints[i].queryIdx].pt);
    //     imagePoints1.push_back(keyPoint1[GoodMatchePoints[i].trainIdx].pt);
    // }

    // //获取图像1到图像2的投影映射矩阵 尺寸为3*3
    // Mat homo = findHomography(imagePoints1, imagePoints2, cv::RANSAC);
    // ////也可以使用getPerspectiveTransform方法获得透视变换矩阵，不过要求只能有4个点，效果稍差
    // // Mat   homo=getPerspectiveTransform(imagePoints1,imagePoints2);
    // cout << "变换矩阵为：\n"
    //      << homo << endl
    //      << endl; //输出映射矩阵

    // //计算配准图的四个顶点坐标
    // CalcCorners(homo, image01);
    // cout << "left_top:" << corners.left_top << endl;
    // cout << "left_bottom:" << corners.left_bottom << endl;
    // cout << "right_top:" << corners.right_top << endl;
    // cout << "right_bottom:" << corners.right_bottom << endl;

    // //图像配准
    // Mat imageTransform1, imageTransform2;
    // warpPerspective(image01, imageTransform1, homo, Size(MAX(corners.right_top.x, corners.right_bottom.x), image02.rows));
    // // warpPerspective(image01, imageTransform2, adjustMat*homo, Size(image02.cols*1.3, image02.rows*1.8));
    // imshow("directly through the perspective matrix transformation", imageTransform1);
    // imwrite("trans1.jpg", imageTransform1);

    Mat img;
    //导入两张图片进行laplace融合
    img = imread("F:\\vscode_work_place\\cpp_test_demo\\res\\images\\bgc_left.png");
    imgs.push_back(img);
    img = imread("F:\\vscode_work_place\\cpp_test_demo\\res\\images\\bgc_right.png");
    // imgs.push_back(img);
    //直接将右边配准后的图像放入到待融合图像列表中
    imgs.push_back(img);
}
