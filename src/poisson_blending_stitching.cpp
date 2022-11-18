#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <iostream>

using namespace std;
using namespace cv;

typedef struct
{
    Point2f left_top;
    Point2f left_bottom;
    Point2f right_top;
    Point2f right_bottom;
} four_corners_t;

four_corners_t corners;

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

int main(int argc, char *argv[])
{
    // Mat image01 = imread("../res/images/bgc_right.png", 1); //右图
    // Mat image02 = imread("../res/images/bgc_left.png", 1);  //左图

    //在钢板数据集上的测试
    Mat image01 = imread("../res/images/test_right.jpg", 1); //右图
    Mat image02 = imread("../res/images/test_left.jpg", 1);  //左图

    //灰度图转换
    Mat image1, image2;
    cvtColor(image01, image1, cv::COLOR_BGR2GRAY);
    cvtColor(image02, image2, cv::COLOR_BGR2GRAY);

    //创建detector存放到KeyPoints中
    Ptr<SIFT> detector = SIFT::create(2000);
    vector<KeyPoint> keyPoint1, keyPoint2;
    detector->detect(image1, keyPoint1);
    detector->detect(image2, keyPoint2);

    Ptr<SiftDescriptorExtractor> descriptor = SiftDescriptorExtractor::create();
    Mat imageDesc1, imageDesc2;
    descriptor->compute(image1, keyPoint1, imageDesc1);
    descriptor->compute(image2, keyPoint2, imageDesc2);

    FlannBasedMatcher matcher;
    vector<vector<DMatch>> matchePoints;
    vector<DMatch> GoodMatchePoints;

    vector<Mat> train_desc(1, imageDesc1);
    matcher.add(train_desc);
    matcher.train();

    matcher.knnMatch(imageDesc2, matchePoints, 2);
    cout << "total match points: " << matchePoints.size() << endl;

    // Lowe's algorithm,获取优秀匹配点
    for (int i = 0; i < matchePoints.size(); i++)
    {
        if (matchePoints[i][0].distance < 0.4 * matchePoints[i][1].distance)
        {
            GoodMatchePoints.push_back(matchePoints[i][0]);
        }
    }

    Mat first_match;
    drawMatches(image02, keyPoint2, image01, keyPoint1, GoodMatchePoints, first_match);

    vector<Point2f> imagePoints1, imagePoints2;

    for (int i = 0; i < GoodMatchePoints.size(); i++)
    {
        imagePoints2.push_back(keyPoint2[GoodMatchePoints[i].queryIdx].pt);
        imagePoints1.push_back(keyPoint1[GoodMatchePoints[i].trainIdx].pt);
    }

    //获取图像1到图像2的投影映射矩阵 尺寸为3*3
    Mat homo = findHomography(imagePoints1, imagePoints2, cv::RANSAC);
    ////也可以使用getPerspectiveTransform方法获得透视变换矩阵，不过要求只能有4个点，效果稍差
    // Mat   homo=getPerspectiveTransform(imagePoints1,imagePoints2);
    cout << "变换矩阵为：\n"
         << homo << endl
         << endl; //输出映射矩阵

    //计算配准图的四个顶点坐标
    CalcCorners(homo, image01);
    cout << "left_top:" << corners.left_top << endl;
    cout << "left_bottom:" << corners.left_bottom << endl;
    cout << "right_top:" << corners.right_top << endl;
    cout << "right_bottom:" << corners.right_bottom << endl;

    //图像配准
    Mat imageTransform1, imageTransform2;
    warpPerspective(image01, imageTransform1, homo, Size(MAX(corners.right_top.x, corners.right_bottom.x), image02.rows));

    // imshow("imageTransform1", imageTransform1);

    //创建拼接后的图,需提前计算图的大小
    int dst_width = imageTransform1.cols; //取最右点的长度为拼接图的长度
    int dst_height = image02.rows;

    //首先得到重叠部分图像在各自对应的位置截图
    double mask_left = MIN(corners.left_top.x, corners.left_bottom.x); //开始位置，即重叠区域的左边界
    double mask_width = image02.cols - mask_left;                      //重叠区域的宽度

    Mat new_left(dst_height, dst_width, CV_8UC3);
    new_left.setTo(0);
    image02.copyTo(new_left(Rect(0, 0, image02.cols, image02.rows)));

    Mat_<Vec3f> l;
    image02.convertTo(l, CV_32F, 1.0 / 255.0); // Vec3f表示有三个通道，即 l[row][column][depth]
    Mat_<Vec3f> r;
    imageTransform1.convertTo(r, CV_32F, 1.0 / 255.0);

    // Mat dst(dst_height, dst_width, CV_8UC3);
    // dst.setTo(0);
    // imageTransform1.copyTo(dst(Rect(0, 0, imageTransform1.cols, imageTransform1.rows)));
    // // image02.copyTo(dst(Rect(0, 0, image02.cols, image02.rows)));
    // image02(Rect(0, 0, image02.cols - mask_width, image02.rows)).copyTo(dst(Rect(0, 0, image02.cols - mask_width, image02.rows)));
    // imshow("copy", dst);

    // //测试先补充一下trans图的左边部分
    // for (int i = 0; i < image02.rows; i++)
    // { //的图像
    //     for (int j = 0; j < image02.cols - mask_width; j++)
    //     {
    //         image_blend.at<uchar>(i, j * 3 + 0) = image02.at<uchar>(i, j * 3 + 0); // B通道
    //         image_blend.at<uchar>(i, j * 3 + 1) = image02.at<uchar>(i, j * 3 + 1); // G通道
    //         image_blend.at<uchar>(i, j * 3 + 2) = image02.at<uchar>(i, j * 3 + 2); // R通道
    //     }
    // }
    // imshow("imageTransform1", imageTransform1);
    // image02(Rect(0, 0, image02.cols - mask_width, image02.rows)).copyTo(imageTransform1(Rect(0, 0, image02.cols - mask_width, image02.rows)));
    // imshow("test", imageTransform1);
    //创建按照左边界拼接生成的目标底部
    Mat dst_accord_to_mask_left(dst_height, dst_width, CV_8UC3);
    dst_accord_to_mask_left.setTo(0);
    imageTransform1.copyTo(dst_accord_to_mask_left(Rect(0, 0, imageTransform1.cols, imageTransform1.rows)));
    image02(Rect(0, 0, image02.cols - mask_width, image02.rows)).copyTo(dst_accord_to_mask_left(Rect(0, 0, image02.cols - mask_width, image02.rows)));
    imshow("dst_accord_to_mask_left", dst_accord_to_mask_left);
    //创建按照右边界拼接生成的目标底部
    Mat dst_accord_to_mask_right(dst_height, dst_width, CV_8UC3);
    dst_accord_to_mask_right.setTo(0);
    imageTransform1.copyTo(dst_accord_to_mask_right(Rect(0, 0, imageTransform1.cols, imageTransform1.rows)));
    image02(Rect(0, 0, image02.cols, image02.rows)).copyTo(dst_accord_to_mask_right(Rect(0, 0, image02.cols, image02.rows)));
    // imshow("dst_accord_to_mask_right", dst_accord_to_mask_right);

    //执行泊松融合
    //按照opencv中使用poission算法的需要准备mask
    //这里随便选择根据左边或右边边界生成的目标dst的cols和rows即可
    Mat mask = 255 * Mat::ones(image02.rows, image02.cols, image02.depth());
    imwrite("mask.png", mask);
    Mat image_blend;
    Point p(mask.cols / 2, mask.rows / 2); //表示left图从右边图中的x,y位置开始放置
    seamlessClone(image02, dst_accord_to_mask_right, mask, p, image_blend, MIXED_CLONE);
    // imshow("result", image_blend);

    //修改image_blend的左边部分图片的RGB通道值和左边对应的原始图像保持一致
    for (int i = 0; i < image02.rows; i++)
    { //的图像
        for (int j = 0; j < mask_left; j++)
        {
            image_blend.at<uchar>(i, j * 3 + 0) = image02.at<uchar>(i, j * 3 + 0); // B通道
            image_blend.at<uchar>(i, j * 3 + 1) = image02.at<uchar>(i, j * 3 + 1); // G通道
            image_blend.at<uchar>(i, j * 3 + 2) = image02.at<uchar>(i, j * 3 + 2); // R通道
        }
    }
    imwrite("../output/poisson/poisson_blending_result.png", image_blend);
    imshow("poisson_blending_result", image_blend);

    waitKey();
    return 0;
}
