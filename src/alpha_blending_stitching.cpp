#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

void OptimizeSeam(Mat &img1, Mat &trans, Mat &dst);

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
    //在demo数据集上的实验
    Mat image01 = imread("../res/images/bgc_right.png", 1); //右图
    Mat image02 = imread("../res/images/bgc_left.png", 1);  //左图

    //在钢板数据集上的测试
    // Mat image01 = imread("../res/images/test_right.jpg", 1); //右图
    // Mat image02 = imread("../res/images/test_left.jpg", 1);  //左图

    // imshow("p2", image01);
    // imshow("p1", image02);

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
    // imshow("first_match ", first_match);

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

    //计算配准图重叠部分的四个顶点坐标
    CalcCorners(homo, image01);
    cout << "left_top:" << corners.left_top << endl;
    cout << "left_bottom:" << corners.left_bottom << endl;
    cout << "right_top:" << corners.right_top << endl;
    cout << "right_bottom:" << corners.right_bottom << endl;

    //图像配准
    Mat imageTransform1, imageTransform2;
    warpPerspective(image01, imageTransform1, homo, Size(MAX(corners.right_top.x, corners.right_bottom.x), image02.rows));
    // warpPerspective(image01, imageTransform2, adjustMat*homo, Size(image02.cols*1.3, image02.rows*1.8));
    // imshow("directly through the perspective matrix transformation", imageTransform1);
    // imwrite("trans1.jpg", imageTransform1);

    //创建拼接后的图,需提前计算图的大小
    int dst_width = imageTransform1.cols; //取最右点的长度为拼接图的长度
    int dst_height = image02.rows;

    Mat dst(dst_height, dst_width, CV_8UC3);
    dst.setTo(0);

    imageTransform1.copyTo(dst(Rect(0, 0, imageTransform1.cols, imageTransform1.rows)));
    image02.copyTo(dst(Rect(0, 0, image02.cols, image02.rows)));

    OptimizeSeam(image02, imageTransform1, dst);
    imwrite("../output/alpha/alpha_blending_result.png", dst);
    imshow("alpha_blending_result", dst);

    waitKey();

    return 0;
}

//优化两图的连接处，使得拼接自然
void OptimizeSeam(Mat &img1, Mat &trans, Mat &dst)
{
    int start = MIN(corners.left_top.x, corners.left_bottom.x); //开始位置，即重叠区域的左边界

    double processWidth = img1.cols - start; //重叠区域的宽度
    int rows = dst.rows;
    int cols = img1.cols; //注意，是列数*通道数
    double alpha = 1;     // img1中像素的权重
    for (int i = 0; i < rows; i++)
    {
        uchar *p = img1.ptr<uchar>(i);  //获取第i行的首地址
        uchar *t = trans.ptr<uchar>(i); //获得变换之后的目标图像的首地址
        uchar *d = dst.ptr<uchar>(i);   //获得拼接后的图的首地址
        for (int j = start; j < cols; j++)
        {
            //如果遇到图像trans中无像素的黑点，则完全拷贝img1中的数据
            if (t[j * 3] == 0 && t[j * 3 + 1] == 0 && t[j * 3 + 2] == 0)
            {
                alpha = 1;
            }
            else
            {
                // img1中像素的权重，与当前处理点距重叠区域左边界的距离成正比，实验证明，这种方法确实好
                alpha = (processWidth - (j - start)) / processWidth;
            }

            d[j * 3] = p[j * 3] * alpha + t[j * 3] * (1 - alpha);
            d[j * 3 + 1] = p[j * 3 + 1] * alpha + t[j * 3 + 1] * (1 - alpha);
            d[j * 3 + 2] = p[j * 3 + 2] * alpha + t[j * 3 + 2] * (1 - alpha);
        }
    }
}
