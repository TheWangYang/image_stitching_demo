#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

// void OptimizeSeam(Mat &img1, Mat &trans, Mat &dst);

class LaplacianBlending
{
private:
    Mat_<Vec3f> left;
    Mat_<Vec3f> right;
    Mat_<float> blendMask;

    vector<Mat_<Vec3f>> leftLapPyr, rightLapPyr, resultLapPyr; // Laplacian Pyramids
    Mat leftHighestLevel, rightHighestLevel, resultHighestLevel;
    vector<Mat_<Vec3f>> maskGaussianPyramid; // masks are 3-channels for easier multiplication with RGB

    int levels;

    void buildPyramids()
    {
        buildLaplacianPyramid(left, leftLapPyr, leftHighestLevel);
        buildLaplacianPyramid(right, rightLapPyr, rightHighestLevel);
        buildGaussianPyramid();
    }

    void buildGaussianPyramid()
    { //金字塔内容为每一层的掩模
        assert(leftLapPyr.size() > 0);

        maskGaussianPyramid.clear();
        Mat currentImg;
        cvtColor(blendMask, currentImg, COLOR_GRAY2BGR); // store color img of blend mask into maskGaussianPyramid
        maskGaussianPyramid.push_back(currentImg);       // 0-level

        currentImg = blendMask;
        for (int l = 1; l < levels + 1; l++)
        {
            Mat _down;
            if (leftLapPyr.size() > l)
                pyrDown(currentImg, _down, leftLapPyr[l].size());
            else
                pyrDown(currentImg, _down, leftHighestLevel.size()); // lowest level

            Mat down;
            cvtColor(_down, down, COLOR_GRAY2BGR);
            maskGaussianPyramid.push_back(down); // add color blend mask into mask Pyramid
            currentImg = _down;
        }
    }

    void buildLaplacianPyramid(const Mat &img, vector<Mat_<Vec3f>> &lapPyr, Mat &HighestLevel)
    {
        lapPyr.clear();
        Mat currentImg = img;
        for (int l = 0; l < levels; l++)
        {
            Mat down, up;
            pyrDown(currentImg, down);
            pyrUp(down, up, currentImg.size());
            Mat lap = currentImg - up;
            lapPyr.push_back(lap);
            currentImg = down;
        }
        currentImg.copyTo(HighestLevel);
    }

    Mat_<Vec3f> reconstructImgFromLapPyramid()
    {
        //将左右laplacian图像拼成的resultLapPyr金字塔中每一层
        //从上到下插值放大并相加，即得blend图像结果
        Mat currentImg = resultHighestLevel;
        for (int l = levels - 1; l >= 0; l--)
        {
            Mat up;

            pyrUp(currentImg, up, resultLapPyr[l].size());
            currentImg = up + resultLapPyr[l];
        }
        return currentImg;
    }

    void blendLapPyrs()
    {
        //获得每层金字塔中直接用左右两图Laplacian变换拼成的图像resultLapPyr
        resultHighestLevel = leftHighestLevel.mul(maskGaussianPyramid.back()) +
                             rightHighestLevel.mul(Scalar(1.0, 1.0, 1.0) - maskGaussianPyramid.back());
        for (int l = 0; l < levels; l++)
        {
            Mat A = leftLapPyr[l].mul(maskGaussianPyramid[l]);
            Mat antiMask = Scalar(1.0, 1.0, 1.0) - maskGaussianPyramid[l];
            Mat B = rightLapPyr[l].mul(antiMask);
            Mat_<Vec3f> blendedLevel = A + B;

            resultLapPyr.push_back(blendedLevel);
        }
    }

public:
    LaplacianBlending(const Mat_<Vec3f> &_left, const Mat_<Vec3f> &_right, const Mat_<float> &_blendMask, int _levels) : // construct function, used in LaplacianBlending lb(l,r,m,4);
                                                                                                                         left(_left), right(_right), blendMask(_blendMask), levels(_levels)
    {
        buildPyramids(); // construct Laplacian Pyramid and Gaussian Pyramid
        blendLapPyrs();  // blend left & right Pyramids into one Pyramid
    };

    Mat_<Vec3f> blend()
    {
        return reconstructImgFromLapPyramid(); // reconstruct Image from Laplacian Pyramid
    }
};

Mat_<Vec3f> LaplacianBlend(const Mat_<Vec3f> &l, const Mat_<Vec3f> &r, const Mat_<float> &m)
{
    LaplacianBlending lb(l, r, m, 256);
    return lb.blend();
}

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
    //在demo数据上的实验
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

    //保存目标图像
    image02(Rect(0, 0, mask_left, image02.rows)).copyTo(imageTransform1(Rect(0, 0, mask_left, image02.rows)));
    imwrite("dst.png", imageTransform1);

    Mat new_left(dst_height, dst_width, CV_8UC3);
    new_left.setTo(0);
    image02.copyTo(new_left(Rect(0, 0, image02.cols, image02.rows)));
    //保存源图像
    imwrite("src.png", new_left);

    Mat_<Vec3f> l;
    new_left.convertTo(l, CV_32F, 1.0 / 255.0); // Vec3f表示有三个通道，即 l[row][column][depth]
    Mat_<Vec3f> r;
    imageTransform1.convertTo(r, CV_32F, 1.0 / 255.0);

    // create blend mask matrix m
    Mat_<float> m(l.rows, l.cols, 0.0);                      //将m全部赋值为0
    m(Range::all(), Range(0, mask_left + mask_width)) = 255; //取m全部行&[0,m.cols/2]列，赋值为1.0
    cout << m.cols / 2 << endl;
    cout << m.rows / 2 << endl;
    //保存mask掩膜
    imwrite("mask.png", m);

    Mat_<Vec3f> blend = LaplacianBlend(l, r, m);
    // imwrite("../output/laplace/laplace_blending_result_xxx_level.png", blend);
    imshow("laplace_blending_result_xxx_level", blend);

    waitKey();
    return 0;
}
