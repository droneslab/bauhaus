#include "ORBextractor.h"


int main(int argc, char *argv[])
{
    cv::Mat image = cv::imread("/home/sofiya/dataset/sequences/00/image_0/000000.png",cv::IMREAD_GRAYSCALE);

    orb_slam3::ORBextractor* orbextractor = new orb_slam3::ORBextractor(5*2000,1.2,8,20,7, 0, 1000);

    std::vector<int> vLapping = {0,0};

    std::vector<cv::KeyPoint> mvKeys;
    cv::Mat mDescriptors;

    auto monoLeft = orbextractor->extract(image,cv::Mat(),mvKeys,mDescriptors,vLapping);

    std::cout << "NUM extracted: " << monoLeft << std::endl;
}