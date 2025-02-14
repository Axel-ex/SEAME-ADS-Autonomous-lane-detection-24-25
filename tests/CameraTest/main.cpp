#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;

int main()
{
    std::string pipeline =
        "nvarguscamerasrc ! video/x-raw(memory:NVMM), format=NV12, width=1920, "
        "height=1080, framerate=30/1 ! nvvidconv ! video/x-raw, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink";
    VideoCapture capt(pipeline, cv::CAP_GSTREAMER);
    if (!capt.isOpened())
    {
        std::cerr << "Fail opening the cam\n";
        return -1;
    }

    Mat frame;
    capt.read(frame);

    if (frame.empty())
    {
        std::cerr << "Empty frame\n";
        capt.release();
        return -1;
    }
    imwrite("test.jpg", frame);
    waitKey();
    capt.release();

    return 0;
}
