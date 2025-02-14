#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;

int main()
{
    VideoCapture capt(
        "v4l2src device=/dev/video0 ! video/x-raw, format=RG10, width=1920, "
        "height=1080, framerate=30/1 ! bayer2rgb ! videoconvert ! appsink",
        cv::CAP_GSTREAMER);
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
        return -1;
    }
    imshow("captured frame", frame);
    waitKey();

    return 0;
}
