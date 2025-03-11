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
    int frame_width = 1920;
    int frame_height = 1080;
    int fps = 30;
    int duration_seconds = 120; // 2 minutes
    int total_frames = fps * duration_seconds;
    VideoWriter videoWriter("output.avi",
                            VideoWriter::fourcc('M', 'J', 'P', 'G'), fps,
                            Size(frame_width, frame_height));
    if (!videoWriter.isOpened())
    {
        std::cerr << "Error: Cannot open video writer\n";
        return -1;
    }
    Mat frame;
    int frame_count = 0;
    while (frame_count < total_frames)
    {
        capt.read(frame);
        if (frame.empty())
        {
            std::cerr << "Empty frame captured\n";
            break;
        }
        videoWriter.write(frame);
        frame_count++;
        // Show the frame while recording (optional)
        imshow("Recording", frame);
        // if (waitKey(1) == 27) // Press 'ESC' to stop early
        //    break;
    }
    capt.release();
    videoWriter.release();
    destroyAllWindows();
    std::cout << "Video recording completed: output.avi\n";
    return 0;
}
