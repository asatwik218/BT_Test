#include <opencv2/opencv.hpp>
#include "Camera.hpp"
#include "AravisBackend.hpp"
#include <cstdio>

using namespace cynlr::camera;

int main() {
    printf("Camera Grabber Test\n");

    const char* cam_name = "FLIR-1E1001579921-01579921";
    auto aravisBackend = AravisBackend::create(cam_name);
    if (aravisBackend == nullptr) {
        printf("Aravis backend camera could not be created\n");
        return -1;
    }
    Camera cam(std::move(aravisBackend));

    printf("Configuring camera...\n");

    abortOnError(cam.stopAcquisition());
    abortOnError(cam.setPixelFormat(PixelFormat::MONO8));
    abortOnError(cam.setBinning(4, 4));
    abortOnError(cam.setFrameRate(60.0));
    abortOnError(cam.setAcquisitionMode(AcquisitionMode::ACQUISITION_MODE_CONTINUOUS));

    printf("Starting acquisition...\n");
    abortOnError(cam.startAcquisition());

    FrameBuffer curr_frame;
    FrameBuffer prev_frame;

    // Borrow first frame
    abortOnError(cam.borrowNewestFrame(prev_frame));
    printf("First frame: %dx%d\n", prev_frame.width, prev_frame.height);

    // Grab a second frame
    abortOnError(cam.borrowNewestFrame(curr_frame));
    printf("Second frame: %dx%d\n", curr_frame.width, curr_frame.height);

    // Save to disk
    cv::Mat cv_frame(curr_frame.height, curr_frame.width, CV_8UC1, curr_frame.data);
    bool ok = cv::imwrite("C:/Users/SatwikAgarwal/Documents/cynlr_software/behaviour_trees_exploration/test_capture.png", cv_frame);
    printf("Saved test_capture.png: %s\n", ok ? "SUCCESS" : "FAILED");

    // Cleanup
    cam.releaseFrame(prev_frame);
    cam.releaseFrame(curr_frame);

    abortOnError(cam.stopAcquisition());
    printf("Done.\n");
    return 0;
}
