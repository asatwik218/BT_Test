#include <opencv2/opencv.hpp>
#include "Camera.hpp"
#include "AravisBackend.hpp"
#include <cstdio>

using namespace cynlr::camera;

int main() {
    printf("Stereo Camera Live View\n");

    const char* left_name  = "FLIR-1E1001647846-01647846";
    const char* right_name = "FLIR-1E1001644882-01644882";

    // Create backends
    auto left_backend = AravisBackend::create(left_name);
    if (!left_backend) {
        printf("Failed to create left camera backend: %s\n", left_name);
        return -1;
    }

    auto right_backend = AravisBackend::create(right_name);
    if (!right_backend) {
        printf("Failed to create right camera backend: %s\n", right_name);
        return -1;
    }

    Camera left_cam(std::move(left_backend));
    Camera right_cam(std::move(right_backend));

    // Configure both cameras
    printf("Configuring cameras...\n");

    abortOnError(left_cam.stopAcquisition());
    abortOnError(left_cam.setPixelFormat(PixelFormat::MONO8));
    abortOnError(left_cam.setBinning(4, 4));
    abortOnError(left_cam.setFrameRate(60.0));
    abortOnError(left_cam.setAcquisitionMode(AcquisitionMode::ACQUISITION_MODE_CONTINUOUS));

    abortOnError(right_cam.stopAcquisition());
    abortOnError(right_cam.setPixelFormat(PixelFormat::MONO8));
    abortOnError(right_cam.setBinning(4, 4));
    abortOnError(right_cam.setFrameRate(60.0));
    abortOnError(right_cam.setAcquisitionMode(AcquisitionMode::ACQUISITION_MODE_CONTINUOUS));

    // Start acquisition
    printf("Starting acquisition...\n");
    abortOnError(left_cam.startAcquisition());
    abortOnError(right_cam.startAcquisition());

    cv::namedWindow("Left Camera", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Right Camera", cv::WINDOW_AUTOSIZE);

    printf("Streaming... Press 'q' to quit, 's' to save a snapshot.\n");

    FrameBuffer left_frame{}, right_frame{};
    bool left_borrowed = false, right_borrowed = false;

    while (true) {
        // Release previous frames
        if (left_borrowed)  { left_cam.releaseFrame(left_frame);  left_borrowed = false; }
        if (right_borrowed) { right_cam.releaseFrame(right_frame); right_borrowed = false; }

        // Borrow new frames
        auto left_err = left_cam.borrowNewestFrame(left_frame);
        if (left_err.has_value()) {
            printf("Left frame error: %s\n", left_err->message);
            continue;
        }
        left_borrowed = true;

        auto right_err = right_cam.borrowNewestFrame(right_frame);
        if (right_err.has_value()) {
            printf("Right frame error: %s\n", right_err->message);
            continue;
        }
        right_borrowed = true;

        // Wrap in cv::Mat (zero-copy)
        cv::Mat left_mat(left_frame.height, left_frame.width, CV_8UC1, left_frame.data);
        cv::Mat right_mat(right_frame.height, right_frame.width, CV_8UC1, right_frame.data);

        cv::imshow("Left Camera", left_mat);
        cv::imshow("Right Camera", right_mat);

        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) break;  // q or ESC
        if (key == 's') {
            cv::imwrite("left_snapshot.png", left_mat);
            cv::imwrite("right_snapshot.png", right_mat);
            printf("Snapshots saved.\n");
        }
    }

    // Cleanup
    if (left_borrowed)  left_cam.releaseFrame(left_frame);
    if (right_borrowed) right_cam.releaseFrame(right_frame);

    abortOnError(left_cam.stopAcquisition());
    abortOnError(right_cam.stopAcquisition());

    cv::destroyAllWindows();
    printf("Done.\n");
    return 0;
}
