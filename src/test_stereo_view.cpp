#include <opencv2/opencv.hpp>
#include "Camera.hpp"
#include "AravisBackend.hpp"
#include <cstdio>
#include <string>
#include <vector>
#include <thread>
#include <chrono>

using namespace cynlr::camera;

int main() {
    printf("Stereo Camera Live View with Focus Control\n");

    // Enumerate all available cameras
    printf("\nScanning for cameras...\n");
    auto devices = AravisBackend::listCameras();
    if (devices.empty()) {
        printf("ERROR: No cameras found!\n");
        return -1;
    }

    printf("Found %zu camera(s):\n", devices.size());
    for (size_t i = 0; i < devices.size(); i++) {
        printf("  [%zu] %s\n", i, devices[i].c_str());
    }

    if (devices.size() < 2) {
        printf("ERROR: Need at least 2 cameras for stereo view, found %zu\n", devices.size());
        return -1;
    }

    // Use first two cameras as left and right
    std::string left_name  = devices[0];
    std::string right_name = devices[1];
    printf("\nUsing:\n  Left:  %s\n  Right: %s\n\n", left_name.c_str(), right_name.c_str());

    // Create backends
    auto left_backend = AravisBackend::create(left_name.c_str());
    if (!left_backend) {
        printf("Failed to create left camera backend: %s\n", left_name.c_str());
        return -1;
    }

    auto right_backend = AravisBackend::create(right_name.c_str());
    if (!right_backend) {
        printf("Failed to create right camera backend: %s\n", right_name.c_str());
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

    // Enable 3.3V lens power on both cameras
    printf("Enabling 3.3V lens power...\n");
    abortOnError(left_cam.enableLensPower(true));
    abortOnError(right_cam.enableLensPower(true));

    // Setup lens serial communication (may not be available on USB3 cameras — non-fatal)
    printf("Setting up lens serial...\n");
    auto left_serial_err = left_cam.setupLensSerial("Baud57600");
    if (left_serial_err.has_value())
        printf("  Left serial setup skipped: %s (focus may still work via FileAccess)\n", left_serial_err->message);
    auto right_serial_err = right_cam.setupLensSerial("Baud57600");
    if (right_serial_err.has_value())
        printf("  Right serial setup skipped: %s (focus may still work via FileAccess)\n", right_serial_err->message);

    // Initial focus voltage (middle of 24-70V range)
    double focus_voltage = 47.0;
    double focus_step = 0.5;

    printf("Setting initial focus to %.1fV...\n", focus_voltage);
    abortOnError(left_cam.setLensFocus(focus_voltage));
    abortOnError(right_cam.setLensFocus(focus_voltage));

    // Start acquisition
    printf("Starting acquisition...\n");
    abortOnError(left_cam.startAcquisition());
    abortOnError(right_cam.startAcquisition());

    // Allow buffers to fill
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    cv::namedWindow("Left Camera", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Right Camera", cv::WINDOW_AUTOSIZE);

    printf("\n=== Controls ===\n");
    printf("  w / UP arrow  : Focus up   (+step)\n");
    printf("  x / DOWN arrow: Focus down (-step)\n");
    printf("  e / Page Up   : Focus up   (+5.0V)\n");
    printf("  z / Page Down : Focus down (-5.0V)\n");
    printf("  +/-           : Change step size\n");
    printf("  s             : Save snapshots\n");
    printf("  q / ESC       : Quit\n");
    printf("  Focus range   : 24.0V - 70.0V\n");
    printf("================\n\n");

    FrameBuffer left_frame{}, right_frame{};
    bool left_borrowed = false, right_borrowed = false;

    while (true) {
        // Release previous frames
        if (left_borrowed)  { left_cam.releaseFrame(left_frame);  left_borrowed = false; }
        if (right_borrowed) { right_cam.releaseFrame(right_frame); right_borrowed = false; }

        // Borrow new frames
        auto left_err = left_cam.borrowOldestFrame(left_frame);
        if (left_err.has_value()) continue;
        left_borrowed = true;

        auto right_err = right_cam.borrowOldestFrame(right_frame);
        if (right_err.has_value()) continue;
        right_borrowed = true;

        // Wrap in cv::Mat (zero-copy)
        cv::Mat left_mat(left_frame.height, left_frame.width, CV_8UC1, left_frame.data);
        cv::Mat right_mat(right_frame.height, right_frame.width, CV_8UC1, right_frame.data);

        // Draw focus info overlay on both frames
        cv::Mat left_display, right_display;
        cv::cvtColor(left_mat, left_display, cv::COLOR_GRAY2BGR);
        cv::cvtColor(right_mat, right_display, cv::COLOR_GRAY2BGR);

        char info_text[128];
        snprintf(info_text, sizeof(info_text), "Focus: %.1fV  Step: %.1fV", focus_voltage, focus_step);

        cv::putText(left_display, info_text, cv::Point(10, 25),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        cv::putText(left_display, "UP/DOWN: focus  +/-: step  s: save  q: quit", cv::Point(10, 50),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 200, 200), 1);

        cv::putText(right_display, info_text, cv::Point(10, 25),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

        cv::imshow("Left Camera", left_display);
        cv::imshow("Right Camera", right_display);

        int key = cv::waitKeyEx(1);
        if (key == 'q' || key == 27) break;

        bool focus_changed = false;

        switch (key) {
            case 'w': case 'W': case 2490368: // w or UP arrow
                focus_voltage = std::min(focus_voltage + focus_step, 70.0);
                focus_changed = true;
                break;
            case 'x': case 'X': case 2621440: // x or DOWN arrow
                focus_voltage = std::max(focus_voltage - focus_step, 24.0);
                focus_changed = true;
                break;
            case 'e': case 'E': case 2162688: // e or Page Up
                focus_voltage = std::min(focus_voltage + 5.0, 70.0);
                focus_changed = true;
                break;
            case 'z': case 'Z': case 2228224: // z or Page Down
                focus_voltage = std::max(focus_voltage - 5.0, 24.0);
                focus_changed = true;
                break;
            case '+': case '=':
                focus_step = std::min(focus_step * 2.0, 5.0);
                printf("Step size: %.1fV\n", focus_step);
                break;
            case '-': case '_':
                focus_step = std::max(focus_step / 2.0, 0.1);
                printf("Step size: %.1fV\n", focus_step);
                break;
            case 's':
                cv::imwrite("left_snapshot.png", left_mat);
                cv::imwrite("right_snapshot.png", right_mat);
                printf("Snapshots saved (focus=%.1fV).\n", focus_voltage);
                break;
        }

        if (key != -1 && !focus_changed) {
            printf("Key pressed: %d (0x%X)\n", key, key);
        }

        if (focus_changed) {
            printf("Focus: %.1fV\n", focus_voltage);
            auto lerr = left_cam.setLensFocus(focus_voltage);
            if (lerr.has_value())
                printf("  Left setLensFocus FAILED: %s\n", lerr->message);
            auto rerr = right_cam.setLensFocus(focus_voltage);
            if (rerr.has_value())
                printf("  Right setLensFocus FAILED: %s\n", rerr->message);
        }
    }

    // Cleanup
    if (left_borrowed)  left_cam.releaseFrame(left_frame);
    if (right_borrowed) right_cam.releaseFrame(right_frame);

    abortOnError(left_cam.stopAcquisition());
    abortOnError(right_cam.stopAcquisition());

    // Disable lens power
    left_cam.enableLensPower(false);
    right_cam.enableLensPower(false);

    cv::destroyAllWindows();
    printf("Done. Final focus was %.1fV\n", focus_voltage);
    return 0;
}
