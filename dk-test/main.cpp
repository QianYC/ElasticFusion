#include <iostream>
#include <k4a/k4a.hpp>
#include <opencv2/opencv.hpp>
#include <memory>

using namespace std;

int main() {
    /**
     * connect to dk
     */
    uint32_t count = k4a::device::get_installed_count();
    cout << "found " << count << " device" << endl;
    if (count < 1) {
        cout << "found no device!" << endl;
        return -1;
    }

    k4a::device device;
    for (int i = 0; i < count; ++i) {
        try {
            device = k4a::device::open(i);
        } catch (k4a::error &error) {
            cout << "failed to open " << i << " device" << endl;
            continue;
        }
        cout << "succeeded to open " << i << " device" << endl;
        break;
    }
    if (!device) {
        cout << "failed to open any device!" << endl;
        return -2;
    }

    /**
     * set configuration
     */
    k4a_device_configuration_t conf = {
            K4A_IMAGE_FORMAT_COLOR_BGRA32,
            K4A_COLOR_RESOLUTION_720P,
            K4A_DEPTH_MODE_NFOV_UNBINNED,
            K4A_FRAMES_PER_SECOND_30,
            true,
            0,
            K4A_WIRED_SYNC_MODE_STANDALONE,
            0,
            false
    };
    k4a::calibration calibration = device.get_calibration(conf.depth_mode, conf.color_resolution);

    /**
     * start dk
     */
    device.start_cameras(&conf);
    device.start_imu();
    std::shared_ptr<k4a::capture> capture_ptr = std::make_shared<k4a::capture>();
    while (true) {
        if (!device.get_capture(capture_ptr.get(), std::chrono::milliseconds(K4A_WAIT_INFINITE))) {
            cout << "failed to read camera!" << endl;
            return -3;
        }
        k4a::image color = capture_ptr->get_color_image();
        k4a::image depth = capture_ptr->get_depth_image();
        cout << "rgb size : " << color.get_size() << endl;
        cout << "depth size : " << depth.get_size() << endl;
        cv::Mat mColor(color.get_width_pixels(), color.get_height_pixels(), CV_8UC4, (void *) color.get_buffer());
        cv::Mat mDepth(depth.get_width_pixels(), depth.get_height_pixels(), CV_16U, (void *) depth.get_buffer());

        cv::imshow("rgb", mColor);
        cv::imshow("depth", mDepth);
        cv::waitKey(1);
    }
    return 0;
}