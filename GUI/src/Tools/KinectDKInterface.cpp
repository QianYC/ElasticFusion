//
// Created by yc_qian on 20-4-26.
//

#include "KinectDKInterface.hpp"

KinectDKInterface::KinectDKInterface(int camIdx) :
        initSuccessful(true),
        capture_ptr(std::make_shared<k4a::capture>()),
        running(true),
        width(640),
        height(576),
        cWidth(1280),
        cHeight(720),
        dWidth(640),
        dHeight(576) {

    /**
     * connect to dk
     */
    uint32_t count = k4a::device::get_installed_count();
    std::cout << "found " << count << " device" << std::endl;
    if (count < 1) {
        errorText = "found no device!";
        initSuccessful = false;
        return;
    }
    if (camIdx >= count) {
        errorText = "camIdx exceed device num!";
        initSuccessful = false;
        return;
    }
    device = k4a::device::open(camIdx);
    if (!device) {
        errorText = "failed to open device!";
        initSuccessful = false;
        return;
    }

    /**
     * set dk configuration
     * and read camera intrinsics
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
     * initialize buffers
     */
    for (int i = 0; i < numBuffers; i++) {
        uint8_t *newImage = (uint8_t *) calloc(width * height * 3, sizeof(uint8_t));
//        uint8_t *newImage = (uint8_t *) calloc(cWidth * cHeight * 3, sizeof(uint8_t));
        rgbBuffers[i] = std::pair<uint8_t *, int64_t>(newImage, 0);
    }
    for (int i = 0; i < numBuffers; ++i) {
        uint8_t *newDepth = (uint8_t *) calloc(width * height * 2, sizeof(uint8_t));
//        uint8_t *newDepth = (uint8_t *) calloc(dWidth * dHeight * 2, sizeof(uint8_t));
//        uint8_t *newImage = (uint8_t *) calloc(cWidth * cHeight * 3, sizeof(uint8_t));
        uint8_t *newImage = (uint8_t *) calloc(width * height * 3, sizeof(uint8_t));
        frameBuffers[i] = std::pair < std::pair < uint8_t *, uint8_t * >, int64_t >
                                                                          (std::pair<uint8_t *, uint8_t *>(newDepth,
                                                                                                           newImage), 0);
    }

    /**
     * initialize callbacks
     */
    latestDepthIndex.assign(-1);
    latestRgbIndex.assign(-1);
    rgbCallback = new RGBCallback(lastRgbTime, latestRgbIndex, rgbBuffers, width, height, cWidth, cHeight);
    depthCallback = new DepthCallback(lastDepthTime, latestDepthIndex, latestRgbIndex, rgbBuffers, frameBuffers, width,
                                      height, cWidth, cHeight, dWidth, dHeight);

    /**
     * start dk
     */
    device.start_cameras(&conf);
    device.start_imu();

    /**
     * start the worker
     */
    std::cout << "start the worker" << std::endl;
    worker = std::thread([&] {
        std::unique_lock <std::mutex> lk(mtx);
        while (true) {
            std::cout << "loop" << std::endl;
            if (!running) {
                break;
            }
            if (device.get_capture(capture_ptr.get(), std::chrono::milliseconds(K4A_WAIT_INFINITE))) {
                std::cout << "get a frame" << std::endl;
                lk.unlock();
                rgbCallback->process(capture_ptr->get_color_image());
                depthCallback->process(capture_ptr->get_depth_image());
                lk.lock();
            }
        }
    });
    worker.detach();
}

KinectDKInterface::~KinectDKInterface() {
    if (initSuccessful) {
        /**
         * stop the worker
         */
        {
            std::lock_guard <std::mutex> lk(mtx);
            running = false;
        }
        /**
         * stop device
         */
        device.stop_cameras();
        device.stop_imu();

        /**
         * free buffers
         */
        for (int i = 0; i < numBuffers; i++) {
            free(rgbBuffers[i].first);
        }
        for (int i = 0; i < numBuffers; i++) {
            free(frameBuffers[i].first.first);
            free(frameBuffers[i].first.second);
        }

        /**
         * delete callbacks
         */
        delete rgbCallback;
        delete depthCallback;
    }
}

void KinectDKInterface::setAutoExposure(bool value) {}

void KinectDKInterface::setAutoWhiteBalance(bool value) {}

