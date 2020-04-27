//
// Created by yc_qian on 20-4-26.
//

#ifndef ELASTICFUSION_KINECTDKINTERFACE_HPP
#define ELASTICFUSION_KINECTDKINTERFACE_HPP
#include "ThreadMutexObject.h"
#include "CameraInterface.h"
#include <k4a/k4a.hpp>
#include <string>
#include <iostream>
#include <memory>

class KinectDKInterface : public CameraInterface {
public:
    KinectDKInterface(int camIdx = 0);

    virtual ~KinectDKInterface();

    virtual void setAutoExposure(bool value);
    virtual void setAutoWhiteBalance(bool value);

    virtual bool ok()
    {
        return initSuccessful;
    }

    virtual std::string error()
    {
        return errorText;
    }

    int width, height;
    float fx, fy, cx, cy;

    struct RGBCallback {
    public:
        RGBCallback(int64_t &lastRgbTime,
                    ThreadMutexObject<int> &latestRgbIndex,
                    std::pair<uint8_t *, int64_t> *rgbBuffers) :
                lastRgbTime(lastRgbTime),
                latestRgbIndex(latestRgbIndex),
                rgbBuffers(rgbBuffers) {

        }

        void process(k4a::image image) {
            lastRgbTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count();

            int bufferIndex = (latestRgbIndex.getValue() + 1) % numBuffers;

            memcpy(rgbBuffers[bufferIndex].first, image.get_buffer(),
                   image.get_width_pixels() * image.get_height_pixels() * 3);

            rgbBuffers[bufferIndex].second = lastRgbTime;

            latestRgbIndex++;
        }
    private:
        int64_t & lastRgbTime;
        ThreadMutexObject<int> & latestRgbIndex;
        std::pair<uint8_t *,int64_t> * rgbBuffers;
    };

    struct DepthCallback {
    public:
        DepthCallback(int64_t &lastDepthTime,
                      ThreadMutexObject<int> &latestDepthIndex,
                      ThreadMutexObject<int> &latestRgbIndex,
                      std::pair<uint8_t *, int64_t> *rgbBuffers,
                      std::pair <std::pair<uint8_t *, uint8_t *>, int64_t> *frameBuffers) :
                lastDepthTime(lastDepthTime),
                latestDepthIndex(latestDepthIndex),
                latestRgbIndex(latestRgbIndex),
                rgbBuffers(rgbBuffers),
                frameBuffers(frameBuffers) {

        }

        void process(k4a::image image) {
            lastDepthTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count();

            int bufferIndex = (latestDepthIndex.getValue() + 1) % numBuffers;

            // The multiplication by 2 is here because the depth is actually uint16_t
            memcpy(frameBuffers[bufferIndex].first.first, image.get_buffer(),
                   image.get_width_pixels() * image.get_height_pixels() * 2);

            frameBuffers[bufferIndex].second = lastDepthTime;

            int lastImageVal = latestRgbIndex.getValue();

            if(lastImageVal == -1)
            {
                return;
            }

            lastImageVal %= numBuffers;

            memcpy(frameBuffers[bufferIndex].first.second, rgbBuffers[lastImageVal].first,
                   image.get_width_pixels() * image.get_height_pixels() * 3);

            latestDepthIndex++;
        }

    private:
        int64_t &lastDepthTime;
        ThreadMutexObject<int> &latestDepthIndex;
        ThreadMutexObject<int> &latestRgbIndex;

        std::pair<uint8_t *, int64_t> *rgbBuffers;
        std::pair <std::pair<uint8_t *, uint8_t *>, int64_t> *frameBuffers;
    };

private:
    bool initSuccessful;
    std::string errorText;
    k4a::device device;
    std::shared_ptr <k4a::capture> capture_ptr;
    RGBCallback *rgbCallback;
    DepthCallback *depthCallback;
    std::pair<uint8_t *, int64_t> rgbBuffers[numBuffers];
    ThreadMutexObject<int> latestRgbIndex;

    int64_t lastRgbTime;
    int64_t lastDepthTime;

    std::thread worker;
    std::mutex mtx;
    bool running;
};
#endif //ELASTICFUSION_KINECTDKINTERFACE_HPP
