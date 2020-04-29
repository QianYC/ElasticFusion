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
#include <opencv2/opencv.hpp>
#include <string.h>

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

    //width height for color image
    int cWidth, cHeight;
    //width height for depth image
    int dWidth, dHeight;
    //recommended resolution
    int width, height;

    struct RGBCallback {
    public:
        RGBCallback(int64_t &lastRgbTime,
                    ThreadMutexObject<int> &latestRgbIndex,
                    std::pair<uint8_t *, int64_t> *rgbBuffers,
                    int width, int height,
                    int cWidth, int cHeight) :
                lastRgbTime(lastRgbTime),
                latestRgbIndex(latestRgbIndex),
                rgbBuffers(rgbBuffers),
                cWidth(cWidth), cHeight(cHeight),
                width(width), height(height) {

        }

        void process(k4a::image image) {

            lastRgbTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count();

            int bufferIndex = (latestRgbIndex.getValue() + 1) % numBuffers;

            /**
             * BGRA -> RGB
             */
            cv::Mat bgra(cHeight, cWidth, CV_8UC4, (void *) image.get_buffer());
            cv::Mat rgb;
            cv::cvtColor(bgra, rgb, cv::COLOR_BGRA2RGB);

            memcpy(rgbBuffers[bufferIndex].first, rgb.data,
                   cWidth * cHeight * 3);

            rgbBuffers[bufferIndex].second = lastRgbTime;

            /**
             * save image
             */
            std::stringstream ss;
            ss << "rgb" << lastRgbTime << ".png";
            cv::Mat color(image.get_height_pixels(), image.get_width_pixels(), CV_8UC4, (void *) image.get_buffer());
            cv::imwrite(ss.str(), color);

            latestRgbIndex++;
        }
    private:
        int64_t &lastRgbTime;
        ThreadMutexObject<int> &latestRgbIndex;
        std::pair<uint8_t *, int64_t> *rgbBuffers;
        //width height for color image
        int cWidth, cHeight;
        //recommended resolution
        int width, height;
    };

    struct DepthCallback {
    public:
        DepthCallback(int64_t &lastDepthTime,
                      ThreadMutexObject<int> &latestDepthIndex,
                      ThreadMutexObject<int> &latestRgbIndex,
                      std::pair<uint8_t *, int64_t> *rgbBuffers,
                      std::pair <std::pair<uint8_t *, uint8_t *>, int64_t> *frameBuffers,
                      int width, int height,
                      int cWidth, int cHeight,
                      int dWidth, int dHeight) :
                lastDepthTime(lastDepthTime),
                latestDepthIndex(latestDepthIndex),
                latestRgbIndex(latestRgbIndex),
                rgbBuffers(rgbBuffers),
                frameBuffers(frameBuffers),
                width(width), height(height),
                cWidth(cWidth), cHeight(cHeight),
                dWidth(dWidth), dHeight(dHeight) {

        }

        void process(k4a::image image) {
            lastDepthTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count();

            int bufferIndex = (latestDepthIndex.getValue() + 1) % numBuffers;

            // The multiplication by 2 is here because the depth is actually uint16_t
            memcpy(frameBuffers[bufferIndex].first.first, image.get_buffer(),
                   dWidth * dHeight * 2);

            frameBuffers[bufferIndex].second = lastDepthTime;

            int lastImageVal = latestRgbIndex.getValue();

            if(lastImageVal == -1)
            {
                return;
            }

            lastImageVal %= numBuffers;

            memcpy(frameBuffers[bufferIndex].first.second, rgbBuffers[lastImageVal].first,
                   cWidth * cHeight * 3);

            /**
             * save image
             */
            std::stringstream ss;
            ss << "depth" << lastDepthTime << ".png";
            cv::Mat depth(image.get_height_pixels(), image.get_width_pixels(), CV_16U, (void *) image.get_buffer());
            cv::imwrite(ss.str(), depth);

            latestDepthIndex++;
        }

    private:
        int64_t &lastDepthTime;
        ThreadMutexObject<int> &latestDepthIndex;
        ThreadMutexObject<int> &latestRgbIndex;

        std::pair<uint8_t *, int64_t> *rgbBuffers;
        std::pair <std::pair<uint8_t *, uint8_t *>, int64_t> *frameBuffers;

        //width height for color image
        int cWidth, cHeight;
        //width height for depth image
        int dWidth, dHeight;
        //recommended resolution
        int width, height;
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
