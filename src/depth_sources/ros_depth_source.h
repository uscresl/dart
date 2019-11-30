/**
 * Author: Yizhou Sheng
 * Last Updated: Nov. 13, 2019
 * A depth source that utilizes message received from a virtual camera
 * 
 */
#ifndef ROS_DEPTH_SOURCE_H
#define ROS_DEPTH_SOURCE_H

#include "depth_source.h"

#include <iostream>
#include <fstream>
#include <vector>
#include "util/string_format.h"

#include "util/mirrored_memory.h"
#include "vector_types.h"

#include "ros/ros.h"
#include "image_transport/image_transport.h"

namespace dart {

/**
 * A depth source class that reads the image from image messages
 * provided by a (virtual) depth camera of another ROS node
 * ASSUMPTION: The image is depth image only and doesn't utilize
 * color messages. 
 * TODO: Current implementation assumes te width and height are 
 * known. Further improvements can automatically deduct from the 
 * first image it has received.
 */
template <typename DepthType, typename ColorType>
class RosDepthSource : public DepthSource<DepthType, ColorType>
{
public:
    RosDepthSource();
    ~RosDepthSource();

    /**
     * An initializer that takes in the dimensions and camera info
     */
    bool initialize(const float2 focalLength,
                    const float2 principalPoint = make_float2(0,0),
                    const sensor_msgs::ImageConstPtr& img_ptr = nullptr,
                    const uint depthWidth = 0,
                    const uint depthHeight = 0,
                    const float scaleToMeters = 1.0f,
                    const std::vector<ulong>* depthTimes = nullptr,
                    const bool hasColor = false,
                    const uint colot_width = 0,
                    const uint colorHeight = 0,
                    const std::vector<long>* colorTimes = nullptr);

#ifdef CUDA_BUILD
    const DepthType* getDepth() const {return _depthData->hostPtr();}
    const DepthType* getDeviceDepth() const {return _depthData->devicePtr();}

#else
    const DepthType* getDepth() const {return _depthData;}
    const DepthType* getDeviceDepth() const {return NULL;}
#endif
    
    const ColorType* getColor() const {return _colorData;}
    
    ColorLayout getColorLayout() const {return LAYOUT_RGB;}
    uint64_t getDepthTime() const {return _depthTimes[this->_frame];}
    uint64_t getColorTime() const {return _colorTimes[this->_frame];}
    void setFrame(const uint frame) {std::cerr << "WARNING: set frame not supported by RosDepthSource." << std::endl;}
    void advance();
    bool hasRadialDistortionParams() const {return false;}
    float getScaleToMeters() const {return _scaleToMeters;}
    // set png swap?
    // getNumDepthFrames is disabled

    void setMsg(const sensor_msgs::ImageConstPtr& img_ptr) {this->_imageMsg = img_ptr; _msgRead = true; }
    bool getMsgRead() const { return _msgRead; }
    
    /**
     * Read from the stored pointer
     */
    void readDepthLocal() {this->readDepth(_imageMsg);}


private:
    /**
     * Read the image from the image message
     */
    void readDepth(const sensor_msgs::ImageConstPtr& img_ptr);

    

#ifdef CUDA_BUILD
    MirroredVector<DepthType>* _depthData;
#else
    DepthType* _depthData;
#endif // CUDA_BUILD
    ColorType* _colorData;
    std::vector<ulong> _depthTimes;
    std::vector<ulong> _colorTimes;
    std::vector<uint> _correspondingColorFrames;
    float _scaleToMeters;
    sensor_msgs::ImageConstPtr _imageMsg;

    bool _msgRead;
};

// Implementation
template <typename DepthType, typename ColorType>
RosDepthSource<DepthType, ColorType>::RosDepthSource() :
    DepthSource<DepthType, ColorType>(), 
    _depthData(nullptr)
    {}

template <typename DepthType, typename ColorType>
RosDepthSource<DepthType, ColorType>::~RosDepthSource() {
#ifdef CUDA_BUILD
    delete _depthData;
#else
    delete [] _depthData;
#endif // CUDA_BUILD
}

template <typename DepthType, typename ColorType>
bool RosDepthSource<DepthType, ColorType>::initialize(
                    const float2 focalLength,
                    const float2 principalPoint,
                    const sensor_msgs::ImageConstPtr& img_ptr,
                    const uint depthWidth,
                    const uint depthHeight,
                    const float scaleToMeters,
                    const std::vector<ulong>* depthTimes,
                    const bool hasColor,
                    const uint colot_width,
                    const uint colorHeight,
                    const std::vector<long>* colorTimes)
{
    this->_frame = 0;
    _scaleToMeters = scaleToMeters;
    this->_focalLength = focalLength;

    // TODO: Figure out whether the depthWidth and depthHeight can be inferred from the first image

    if (depthWidth > 0 && depthHeight > 0) {
        this->_depthWidth = depthWidth;
        this->_depthHeight = depthHeight;
    } else if (img_ptr != nullptr) {
        // TODO: Figure out whether the depthWidth and depthHeight can be inferred from the first image/
        // Can be done in the first readDepth?
        std::cerr << "WARNING: Depth Image size info is not guaranteed to work." << std::endl;
        this->_depthWidth = img_ptr->width;
        this->_depthHeight = img_ptr->height;
    } else {
        std::cerr << "WARNING: Depth Image size info required." << std::endl;
        return false;
    }

    if (principalPoint.x == 0) {
        this->_principalPoint = make_float2(this->_depthWidth/2, this->_depthHeight/2);
    } else {
        this->_principalPoint = principalPoint;
    }

    // allocate data
#ifdef CUDA_BUILD
    _depthData = new MirroredVector<DepthType>(this->_depthWidth*this->_depthHeight);
#else
    _depthData = new DepthType[this->_depthWidth*this->_depthHeight];
#endif

    if (depthTimes) {
        this->_hasTimestamps = true;
        _depthTimes = *depthTimes;
    } else {
        this->_hasTimestamps = false;
    }

    _msgRead = false;

    // Initial Read
    // this->readDepth(img_ptr);


    // Currently omitting color for simplicity
    if (hasColor) {
        std::cerr << "WARNING: RosDepthSource currently does not support reading from color images. " << std::endl;
    }

    return true;

}

template <typename DepthType, typename ColorType>
void RosDepthSource<DepthType, ColorType>::readDepth(const sensor_msgs::ImageConstPtr& img_ptr)
{
    _msgRead = false;
    if (img_ptr == nullptr) {
        std::cerr << "Warning: Attempting to read from a NULL pointer" << std::endl;
        return;
    }
    this->_imageMsg = img_ptr;
    if (img_ptr->width != this->_depthWidth || img_ptr->height != this->_depthHeight) {
        std::cerr << "WARNING: expected width: " << this->_depthWidth << " and height: " << this->_depthHeight << " but get w: " << img_ptr->width << " and h: " << img_ptr->height << std::endl;
        // return;
    }

    if (img_ptr->data.size() != sizeof(DepthType)*(this->_depthWidth)*(this->_depthHeight)) {
        std::cerr << "WARNING: unmatched row step" << std::endl;
    }

    // DEBUG
    // long long counter = 0;

    std::cerr << "Reading Depth Image" << std::endl;
    // Converting image data from message to dart source
    DepthType curr = 0;
    for (int i = 0; i < this->_depthHeight; ++i) {
        for (int j = 0; j < this->_depthWidth; ++j) {
            for (int k = 0; k < sizeof(DepthType); ++k) {
                // DEBUG
                // if (img_ptr->data[i*this->_depthWidth*sizeof(DepthType) + j*sizeof(DepthType) + k] != 0) {
                //     std::cout << "Not Zero pixel: " << i*this->_depthWidth*sizeof(DepthType) + j*sizeof(DepthType) + k 
                //               << " " << (unsigned int)(img_ptr->data[i*this->_depthWidth*sizeof(DepthType) + j*sizeof(DepthType) + k])
                //               << std::endl;
                // }
                // if (i*this->_depthWidth+j == 1921) {
                //     std::cerr << "Pixel 1921: Curr = " << curr << std::endl;
                // }
                // curr |= img_ptr->data[i*this->_depthWidth*sizeof(DepthType) + j*sizeof(DepthType) + k];
                DepthType temp = img_ptr->data[i*this->_depthWidth*sizeof(DepthType) + j*sizeof(DepthType) + k];
                // if (i*this->_depthWidth+j == 1921) {
                //     std::cerr << "Pixel 1921: after OR curr = " << curr << std::endl;
                // }
                // if (k != sizeof(DepthType)-1) {
                //     curr = curr << 8;
                // }
                temp = temp << (k*8);
                curr |= temp;
                // if (i*this->_depthWidth+j == 1921) {
                //     std::cerr << "Pixel 1921: after SHIFT curr = " << curr << std::endl;
                // }

                // if (counter != i*this->_depthWidth*sizeof(DepthType) + j*sizeof(DepthType) + k) {
                //     std::cerr << "Unexpected iteration error: counter = " << counter << " but actually reading "
                //               << i*this->_depthWidth*sizeof(DepthType) + j*sizeof(DepthType) + k << std::endl;
                // }
                // counter++;
            }
            #ifdef CUDA_BUILD
                this->_depthData->hostPtr()[i*this->_depthWidth+j] = curr;
                
            #else
                this->_depthData[i*this->_depthWidth+j] = curr;
            #endif
            curr = 0;
        }
    }

    // DEBUG print the first ten pixels
    // #ifdef CUDA_BUILD
        // std::cout << "DEBUG: First Ten Pixels of the Read Image" << std::endl;
        // for (int i = 1900; i < 2000; i++) {
        //     std::cout << i << ": " << this->_depthData->hostPtr()[i] << std::endl;
        // }
    // #endif

    
}

template <typename DepthType, typename ColorType>
void RosDepthSource<DepthType,ColorType>::advance() 
{
    std::cerr << "ADVANCE" << std::endl;
    _msgRead=false;
    this->_frame++;
    readDepthLocal();
#ifdef CUDA_BUILD
    _depthData->syncHostToDevice();
#endif

    // Read Color disabled for simplicity
}

    

}


#endif