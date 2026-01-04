/*
 *  SLAMTEC Aurora
 *  Copyright 2013 - 2024 SLAMTEC Co., Ltd.
 *
 *  http://www.slamtec.com
 *
 *  Aurora Remote SDK
 *  C++ Wrapper Header of the SDK
 *
 *  At lease C++ 14 is required
 */

#include <type_traits>
#include <sstream>
#include <optional>
#include <array>

#pragma once

namespace cv {
    class Mat; // in case of opencv
}

namespace rp { namespace standalone { namespace aurora {

// Type trait to detect Eigen Matrix types
template<typename T>
struct is_eigen_matrix : std::false_type {};

// Specialization will be provided when Eigen is included
#ifdef EIGEN_WORLD_VERSION
template<typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
struct is_eigen_matrix<Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>> : std::true_type {};
#endif 


class Noncopyable {
protected:
    Noncopyable() = default;
    ~Noncopyable() = default;

    Noncopyable(const Noncopyable&) = delete;
    Noncopyable& operator=(const Noncopyable&) = delete;

    Noncopyable(Noncopyable&&) = default;
    Noncopyable& operator=(Noncopyable&&) = default;
};

/**
 * @brief The connection info class
 * @details This class is used to store the connection information.
 * @ingroup Cxx_Controller_Operations Controller Operations
 */
class SDKConnectionInfo : public slamtec_aurora_sdk_connection_info_t
{
public:
    SDKConnectionInfo() : slamtec_aurora_sdk_connection_info_t() {
        memset(this, 0, sizeof(slamtec_aurora_sdk_connection_info_t));
    }

    /**
     * @brief The constructor with IP address, port and protocol
     * @details This constructor is used to create a connection info with IP address, port and protocol.
     * @param[in] ip The IP address
     * @param[in] port The port
     * @param[in] proto The protocol
     */
    SDKConnectionInfo(const char* ip, int port = SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_PORT, const char * proto = SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_PROTOCOL) : SDKConnectionInfo() {
        snprintf(this->address, sizeof(this->address), "%s", ip);
        this->port = port;
        snprintf(this->protocol_type, sizeof(this->protocol_type), "%s", proto);
    }

    SDKConnectionInfo(const SDKConnectionInfo& other) {
        memcpy(this, &other, sizeof(SDKConnectionInfo));
    }

    SDKConnectionInfo& operator=(const SDKConnectionInfo& other) {
        memcpy(this, &other, sizeof(SDKConnectionInfo));
        return *this;
    }

    SDKConnectionInfo(const slamtec_aurora_sdk_connection_info_t& other) {
        memcpy(this, &other, sizeof(slamtec_aurora_sdk_connection_info_t));
    }


    /**
     * @brief Convert the connection info to a locator string
     * @details This function is used to convert the connection info to a locator string.
     * @details The format of the locator string is "[protocol/]ip[:port]".
     * @return The locator string
     */
    std::string toLocatorString() const {
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "%s/%s:%d", protocol_type, address, port);
        return std::string(buffer);
    }

    /**
     * @brief Parse the connection info from a locator string
     * @details This function is used to parse the connection info from a locator string.
     * @details The format of the locator string is "[protocol/]ip[:port]".
     * @param[in] input The locator string
     * @return True if the parsing is successful, false otherwise
     */
    bool fromLocatorString(const char* input) {
        std::string protocol;
        std::string ip;
        uint16_t tport = SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_PORT;

        std::string inputWrapper = input;

        size_t protocol_pos = inputWrapper.find('/');
        if (protocol_pos != std::string::npos) {
            protocol = inputWrapper.substr(0, protocol_pos);
        }
        else {
            protocol = SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_PROTOCOL;
            protocol_pos = -1;
        }

        size_t port_pos = inputWrapper.rfind(':');
        if (port_pos != std::string::npos) {
            ip = inputWrapper.substr(protocol_pos + 1, port_pos - protocol_pos - 1);
            auto&& portStr = inputWrapper.substr(port_pos + 1);
            tport = (uint16_t)std::stoi(portStr);
        }
        else {
            ip = inputWrapper.substr(protocol_pos + 1);
        }

        snprintf(protocol_type, sizeof(protocol_type), "%s", protocol.c_str());
        snprintf(address, sizeof(address), "%s", ip.c_str());
        this->port = tport;

        return true;
    }
};


/**
 * @brief The server connection description class
 * @details This class is used to store the server connection information.
 * @ingroup Cxx_Controller_Operations Session Management
 */
class SDKServerConnectionDesc : public slamtec_aurora_sdk_server_connection_info_t
{
public:
    SDKServerConnectionDesc() : slamtec_aurora_sdk_server_connection_info_t() {
        memset(this, 0, sizeof(slamtec_aurora_sdk_server_connection_info_t));
    }

    /**
     * @brief Create the server connection description with a server connection info structure
     * @details This constructor is used to create a server connection description with a server connection info structure.
     * @param[in] info The server connection info
     */
    SDKServerConnectionDesc(const slamtec_aurora_sdk_server_connection_info_t & info)
    {
        memcpy(this, &info, sizeof(slamtec_aurora_sdk_server_connection_info_t));
    }


    SDKServerConnectionDesc(const SDKServerConnectionDesc& other) : SDKServerConnectionDesc() {
        memcpy(this, &other, sizeof(SDKServerConnectionDesc));
    }

    SDKServerConnectionDesc(const std::vector<SDKConnectionInfo>& src) {
        memset(this, 0, sizeof(slamtec_aurora_sdk_server_connection_info_t));
        for (auto&& info : src) {
            push_back(info);
        }
    }

    /**
     * @brief Create the server connection description with one connection option with IP address, port and protocol
     * @details This constructor is used to create a server connection description with IP address, port and protocol.
     * @param[in] ip The IP address
     * @param[in] port The port
     * @param[in] proto The protocol
     */
    SDKServerConnectionDesc(const char* ip, int port = SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_PORT, const char* proto = SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_PROTOCOL)
    {
        memset(this, 0, sizeof(slamtec_aurora_sdk_server_connection_info_t));
        push_back(SDKConnectionInfo(ip, port, proto));
    }

    SDKServerConnectionDesc& operator=(const SDKServerConnectionDesc& other) {
        memcpy(this, &other, sizeof(SDKServerConnectionDesc));
        return *this;
    }

    /**
     * @brief Convert the server connection description to a vector of connection info
     * @details This function is used to convert the server connection description to a vector of connection info.
     * @return The vector of connection info
     */
    std::vector<SDKConnectionInfo> toVector() const {
        std::vector<SDKConnectionInfo> result;
        for (size_t i = 0; i < connection_count; i++) {
            result.push_back(connection_info[i]);
        }
        return result;
    }

    SDKServerConnectionDesc& operator=(const std::vector<SDKConnectionInfo>& src) {
        memset(this, 0, sizeof(slamtec_aurora_sdk_server_connection_info_t));
        for (auto&& info : src) {
            push_back(info);
        }
        return *this;
    }

    /**
     * @brief Get the count of the server connection description
     * @details This function is used to get the count of the server connection description.
     * @return The count of the server connection description
     */
    size_t size() const {
        return connection_count;
    }

    /**
     * @brief Get the capacity of the connection description this object can hold
     * @details This function is used to get the capacity of the connection description this object can hold.
     * @return The capacity of the connection description
     */
    size_t capacity() const {
        return sizeof(connection_info) / sizeof(connection_info[0]);
    }

    /**
     * @brief Clear the connection description
     * @details This function is used to clear the connection description.
     */
    void clear() {
        connection_count = 0;
    }

    /**
     * @brief Push a connection info to the server connection description
     * @details This function is used to push a connection info to the server connection description.
     * @param[in] info The connection info
     * @return True if the push is successful, false otherwise
     */
    bool push_back(const slamtec_aurora_sdk_connection_info_t& info) {
        if (connection_count >= capacity()) {
            return false;
        }

        connection_info[connection_count++] = info;
        return true;
    }

    /**
     * @brief Pop a connection info from the server connection description
     * @details This function is used to pop a connection info from the server connection description.
     */
    void pop_back() {
        if (connection_count > 0) {
            connection_count--;
        }
    }

    /**
     * @brief Get the connection info at the specified index
     * @details This function is used to get the connection info at the specified index.
     * @param[in] index The index
     * @return The connection info
     */
    const SDKConnectionInfo& operator[](size_t index) const {
        return *(const SDKConnectionInfo *) ( & connection_info[index]);
    }

    /**
     * @brief Get the connection info at the specified index
     * @details This function is used to get the connection info at the specified index.
     * @param[in] index The index
     * @return The connection info
     */
    const SDKConnectionInfo& at(size_t index) const {
        return *(const SDKConnectionInfo*)(&connection_info[index]);
    }


};


class RemoteDeviceBasicInfo : public slamtec_aurora_sdk_device_basic_info_t {
public:
    RemoteDeviceBasicInfo() : slamtec_aurora_sdk_device_basic_info_t() {
        memset(this, 0, sizeof(slamtec_aurora_sdk_device_basic_info_t));
    }

    RemoteDeviceBasicInfo(const slamtec_aurora_sdk_device_basic_info_t& other) {
        memcpy(this, &other, sizeof(slamtec_aurora_sdk_device_basic_info_t));
    }

    RemoteDeviceBasicInfo(const RemoteDeviceBasicInfo& other) {
        memcpy(this, &other, sizeof(slamtec_aurora_sdk_device_basic_info_t));
    }

    RemoteDeviceBasicInfo& operator=(const slamtec_aurora_sdk_device_basic_info_t& other) {
        memcpy(this, &other, sizeof(slamtec_aurora_sdk_device_basic_info_t));
        return *this;
    }

    RemoteDeviceBasicInfo& operator=(const RemoteDeviceBasicInfo& other) {
        memcpy(this, &other, sizeof(slamtec_aurora_sdk_device_basic_info_t));
        return *this;
    }

    std::string getDeviceSerialNumberString() const {
        std::string sn;
        sn.resize(32);

        for (int i = 0; i < 16; ++i)
        {
            snprintf(&sn[i * 2], 3, "%02X", device_sn[i]);
        }

        return sn;
    }

    std::string getDeviceModelString() const {
        std::stringstream ss;
        if (model_major == 0 && model_sub == 0) {
            ss << "A1M1";
        }
        else {
            
            ss << "A" << std::to_string(model_major) << "M" << std::to_string(model_sub);
        }

        if (model_revision) {
            ss << "-r" << std::to_string(model_revision);
        }
        return ss.str();
    }


    bool isSupportDepthCamera() const {
        return sensing_feature_bitmaps & SLAMTEC_AURORA_SDK_SENSING_FEATURE_BIT_STEREO_DENSE_DISPARITY;
    }

    bool isSupportSemanticSegmentation() const {
        return sensing_feature_bitmaps & SLAMTEC_AURORA_SDK_SENSING_FEATURE_BIT_SEMANTIC_SEGMENTATION;
    }

    bool isSupportCameraPreviewStream() const {
        return swfeature_bitmaps & SLAMTEC_AURORA_SDK_SW_FEATURE_BIT_CAMREA_PREVIEW_STREAM;
    }
    
};


/**
 * @brief The image reference class wraps the image description and data
 * @details This class is used to wrap the image description and data.
 * @ingroup Cxx_DataProvider_Operations Data Provider Operations
 */
class RemoteImageRef {
public:
    
    RemoteImageRef(const slamtec_aurora_sdk_image_desc_t& desc, const void * data)
        : _desc(desc), _data(data)
    {
    }

    /**
     * @brief The data pointer of the image
     */
    const void* _data;

    /*
    * @brief The description of the image
    */
    const slamtec_aurora_sdk_image_desc_t & _desc;


public:
    /**
     * @brief Convert the image to a cv::Mat object
     * @details This function is used to convert the image to a cv::Mat object. 
     * @details This function is only available when OpenCV headers are included.
     * @param[out] mat The cv::Mat object
     * @return True if the conversion is successful, false otherwise
     */
    template <typename T>
    typename std::enable_if<std::is_same<T, cv::Mat>::value, bool>::type
    toMat(T &mat)const {
        if (isEmpty()) {
            mat = T();
            return false;
        }

        switch (_desc.format)
        {
        case 0: // mono
            mat = T(_desc.height, _desc.width, 0, (void*)_data, _desc.stride);
            break;
        case 1: // bgr
            mat = T(_desc.height, _desc.width, 0x10, (void*)_data, _desc.stride);
            break;
        case 2: // rgba
            mat = T(_desc.height, _desc.width, 0x18, (void*)_data, _desc.stride);
            break;
        case 3: // depth (float32)
            mat = T(_desc.height, _desc.width, 5, (void*)_data, _desc.stride);
            break;
        case 4: // point3D (floatx3 XYZ)
            mat = T(_desc.height, _desc.width, 0x15, (void*)_data, _desc.stride);
            break;

            
        default:
            mat = T();
            return false;
        }

        return true;
    }

    /**
     * @brief Convert the image to a point cloud array
     * @details If the image format is not point3D, the function will return nullptr.
     * @return The point cloud array
     */
    const std::array<float, 3>* toPoint3D() const {
        if (_desc.format != 4) {
            return nullptr;
        }

        return reinterpret_cast<const std::array<float,3>*>(_data);
    }



    size_t getPointCount() const {
        return _desc.height * _desc.width;
    }


    bool isEmpty() const {
        return (_data == nullptr || _desc.height == 0 || _desc.width == 0);
    }

};

/**
 * @brief The tracking frame info class wraps the tracking information and its data
 * @details This class is used to wrap the tracking information and its data.
 * @ingroup Cxx_DataProvider_Operations Data Provider Operations
 */
class RemoteTrackingFrameInfo {
public:
    RemoteTrackingFrameInfo()
        : leftImage(trackingInfo.left_image_desc, nullptr)
        , rightImage(trackingInfo.right_image_desc, nullptr)
        , _keypoints_left(_keypoints_buffer_left.data())
        , _keypoints_right(_keypoints_buffer_rightf.data())
    {
        memset(&trackingInfo, 0, sizeof(slamtec_aurora_sdk_tracking_info_t));
    }

    /**
     * @brief Create the tracking frame info with the tracking information and the tracking data buffer
     * @details This constructor is used to create the tracking frame info with the tracking information and the tracking data buffer.
     * @param[in] info The tracking information
     * @param[in] buffer The tracking data buffer
     */
    RemoteTrackingFrameInfo(const slamtec_aurora_sdk_tracking_info_t& info, const slamtec_aurora_sdk_tracking_data_buffer_t & buffer)
        : trackingInfo(info)
        , leftImage(info.left_image_desc, buffer.imgdata_left)
        , rightImage(info.right_image_desc, buffer.imgdata_right)
        , _keypoints_left(buffer.keypoints_left)
        , _keypoints_right(buffer.keypoints_right)
    {
    }

    /**
     * @brief Create the tracking frame info with the tracking information and the image data buffer
     * @details This constructor is used to create the tracking frame info with the tracking information and the image data buffer.
     * @details The image data buffer and the keypoints data buffer are moved from the input parameters.
     * @param[in] info The tracking information
     * @param[in] imgbuffer_left The left image data buffer
     * @param[in] imgbuffer_right The right image data buffer
     * @param[in] keypoints_buffer_left The left keypoints data buffer
     * @param[in] keypoints_buffer_right The right keypoints data buffer
     */
    RemoteTrackingFrameInfo(const slamtec_aurora_sdk_tracking_info_t& info,
        std::vector<uint8_t>&& imgbuffer_left,
        std::vector<uint8_t>&& imgbuffer_right,
        std::vector< slamtec_aurora_sdk_keypoint_t>&& keypoints_buffer_left,
        std::vector< slamtec_aurora_sdk_keypoint_t>&& keypoints_buffer_right)
        : trackingInfo(info)
        , leftImage(info.left_image_desc, imgbuffer_left.data())
        , rightImage(info.right_image_desc, imgbuffer_right.data())
        , _keypoints_left(keypoints_buffer_left.data())
        , _keypoints_right(keypoints_buffer_right.data())
        , _imgbuffer_left(std::move(imgbuffer_left))
        , _imgbuffer_right(std::move(imgbuffer_right))
        , _keypoints_buffer_left(std::move(keypoints_buffer_left))
        , _keypoints_buffer_rightf(std::move(keypoints_buffer_right))
    {}

    RemoteTrackingFrameInfo(const RemoteTrackingFrameInfo& other) 
        : trackingInfo(other.trackingInfo)
        , leftImage(trackingInfo.left_image_desc, nullptr)
        , rightImage(trackingInfo.right_image_desc, nullptr)
        , _keypoints_left(nullptr)
        , _keypoints_right(nullptr)
    {
        _copyFrom(other);
    }

    RemoteTrackingFrameInfo(RemoteTrackingFrameInfo&& other)
        : trackingInfo(other.trackingInfo)
        , leftImage(trackingInfo.left_image_desc, nullptr)
        , rightImage(trackingInfo.right_image_desc, nullptr)
        , _keypoints_left(nullptr)
        , _keypoints_right(nullptr)
    {
        if (!other._isOwnBuffer()) {
            _copyFrom(other);
        }
        else {
            _moveFrom(other);
        }
    }


    RemoteTrackingFrameInfo& operator=(const RemoteTrackingFrameInfo& other) {
        trackingInfo = other.trackingInfo;
        _copyFrom(other);
        return *this;
    }

    RemoteTrackingFrameInfo& operator=(RemoteTrackingFrameInfo&& other) {
        trackingInfo = other.trackingInfo;
        if (!other._isOwnBuffer()) {
            _copyFrom(other);
        }
        else {
            _moveFrom(other);
        }
        return *this;
    }

    /**
     * @brief Get the left keypoints buffer
     * @details This function is used to get the left keypoints buffer.
     * @return The left keypoints buffer
     */
    const slamtec_aurora_sdk_keypoint_t* getKeypointsLeftBuffer() const {
        return _keypoints_left;
    }

    /**
     * @brief Get the right keypoints buffer
     * @details This function is used to get the right keypoints buffer.
     * @return The right keypoints buffer
     */
    const slamtec_aurora_sdk_keypoint_t* getKeypointsRightBuffer() const {
        return _keypoints_right;
    }

    /**
     * @brief Get the left keypoints count
     * @details This function is used to get the left keypoints count.
     * @return The left keypoints count
     */
    size_t getKeypointsLeftCount() const {
        return trackingInfo.keypoints_left_count;
    }

    /**
     * @brief Get the right keypoints count
     * @details This function is used to get the right keypoints count.
     * @return The right keypoints count
     */
    size_t getKeypointsRightCount() const {
        return trackingInfo.keypoints_right_count;
    }

    void reset() {
        memset(&trackingInfo, 0, sizeof(slamtec_aurora_sdk_tracking_info_t));
        leftImage._data = nullptr;
        rightImage._data = nullptr;
        _keypoints_left = nullptr;
        _keypoints_right = nullptr;
        _imgbuffer_left.clear();
        _imgbuffer_right.clear();
        _keypoints_buffer_left.clear();
        _keypoints_buffer_rightf.clear();
    }



public: 

    /**
     * @brief The left image reference
     */
    RemoteImageRef leftImage;

    /**
     * @brief The right image reference
     */
    RemoteImageRef rightImage;
  
    /**
     * @brief The tracking information
     */
    slamtec_aurora_sdk_tracking_info_t trackingInfo;
    
protected:
    bool _isOwnBuffer() const {
        return (_keypoints_left == _keypoints_buffer_left.data());
    }

    void _moveFrom(RemoteTrackingFrameInfo& other) {
        _imgbuffer_left = std::move(other._imgbuffer_left);
        _imgbuffer_right = std::move(other._imgbuffer_right);
        _keypoints_buffer_left = std::move(other._keypoints_buffer_left);
        _keypoints_buffer_rightf = std::move(other._keypoints_buffer_rightf);

        leftImage._data = _imgbuffer_left.data();
        rightImage._data = _imgbuffer_right.data();
        _keypoints_left = _keypoints_buffer_left.data();
        _keypoints_right = _keypoints_buffer_rightf.data();

    }

    void _copyFrom(const RemoteTrackingFrameInfo& other) {
        if (other.leftImage._data) {
            _imgbuffer_left.resize(other.trackingInfo.left_image_desc.data_size);
            memcpy(_imgbuffer_left.data(), other.leftImage._data, other.trackingInfo.left_image_desc.data_size);
            leftImage._data = _imgbuffer_left.data();
        }
        else {
            leftImage._data = nullptr;
            _imgbuffer_left.clear();
        }

        if (other.rightImage._data) {
            _imgbuffer_right.resize(other.trackingInfo.right_image_desc.data_size);
            memcpy(_imgbuffer_right.data(), other.rightImage._data, other.trackingInfo.right_image_desc.data_size);
            rightImage._data = _imgbuffer_right.data();
        }
        else {
            rightImage._data = nullptr;
            _imgbuffer_right.clear();
        }

        if (other._keypoints_left) {
            _keypoints_buffer_left.resize(other.trackingInfo.keypoints_left_count);
            memcpy(_keypoints_buffer_left.data(), other._keypoints_left, other.trackingInfo.keypoints_left_count * sizeof(slamtec_aurora_sdk_keypoint_t));
            _keypoints_left = _keypoints_buffer_left.data();
        }
        else {
            _keypoints_left = nullptr;
            _keypoints_buffer_left.clear();
        }

        if (other._keypoints_right) {
            _keypoints_buffer_rightf.resize(other.trackingInfo.keypoints_right_count);
            memcpy(_keypoints_buffer_rightf.data(), other._keypoints_right, other.trackingInfo.keypoints_right_count * sizeof(slamtec_aurora_sdk_keypoint_t));
            _keypoints_right = _keypoints_buffer_rightf.data();
        }
        else {
            _keypoints_right = nullptr;
            _keypoints_buffer_rightf.clear();
        }
    }


    const slamtec_aurora_sdk_keypoint_t* _keypoints_left;
    const slamtec_aurora_sdk_keypoint_t* _keypoints_right;


    std::vector<uint8_t> _imgbuffer_left;
    std::vector<uint8_t> _imgbuffer_right;
    std::vector< slamtec_aurora_sdk_keypoint_t> _keypoints_buffer_left;
    std::vector< slamtec_aurora_sdk_keypoint_t> _keypoints_buffer_rightf;

};


struct RemoteStereoImagePair {

public:
    RemoteStereoImagePair() 
        : leftImage(desc.left_image_desc, nullptr)
        , rightImage(desc.right_image_desc, nullptr) 
    {
        memset(&desc, 0, sizeof(slamtec_aurora_sdk_stereo_image_pair_desc_t));
    }

    RemoteStereoImagePair(const slamtec_aurora_sdk_stereo_image_pair_desc_t& desc, const slamtec_aurora_sdk_stereo_image_pair_buffer_t& buffer)
        : desc(desc)
        , leftImage(desc.left_image_desc, buffer.imgdata_left)
        , rightImage(desc.right_image_desc, buffer.imgdata_right)
    {
    }

    RemoteStereoImagePair(const slamtec_aurora_sdk_stereo_image_pair_desc_t & info,
        std::vector<uint8_t> && left_buffer,
        std::vector<uint8_t> && right_buffer
    )
        : desc(info)
        , leftImage(info.left_image_desc, left_buffer.data())
        , rightImage(info.right_image_desc, right_buffer.data())
        , _imgbuffer_left(std::move(left_buffer))
        , _imgbuffer_right(std::move(right_buffer))
    {
    }

    RemoteStereoImagePair(const RemoteStereoImagePair& other)
        : desc(other.desc)
        , leftImage(desc.left_image_desc, nullptr)
        , rightImage(desc.right_image_desc, nullptr)
    {
        _copyFrom(other);
    }

    RemoteStereoImagePair(RemoteStereoImagePair && other)
        : desc(other.desc)
        , leftImage(desc.left_image_desc, nullptr)
        , rightImage(desc.right_image_desc, nullptr)
    {
        if (!other._isOwnBuffer()) {
            _copyFrom(other);
        } else {
            _moveFrom(other);
        }
    }

    RemoteStereoImagePair& operator=(const RemoteStereoImagePair& other) {
        desc = other.desc;
        _copyFrom(other);
        return *this;
    }

    RemoteStereoImagePair& operator=(RemoteStereoImagePair && other) {
        desc = other.desc;
        if (!other._isOwnBuffer()) {
            _copyFrom(other);
        } else {
            _moveFrom(other);
        }
        return *this;
    }

public:
    RemoteImageRef leftImage;
    RemoteImageRef rightImage;
    slamtec_aurora_sdk_stereo_image_pair_desc_t desc;


protected:
    bool _isOwnBuffer() const {
        return (_imgbuffer_left.data() == leftImage._data);
    }

    void _moveFrom(RemoteStereoImagePair& other) {
        _imgbuffer_left = std::move(other._imgbuffer_left);
        _imgbuffer_right = std::move(other._imgbuffer_right);


        leftImage._data = _imgbuffer_left.data();
        rightImage._data = _imgbuffer_right.data();

    }

    void _copyFrom(const RemoteStereoImagePair& other) {
        if (other.leftImage._data) {
            _imgbuffer_left.resize(other.desc.left_image_desc.data_size);
            memcpy(_imgbuffer_left.data(), other.leftImage._data, other.desc.left_image_desc.data_size);
            leftImage._data = _imgbuffer_left.data();
        }
        else {
            leftImage._data = nullptr;
            _imgbuffer_left.clear();
        }

        if (other.rightImage._data) {
            _imgbuffer_right.resize(other.desc.right_image_desc.data_size);
            memcpy(_imgbuffer_right.data(), other.rightImage._data, other.desc.right_image_desc.data_size);
            rightImage._data = _imgbuffer_right.data();
        }
        else {
            rightImage._data = nullptr;
            _imgbuffer_right.clear();
        }
    }

protected:
    std::vector<uint8_t> _imgbuffer_left;
    std::vector<uint8_t> _imgbuffer_right;
};


struct RemoteEnhancedImagingFrame {

public:
    RemoteEnhancedImagingFrame() 
        : image(desc.image_desc, nullptr)
    {
        memset(&desc, 0, sizeof(slamtec_aurora_sdk_enhanced_imaging_frame_desc_t));
    }

    RemoteEnhancedImagingFrame(const slamtec_aurora_sdk_enhanced_imaging_frame_desc_t& desc, const slamtec_aurora_sdk_enhanced_imaging_frame_buffer_t& buffer)
        : desc(desc)
        , image(desc.image_desc, buffer.frame_data)
    {
    }

    RemoteEnhancedImagingFrame(const slamtec_aurora_sdk_enhanced_imaging_frame_desc_t & info,
        std::vector<uint8_t> && buffer
    )
        : desc(info)
        , image(info.image_desc, buffer.data())
        , _imgbuffer(std::move(buffer))
    {
    }

    RemoteEnhancedImagingFrame(const RemoteEnhancedImagingFrame& other)
        : desc(other.desc)
        , image(desc.image_desc, nullptr)
    {
        _copyFrom(other);
    }

    RemoteEnhancedImagingFrame(RemoteEnhancedImagingFrame && other)
        : desc(other.desc)
        , image(desc.image_desc, nullptr)
    {
        if (!other._isOwnBuffer()) {
            _copyFrom(other);
        } else {
            _moveFrom(other);
        }
    }

    RemoteEnhancedImagingFrame& operator=(const RemoteEnhancedImagingFrame& other) {
        desc = other.desc;
        _copyFrom(other);
        return *this;
    }

    RemoteEnhancedImagingFrame& operator=(RemoteEnhancedImagingFrame && other) {
        desc = other.desc;
        if (!other._isOwnBuffer()) {
            _copyFrom(other);
        } else {
            _moveFrom(other);
        }
        return *this;
    }

public:
    RemoteImageRef image;
    slamtec_aurora_sdk_enhanced_imaging_frame_desc_t desc;


protected:
    bool _isOwnBuffer() const {
        return (_imgbuffer.data() == image._data);
    }

    void _moveFrom(RemoteEnhancedImagingFrame& other) {
        _imgbuffer = std::move(other._imgbuffer);


        image._data = _imgbuffer.data();

    }

    void _copyFrom(const RemoteEnhancedImagingFrame& other) {
        if (other.image._data) {
            _imgbuffer.resize(other.desc.image_desc.data_size);
            memcpy(_imgbuffer.data(), other.image._data, other.desc.image_desc.data_size);
            image._data = _imgbuffer.data();
        }
        else {
            image._data = nullptr;
            _imgbuffer.clear();
        }

        if (other.image._data) {
            _imgbuffer.resize(other.desc.image_desc.data_size);
            memcpy(_imgbuffer.data(), other.image._data, other.desc.image_desc.data_size);
            image._data = _imgbuffer.data();
        }
        else {
            image._data = nullptr;
            _imgbuffer.clear();
        }
    }

protected:
    std::vector<uint8_t> _imgbuffer;
};


/**
 * @brief The camera mask image class wraps the camera mask image data
 * @details This class is used to wrap the camera mask image data with RAII semantics
 * @ingroup Cxx_CameraMask_Operations Camera Mask Operations
 */
struct RemoteCameraMaskImage {

public:
    RemoteCameraMaskImage()
        : image(desc, nullptr)
    {
        memset(&desc, 0, sizeof(slamtec_aurora_sdk_image_desc_t));
    }

    RemoteCameraMaskImage(const slamtec_aurora_sdk_image_desc_t& desc, const slamtec_aurora_sdk_camera_mask_image_buffer_t& buffer)
        : desc(desc)
        , image(desc, buffer.image_data)
    {
    }

    RemoteCameraMaskImage(const slamtec_aurora_sdk_image_desc_t& info,
        std::vector<uint8_t>&& buffer
    )
        : desc(info)
        , image(info, buffer.data())
        , _imgbuffer(std::move(buffer))
    {
    }

    RemoteCameraMaskImage(const RemoteCameraMaskImage& other)
        : desc(other.desc)
        , image(desc, nullptr)
    {
        _copyFrom(other);
    }

    RemoteCameraMaskImage(RemoteCameraMaskImage&& other)
        : desc(other.desc)
        , image(desc, nullptr)
    {
        if (!other._isOwnBuffer()) {
            _copyFrom(other);
        } else {
            _moveFrom(other);
        }
    }

    RemoteCameraMaskImage& operator=(const RemoteCameraMaskImage& other) {
        desc = other.desc;
        _copyFrom(other);
        return *this;
    }

    RemoteCameraMaskImage& operator=(RemoteCameraMaskImage&& other) {
        desc = other.desc;
        if (!other._isOwnBuffer()) {
            _copyFrom(other);
        } else {
            _moveFrom(other);
        }
        return *this;
    }

    bool isValid() const {
        return desc.width > 0 && desc.height > 0 && image._data != nullptr;
    }

public:
    RemoteImageRef image;
    slamtec_aurora_sdk_image_desc_t desc;


protected:
    bool _isOwnBuffer() const {
        return (_imgbuffer.data() == image._data);
    }

    void _moveFrom(RemoteCameraMaskImage& other) {
        _imgbuffer = std::move(other._imgbuffer);
        image._data = _imgbuffer.data();
    }

    void _copyFrom(const RemoteCameraMaskImage& other) {
        if (other.image._data) {
            _imgbuffer.resize(other.desc.data_size);
            memcpy(_imgbuffer.data(), other.image._data, other.desc.data_size);
            image._data = _imgbuffer.data();
        }
        else {
            image._data = nullptr;
            _imgbuffer.clear();
        }
    }

protected:
    std::vector<uint8_t> _imgbuffer;
};


/**
 * @brief Dashcam recorder status - C++ extension of C struct
 * @details Inherits from C struct and adds convenience methods.
 * @ingroup Cxx_DashcamRecorder_Operations Dashcam Recorder Operations
 */
struct RemoteDashcamStatus : public slamtec_aurora_sdk_dashcam_status_t {
    RemoteDashcamStatus() {
        memset(this, 0, sizeof(slamtec_aurora_sdk_dashcam_status_t));
    }

    RemoteDashcamStatus(const slamtec_aurora_sdk_dashcam_status_t& c_struct) {
        memcpy(this, &c_struct, sizeof(slamtec_aurora_sdk_dashcam_status_t));
    }

    bool isEnabled() const { return enabled != 0; }
    bool isRecording() const { return recording != 0; }
    float getSizeLimitGB() const { return size_limit_gb; }
    uint64_t getCurrentSizeBytes() const { return current_size_bytes; }
    slamtec_aurora_sdk_dashcam_working_state_t getWorkingState() const { return working_state; }
    const char* getWorkingMessage() const { return working_message; }
    uint64_t getWorkingTimestamp() const { return working_timestamp; }
};

/**
 * @brief Dashcam storage status - C++ extension of C struct
 * @details Inherits from C struct and adds convenience methods.
 * @ingroup Cxx_DashcamRecorder_Operations Dashcam Recorder Operations
 */
struct RemoteDashcamStorageStatus : public slamtec_aurora_sdk_dashcam_storage_status_t {
    RemoteDashcamStorageStatus() {
        memset(this, 0, sizeof(slamtec_aurora_sdk_dashcam_storage_status_t));
    }

    RemoteDashcamStorageStatus(const slamtec_aurora_sdk_dashcam_storage_status_t& c_struct) {
        memcpy(this, &c_struct, sizeof(slamtec_aurora_sdk_dashcam_storage_status_t));
    }

    bool isExternalStoragePresent() const { return external_storage_present != 0; }
    bool isExternalStorageMounted() const { return external_storage_mounted != 0; }
    bool isUsingExternalStorage() const { return using_external_storage != 0; }
    uint64_t getTotalSpaceBytes() const { return total_space_bytes; }
    uint64_t getFreeSpaceBytes() const { return free_space_bytes; }
    uint64_t getUsedByDashcamBytes() const { return used_by_dashcam_bytes; }
    uint64_t getLastUpdateTime() const { return last_update_time; }
};

/**
 * @brief Dashcam session info - C++ extension of C struct
 * @details Inherits from C struct and adds convenience methods.
 * @ingroup Cxx_DashcamRecorder_Operations Dashcam Recorder Operations
 */
struct RemoteDashcamSessionInfo : public slamtec_aurora_sdk_dashcam_session_info_t {
    RemoteDashcamSessionInfo() {
        memset(this, 0, sizeof(slamtec_aurora_sdk_dashcam_session_info_t));
    }

    RemoteDashcamSessionInfo(const slamtec_aurora_sdk_dashcam_session_info_t& c_struct) {
        memcpy(this, &c_struct, sizeof(slamtec_aurora_sdk_dashcam_session_info_t));
    }

    uint32_t getSessionId() const { return session_id; }
    uint64_t getStartTime() const { return start_time; }
    uint64_t getEndTime() const { return end_time; }
    uint64_t getSize() const { return size; }
    uint64_t getStartBlobIndex() const { return start_blob_index; }
    uint64_t getEndBlobIndex() const { return end_blob_index; }
    uint64_t getBlobIdxCount() const { return blob_idx_count; }
};


/**
 * @brief The keyframe data class wraps the keyframe description and its data
 * @details This class is used to wrap the keyframe description and its data.
 * @ingroup Cxx_DataProvider_Operations Data Provider Operations
 */
class RemoteKeyFrameData {
public:
    RemoteKeyFrameData() : desc{ 0 } {
    }

    RemoteKeyFrameData(const slamtec_aurora_sdk_keyframe_desc_t& desc, const uint64_t * lcIDs, const uint64_t * connIDs, const uint64_t * relatedMPIDs)
        : desc(desc)
    {
        if (lcIDs && desc.looped_frame_count) {
            loopedKeyFrameIDs.reserve(desc.looped_frame_count);
            loopedKeyFrameIDs.insert(loopedKeyFrameIDs.end(), lcIDs, lcIDs + desc.looped_frame_count);
        }

        if (connIDs && desc.connected_frame_count) {
            connectedKeyFrameIDs.reserve(desc.connected_frame_count);
            connectedKeyFrameIDs.insert(connectedKeyFrameIDs.end(), connIDs, connIDs + desc.connected_frame_count);
        }

        if (relatedMPIDs && desc.related_mp_count) {
            relatedMapPointIDs.reserve(desc.related_mp_count);
            relatedMapPointIDs.insert(relatedMapPointIDs.end(), relatedMPIDs, relatedMPIDs + desc.related_mp_count);
        }
    }

    RemoteKeyFrameData(const RemoteKeyFrameData& other) : desc(other.desc), loopedKeyFrameIDs(other.loopedKeyFrameIDs), connectedKeyFrameIDs(other.connectedKeyFrameIDs), relatedMapPointIDs(other.relatedMapPointIDs) {
    }

    RemoteKeyFrameData& operator=(const RemoteKeyFrameData& other) {
        desc = other.desc;
        loopedKeyFrameIDs = other.loopedKeyFrameIDs;
        connectedKeyFrameIDs = other.connectedKeyFrameIDs;
        relatedMapPointIDs = other.relatedMapPointIDs;
        return *this;
    }

    RemoteKeyFrameData(RemoteKeyFrameData&& other) : desc(other.desc), loopedKeyFrameIDs(std::move(other.loopedKeyFrameIDs)), connectedKeyFrameIDs(std::move(other.connectedKeyFrameIDs)), relatedMapPointIDs(std::move(other.relatedMapPointIDs)) {
    }

    RemoteKeyFrameData& operator=(RemoteKeyFrameData&& other) {
        desc = other.desc;
        loopedKeyFrameIDs = std::move(other.loopedKeyFrameIDs);
        connectedKeyFrameIDs = std::move(other.connectedKeyFrameIDs);
        relatedMapPointIDs = std::move(other.relatedMapPointIDs);
        return *this;
    }

public:
    /**
     * @brief The keyframe description
     */
    slamtec_aurora_sdk_keyframe_desc_t desc;

    /**
     * @brief The looped keyframe IDs
     */
    std::vector<uint64_t> loopedKeyFrameIDs;

    /**
     * @brief The connected keyframe IDs
     */
    std::vector<uint64_t> connectedKeyFrameIDs;

    /**
     * @brief The related map point IDs
     */
    std::vector<uint64_t> relatedMapPointIDs;
};


/**
 * @brief The single layer LIDAR scan data class wraps the LIDAR scan data and its description
 * @details This class is used to wrap the LIDAR scan data and its description.
 * @ingroup Cxx_DataProvider_Operations Data Provider Operations
 */
class SingleLayerLIDARScan {
public:
    SingleLayerLIDARScan()  {
        memset(&info, 0, sizeof(slamtec_aurora_sdk_lidar_singlelayer_scandata_info_t));
    }

    SingleLayerLIDARScan(const SingleLayerLIDARScan& other) : info(other.info), scanData(other.scanData) {
    }

    SingleLayerLIDARScan& operator=(const SingleLayerLIDARScan& other) {
        info = other.info;
        scanData = other.scanData;
        return *this;
    }

    SingleLayerLIDARScan(SingleLayerLIDARScan&& other) : info(other.info), scanData(std::move(other.scanData)) {
    }

    SingleLayerLIDARScan& operator=(SingleLayerLIDARScan&& other) {
        info = other.info;
        scanData = std::move(other.scanData);
        return *this;
    }

public:
    /**
     * @brief The LIDAR scan data info
     */
    slamtec_aurora_sdk_lidar_singlelayer_scandata_info_t info;

    /**
     * @brief The LIDAR scan data
     */
    std::vector< slamtec_aurora_sdk_lidar_scan_point_t> scanData;

};

/**
 * @brief The 2D gridmap generation options class wraps the 2D gridmap generation options
 * @details This class is used to wrap the 2D gridmap generation options.
 * @ingroup Cxx_LIDAR_2DMap_Operations LIDAR 2D GridMap Operations
 */
class LIDAR2DGridMapGenerationOptions : public slamtec_aurora_sdk_2d_gridmap_generation_options_t {
public:
    LIDAR2DGridMapGenerationOptions()
    {
        memset(this, 0, sizeof(slamtec_aurora_sdk_2d_gridmap_generation_options_t));
        loadDefaults();
    }


    LIDAR2DGridMapGenerationOptions(const slamtec_aurora_sdk_2d_gridmap_generation_options_t& other) {
        memcpy(this, &other, sizeof(slamtec_aurora_sdk_2d_gridmap_generation_options_t));
    }



    LIDAR2DGridMapGenerationOptions(const LIDAR2DGridMapGenerationOptions& other) {
        memcpy(this, &other, sizeof(LIDAR2DGridMapGenerationOptions));
    }

    LIDAR2DGridMapGenerationOptions& operator=(const LIDAR2DGridMapGenerationOptions& other) {
        memcpy(this, &other, sizeof(LIDAR2DGridMapGenerationOptions));
        return *this;
    }


    
    /**
     * @brief Load the default 2D gridmap generation options
     */
    void loadDefaults() {
        this->resolution = SLAMTEC_AURORA_SDK_LIDAR_2D_GRIDMAP_DEFAULT_RESOLUTION;
        this->map_canvas_width = SLAMTEC_AURORA_SDK_LIDAR_2D_GRIDMAP_DEFAULT_WIDTH;
        this->map_canvas_height = SLAMTEC_AURORA_SDK_LIDAR_2D_GRIDMAP_DEFAULT_HEIGHT;
        this->active_map_only = 1;
        this->height_range_specified = 0;
    }

    /**
     * @brief Set the height range for the 2D gridmap generation
     * @param[in] minHeight The minimum height
     * @param[in] maxHeight The maximum height
     */
    void setHeightRange(float minHeight, float maxHeight) {
        this->height_range_specified = 1;
        this->min_height = minHeight;
        this->max_height = maxHeight;
    }

    /**
     * @brief Clear the height range for the 2D gridmap generation
     */
    void clearHeightRange() {
        this->height_range_specified = 0;
        this->min_height = 0;
        this->max_height = 0;
    }


};

/**
 * @brief The floor detection histogram class wraps the floor detection histogram data and its description
 * @details This class is used to wrap the floor detection histogram data and its description.
 * @ingroup Cxx_Auto_Floor_Detection_Operations LIDAR Auto Floor Detection Operations
 */
class FloorDetectionHistogram {
public:
    FloorDetectionHistogram() {
        memset(&info, 0, sizeof(slamtec_aurora_sdk_floor_detection_histogram_info_t));
    }

    FloorDetectionHistogram(const FloorDetectionHistogram& other) : info(other.info), histogramData(other.histogramData) {
    }

    FloorDetectionHistogram& operator=(const FloorDetectionHistogram& other) {
        info = other.info;
        histogramData = other.histogramData;
        return *this;
    }

    FloorDetectionHistogram(FloorDetectionHistogram&& other) : info(other.info), histogramData(std::move(other.histogramData)) {
    }

    FloorDetectionHistogram& operator=(FloorDetectionHistogram&& other)  {
        info = other.info;
        histogramData = std::move(other.histogramData);
        return *this;
    }

public:
    /**
     * @brief The floor detection histogram info
     */
    slamtec_aurora_sdk_floor_detection_histogram_info_t info;

    /**
     * @brief The floor detection histogram data
     */
    std::vector<float> histogramData;
};

/**
 * @brief The pose covariance class wraps the pose covariance data
 * @details This class provides zero-copy access to covariance matrix data.
 * @details It can reference external buffers (zero-copy) or own its own buffer.
 * @details When Eigen library is available, it provides conversion to Eigen::Matrix.
 * @ingroup Cxx_Data_Types Data Types
 */
class PoseCovariance {
public:
    /**
     * @brief Default constructor - creates with owned zero-initialized buffer
     */
    PoseCovariance() : _data_ptr(nullptr), _owned_buffer(36, 0.0f) {
        _data_ptr = _owned_buffer.data();
    }

    /**
     * @brief Construct from external buffer (zero-copy, references external data)
     * @param external_buffer Pointer to 36 floats in column-major order
     * @warning The external buffer must remain valid for the lifetime of this object
     */
    explicit PoseCovariance(const float* external_buffer) : _data_ptr(external_buffer) {
        // No copy - just store the pointer
    }

    /**
     * @brief Construct from C structure (copies data to owned buffer)
     */
    PoseCovariance(const slamtec_aurora_sdk_pose_covariance_t& c_struct)
        : _data_ptr(nullptr), _owned_buffer(c_struct.covariance_matrix, c_struct.covariance_matrix + 36) {
        _data_ptr = _owned_buffer.data();
    }

    /**
     * @brief Copy constructor
     */
    PoseCovariance(const PoseCovariance& other) {
        if (other._owned_buffer.empty()) {
            // Other is using foreign buffer, copy the data to our own buffer
            _owned_buffer.assign(other._data_ptr, other._data_ptr + 36);
            _data_ptr = _owned_buffer.data();
        } else {
            // Other owns its buffer, copy it
            _owned_buffer = other._owned_buffer;
            _data_ptr = _owned_buffer.data();
        }
    }

    /**
     * @brief Assignment operator
     */
    PoseCovariance& operator=(const PoseCovariance& other) {
        if (this != &other) {
            if (other._owned_buffer.empty()) {
                // Other is using foreign buffer, copy the data to our own buffer
                _owned_buffer.assign(other._data_ptr, other._data_ptr + 36);
                _data_ptr = _owned_buffer.data();
            } else {
                // Other owns its buffer, copy it
                _owned_buffer = other._owned_buffer;
                _data_ptr = _owned_buffer.data();
            }
        }
        return *this;
    }

    /**
     * @brief Assignment operator from C structure
     * @param c_struct The C structure to assign from
     * @return Reference to this object
     */
    PoseCovariance& operator=(const slamtec_aurora_sdk_pose_covariance_t& c_struct) {
        _owned_buffer.assign(c_struct.covariance_matrix, c_struct.covariance_matrix + 36);
        _data_ptr = _owned_buffer.data();
        return *this;
    }

    /**
     * @brief Conversion operator to C structure
     * @return C structure with covariance data
     */
    operator slamtec_aurora_sdk_pose_covariance_t() const {
        return toCStruct();
    }

    /**
     * @brief Get raw pointer to covariance matrix data (36 floats, column-major)
     */
    const float* data() const { return _data_ptr; }

    /**
     * @brief Get element at (row, col)
     */
    float operator()(int row, int col) const {
        return _data_ptr[col * 6 + row]; // column-major
    }

    /**
     * @brief Convert to C structure (always copies data)
     */
    slamtec_aurora_sdk_pose_covariance_t toCStruct() const {
        slamtec_aurora_sdk_pose_covariance_t result;
        memcpy(result.covariance_matrix, _data_ptr, sizeof(result.covariance_matrix));
        return result;
    }

    /**
     * @brief Convert to Eigen::Matrix<float,6,6>
     * @details This function is only available when Eigen library headers are included.
     * @details The covariance matrix is stored in column-major order, compatible with Eigen's default.
     * @param[out] mat The output Eigen matrix
     * @return true if the conversion is successful, false otherwise
     */
    template <typename T>
    typename std::enable_if<is_eigen_matrix<T>::value && std::is_same<typename T::Scalar, float>::value && T::RowsAtCompileTime == 6 && T::ColsAtCompileTime == 6, bool>::type
    toEigenMatrix(T& mat) const {
        memcpy(mat.data(), _data_ptr, 36 * sizeof(float));
        return true;
    }

    /**
     * @brief Construct from Eigen::Matrix<float,6,6>
     * @details This function is only available when Eigen library headers are included.
     * @param mat The 6x6 covariance matrix
     * @return true if the conversion is successful, false otherwise
     */
    template <typename T>
    typename std::enable_if<is_eigen_matrix<T>::value && std::is_same<typename T::Scalar, float>::value && T::RowsAtCompileTime == 6 && T::ColsAtCompileTime == 6, bool>::type
    fromEigenMatrix(const T& mat) {
        _owned_buffer.assign(mat.data(), mat.data() + 36);
        _data_ptr = _owned_buffer.data();
        return true;
    }

    /**
     * @brief Convert to human-readable covariance metrics
     * @param[out] readable_out The output readable covariance metrics
     * @return true if the conversion is successful, false otherwise
     */
    bool toHumanReadable(slamtec_aurora_sdk_pose_covariance_readable_t& readable_out) const {
        slamtec_aurora_sdk_pose_covariance_t temp = toCStruct();
        return slamtec_aurora_sdk_convert_pose_covariance_to_readable(&temp, &readable_out) == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

    /**
     * @brief Get pointer to C struct representation (for compatibility)
     * @warning This returns a temporary - do not store the pointer!
     * @warning Only valid for owned buffer PoseCovariance objects
     */
    const slamtec_aurora_sdk_pose_covariance_t* getCStructPtr() const {
        // For compatibility with old code that expects a pointer to C struct
        // Note: This is a workaround and should be avoided in new code
        static thread_local slamtec_aurora_sdk_pose_covariance_t temp;
        temp = toCStruct();
        return &temp;
    }

private:
    const float* _data_ptr;              // Points to either _owned_buffer or external buffer
    std::vector<float> _owned_buffer;    // Owned buffer (empty if using foreign buffer)
};

/**
 * @brief The pose covariance readable values class wraps the human-readable covariance metrics
 * @details This class extends the C structure with convenient C++ features.
 * @ingroup Cxx_Data_Types Data Types
 */
class PoseCovarianceReadable : public slamtec_aurora_sdk_pose_covariance_readable_t {
public:
    PoseCovarianceReadable() : slamtec_aurora_sdk_pose_covariance_readable_t() {
        memset(this, 0, sizeof(slamtec_aurora_sdk_pose_covariance_readable_t));
    }

    PoseCovarianceReadable(const slamtec_aurora_sdk_pose_covariance_readable_t& other) {
        memcpy(this, &other, sizeof(slamtec_aurora_sdk_pose_covariance_readable_t));
    }

    PoseCovarianceReadable& operator=(const slamtec_aurora_sdk_pose_covariance_readable_t& other) {
        memcpy(this, &other, sizeof(slamtec_aurora_sdk_pose_covariance_readable_t));
        return *this;
    }

    /**
     * @brief Get the 95% confidence ellipsoid semi-axes for position (x, y, z) in meters
     * @return std::array<float,3> containing [x, y, z] semi-axes
     */
    std::array<float,3> getPositionEllipsoid95() const {
        return {position_ellipsoid_95_xyz[0], position_ellipsoid_95_xyz[1], position_ellipsoid_95_xyz[2]};
    }

    /**
     * @brief Get the 95% confidence radius for 2D position (x, y) in meters
     * @return float radius in meters
     */
    float getPositionRadius95XY() const {
        return position_radius_95_xy;
    }

    /**
     * @brief Get the 1-sigma standard deviation for rotation (roll, pitch, yaw) in degrees
     * @return std::array<float,3> containing [roll, pitch, yaw] in degrees
     */
    std::array<float,3> getRotation1SigmaRPY() const {
        return {rotation_1sigma_rpy_deg[0], rotation_1sigma_rpy_deg[1], rotation_1sigma_rpy_deg[2]};
    }
};


}}} // namespace rp::standalone::aurora