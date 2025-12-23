/*
 *  SLAMTEC Aurora
 *  Copyright 2013 - 2024 SLAMTEC Co., Ltd.
 *
 *  http://www.slamtec.com
 *
 *  Aurora Remote SDK
 *  Configuration Handlers - Config Manager, Transform Manager, Camera Mask Manager
 *
 *  At lease C++ 14 is required
 */


#pragma once

#include "slamtec_remote_objects.hxx"

/**
 * @defgroup Cxx_Config_Operations Config Operations
 * @brief The configuration management classes
 */

/**
 * @defgroup Cxx_TransformManager_Operations Transform Manager Operations
 * @brief The transform manager classes
 */

/**
 * @defgroup Cxx_CameraMask_Operations Camera Mask Operations
 * @brief The camera mask manager classes
 */


namespace rp { namespace standalone { namespace aurora {

// Forward declarations
class ConfigManager;

/**
 * @brief Wrapper class for config data
 * @details Encapsulates the config data object and provides convenient methods for manipulation
 * @ingroup Cxx_Config_Operations Config Operations
 */
class ConfigData : public Noncopyable {
public:
    /**
     * @brief Constructor - creates a new config data object
     */
    ConfigData() {
        _handle = slamtec_aurora_sdk_config_data_create();
    }

    /**
     * @brief Destructor - destroys the config data object
     */
    ~ConfigData() {
        if (_handle) {
            slamtec_aurora_sdk_config_data_destroy(_handle);
        }
    }

    /**
     * @brief Move constructor - transfers ownership from another object
     * @param[in] other The source object to move from
     */
    ConfigData(ConfigData&& other) noexcept {
        _handle = other._handle;
        other._handle = nullptr;
    }

    /**
     * @brief Move assignment operator - transfers ownership from another object
     * @param[in] other The source object to move from
     * @return Reference to this object
     */
    ConfigData& operator=(ConfigData&& other) noexcept {
        if (this != &other) {
            // Destroy current handle if exists
            if (_handle) {
                slamtec_aurora_sdk_config_data_destroy(_handle);
            }
            // Transfer ownership
            _handle = other._handle;
            other._handle = nullptr;
        }
        return *this;
    }

    /**
     * @brief Check if the config data object is valid
     * @return True if valid, false otherwise
     */
    bool isValid() const {
        return _handle != nullptr;
    }

    /**
     * @brief Load config data from JSON string
     * @param[in] json_string The JSON string to load
     * @param[out] errcode The error code, set to nullptr if not interested
     * @return True if loaded successfully, false otherwise
     */
    bool loadFromString(const std::string& json_string, slamtec_aurora_sdk_errorcode_t* errcode = nullptr) {
        if (!_handle) {
            if (errcode) *errcode = SLAMTEC_AURORA_SDK_ERRORCODE_INVALID_ARGUMENT;
            return false;
        }

        auto result = slamtec_aurora_sdk_config_data_load_from_string(_handle, json_string.c_str());
        if (errcode) *errcode = result;
        return result == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

    /**
     * @brief Dump config data to JSON string
     * @param[out] errcode The error code, set to nullptr if not interested
     * @return The JSON string, empty if failed
     */
    std::string dumpToString(slamtec_aurora_sdk_errorcode_t* errcode = nullptr) const {
        if (!_handle) {
            if (errcode) *errcode = SLAMTEC_AURORA_SDK_ERRORCODE_INVALID_ARGUMENT;
            return std::string();
        }

        size_t size = 0;
        const char* dumpStr = slamtec_aurora_sdk_config_data_create_string_dump(_handle, &size);

        if (!dumpStr) {
            if (errcode) *errcode = SLAMTEC_AURORA_SDK_ERRORCODE_OP_FAILED;
            return std::string();
        }

        // Copy the string data
        std::string result(dumpStr, size);

        // Destroy the cached string dump to release memory
        slamtec_aurora_sdk_config_data_destroy_string_dump(_handle);

        if (errcode) *errcode = SLAMTEC_AURORA_SDK_ERRORCODE_OK;
        return result;
    }

    /**
     * @brief Load config data from a file
     * @param[in] file_path Path to the file
     * @param[out] errcode The error code, set to nullptr if not interested
     * @return True if loaded successfully, false otherwise
     */
    bool loadFromFile(const std::string& file_path, slamtec_aurora_sdk_errorcode_t* errcode = nullptr) {
        if (!_handle) {
            if (errcode) *errcode = SLAMTEC_AURORA_SDK_ERRORCODE_INVALID_ARGUMENT;
            return false;
        }

        auto result = slamtec_aurora_sdk_config_data_load_from_file(_handle, file_path.c_str());
        if (errcode) *errcode = result;
        return result == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

    /**
     * @brief Save config data to a file
     * @param[in] file_path Path to the file
     * @param[out] errcode The error code, set to nullptr if not interested
     * @return True if saved successfully, false otherwise
     */
    bool saveToFile(const std::string& file_path, slamtec_aurora_sdk_errorcode_t* errcode = nullptr) const {
        if (!_handle) {
            if (errcode) *errcode = SLAMTEC_AURORA_SDK_ERRORCODE_INVALID_ARGUMENT;
            return false;
        }

        auto result = slamtec_aurora_sdk_config_data_save_to_file(_handle, file_path.c_str());
        if (errcode) *errcode = result;
        return result == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

    /**
     * @brief Get the underlying C handle
     * @return The C handle
     */
    slamtec_aurora_sdk_config_data_t getHandle() const {
        return _handle;
    }

private:
    slamtec_aurora_sdk_config_data_t _handle;
};


/**
 * @brief The persistent configuration manager class for managing device configuration
 * @details Use this class to get/set/reset persistent configuration entries on the remote device
 * @ingroup Cxx_Config_Operations Config Operations
 */
class PersistentConfigManager : public Noncopyable {
    friend class RemoteSDK;
public:

    /**
     * @brief Enumerate all available persistent config entries
     * @param[in] timeout_ms The timeout in milliseconds, default is 5000ms
     * @param[out] errcode The error code, set to nullptr if not interested
     * @return A vector of filter path strings, empty if failed
     */
    std::vector<std::string> enumAllEntries(uint64_t timeout_ms = SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_TIMEOUT, slamtec_aurora_sdk_errorcode_t* errcode = nullptr) {
        std::vector<std::string> result;

        auto entryList = slamtec_aurora_sdk_persistent_config_enum_all_entries(_sdk, timeout_ms);

        if (!entryList) {
            if (errcode) {
                *errcode = SLAMTEC_AURORA_SDK_ERRORCODE_OP_FAILED;
            }
            return result;
        }

        if (errcode) {
            *errcode = SLAMTEC_AURORA_SDK_ERRORCODE_OK;
        }

        int count = slamtec_aurora_sdk_config_entry_list_get_count(entryList);
        result.reserve(count);
        for (int i = 0; i < count; i++) {
            const char* entry = slamtec_aurora_sdk_config_entry_list_get_entry(entryList, i);
            if (entry) {
                result.push_back(entry);
            }
        }

        slamtec_aurora_sdk_config_entry_list_destroy(entryList);
        return result;
    }

    /**
     * @brief Reset a specific persistent config entry to default
     * @param[in] filter_path The filter path of the config entry to reset
     * @param[in] timeout_ms The timeout in milliseconds, default is 5000ms
     * @param[out] errcode The error code, set to nullptr if not interested
     * @return True if reset successfully, false otherwise
     */
    bool resetConfig(const std::string& filter_path, uint64_t timeout_ms = SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_TIMEOUT, slamtec_aurora_sdk_errorcode_t* errcode = nullptr) {
        auto result = slamtec_aurora_sdk_persistent_config_reset_config(_sdk, filter_path.c_str(), timeout_ms);
        if (errcode) {
            *errcode = result;
        }
        return result == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

    /**
     * @brief Reset all persistent config entries to default
     * @param[in] timeout_ms The timeout in milliseconds, default is 5000ms
     * @param[out] errcode The error code, set to nullptr if not interested
     * @return True if reset successfully, false otherwise
     */
    bool resetAllConfig(uint64_t timeout_ms = SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_TIMEOUT, slamtec_aurora_sdk_errorcode_t* errcode = nullptr) {
        auto result = slamtec_aurora_sdk_persistent_config_reset_all_config(_sdk, timeout_ms);
        if (errcode) {
            *errcode = result;
        }
        return result == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

    /**
     * @brief Set a persistent config entry with config data object
     * @param[in] filter_path The filter path of the config entry
     * @param[in] key The key for merging config, use "@overwrite" to overwrite the entire config
     * @param[in] config_data The config data object
     * @param[in] timeout_ms The timeout in milliseconds, default is 5000ms
     * @param[out] errcode The error code, set to nullptr if not interested
     * @return True if set successfully, false otherwise
     */
    bool setConfig(const std::string& filter_path, const std::string& key, const ConfigData& config_data, uint64_t timeout_ms = SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_TIMEOUT, slamtec_aurora_sdk_errorcode_t* errcode = nullptr) {
        if (!config_data.isValid()) {
            if (errcode) {
                *errcode = SLAMTEC_AURORA_SDK_ERRORCODE_INVALID_ARGUMENT;
            }
            return false;
        }

        auto result = slamtec_aurora_sdk_persistent_config_set_config(_sdk, filter_path.c_str(), key.c_str(), config_data.getHandle(), timeout_ms);

        if (errcode) {
            *errcode = result;
        }
        return result == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

    /**
     * @brief Set a persistent config entry with JSON string (convenience method)
     * @param[in] filter_path The filter path of the config entry
     * @param[in] key The key for merging config, use "@overwrite" to overwrite the entire config
     * @param[in] config_json The config in JSON string format
     * @param[in] timeout_ms The timeout in milliseconds, default is 5000ms
     * @param[out] errcode The error code, set to nullptr if not interested
     * @return True if set successfully, false otherwise
     */
    bool setConfig(const std::string& filter_path, const std::string& key, const std::string& config_json, uint64_t timeout_ms = SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_TIMEOUT, slamtec_aurora_sdk_errorcode_t* errcode = nullptr) {
        ConfigData configData;
        if (!configData.isValid()) {
            if (errcode) {
                *errcode = SLAMTEC_AURORA_SDK_ERRORCODE_OP_FAILED;
            }
            return false;
        }

        if (!configData.loadFromString(config_json, errcode)) {
            return false;
        }

        return setConfig(filter_path, key, configData, timeout_ms, errcode);
    }

    /**
     * @brief Get a persistent config entry and populate config data object
     * @param[in] filter_path The filter path of the config entry
     * @param[out] config_data The config data object to populate
     * @param[in] timeout_ms The timeout in milliseconds, default is 5000ms
     * @param[out] errcode The error code, set to nullptr if not interested
     * @return True if get successfully, false otherwise
     */
    bool getConfig(const std::string& filter_path, ConfigData& config_data, uint64_t timeout_ms = SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_TIMEOUT, slamtec_aurora_sdk_errorcode_t* errcode = nullptr) {
        if (!config_data.isValid()) {
            if (errcode) {
                *errcode = SLAMTEC_AURORA_SDK_ERRORCODE_INVALID_ARGUMENT;
            }
            return false;
        }

        auto result = slamtec_aurora_sdk_persistent_config_get_config(_sdk, filter_path.c_str(), config_data.getHandle(), timeout_ms);

        if (errcode) {
            *errcode = result;
        }

        return result == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

    /**
     * @brief Get a persistent config entry as JSON string (convenience method)
     * @param[in] filter_path The filter path of the config entry
     * @param[in] timeout_ms The timeout in milliseconds, default is 5000ms
     * @param[out] errcode The error code, set to nullptr if not interested
     * @return The config JSON string, empty if failed
     */
    std::string getConfig(const std::string& filter_path, uint64_t timeout_ms = SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_TIMEOUT, slamtec_aurora_sdk_errorcode_t* errcode = nullptr) {
        ConfigData configData;
        if (!configData.isValid()) {
            if (errcode) {
                *errcode = SLAMTEC_AURORA_SDK_ERRORCODE_OP_FAILED;
            }
            return std::string();
        }

        if (!getConfig(filter_path, configData, timeout_ms, errcode)) {
            return std::string();
        }

        return configData.dumpToString(errcode);
    }

protected:
    PersistentConfigManager(slamtec_aurora_sdk_session_handle_t& sdk)
        : _sdk(sdk)
    {}

    slamtec_aurora_sdk_session_handle_t _sdk;
};
/**
 * @brief The Transform Manager class for managing configurable transforms
 * @details Use this class to get/set/reset transforms by name
 * @ingroup Cxx_TransformManager_Operations Transform Manager Operations
 */
class TransformManager : public Noncopyable {
    friend class RemoteSDK;
public:

    ~TransformManager() {
        if (_handle) {
            slamtec_aurora_sdk_transform_manager_destroy(_handle);
        }
    }

    /**
     * @brief Get all transform names
     * @param[in] timeout_ms The timeout in milliseconds, default is 5000ms
     * @param[out] errcode The error code, set to nullptr if not interested
     * @return A vector of transform names, empty if failed
     */
    std::vector<std::string> getAllTransformNames(uint64_t timeout_ms = SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_TIMEOUT, slamtec_aurora_sdk_errorcode_t* errcode = nullptr) {
        std::vector<std::string> result;

        auto nameList = slamtec_aurora_sdk_transform_manager_get_all_transform_names(_handle, timeout_ms);
        if (!nameList) {
            if (errcode) *errcode = SLAMTEC_AURORA_SDK_ERRORCODE_OP_FAILED;
            return result;
        }

        int count = slamtec_aurora_sdk_transform_manager_name_list_get_count(nameList);
        for (int i = 0; i < count; i++) {
            const char* name = slamtec_aurora_sdk_transform_manager_name_list_get_entry(nameList, i);
            if (name) {
                result.push_back(name);
            }
        }

        slamtec_aurora_sdk_transform_manager_name_list_destroy(nameList);

        if (errcode) *errcode = SLAMTEC_AURORA_SDK_ERRORCODE_OK;
        return result;
    }

    /**
     * @brief Get a transform by name
     * @param[in] name The transform name
     * @param[out] transform The output SE3 pose
     * @param[in] timeout_ms The timeout in milliseconds, default is 5000ms
     * @param[out] errcode The error code, set to nullptr if not interested
     * @return True if successful, false otherwise
     */
    bool getTransform(const std::string& name, slamtec_aurora_sdk_pose_se3_t& transform, uint64_t timeout_ms = SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_TIMEOUT, slamtec_aurora_sdk_errorcode_t* errcode = nullptr) {
        auto result = slamtec_aurora_sdk_transform_manager_get_transform(_handle, name.c_str(), &transform, timeout_ms);
        if (errcode) *errcode = result;
        return result == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

    /**
     * @brief Set a transform by name
     * @param[in] name The transform name
     * @param[in] transform The SE3 pose to set
     * @param[in] timeout_ms The timeout in milliseconds, default is 5000ms
     * @param[out] errcode The error code, set to nullptr if not interested
     * @return True if successful, false otherwise
     */
    bool setTransform(const std::string& name, const slamtec_aurora_sdk_pose_se3_t& transform, uint64_t timeout_ms = SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_TIMEOUT, slamtec_aurora_sdk_errorcode_t* errcode = nullptr) {
        auto result = slamtec_aurora_sdk_transform_manager_set_transform(_handle, name.c_str(), &transform, timeout_ms);
        if (errcode) *errcode = result;
        return result == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

    /**
     * @brief Reset a transform to identity by name
     * @param[in] name The transform name
     * @param[in] timeout_ms The timeout in milliseconds, default is 5000ms
     * @param[out] errcode The error code, set to nullptr if not interested
     * @return True if successful, false otherwise
     */
    bool resetTransform(const std::string& name, uint64_t timeout_ms = SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_TIMEOUT, slamtec_aurora_sdk_errorcode_t* errcode = nullptr) {
        auto result = slamtec_aurora_sdk_transform_manager_reset_transform(_handle, name.c_str(), timeout_ms);
        if (errcode) *errcode = result;
        return result == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

    /**
     * @brief Check if a transform exists
     * @param[in] name The transform name
     * @param[in] timeout_ms The timeout in milliseconds, default is 5000ms
     * @param[out] errcode The error code, set to nullptr if not interested
     * @return True if transform exists, false otherwise
     */
    bool hasTransform(const std::string& name, uint64_t timeout_ms = SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_TIMEOUT, slamtec_aurora_sdk_errorcode_t* errcode = nullptr) {
        int result = slamtec_aurora_sdk_transform_manager_has_transform(_handle, name.c_str(), timeout_ms);
        if (result < 0) {
            if (errcode) *errcode = SLAMTEC_AURORA_SDK_ERRORCODE_OP_FAILED;
            return false;
        }
        if (errcode) *errcode = SLAMTEC_AURORA_SDK_ERRORCODE_OK;
        return result != 0;
    }

    /**
     * @brief Refresh the transform manager configuration from device
     * @details Forces a reload of configuration from the device, bypassing the cache
     * @param[in] timeout_ms The timeout in milliseconds, default is 5000ms
     * @param[out] errcode The error code, set to nullptr if not interested
     * @return True if successful, false otherwise
     */
    bool refresh(uint64_t timeout_ms = SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_TIMEOUT, slamtec_aurora_sdk_errorcode_t* errcode = nullptr) {
        auto result = slamtec_aurora_sdk_transform_manager_refresh(_handle, timeout_ms);
        if (errcode) *errcode = result;
        return result == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

protected:
    TransformManager(slamtec_aurora_sdk_session_handle_t& sdk)
        : _handle(nullptr)
    {
        _handle = slamtec_aurora_sdk_transform_manager_create(sdk);
    }

    slamtec_aurora_sdk_transform_manager_t _handle;
};


/**
 * @brief The Camera Mask Manager class for managing camera input masks
 * @details Use this class to enable/disable and get/set camera masks
 * @ingroup Cxx_CameraMask_Operations Camera Mask Operations
 */
class CameraMaskManager : public Noncopyable {
    friend class RemoteSDK;
public:

    ~CameraMaskManager() {
        if (_handle) {
            slamtec_aurora_sdk_camera_mask_destroy(_handle);
        }
    }

    /**
     * @brief Check if static mask is enabled
     * @param[in] timeout_ms The timeout in milliseconds, default is 5000ms
     * @param[out] errcode The error code, set to nullptr if not interested
     * @return True if enabled, false otherwise
     */
    bool isStaticMaskEnabled(uint64_t timeout_ms = SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_TIMEOUT, slamtec_aurora_sdk_errorcode_t* errcode = nullptr) {
        int result = slamtec_aurora_sdk_camera_mask_is_static_mask_enabled(_handle, timeout_ms);
        if (result < 0) {
            if (errcode) *errcode = SLAMTEC_AURORA_SDK_ERRORCODE_OP_FAILED;
            return false;
        }
        if (errcode) *errcode = SLAMTEC_AURORA_SDK_ERRORCODE_OK;
        return result != 0;
    }

    /**
     * @brief Enable or disable static mask
     * @param[in] enable True to enable, false to disable
     * @param[in] timeout_ms The timeout in milliseconds, default is 5000ms
     * @param[out] errcode The error code, set to nullptr if not interested
     * @return True if successful, false otherwise
     */
    bool setStaticMaskEnable(bool enable, uint64_t timeout_ms = SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_TIMEOUT, slamtec_aurora_sdk_errorcode_t* errcode = nullptr) {
        auto result = slamtec_aurora_sdk_camera_mask_set_static_mask_enable(_handle, enable ? 1 : 0, timeout_ms);
        if (errcode) *errcode = result;
        return result == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

    /**
     * @brief Get static camera mask image by camera index
     * @param[in] camera_index The camera index (0 for left, 1 for right, etc.)
     * @param[in] timeout_ms The timeout in milliseconds, default is 5000ms
     * @param[out] errcode The error code, set to nullptr if not interested
     * @return RemoteCameraMaskImage object with image data, check isValid() to see if successful
     */
    RemoteCameraMaskImage getStaticCameraMaskImage(int camera_index, uint64_t timeout_ms = SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_TIMEOUT, slamtec_aurora_sdk_errorcode_t* errcode = nullptr) {
        RemoteCameraMaskImage result;

        slamtec_aurora_sdk_image_desc_t image_desc = {};
        slamtec_aurora_sdk_camera_mask_image_buffer_t image_buffer = {};

        // First call to get required buffer size
        auto err = slamtec_aurora_sdk_camera_mask_get_static_camera_mask_image(_handle, camera_index, &image_desc, &image_buffer, timeout_ms);
        if (err == SLAMTEC_AURORA_SDK_ERRORCODE_INSUFFICIENT_BUFFER) {
            // Allocate buffer and try again
            std::vector<uint8_t> buffer(image_desc.data_size);
            image_buffer.image_data = buffer.data();
            image_buffer.image_data_size = image_desc.data_size;

            err = slamtec_aurora_sdk_camera_mask_get_static_camera_mask_image(_handle, camera_index, &image_desc, &image_buffer, timeout_ms);
            if (err == SLAMTEC_AURORA_SDK_ERRORCODE_OK) {
                result = RemoteCameraMaskImage(image_desc, std::move(buffer));
                if (errcode) *errcode = SLAMTEC_AURORA_SDK_ERRORCODE_OK;
                return result;
            }
        }

        if (errcode) *errcode = err;
        return result;
    }

    /**
     * @brief Set static camera mask image for a camera index
     * @param[in] camera_index The camera index (0 for left, 1 for right, etc.)
     * @param[in] width Image width
     * @param[in] height Image height
     * @param[in] data Pointer to grayscale image data (8-bit per pixel)
     * @param[in] data_size Size of data in bytes
     * @param[in] timeout_ms The timeout in milliseconds, default is 5000ms
     * @param[out] errcode The error code, set to nullptr if not interested
     * @return True if successful, false otherwise
     */
    bool setStaticCameraMaskImage(int camera_index, uint32_t width, uint32_t height, const uint8_t* data, size_t data_size, uint64_t timeout_ms = SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_TIMEOUT, slamtec_aurora_sdk_errorcode_t* errcode = nullptr) {
        slamtec_aurora_sdk_image_desc_t image_desc = {};
        image_desc.width = width;
        image_desc.height = height;
        image_desc.stride = width;  // Assuming no padding for grayscale
        image_desc.format = 0;  // gray
        image_desc.data_size = static_cast<uint32_t>(data_size);

        auto result = slamtec_aurora_sdk_camera_mask_set_static_camera_mask_image(_handle, camera_index, &image_desc, data, timeout_ms);
        if (errcode) *errcode = result;
        return result == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

    /**
     * @brief Get list of camera indices that have static masks configured
     * @param[in] timeout_ms The timeout in milliseconds, default is 5000ms
     * @param[out] errcode The error code, set to nullptr if not interested
     * @return A vector of camera indices, empty if failed
     */
    std::vector<int> getStaticCameraMaskImageIdList(uint64_t timeout_ms = SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_TIMEOUT, slamtec_aurora_sdk_errorcode_t* errcode = nullptr) {
        std::vector<int> result;

        int count = slamtec_aurora_sdk_camera_mask_get_static_camera_mask_image_id_count(_handle, timeout_ms);
        if (count <= 0) {
            if (errcode) *errcode = count < 0 ? SLAMTEC_AURORA_SDK_ERRORCODE_OP_FAILED : SLAMTEC_AURORA_SDK_ERRORCODE_OK;
            return result;
        }

        result.resize(count);
        int actual_count = slamtec_aurora_sdk_camera_mask_get_static_camera_mask_image_indices(_handle, result.data(), count, timeout_ms);
        if (actual_count > 0) {
            result.resize(actual_count);
            if (errcode) *errcode = SLAMTEC_AURORA_SDK_ERRORCODE_OK;
        } else {
            result.clear();
            if (errcode) *errcode = SLAMTEC_AURORA_SDK_ERRORCODE_OP_FAILED;
        }

        return result;
    }

    /**
     * @brief Remove static camera mask for a specific camera index
     * @param[in] camera_index The camera index to remove mask for
     * @param[in] timeout_ms The timeout in milliseconds, default is 5000ms
     * @param[out] errcode The error code, set to nullptr if not interested
     * @return True if successful, false otherwise
     */
    bool removeStaticCameraMaskImage(int camera_index, uint64_t timeout_ms = SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_TIMEOUT, slamtec_aurora_sdk_errorcode_t* errcode = nullptr) {
        auto result = slamtec_aurora_sdk_camera_mask_remove_static_camera_mask_image(_handle, camera_index, timeout_ms);
        if (errcode) *errcode = result;
        return result == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

    /**
     * @brief Refresh the camera mask configuration from device
     * @details Forces a reload of configuration from the device, bypassing the cache
     * @param[in] timeout_ms The timeout in milliseconds, default is 5000ms
     * @param[out] errcode The error code, set to nullptr if not interested
     * @return True if successful, false otherwise
     */
    bool refresh(uint64_t timeout_ms = SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_TIMEOUT, slamtec_aurora_sdk_errorcode_t* errcode = nullptr) {
        auto result = slamtec_aurora_sdk_camera_mask_refresh(_handle, timeout_ms);
        if (errcode) *errcode = result;
        return result == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

protected:
    CameraMaskManager(slamtec_aurora_sdk_session_handle_t& sdk)
        : _handle(nullptr)
    {
        _handle = slamtec_aurora_sdk_camera_mask_create(sdk);
    }

    slamtec_aurora_sdk_camera_mask_t _handle;
};


}}} // namespace rp::standalone::aurora
