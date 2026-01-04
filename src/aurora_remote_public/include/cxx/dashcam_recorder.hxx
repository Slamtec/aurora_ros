/*
 *  SLAMTEC Aurora Remote SDK
 *  Dashcam Recorder C++ Wrapper
 *
 *  Copyright 2013 - 2025 SLAMTEC Co., Ltd.
 *  http://www.slamtec.com
 *
 *  At least C++14 is required
 */

#pragma once

#include "slamtec_remote_objects.hxx"

namespace rp { namespace standalone { namespace aurora {

/**
 * @brief Dashcam Storage Info wrapper class
 * @details C++ wrapper class around slamtec_aurora_sdk_dashcam_storage_info_t handle.
 *          Provides convenient access to storage information via accessor methods.
 *          The handle is automatically destroyed when this object goes out of scope.
 * @ingroup Cxx_DashcamRecorder_Operations Dashcam Recorder Operations
 */
class RemoteDashcamStorageInfo : public Noncopyable {
public:
    /**
     * @brief Construct from handle (takes ownership)
     * @param[in] handle The storage info handle (ownership is transferred)
     */
    explicit RemoteDashcamStorageInfo(slamtec_aurora_sdk_dashcam_storage_info_t handle = nullptr)
        : _handle(handle)
    {}

    /**
     * @brief Move constructor
     */
    RemoteDashcamStorageInfo(RemoteDashcamStorageInfo&& other) noexcept
        : _handle(other._handle)
    {
        other._handle = nullptr;
    }

    /**
     * @brief Move assignment operator
     */
    RemoteDashcamStorageInfo& operator=(RemoteDashcamStorageInfo&& other) noexcept {
        if (this != &other) {
            if (_handle) {
                slamtec_aurora_sdk_dashcam_storage_info_destroy(_handle);
            }
            _handle = other._handle;
            other._handle = nullptr;
        }
        return *this;
    }

    /**
     * @brief Destructor - destroys the handle
     */
    ~RemoteDashcamStorageInfo() {
        if (_handle) {
            slamtec_aurora_sdk_dashcam_storage_info_destroy(_handle);
        }
    }

    /**
     * @brief Check if the handle is valid
     * @return True if valid, false otherwise
     */
    bool isValid() const { return _handle != nullptr; }

    /**
     * @brief Bool conversion operator
     */
    explicit operator bool() const { return isValid(); }

    /**
     * @brief Get the underlying handle
     * @return The storage info handle
     */
    slamtec_aurora_sdk_dashcam_storage_info_t getHandle() const { return _handle; }

    /**
     * @brief Get storage path
     * @return Storage path string, or empty string on error
     */
    const char* getPath() const {
        if (!_handle) return "";
        const char* path = slamtec_aurora_sdk_dashcam_storage_info_get_path(_handle);
        return path ? path : "";
    }

    /**
     * @brief Check if storage status is available
     * @return True if storage status is available
     */
    bool hasStorageStatus() const {
        if (!_handle) return false;
        return slamtec_aurora_sdk_dashcam_storage_info_has_storage_status(_handle) > 0;
    }

    /**
     * @brief Get storage status
     * @param[out] status_out Output storage status structure
     * @return True on success, false if not available or on error
     */
    bool getStorageStatus(RemoteDashcamStorageStatus& status_out) const {
        if (!_handle) return false;
        return slamtec_aurora_sdk_dashcam_storage_info_get_storage_status(_handle, &status_out) == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

    /**
     * @brief Get existing session count
     * @return Number of existing sessions, or 0 on error
     */
    int getSessionCount() const {
        if (!_handle) return 0;
        int count = slamtec_aurora_sdk_dashcam_storage_info_get_session_count(_handle);
        return count > 0 ? count : 0;
    }

    /**
     * @brief Get existing session info by index
     * @param[in] index Session index (0-based)
     * @param[out] session_out Output session info structure
     * @return True on success, false on error
     */
    bool getSession(int index, RemoteDashcamSessionInfo& session_out) const {
        if (!_handle) return false;
        return slamtec_aurora_sdk_dashcam_storage_info_get_session(_handle, index, &session_out) == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

    /**
     * @brief Check if current (active) session exists
     * @return True if current session exists
     */
    bool hasCurrentSession() const {
        if (!_handle) return false;
        return slamtec_aurora_sdk_dashcam_storage_info_has_current_session(_handle) > 0;
    }

    /**
     * @brief Get current (active) session info
     * @param[out] session_out Output session info structure
     * @return True on success, false if not available or on error
     */
    bool getCurrentSession(RemoteDashcamSessionInfo& session_out) const {
        if (!_handle) return false;
        return slamtec_aurora_sdk_dashcam_storage_info_get_current_session(_handle, &session_out) == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

private:
    slamtec_aurora_sdk_dashcam_storage_info_t _handle;
};

/**
 * @brief Dashcam Recorder Manager
 * @details C++ wrapper class for managing dashcam recording via the Aurora Remote SDK.
 *          This class provides a convenient interface to control dashcam recording, query status, and manage sessions.
 * @ingroup Cxx_DashcamRecorder_Operations Dashcam Recorder Operations
 */
class DashcamRecorderManager : public Noncopyable {
    friend class RemoteSDK;
public:

    /**
     * @brief Get current dashcam recorder status
     * @param[out] status_out Output status structure
     * @param[in] timeout_ms Timeout in milliseconds (default: 5000ms)
     * @param[out] errcode Optional pointer to receive error code
     * @return True on success, false on failure
     */
    bool getStatus(
        RemoteDashcamStatus& status_out,
        uint64_t timeout_ms = SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_TIMEOUT,
        slamtec_aurora_sdk_errorcode_t* errcode = nullptr) const
    {
        auto err = slamtec_aurora_sdk_dashcam_recorder_get_status(_sdk, &status_out, timeout_ms);
        if (errcode) *errcode = err;
        return err == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

    /**
     * @brief Get complete storage information including sessions
     * @param[in] timeout_ms Timeout in milliseconds (default: 5000ms)
     * @return RemoteDashcamStorageInfo wrapper object (check isValid() for success)
     */
    RemoteDashcamStorageInfo getStorageInfo(
        uint64_t timeout_ms = SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_TIMEOUT) const
    {
        auto handle = slamtec_aurora_sdk_dashcam_recorder_get_storage_info(_sdk, timeout_ms);
        return RemoteDashcamStorageInfo(handle);
    }

    /**
     * @brief Enable or disable dashcam recording
     * @param[in] enable True to enable, false to disable
     * @param[in] timeout_ms Timeout in milliseconds (default: 5000ms)
     * @param[out] errcode Optional pointer to receive error code
     * @return True on success, false on failure
     */
    bool setEnable(
        bool enable,
        uint64_t timeout_ms = SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_TIMEOUT,
        slamtec_aurora_sdk_errorcode_t* errcode = nullptr) const
    {
        auto err = slamtec_aurora_sdk_dashcam_recorder_set_enable(_sdk, enable ? 1 : 0, timeout_ms);
        if (errcode) *errcode = err;
        return err == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

    /**
     * @brief Set storage size limit for dashcam recordings
     * @param[in] sizeLimitGB Maximum storage size in gigabytes
     * @param[in] timeout_ms Timeout in milliseconds (default: 5000ms)
     * @param[out] errcode Optional pointer to receive error code
     * @return True on success, false on failure
     */
    bool setSizeLimit(
        float sizeLimitGB,
        uint64_t timeout_ms = SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_TIMEOUT,
        slamtec_aurora_sdk_errorcode_t* errcode = nullptr) const
    {
        auto err = slamtec_aurora_sdk_dashcam_recorder_set_size_limit(_sdk, sizeLimitGB, timeout_ms);
        if (errcode) *errcode = err;
        return err == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

    /**
     * @brief Invalidate (delete) all recording sessions
     * @param[in] timeout_ms Timeout in milliseconds (default: 10000ms)
     * @param[out] errcode Optional pointer to receive error code
     * @return True on success, false on failure
     */
    bool invalidateSessions(
        uint64_t timeout_ms = 10000,
        slamtec_aurora_sdk_errorcode_t* errcode = nullptr) const
    {
        auto err = slamtec_aurora_sdk_dashcam_recorder_invalidate_sessions(_sdk, timeout_ms);
        if (errcode) *errcode = err;
        return err == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

protected:
    DashcamRecorderManager(slamtec_aurora_sdk_session_handle_t& sdk)
        : _sdk(sdk)
    {}

    slamtec_aurora_sdk_session_handle_t _sdk;
};

}}} // namespace rp::standalone::aurora
