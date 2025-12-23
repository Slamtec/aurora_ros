/*
 *  SLAMTEC Aurora Remote SDK
 *  Dashcam Recorder Public API (Pure C)
 *
 *  Copyright 2013 - 2025 SLAMTEC Co., Ltd.
 *  http://www.slamtec.com
 */

#pragma once

// NOTE: This header is included from aurora_pubsdk_inc.h
// It expects aurora_pubsdk_common_def.h and aurora_pubsdk_objects.h to be already included

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup DashcamRecorder_Operations Dashcam Recorder Operations
 * @brief Functions for managing dashcam recording
 * @{
 */

// Status Operations
////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Get current dashcam recorder status
 * @param[in] session_handle The session handle
 * @param[out] status_out Output status structure
 * @param[in] timeout_ms Timeout in milliseconds
 * @return Error code
 * @ingroup DashcamRecorder_Operations
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API slamtec_aurora_sdk_dashcam_recorder_get_status(
    slamtec_aurora_sdk_session_handle_t session_handle,
    slamtec_aurora_sdk_dashcam_status_t* status_out,
    uint64_t timeout_ms);

// Storage Info Operations
////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Get complete dashcam storage information in a single call
 * @param[in] session_handle The session handle
 * @param[in] timeout_ms Timeout in milliseconds
 * @return Storage info handle on success, NULL on failure. Must be destroyed with
 *         slamtec_aurora_sdk_dashcam_storage_info_destroy() when done.
 * @note This retrieves all storage info (path, status, sessions) in one request.
 * @ingroup DashcamRecorder_Operations
 */
slamtec_aurora_sdk_dashcam_storage_info_t AURORA_SDK_API slamtec_aurora_sdk_dashcam_recorder_get_storage_info(
    slamtec_aurora_sdk_session_handle_t session_handle,
    uint64_t timeout_ms);

/**
 * @brief Destroy a dashcam storage info handle
 * @param[in] storage_info The storage info handle to destroy
 * @ingroup DashcamRecorder_Operations
 */
void AURORA_SDK_API slamtec_aurora_sdk_dashcam_storage_info_destroy(
    slamtec_aurora_sdk_dashcam_storage_info_t storage_info);

/**
 * @brief Get storage path from storage info handle
 * @param[in] storage_info The storage info handle
 * @return Storage path string, or NULL on error. Valid until handle is destroyed.
 * @ingroup DashcamRecorder_Operations
 */
AURORA_SDK_API const char* slamtec_aurora_sdk_dashcam_storage_info_get_path(
    slamtec_aurora_sdk_dashcam_storage_info_t storage_info);

/**
 * @brief Check if storage status is available from storage info handle
 * @param[in] storage_info The storage info handle
 * @return 1 if storage status is available, 0 if not, -1 on error
 * @ingroup DashcamRecorder_Operations
 */
int AURORA_SDK_API slamtec_aurora_sdk_dashcam_storage_info_has_storage_status(
    slamtec_aurora_sdk_dashcam_storage_info_t storage_info);

/**
 * @brief Get storage status from storage info handle
 * @param[in] storage_info The storage info handle
 * @param[out] storage_status_out Output storage status structure
 * @return Error code (returns SLAMTEC_AURORA_SDK_ERRORCODE_NOT_SUPPORTED if storage status not available)
 * @ingroup DashcamRecorder_Operations
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API slamtec_aurora_sdk_dashcam_storage_info_get_storage_status(
    slamtec_aurora_sdk_dashcam_storage_info_t storage_info,
    slamtec_aurora_sdk_dashcam_storage_status_t* storage_status_out);

/**
 * @brief Get existing session count from storage info handle
 * @param[in] storage_info The storage info handle
 * @return Number of existing sessions, or -1 on error
 * @ingroup DashcamRecorder_Operations
 */
int AURORA_SDK_API slamtec_aurora_sdk_dashcam_storage_info_get_session_count(
    slamtec_aurora_sdk_dashcam_storage_info_t storage_info);

/**
 * @brief Get existing session info by index from storage info handle
 * @param[in] storage_info The storage info handle
 * @param[in] session_index Index of the session (0-based)
 * @param[out] session_info_out Output session info structure
 * @return Error code
 * @ingroup DashcamRecorder_Operations
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API slamtec_aurora_sdk_dashcam_storage_info_get_session(
    slamtec_aurora_sdk_dashcam_storage_info_t storage_info,
    int session_index,
    slamtec_aurora_sdk_dashcam_session_info_t* session_info_out);

/**
 * @brief Check if current (active) session exists from storage info handle
 * @param[in] storage_info The storage info handle
 * @return 1 if current session exists, 0 if not, -1 on error
 * @ingroup DashcamRecorder_Operations
 */
int AURORA_SDK_API slamtec_aurora_sdk_dashcam_storage_info_has_current_session(
    slamtec_aurora_sdk_dashcam_storage_info_t storage_info);

/**
 * @brief Get current (active) session info from storage info handle
 * @param[in] storage_info The storage info handle
 * @param[out] session_info_out Output session info structure
 * @return Error code (returns SLAMTEC_AURORA_SDK_ERRORCODE_NOT_SUPPORTED if no current session)
 * @ingroup DashcamRecorder_Operations
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API slamtec_aurora_sdk_dashcam_storage_info_get_current_session(
    slamtec_aurora_sdk_dashcam_storage_info_t storage_info,
    slamtec_aurora_sdk_dashcam_session_info_t* session_info_out);

// Configuration Operations
////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Enable or disable dashcam recording
 * @param[in] session_handle The session handle
 * @param[in] enable 1 to enable, 0 to disable
 * @param[in] timeout_ms Timeout in milliseconds
 * @return Error code
 * @ingroup DashcamRecorder_Operations
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API slamtec_aurora_sdk_dashcam_recorder_set_enable(
    slamtec_aurora_sdk_session_handle_t session_handle,
    int enable,
    uint64_t timeout_ms);

/**
 * @brief Set the storage size limit for dashcam recordings
 * @param[in] session_handle The session handle
 * @param[in] size_limit_gb Size limit in gigabytes
 * @param[in] timeout_ms Timeout in milliseconds
 * @return Error code
 * @ingroup DashcamRecorder_Operations
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API slamtec_aurora_sdk_dashcam_recorder_set_size_limit(
    slamtec_aurora_sdk_session_handle_t session_handle,
    float size_limit_gb,
    uint64_t timeout_ms);

/**
 * @brief Invalidate (delete) all recording sessions
 * @param[in] session_handle The session handle
 * @param[in] timeout_ms Timeout in milliseconds
 * @return Error code
 * @ingroup DashcamRecorder_Operations
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API slamtec_aurora_sdk_dashcam_recorder_invalidate_sessions(
    slamtec_aurora_sdk_session_handle_t session_handle,
    uint64_t timeout_ms);

/** @} */ // end of DashcamRecorder_Operations

#ifdef __cplusplus
}
#endif
