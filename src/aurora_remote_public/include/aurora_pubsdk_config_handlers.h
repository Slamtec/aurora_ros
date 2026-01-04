/*
 *  SLAMTEC Aurora
 *  Copyright 2013 - 2024 SLAMTEC Co., Ltd.
 *
 *  http://www.slamtec.com
 *
 *  Aurora Remote SDK
 *  Configuration Handlers - Config Manager, Transform Manager, Camera Mask Manager
 *  Pure C Interface
 */

#pragma once

// NOTE: This header is included from aurora_pubsdk_inc.h
// It expects aurora_pubsdk_common_def.h and aurora_pubsdk_objects.h to be already included

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup Config_Operations Config Operations
 * @brief Functions for managing configuration settings
 */

/**
 * @defgroup TransformManager_Operations Transform Manager Operations
 * @brief Functions for managing configurable transforms
 */

/**
 * @defgroup CameraMask_Operations Camera Mask Operations
 * @brief Functions for managing camera input masks
 */

// Config Operations
////////////////////////////////////////////////////////////////////////////////////////

// Entry List Object Operations

/**
 * @brief Destroy a config entry list object
 * @ingroup Config_Operations Config Operations
 *
 * @param list - the entry list handle to destroy
 */
void AURORA_SDK_API slamtec_aurora_sdk_config_entry_list_destroy(slamtec_aurora_sdk_config_entry_list_t list);

/**
 * @brief Get the count of entries in the list
 * @ingroup Config_Operations Config Operations
 *
 * @param list - the entry list handle
 * @return the number of entries, <0 means error
 */
int AURORA_SDK_API slamtec_aurora_sdk_config_entry_list_get_count(slamtec_aurora_sdk_config_entry_list_t list);

/**
 * @brief Get an entry from the list by index
 * @ingroup Config_Operations Config Operations
 *
 * @param list - the entry list handle
 * @param index - the index of the entry (0-based)
 * @return the filter path string, NULL if index out of range
 */
AURORA_SDK_API const char* slamtec_aurora_sdk_config_entry_list_get_entry(slamtec_aurora_sdk_config_entry_list_t list, int index);

// Config Data Object Operations

/**
 * @brief Create an empty config data object
 * @ingroup Config_Operations Config Operations
 *
 * @return the config data handle, NULL if failed
 */
slamtec_aurora_sdk_config_data_t AURORA_SDK_API slamtec_aurora_sdk_config_data_create();

/**
 * @brief Destroy a config data object
 * @ingroup Config_Operations Config Operations
 *
 * @param data - the config data handle to destroy
 */
void AURORA_SDK_API slamtec_aurora_sdk_config_data_destroy(slamtec_aurora_sdk_config_data_t data);

/**
 * @brief Load config data from JSON string
 * @ingroup Config_Operations Config Operations
 *
 * @param data - the config data handle
 * @param json_string - the JSON string to load
 * @return the error code
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API slamtec_aurora_sdk_config_data_load_from_string(slamtec_aurora_sdk_config_data_t data, const char* json_string);

/**
 * @brief Create a string dump of config data
 * @ingroup Config_Operations Config Operations
 *
 * @param data - the config data handle
 * @param size_out - optional pointer to receive the string length (excluding null terminator)
 * @return pointer to the dumped JSON string (null-terminated), NULL if failed. The returned string is valid until slamtec_aurora_sdk_config_data_destroy_string_dump or slamtec_aurora_sdk_config_data_destroy is called.
 */
AURORA_SDK_API const char* slamtec_aurora_sdk_config_data_create_string_dump(slamtec_aurora_sdk_config_data_t data, size_t* size_out);

/**
 * @brief Destroy the cached string dump
 * @ingroup Config_Operations Config Operations
 *
 * @param data - the config data handle
 */
void AURORA_SDK_API slamtec_aurora_sdk_config_data_destroy_string_dump(slamtec_aurora_sdk_config_data_t data);

/**
 * @brief Load config data from a file
 * @ingroup Config_Operations Config Operations
 *
 * @param data - the config data handle
 * @param file_path - path to the file
 * @return the error code
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API slamtec_aurora_sdk_config_data_load_from_file(slamtec_aurora_sdk_config_data_t data, const char* file_path);

/**
 * @brief Save config data to a file
 * @ingroup Config_Operations Config Operations
 *
 * @param data - the config data handle
 * @param file_path - path to the file
 * @return the error code
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API slamtec_aurora_sdk_config_data_save_to_file(slamtec_aurora_sdk_config_data_t data, const char* file_path);

// Persistent Config Management Operations

/**
 * @brief Enumerate all persistent config entries (filter paths) available
 * @ingroup Config_Operations Config Operations
 *
 * @param handle - the session handle
 * @param timeout_ms - timeout in milliseconds
 * @return the entry list handle, NULL if failed (caller should destroy using slamtec_aurora_sdk_config_entry_list_destroy)
 */
slamtec_aurora_sdk_config_entry_list_t AURORA_SDK_API slamtec_aurora_sdk_persistent_config_enum_all_entries(slamtec_aurora_sdk_session_handle_t handle, uint64_t timeout_ms);

/**
 * @brief Reset a specific persistent config entry to default
 * @ingroup Config_Operations Config Operations
 *
 * @param handle - the session handle
 * @param filter_path - the filter path of the config entry to reset
 * @param timeout_ms - timeout in milliseconds
 * @return the error code
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API slamtec_aurora_sdk_persistent_config_reset_config(slamtec_aurora_sdk_session_handle_t handle, const char* filter_path, uint64_t timeout_ms);

/**
 * @brief Reset all persistent config entries to default
 * @ingroup Config_Operations Config Operations
 *
 * @param handle - the session handle
 * @param timeout_ms - timeout in milliseconds
 * @return the error code
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API slamtec_aurora_sdk_persistent_config_reset_all_config(slamtec_aurora_sdk_session_handle_t handle, uint64_t timeout_ms);

/**
 * @brief Set persistent config entry with config data object
 * @ingroup Config_Operations Config Operations
 *
 * @param handle - the session handle
 * @param filter_path - the filter path of the config entry
 * @param key - the key for merging config, use "@overwrite" to overwrite the entire config
 * @param config_data - the config data handle
 * @param timeout_ms - timeout in milliseconds
 * @return the error code
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API slamtec_aurora_sdk_persistent_config_set_config(slamtec_aurora_sdk_session_handle_t handle, const char* filter_path, const char* key, slamtec_aurora_sdk_config_data_t config_data, uint64_t timeout_ms);

/**
 * @brief Get persistent config entry and populate config data object
 * @ingroup Config_Operations Config Operations
 *
 * @param handle - the session handle
 * @param filter_path - the filter path of the config entry
 * @param config_data - the config data handle to populate (must be created by caller)
 * @param timeout_ms - timeout in milliseconds
 * @return the error code
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API slamtec_aurora_sdk_persistent_config_get_config(slamtec_aurora_sdk_session_handle_t handle, const char* filter_path, slamtec_aurora_sdk_config_data_t config_data, uint64_t timeout_ms);


// Transform Manager Operations
////////////////////////////////////////////////////////////////////////////////////////
/**
 * @ingroup TransformManager_Operations Transform Manager Operations
 * @brief Create a transform manager instance
 * @param[in] session_handle The session handle
 * @return The transform manager instance handle, NULL if failed
 */
slamtec_aurora_sdk_transform_manager_t AURORA_SDK_API slamtec_aurora_sdk_transform_manager_create(slamtec_aurora_sdk_session_handle_t session_handle);

/**
 * @ingroup TransformManager_Operations Transform Manager Operations
 * @brief Destroy a transform manager instance
 * @param[in] handle The transform manager instance handle to destroy
 */
void AURORA_SDK_API slamtec_aurora_sdk_transform_manager_destroy(slamtec_aurora_sdk_transform_manager_t handle);

/**
 * @ingroup TransformManager_Operations Transform Manager Operations
 * @brief Destroy a transform name list
 * @param[in] list The transform name list handle to destroy
 */
void AURORA_SDK_API slamtec_aurora_sdk_transform_manager_name_list_destroy(slamtec_aurora_sdk_transform_name_list_t list);

/**
 * @ingroup TransformManager_Operations Transform Manager Operations
 * @brief Get the count of transform names in the list
 * @param[in] list The transform name list handle
 * @return The count of transform names, -1 if list is invalid
 */
int AURORA_SDK_API slamtec_aurora_sdk_transform_manager_name_list_get_count(slamtec_aurora_sdk_transform_name_list_t list);

/**
 * @ingroup TransformManager_Operations Transform Manager Operations
 * @brief Get a transform name from the list by index
 * @param[in] list The transform name list handle
 * @param[in] index The index of the transform name (0-based)
 * @return The transform name string, NULL if index is out of range or list is invalid
 */
AURORA_SDK_API const char* slamtec_aurora_sdk_transform_manager_name_list_get_entry(slamtec_aurora_sdk_transform_name_list_t list, int index);

/**
 * @ingroup TransformManager_Operations Transform Manager Operations
 * @brief Get all transform names
 * @param[in] handle The transform manager instance handle
 * @param[in] timeout_ms The timeout in milliseconds
 * @return The transform name list handle, NULL if failed
 */
slamtec_aurora_sdk_transform_name_list_t AURORA_SDK_API slamtec_aurora_sdk_transform_manager_get_all_transform_names(slamtec_aurora_sdk_transform_manager_t handle, uint64_t timeout_ms);

/**
 * @ingroup TransformManager_Operations Transform Manager Operations
 * @brief Get a transform by name
 * @param[in] handle The transform manager instance handle
 * @param[in] name The transform name
 * @param[out] transform The output transform (SE3 pose)
 * @param[in] timeout_ms The timeout in milliseconds
 * @return SLAMTEC_AURORA_SDK_ERRORCODE_OK if successful, error code otherwise
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API slamtec_aurora_sdk_transform_manager_get_transform(slamtec_aurora_sdk_transform_manager_t handle, const char* name, slamtec_aurora_sdk_pose_se3_t* transform, uint64_t timeout_ms);

/**
 * @ingroup TransformManager_Operations Transform Manager Operations
 * @brief Set a transform by name
 * @param[in] handle The transform manager instance handle
 * @param[in] name The transform name
 * @param[in] transform The transform to set (SE3 pose)
 * @param[in] timeout_ms The timeout in milliseconds
 * @return SLAMTEC_AURORA_SDK_ERRORCODE_OK if successful, error code otherwise
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API slamtec_aurora_sdk_transform_manager_set_transform(slamtec_aurora_sdk_transform_manager_t handle, const char* name, const slamtec_aurora_sdk_pose_se3_t* transform, uint64_t timeout_ms);

/**
 * @ingroup TransformManager_Operations Transform Manager Operations
 * @brief Reset a transform to identity by name
 * @param[in] handle The transform manager instance handle
 * @param[in] name The transform name
 * @param[in] timeout_ms The timeout in milliseconds
 * @return SLAMTEC_AURORA_SDK_ERRORCODE_OK if successful, error code otherwise
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API slamtec_aurora_sdk_transform_manager_reset_transform(slamtec_aurora_sdk_transform_manager_t handle, const char* name, uint64_t timeout_ms);

/**
 * @ingroup TransformManager_Operations Transform Manager Operations
 * @brief Check if a transform exists
 * @param[in] handle The transform manager instance handle
 * @param[in] name The transform name
 * @param[in] timeout_ms The timeout in milliseconds
 * @return 1 if transform exists, 0 if not, -1 if error
 */
int AURORA_SDK_API slamtec_aurora_sdk_transform_manager_has_transform(slamtec_aurora_sdk_transform_manager_t handle, const char* name, uint64_t timeout_ms);

/**
 * @ingroup TransformManager_Operations Transform Manager Operations
 * @brief Refresh the transform manager configuration from device
 * @details This function forces a reload of configuration from the device, bypassing the cache
 * @param[in] handle The transform manager instance handle
 * @param[in] timeout_ms The timeout in milliseconds
 * @return SLAMTEC_AURORA_SDK_ERRORCODE_OK if successful, error code otherwise
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API slamtec_aurora_sdk_transform_manager_refresh(slamtec_aurora_sdk_transform_manager_t handle, uint64_t timeout_ms);


// Camera Mask Operations
////////////////////////////////////////////////////////////////////////////////////////
/**
 * @ingroup CameraMask_Operations Camera Mask Operations
 * @brief Create a camera mask instance
 * @param[in] session_handle The session handle
 * @return The camera mask instance handle, NULL if failed
 */
slamtec_aurora_sdk_camera_mask_t AURORA_SDK_API slamtec_aurora_sdk_camera_mask_create(slamtec_aurora_sdk_session_handle_t session_handle);

/**
 * @ingroup CameraMask_Operations Camera Mask Operations
 * @brief Destroy a camera mask instance
 * @param[in] handle The camera mask instance handle to destroy
 */
void AURORA_SDK_API slamtec_aurora_sdk_camera_mask_destroy(slamtec_aurora_sdk_camera_mask_t handle);

/**
 * @ingroup CameraMask_Operations Camera Mask Operations
 * @brief Check if static mask is enabled
 * @param[in] handle The camera mask instance handle
 * @param[in] timeout_ms The timeout in milliseconds
 * @return 1 if enabled, 0 if disabled, -1 if error
 */
int AURORA_SDK_API slamtec_aurora_sdk_camera_mask_is_static_mask_enabled(slamtec_aurora_sdk_camera_mask_t handle, uint64_t timeout_ms);

/**
 * @ingroup CameraMask_Operations Camera Mask Operations
 * @brief Enable or disable static mask
 * @param[in] handle The camera mask instance handle
 * @param[in] enable 1 to enable, 0 to disable
 * @param[in] timeout_ms The timeout in milliseconds
 * @return SLAMTEC_AURORA_SDK_ERRORCODE_OK if successful, error code otherwise
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API slamtec_aurora_sdk_camera_mask_set_static_mask_enable(slamtec_aurora_sdk_camera_mask_t handle, int enable, uint64_t timeout_ms);

/**
 * @ingroup CameraMask_Operations Camera Mask Operations
 * @brief Get the count of camera indices that have static masks configured
 * @param[in] handle The camera mask instance handle
 * @param[in] timeout_ms The timeout in milliseconds
 * @return The count of camera indices, -1 if error
 */
int AURORA_SDK_API slamtec_aurora_sdk_camera_mask_get_static_camera_mask_image_id_count(slamtec_aurora_sdk_camera_mask_t handle, uint64_t timeout_ms);

/**
 * @ingroup CameraMask_Operations Camera Mask Operations
 * @brief Get camera indices that have static masks configured
 * @param[in] handle The camera mask instance handle
 * @param[out] index_buffer Buffer to receive camera indices
 * @param[in] buffer_count Maximum number of indices the buffer can hold
 * @param[in] timeout_ms The timeout in milliseconds
 * @return The actual number of indices written to buffer, -1 if error
 */
int AURORA_SDK_API slamtec_aurora_sdk_camera_mask_get_static_camera_mask_image_indices(slamtec_aurora_sdk_camera_mask_t handle, int* index_buffer, int buffer_count, uint64_t timeout_ms);

/**
 * @ingroup CameraMask_Operations Camera Mask Operations
 * @brief Get static camera mask image by camera index
 * @param[in] handle The camera mask instance handle
 * @param[in] camera_index The camera index (0 for left, 1 for right, etc.)
 * @param[out] image_desc Image description to be populated
 * @param[in,out] image_buffer User-provided buffer to receive mask image data
 * @param[in] timeout_ms The timeout in milliseconds
 * @return SLAMTEC_AURORA_SDK_ERRORCODE_OK if successful, SLAMTEC_AURORA_SDK_ERRORCODE_INSUFFICIENT_BUFFER if buffer too small (image_buffer->image_data_size will be set to required size)
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API slamtec_aurora_sdk_camera_mask_get_static_camera_mask_image(slamtec_aurora_sdk_camera_mask_t handle, int camera_index, slamtec_aurora_sdk_image_desc_t* image_desc, slamtec_aurora_sdk_camera_mask_image_buffer_t* image_buffer, uint64_t timeout_ms);

/**
 * @ingroup CameraMask_Operations Camera Mask Operations
 * @brief Set static camera mask image for a camera index
 * @param[in] handle The camera mask instance handle
 * @param[in] camera_index The camera index (0 for left, 1 for right, etc.)
 * @param[in] image_desc Image description of the mask
 * @param[in] image_data Pointer to the mask image data (grayscale, 8-bit per pixel)
 * @param[in] timeout_ms The timeout in milliseconds
 * @return SLAMTEC_AURORA_SDK_ERRORCODE_OK if successful, error code otherwise
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API slamtec_aurora_sdk_camera_mask_set_static_camera_mask_image(slamtec_aurora_sdk_camera_mask_t handle, int camera_index, const slamtec_aurora_sdk_image_desc_t* image_desc, const void* image_data, uint64_t timeout_ms);

/**
 * @ingroup CameraMask_Operations Camera Mask Operations
 * @brief Remove static camera mask for a specific camera index
 * @param[in] handle The camera mask instance handle
 * @param[in] camera_index The camera index to remove mask for
 * @param[in] timeout_ms The timeout in milliseconds
 * @return SLAMTEC_AURORA_SDK_ERRORCODE_OK if successful, error code otherwise
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API slamtec_aurora_sdk_camera_mask_remove_static_camera_mask_image(slamtec_aurora_sdk_camera_mask_t handle, int camera_index, uint64_t timeout_ms);

/**
 * @ingroup CameraMask_Operations Camera Mask Operations
 * @brief Refresh the camera mask configuration from device
 * @details This function forces a reload of configuration from the device, bypassing the cache
 * @param[in] handle The camera mask instance handle
 * @param[in] timeout_ms The timeout in milliseconds
 * @return SLAMTEC_AURORA_SDK_ERRORCODE_OK if successful, error code otherwise
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API slamtec_aurora_sdk_camera_mask_refresh(slamtec_aurora_sdk_camera_mask_t handle, uint64_t timeout_ms);


#ifdef __cplusplus
}
#endif
