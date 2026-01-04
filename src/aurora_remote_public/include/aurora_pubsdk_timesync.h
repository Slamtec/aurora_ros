/*
 * Copyright 2013-2025 SLAMTEC Co., Ltd.
 */

#pragma once

#include "aurora_pubsdk_common_def.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Default port for time synchronization server
 * @ingroup Utility_Functions Utility Functions
 */
#define SLAMTEC_AURORA_SDK_TIMESYNC_DEFAULT_PORT 9527

/**
 * @brief Handle to a time synchronization client instance
 * @ingroup Utility_Functions Utility Functions
 */
typedef void* slamtec_aurora_sdk_timesync_handle_t;

/**
 * @brief Time domain types for synchronization
 * @ingroup Utility_Functions Utility Functions
 */
typedef enum {
    SLAMTEC_AURORA_SDK_TIMESYNC_DOMAIN_STEADY_CLOCK = 0,  /**< Monotonic steady clock */
    SLAMTEC_AURORA_SDK_TIMESYNC_DOMAIN_WALL_CLOCK = 1     /**< System wall clock */
} slamtec_aurora_sdk_timesync_domain_t;

/**
 * @brief Time synchronization client options
 * @ingroup Utility_Functions Utility Functions
 */
typedef struct {
    uint32_t synchronization_interval_ms;  /**< Interval between sync requests (milliseconds) */
    uint32_t sample_window_size;           /**< Number of samples in sliding window */
    double outlier_threshold;              /**< Outlier detection threshold (MAD multiplier) */
    uint32_t min_samples_for_sync;         /**< Minimum samples required before synchronized */
    uint32_t timeout_ms;                   /**< Response timeout (milliseconds) */
    double max_rtt_ms;                     /**< Maximum acceptable RTT (milliseconds) */
    uint32_t initialize_timeout_ms;        /**< Timeout for initialize() operation (milliseconds) */
} slamtec_aurora_sdk_timesync_options_t;

/**
 * @brief Time synchronization quality metrics
 * @ingroup Utility_Functions Utility Functions
 */
typedef struct {
    double rmse_ms;                /**< Root mean square error (milliseconds) */
    double max_error_ms;           /**< Maximum absolute error (milliseconds) */
    double scale;                  /**< Estimated time scale factor */
    double offset_ns;              /**< Estimated time offset (nanoseconds) */
    double scale_std_dev;          /**< Standard deviation of scale estimate */
    double offset_std_dev_ns;      /**< Standard deviation of offset estimate (nanoseconds) */
    uint32_t sample_count;         /**< Number of samples used */
    uint32_t total_sync_count;     /**< Total synchronization attempts */
    uint32_t failed_sync_count;    /**< Failed synchronization attempts */
} slamtec_aurora_sdk_timesync_quality_t;

/**
 * @brief Create a time synchronization client instance
 * @ingroup Utility_Functions Utility Functions
 *
 * @param time_domain The time domain type for this client
 * @param error_code If provided, the error code will be stored in this pointer
 * @return Time sync client handle, or NULL on failure
 */
slamtec_aurora_sdk_timesync_handle_t AURORA_SDK_API
slamtec_aurora_sdk_timesync_create_instance(
    slamtec_aurora_sdk_timesync_domain_t time_domain,
    slamtec_aurora_sdk_errorcode_t* error_code
);

/**
 * @brief Destroy a time synchronization client instance
 * @ingroup Utility_Functions Utility Functions
 *
 * @param handle The time sync client handle to destroy
 */
void AURORA_SDK_API
slamtec_aurora_sdk_timesync_destroy_instance(
    slamtec_aurora_sdk_timesync_handle_t handle
);

/**
 * @brief Connect to a time synchronization server
 * @ingroup Utility_Functions Utility Functions
 *
 * @param handle The time sync client handle
 * @param server_address Server IP address or hostname
 * @param server_port Server port number
 * @return Error code
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API
slamtec_aurora_sdk_timesync_connect(
    slamtec_aurora_sdk_timesync_handle_t handle,
    const char* server_address,
    uint16_t server_port
);

/**
 * @brief Get default time synchronization options
 * @ingroup Utility_Functions Utility Functions
 *
 * @details This function fills the provided structure with default values
 *          that provide good performance for most use cases.
 *
 * @param options_out Pointer to options structure to fill with defaults
 */
void AURORA_SDK_API
slamtec_aurora_sdk_timesync_get_default_options(
    slamtec_aurora_sdk_timesync_options_t* options_out
);

/**
 * @brief Set time synchronization options
 * @ingroup Utility_Functions Utility Functions
 *
 * @param handle The time sync client handle
 * @param options The options to configure
 * @return Error code
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API
slamtec_aurora_sdk_timesync_set_options(
    slamtec_aurora_sdk_timesync_handle_t handle,
    const slamtec_aurora_sdk_timesync_options_t* options
);

/**
 * @brief Initialize time synchronization with first sync request
 * @ingroup Utility_Functions Utility Functions
 *
 * @details The timeout for initialization is configured via setOptions().
 *          If not set, the default timeout will be used.
 *
 * @param handle The time sync client handle
 * @return Error code
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API
slamtec_aurora_sdk_timesync_initialize(
    slamtec_aurora_sdk_timesync_handle_t handle
);

/**
 * @brief Check if time synchronization is established
 * @ingroup Utility_Functions Utility Functions
 *
 * @param handle The time sync client handle
 * @return Non-zero if synchronized, zero otherwise
 */
int AURORA_SDK_API
slamtec_aurora_sdk_timesync_is_synchronized(
    slamtec_aurora_sdk_timesync_handle_t handle
);

/**
 * @brief Translate Aurora timestamp to client time domain
 * @ingroup Utility_Functions Utility Functions
 *
 * @param handle The time sync client handle
 * @param aurora_timestamp_ns Aurora timestamp in nanoseconds
 * @param client_timestamp_ns_out Translated timestamp in client time domain (output)
 * @return Error code (returns NOT_READY if not synchronized)
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API
slamtec_aurora_sdk_timesync_translate_timestamp(
    slamtec_aurora_sdk_timesync_handle_t handle,
    uint64_t aurora_timestamp_ns,
    uint64_t* client_timestamp_ns_out
);

/**
 * @brief Get current synchronization quality metrics
 * @ingroup Utility_Functions Utility Functions
 *
 * @param handle The time sync client handle
 * @param quality_out Quality metrics structure (output)
 * @return Error code
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API
slamtec_aurora_sdk_timesync_get_quality(
    slamtec_aurora_sdk_timesync_handle_t handle,
    slamtec_aurora_sdk_timesync_quality_t* quality_out
);

/**
 * @brief Get current server timestamp translated to client time domain
 * @ingroup Utility_Functions Utility Functions
 *
 * @details This function queries the server for its current timestamp and translates
 *          it to the client's time domain. Useful for validating synchronization.
 *
 * @param handle The time sync client handle
 * @param timeout_ms Timeout for the query (milliseconds)
 * @param server_timestamp_ns_out Translated server timestamp (output)
 * @param client_timestamp_ns_out Current client timestamp at query time (output, can be NULL)
 * @return Error code (returns NOT_READY if not synchronized)
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API
slamtec_aurora_sdk_timesync_get_current_server_timestamp(
    slamtec_aurora_sdk_timesync_handle_t handle,
    uint32_t timeout_ms,
    uint64_t* server_timestamp_ns_out,
    uint64_t* client_timestamp_ns_out
);

/**
 * @brief Stop time synchronization background thread
 * @ingroup Utility_Functions Utility Functions
 *
 * @param handle The time sync client handle
 */
void AURORA_SDK_API
slamtec_aurora_sdk_timesync_stop(
    slamtec_aurora_sdk_timesync_handle_t handle
);

/**
 * @brief Check if time synchronization is running
 * @ingroup Utility_Functions Utility Functions
 *
 * @param handle The time sync client handle
 * @return Non-zero if running, zero otherwise
 */
int AURORA_SDK_API
slamtec_aurora_sdk_timesync_is_running(
    slamtec_aurora_sdk_timesync_handle_t handle
);

/**
 * @brief Wall clock offset result structure
 * @ingroup Utility_Functions Utility Functions
 */
typedef struct {
    int success;               /**< Non-zero if successful, zero otherwise */
    int64_t offset_ns;         /**< Offset between server and client UTC clocks (ns) */
    double rtt_ns;             /**< Round-trip time (nanoseconds) */
    int64_t server_offset_ns;  /**< Server's current wall clock offset adjustment (ns) */
} slamtec_aurora_sdk_wallclock_offset_result_t;

/**
 * @brief Wall clock sync result structure
 * @ingroup Utility_Functions Utility Functions
 */
typedef struct {
    int success;               /**< Non-zero if successful, zero otherwise */
    int64_t applied_offset_ns; /**< Offset that was applied (nanoseconds) */
    uint64_t server_utc_ns;    /**< Server's UTC wall clock after sync (ns since epoch) */
} slamtec_aurora_sdk_wallclock_sync_result_t;

/**
 * @brief Wall clock sync accuracy result structure
 * @ingroup Utility_Functions Utility Functions
 */
typedef struct {
    int success;               /**< Non-zero if successful, zero otherwise */
    int64_t offset_error_ns;   /**< Difference between server and client clocks (ns) */
    double rtt_ns;             /**< Round-trip time estimate (nanoseconds) */
    uint64_t server_utc_ns;    /**< Server's current UTC wall clock (ns since epoch) */
} slamtec_aurora_sdk_wallclock_accuracy_result_t;

/**
 * @brief Get wall clock offset between server and client
 * @ingroup Utility_Functions Utility Functions
 *
 * @details This function queries the offset between the server's UTC wall clock
 *          and the client's UTC wall clock. This is used to evaluate whether the
 *          Aurora device's wall clock needs to be synchronized.
 *
 * @param handle The time sync client handle
 * @param timeout_ms Timeout for the query (milliseconds)
 * @param result_out Pointer to result structure (output)
 * @return Error code
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API
slamtec_aurora_sdk_timesync_get_wallclock_offset(
    slamtec_aurora_sdk_timesync_handle_t handle,
    uint32_t timeout_ms,
    slamtec_aurora_sdk_wallclock_offset_result_t* result_out
);

/**
 * @brief Sync server's wall clock with client's wall clock
 * @ingroup Utility_Functions Utility Functions
 *
 * @details This function automatically queries the current wall clock offset between
 *          server and client, then requests the server to adjust its wall clock to
 *          match the client's UTC wall clock. The offset is calculated internally.
 *
 * @param handle The time sync client handle
 * @param timeout_ms Timeout for the operation (milliseconds)
 * @param result_out Pointer to result structure (output)
 * @return Error code
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API
slamtec_aurora_sdk_timesync_sync_server_wallclock(
    slamtec_aurora_sdk_timesync_handle_t handle,
    uint32_t timeout_ms,
    slamtec_aurora_sdk_wallclock_sync_result_t* result_out
);

/**
 * @brief Evaluate wall clock synchronization accuracy
 * @ingroup Utility_Functions Utility Functions
 *
 * @details This function evaluates the accuracy of the wall clock synchronization
 *          by querying the current offset between server and client wall clocks.
 *
 * @param handle The time sync client handle
 * @param timeout_ms Timeout for the query (milliseconds)
 * @param result_out Pointer to result structure (output)
 * @return Error code
 */
slamtec_aurora_sdk_errorcode_t AURORA_SDK_API
slamtec_aurora_sdk_timesync_evaluate_wallclock_accuracy(
    slamtec_aurora_sdk_timesync_handle_t handle,
    uint32_t timeout_ms,
    slamtec_aurora_sdk_wallclock_accuracy_result_t* result_out
);

#ifdef __cplusplus
}
#endif
