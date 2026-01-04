/*
 * Copyright 2013-2025 SLAMTEC Co., Ltd.
 */

#pragma once

// NOTE: This file should only be included through aurora_pubsdk_inc.h
// The C API types from aurora_pubsdk_timesync.h are already included there

#include <string>
#include <utility>
#include <stdexcept>

/**
 * @defgroup Cxx_TimeSyncUtility_Operations Time Synchronization Utility Operations
 * @brief The time synchronization utility classes
 * @ingroup SDK_Cxx_Wrapper Aurora Remote SDK C++ Interface
 */

// NOTE: This file is included inside the rp::standalone::aurora namespace in slamtec_remote_public.hxx
// So we don't redeclare the namespace here

/**
 * @brief Time domain types for synchronization (C++ enum)
 * @ingroup Cxx_TimeSyncUtility_Operations Time Synchronization Utility Operations
 */
enum class TimeSyncDomain {
    STEADY_CLOCK = SLAMTEC_AURORA_SDK_TIMESYNC_DOMAIN_STEADY_CLOCK,  ///< Monotonic steady clock
    WALL_CLOCK = SLAMTEC_AURORA_SDK_TIMESYNC_DOMAIN_WALL_CLOCK      ///< System wall clock
};

/**
 * @brief Time synchronization client options (C++ struct)
 * @ingroup Cxx_TimeSyncUtility_Operations Time Synchronization Utility Operations
 */
struct TimeSyncOptions {
    uint32_t synchronization_interval_ms = 500;   ///< Interval between sync requests (milliseconds)
    uint32_t sample_window_size = 100;            ///< Number of samples in sliding window
    double outlier_threshold = 3.0;               ///< Outlier detection threshold (MAD multiplier)
    uint32_t min_samples_for_sync = 10;           ///< Minimum samples required before synchronized
    uint32_t timeout_ms = 1000;                   ///< Response timeout (milliseconds)
    double max_rtt_ms = 100.0;                    ///< Maximum acceptable RTT (milliseconds)
    uint32_t initialize_timeout_ms = 15000;       ///< Timeout for initialize() operation (milliseconds)

    /**
     * @brief Get default options with recommended values
     * @return TimeSyncOptions with default values
     */
    static TimeSyncOptions getDefaults() {
        TimeSyncOptions opts;
        slamtec_aurora_sdk_timesync_options_t c_opts;
        slamtec_aurora_sdk_timesync_get_default_options(&c_opts);
        opts.fromC(c_opts);
        return opts;
    }

    /**
     * @brief Initialize from C structure
     */
    void fromC(const slamtec_aurora_sdk_timesync_options_t& c_opts) {
        synchronization_interval_ms = c_opts.synchronization_interval_ms;
        sample_window_size = c_opts.sample_window_size;
        outlier_threshold = c_opts.outlier_threshold;
        min_samples_for_sync = c_opts.min_samples_for_sync;
        timeout_ms = c_opts.timeout_ms;
        max_rtt_ms = c_opts.max_rtt_ms;
        initialize_timeout_ms = c_opts.initialize_timeout_ms;
    }

    /**
     * @brief Convert to C structure
     */
    slamtec_aurora_sdk_timesync_options_t toC() const {
        slamtec_aurora_sdk_timesync_options_t c_opts;
        c_opts.synchronization_interval_ms = synchronization_interval_ms;
        c_opts.sample_window_size = sample_window_size;
        c_opts.outlier_threshold = outlier_threshold;
        c_opts.min_samples_for_sync = min_samples_for_sync;
        c_opts.timeout_ms = timeout_ms;
        c_opts.max_rtt_ms = max_rtt_ms;
        c_opts.initialize_timeout_ms = initialize_timeout_ms;
        return c_opts;
    }
};

/**
 * @brief Time synchronization quality metrics (C++ struct)
 * @ingroup Cxx_TimeSyncUtility_Operations Time Synchronization Utility Operations
 */
struct TimeSyncQuality {
    double rmse_ms;                ///< Root mean square error (milliseconds)
    double max_error_ms;           ///< Maximum absolute error (milliseconds)
    double scale;                  ///< Estimated time scale factor
    double offset_ns;              ///< Estimated time offset (nanoseconds)
    double scale_std_dev;          ///< Standard deviation of scale estimate
    double offset_std_dev_ns;      ///< Standard deviation of offset estimate (nanoseconds)
    uint32_t sample_count;         ///< Number of samples used
    uint32_t total_sync_count;     ///< Total synchronization attempts
    uint32_t failed_sync_count;    ///< Failed synchronization attempts

    /**
     * @brief Construct from C structure
     */
    TimeSyncQuality(const slamtec_aurora_sdk_timesync_quality_t& c_quality)
        : rmse_ms(c_quality.rmse_ms)
        , max_error_ms(c_quality.max_error_ms)
        , scale(c_quality.scale)
        , offset_ns(c_quality.offset_ns)
        , scale_std_dev(c_quality.scale_std_dev)
        , offset_std_dev_ns(c_quality.offset_std_dev_ns)
        , sample_count(c_quality.sample_count)
        , total_sync_count(c_quality.total_sync_count)
        , failed_sync_count(c_quality.failed_sync_count)
    {}

    TimeSyncQuality() = default;
};

/**
 * @brief Remote time synchronization client (C++ wrapper)
 * @details RAII-style wrapper for time synchronization client
 * @ingroup Cxx_TimeSyncUtility_Operations Time Synchronization Utility Operations
 *
 * Example usage:
 * @code
 * RemoteTimeSyncClient client(TimeSyncDomain::STEADY_CLOCK);
 * if (client.connect("192.168.1.100", 9527)) {
 *     TimeSyncOptions options;
 *     options.synchronization_interval_ms = 1000;
 *     client.setOptions(options);
 *
 *     if (client.initialize(5000)) {
 *         while (!client.isSynchronized()) {
 *             std::this_thread::sleep_for(std::chrono::milliseconds(100));
 *         }
 *
 *         uint64_t aurora_time = 123456789000;
 *         uint64_t client_time;
 *         if (client.translateTimestamp(aurora_time, &client_time)) {
 *             std::cout << "Translated time: " << client_time << " ns\n";
 *         }
 *     }
 * }
 * @endcode
 */
class RemoteTimeSyncClient {
public:
    /**
     * @brief Construct a time synchronization client
     * @param domain The time domain type for this client
     * @throws std::runtime_error if creation fails
     */
    explicit RemoteTimeSyncClient(TimeSyncDomain domain)
        : handle_(nullptr)
    {
        slamtec_aurora_sdk_errorcode_t err;
        handle_ = slamtec_aurora_sdk_timesync_create_instance(
            static_cast<slamtec_aurora_sdk_timesync_domain_t>(domain),
            &err
        );

        if (!handle_) {
            throw std::runtime_error("Failed to create time sync client instance");
        }
    }

    /**
     * @brief Destructor - automatically stops and destroys the client
     */
    ~RemoteTimeSyncClient() {
        if (handle_) {
            slamtec_aurora_sdk_timesync_destroy_instance(handle_);
        }
    }

    // Movable but not copyable
    RemoteTimeSyncClient(RemoteTimeSyncClient&& other) noexcept
        : handle_(other.handle_)
    {
        other.handle_ = nullptr;
    }

    RemoteTimeSyncClient& operator=(RemoteTimeSyncClient&& other) noexcept {
        if (this != &other) {
            if (handle_) {
                slamtec_aurora_sdk_timesync_destroy_instance(handle_);
            }
            handle_ = other.handle_;
            other.handle_ = nullptr;
        }
        return *this;
    }

    RemoteTimeSyncClient(const RemoteTimeSyncClient&) = delete;
    RemoteTimeSyncClient& operator=(const RemoteTimeSyncClient&) = delete;

    /**
     * @brief Connect to a time synchronization server
     * @param server_address Server IP address or hostname
     * @param server_port Server port number (default: 9527)
     * @param error_code Optional pointer to store error code
     * @return true if connection succeeded, false otherwise
     */
    bool connect(const std::string& server_address,
                 uint16_t server_port = SLAMTEC_AURORA_SDK_TIMESYNC_DEFAULT_PORT,
                 slamtec_aurora_sdk_errorcode_t* error_code = nullptr) {
        auto result = slamtec_aurora_sdk_timesync_connect(
            handle_, server_address.c_str(), server_port);

        if (error_code) {
            *error_code = result;
        }
        return result == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

    /**
     * @brief Set time synchronization options
     * @param options The options to configure
     * @param error_code Optional pointer to store error code
     * @return true if successful, false otherwise
     */
    bool setOptions(const TimeSyncOptions& options,
                    slamtec_aurora_sdk_errorcode_t* error_code = nullptr) {
        auto c_options = options.toC();
        auto result = slamtec_aurora_sdk_timesync_set_options(handle_, &c_options);

        if (error_code) {
            *error_code = result;
        }
        return result == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

    /**
     * @brief Initialize time synchronization with first sync request
     * @details The timeout is configured via setOptions(). If not set, default timeout will be used.
     * @param error_code Optional pointer to store error code
     * @return true if initialization succeeded, false otherwise
     */
    bool initialize(slamtec_aurora_sdk_errorcode_t* error_code = nullptr) {
        auto result = slamtec_aurora_sdk_timesync_initialize(handle_);

        if (error_code) {
            *error_code = result;
        }
        return result == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

    /**
     * @brief Check if time synchronization is established
     * @return true if synchronized, false otherwise
     */
    bool isSynchronized() const {
        return slamtec_aurora_sdk_timesync_is_synchronized(handle_) != 0;
    }

    /**
     * @brief Check if time synchronization is running
     * @return true if running, false otherwise
     */
    bool isRunning() const {
        return slamtec_aurora_sdk_timesync_is_running(handle_) != 0;
    }

    /**
     * @brief Translate Aurora timestamp to client time domain
     * @param aurora_timestamp_ns Aurora timestamp in nanoseconds
     * @param client_timestamp_ns Pointer to store translated timestamp
     * @param error_code Optional pointer to store error code
     * @return true if translation succeeded, false otherwise (e.g., not synchronized)
     */
    bool translateTimestamp(uint64_t aurora_timestamp_ns,
                           uint64_t* client_timestamp_ns,
                           slamtec_aurora_sdk_errorcode_t* error_code = nullptr) const {
        auto result = slamtec_aurora_sdk_timesync_translate_timestamp(
            handle_, aurora_timestamp_ns, client_timestamp_ns);

        if (error_code) {
            *error_code = result;
        }
        return result == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

    /**
     * @brief Get current synchronization quality metrics
     * @param quality Pointer to quality structure to fill
     * @param error_code Optional pointer to store error code
     * @return true if successful, false otherwise
     */
    bool getQuality(TimeSyncQuality* quality,
                    slamtec_aurora_sdk_errorcode_t* error_code = nullptr) const {
        slamtec_aurora_sdk_timesync_quality_t c_quality;
        auto result = slamtec_aurora_sdk_timesync_get_quality(handle_, &c_quality);

        if (result == SLAMTEC_AURORA_SDK_ERRORCODE_OK && quality) {
            *quality = TimeSyncQuality(c_quality);
        }

        if (error_code) {
            *error_code = result;
        }
        return result == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

    /**
     * @brief Get current server timestamp translated to client time domain
     * @param timeout_ms Timeout for the query (milliseconds)
     * @param server_timestamp_ns Pointer to store translated server timestamp
     * @param client_timestamp_ns Optional pointer to store client timestamp at query time
     * @param error_code Optional pointer to store error code
     * @return true if successful, false otherwise
     */
    bool getCurrentServerTimestamp(uint32_t timeout_ms,
                                   uint64_t* server_timestamp_ns,
                                   uint64_t* client_timestamp_ns = nullptr,
                                   slamtec_aurora_sdk_errorcode_t* error_code = nullptr) const {
        auto result = slamtec_aurora_sdk_timesync_get_current_server_timestamp(
            handle_, timeout_ms, server_timestamp_ns, client_timestamp_ns);

        if (error_code) {
            *error_code = result;
        }
        return result == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

    /**
     * @brief Stop time synchronization background thread
     */
    void stop() {
        slamtec_aurora_sdk_timesync_stop(handle_);
    }

    /**
     * @brief Get the underlying handle (for advanced use)
     * @return The C API handle
     */
    slamtec_aurora_sdk_timesync_handle_t getHandle() const {
        return handle_;
    }

    /**
     * @brief Get wall clock offset between server and client
     * @param timeout_ms Timeout for the query (milliseconds)
     * @param result Pointer to result structure to fill
     * @param error_code Optional pointer to store error code
     * @return true if successful, false otherwise
     */
    bool getWallClockOffset(uint32_t timeout_ms,
                           slamtec_aurora_sdk_wallclock_offset_result_t* result,
                           slamtec_aurora_sdk_errorcode_t* error_code = nullptr) const {
        auto res = slamtec_aurora_sdk_timesync_get_wallclock_offset(
            handle_, timeout_ms, result);

        if (error_code) {
            *error_code = res;
        }
        return res == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

    /**
     * @brief Sync server's wall clock with client's wall clock
     * @details Automatically queries the current wall clock offset and applies
     *          the correction to synchronize server with client's UTC wall clock.
     * @param timeout_ms Timeout for the operation (milliseconds)
     * @param result Pointer to result structure to fill
     * @param error_code Optional pointer to store error code
     * @return true if successful, false otherwise
     */
    bool syncServerWallClock(uint32_t timeout_ms,
                            slamtec_aurora_sdk_wallclock_sync_result_t* result,
                            slamtec_aurora_sdk_errorcode_t* error_code = nullptr) const {
        auto res = slamtec_aurora_sdk_timesync_sync_server_wallclock(
            handle_, timeout_ms, result);

        if (error_code) {
            *error_code = res;
        }
        return res == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

    /**
     * @brief Evaluate wall clock synchronization accuracy
     * @param timeout_ms Timeout for the query (milliseconds)
     * @param result Pointer to result structure to fill
     * @param error_code Optional pointer to store error code
     * @return true if successful, false otherwise
     */
    bool evaluateWallClockSyncAccuracy(uint32_t timeout_ms,
                                       slamtec_aurora_sdk_wallclock_accuracy_result_t* result,
                                       slamtec_aurora_sdk_errorcode_t* error_code = nullptr) const {
        auto res = slamtec_aurora_sdk_timesync_evaluate_wallclock_accuracy(
            handle_, timeout_ms, result);

        if (error_code) {
            *error_code = res;
        }
        return res == SLAMTEC_AURORA_SDK_ERRORCODE_OK;
    }

private:
    slamtec_aurora_sdk_timesync_handle_t handle_;
};

// End of file - namespace closed in slamtec_remote_public.hxx
