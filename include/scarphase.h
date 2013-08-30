/**
 * Copyright (c) 2011-2013 Andreas Sembrant
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  - Neither the name of the copyright holders nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Andreas Sembrant
 *
 */

#ifndef __SCARPHASE_H
#define __SCARPHASE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include <signal.h>

//----------------------------------------------------------------------------//

/**
 * @brief
 */
typedef void* scarphase_handle_t;

//----------------------------------------------------------------------------//

/**
 * @brief
 */
struct scarphase_event_info
{

    /**
     * @brief A handle to the monitor.
     */
    void *handle;

    /**
     * @brief The thread id.
     */
    pid_t tid;

    /**
     * @brief The phase id.
     */
    pid_t phase;

    /**
     * @brief Prediction of the next interval.
     */
    struct prediction
    {

        /**
         * @brief The phase id of the next interval.
         */
        pid_t phase;

        /**
         * @brief How confident we are in our prediction.
         */
        size_t confidence;

    } prediction;

    /**
     * @brief Internal debugging structure.
     */
    void *_debug;

};

/**
 * Typedef for the info struct.
 */
typedef struct scarphase_event_info scarphase_event_info_t;

//----------------------------------------------------------------------------//

/**
 * @brief
 */
struct scarphase_data_accessor
{

    /**
     * @brief Start collection samples.
     */
    int (*start)(void*);

    /**
     * @brief Stop
     */
    int (*stop)(void*);

    /**
     * @brief Forward signal to data accessor.
     * @returns 1 if the signal was handles, 0 if ignored.
     */
    int (*handle_signal)(void*, int signum, siginfo_t *info, void *uc);

    /**
     * @brief Get all samples from within a window.
     */
    int (*get_samples)(void*, const uint64_t **samples, size_t *size);

    /**
     * @brief Get the sample period.
     */
    uint64_t (*get_sample_period)(void*);

    /**
     * @brief Set the sample period.
     */
    int (*set_sample_period)(void*, uint64_t sample_period);

    /**
     * @brief Get the window size.
     */
    uint64_t (*get_window_size)(void*);

    /**
     * @brief Set the window size.
     */
    int (*set_window_size)(void*, uint64_t window_size);

};

/**
 * Typedef for the info struct.
 */
typedef struct scarphase_data_accessor scarphase_data_accessor_t;

//----------------------------------------------------------------------------//

/**
 * @brief Monitor settings.
 */
struct scarphase_monitor_attr
{

    /**
     * @brief The sample period is the number of branches between samples.
     *        A low sample_period results in a high sample rate.
     */
    uint64_t sample_period;

    /**
     * @brief The size of the execution window in number of instructions.
     */
    uint64_t window_size;

    /**
     * @brief Data accessor to collect samples for frequency vector.
     *        e.g. BBV (Basic Block Vector).
     */
    scarphase_data_accessor_t *data_accessor;

    /**
     * @brief User data accessor to collect samples for extra vector.
     *        e.g. Data with CPI, L3 misses, ...
     */
    scarphase_data_accessor_t *user_data_accessor;

    /**
     * @brief Extra hooks into the library.
     */
    struct hooks
    {
        /**
         * @brief Hashfunction.
         */
        uint64_t (*hash_function)(scarphase_handle_t monitor, uint64_t ip);

    } hooks;

    /**
     * @brief Dynamic sample rate.
     */
    struct dsr
    {

        /**
         * @brief Enable dynamic sample rate.
         */
        int enabled;

        /**
         * @brief The maximum size of the sample period.
         *        multiplier * iteration < maximum_size
         */
        int maximum_sample_period;

        /**
         * @brief Similarity threshold when in dsr mode.
         */
        double similarity_threshold;

    } dsr;

    /**
     * @brief Classifier.
     */
    struct classifier
    {

        #define SCARPHASE_CLASSIFIER_TYPE_LEADER_FOLLOWER     0

        /**
         * @brief The type of classifier;
         */
        int type;

        union
        {

            /**
             * @brief Distance based classification
             */
            struct leader_follower
            {

                #define SCARPHASE_CLASSIFIER_LEADER_FOLLOWER_TYPE_BOUNDED     0
                #define SCARPHASE_CLASSIFIER_LEADER_FOLLOWER_TYPE_UNBOUNDED   1
                int type;

                /**
                 * @brief The similarity threshold, [0,1.0]
                 */
                double similarity_threshold;

                /**
                 * @brief Number of intervals a phase must have been seen in
                 *        before beeing assigned a unique phase id.
                 */
                int transition_threshold;

            } leader_follower;

        } attr;

    } classifier;

    /**
     * @brief Predictor.
     */
    struct predictor
    {

        #define SCARPHASE_PREDICTOR_TYPE_LAST_VALUE      0
        #define SCARPHASE_PREDICTOR_TYPE_RUN_LENGTH      1

        /**
         * @brief The type of predictor;
         */
        int type;

        union
        {

            /**
             * @brief Last value predictor.
             */
            struct last_value
            {

            } last_value;

            /**
             * @brief Run length predictor
             *  _ _
             * |_|_| a
             * |_|_| a
             * |_|_| a
             * |_|_| a
             *  b b
             *
             * a = cache size
             * b = pattern length
             *
             */
            struct run_length
            {

                /**
                 * @brief The size of the cache
                 */
                int cache_size;

                /**
                 * @brief The size of the phase history.
                 */
                int pattern_length;

                /**
                 * @brief The number of correct prediction before relying on
                 *        the prediction. Otherwise the last value prediction
                 *        is used.
                 */
                int confidence_threshold;

            } run_length;

        } attr;

    } predictor;
};

/**
 * Typedef for the monitor attr struct.
 */
typedef struct scarphase_monitor_attr scarphase_monitor_attr_t;

//----------------------------------------------------------------------------//

/**
 * @brief Init the montior attributes with sane values.
 * @param attr  The attributes to initialize;
 */
extern void scarphase_init_attr(scarphase_monitor_attr_t *attr);

/**
 * @brief Init the library.
 * @returns 0 on success, otherwise -1.
 */
extern int scarphase_init();

/**
 * @brief Shutdown the library and free any allocated resources.
 *
 * @returns 0 on success, otherwise -1.
 */
extern int scarphase_shutdown();

/**
 * @brief A string representation of the error.
 * @param monitor       Monitor handle. If null get system error.
 */
extern const char *scarphase_strerror(scarphase_handle_t monitor);

/**
 * @brief Create a monitor for a given thread.
 * @param tid       The thread/process id.
 * @param attr      Monitor settings.
 * @param parent    0 if no parent, otherwise parent handle.
 *                  For multi-threaded applications. This is used to share
 *                  certain components, such as classifier, predictors.
 *
 * @returns NULL on failure, otherwise a monitor handle.
 */
extern scarphase_handle_t scarphase_open_monitor(
        pid_t tid, scarphase_monitor_attr_t *attr, scarphase_handle_t parent);

/**
 * @brief Close a monitor.
 * @param monitor   Monitor handle.
 * @returns 0 on success, otherwise -1.
 */
extern int scarphase_close_monitor(scarphase_handle_t monitor);

/**
 * @brief Start monitoring the phases.
 * @param monitor       Monitor handle.
 * @returns 0 on success, otherwise -1.
 */
extern int scarphase_start_monitor(scarphase_handle_t monitor);

/**
 * @brief Stop monitoring the phases.
 * @param monitor       Monitor handle.
 * @returns 0 on success, otherwise -1.
 */
extern int scarphase_stop_monitor(scarphase_handle_t monitor);

/**
 * @brief Add a new vector to scarphase. It will classify it etc, and call the
 *        callback functions.
 *
 *        This function can be used to do analysis offline with different
 *        settings. Save the vector at runtime, and reload it with this
 *        function.
 *
 * @param monitor       Monitor handle.
 *
 * @returns A pointer to an event info structure on success, otherwise NULL.
 *
 */
extern scarphase_event_info_t*
scarphase_handle_signal(scarphase_handle_t monitor,
                        int signum, siginfo_t *info, void *uc);

/**
 * @brief Add a new vector to scarphase. It will classify it etc, and call the
 *        callback functions.
 *
 *        This function can be used to do analysis offline with different
 *        settings. Save the vector at runtime, and reload it with this
 *        function.
 *
 * @param monitor       Monitor handle.
 *
 * @returns A pointer to an event info structure on success, otherwise NULL.
 *
 */
extern scarphase_event_info_t*
scarphase_force_new_window(scarphase_handle_t monitor);

//----------------------------------------------------------------------------//

#ifdef __cplusplus
}
#endif


#endif /* __SCARPHASE_H */
