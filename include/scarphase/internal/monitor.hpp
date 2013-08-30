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

#ifndef __SCARPHASE_INTERNAL_MONITOR_HPP
#define __SCARPHASE_INTERNAL_MONITOR_HPP

#include <stdint.h>
#include <boost/shared_ptr.hpp>

#include <scarphase/internal/classifier/classifier.hpp>

#include <scarphase/internal/predictor/predictor.hpp>

#include <scarphase/internal/util/signature.hpp>
#include <scarphase/internal/util/phase_tracking_buffer.hpp>

#include <scarphase.h>
#include <scarphase/scarphase_debug.h>

namespace scarphase {
namespace internal {

/**
 * @brief Monitor an threads phase behavior.
 */
class Monitor //--------------------------------------------------------------//
{

public: //--------------------------------------------------------------------//

    /**
     * @brief C-tor
     * @param tid     The thread to monitor.
     * @param attr    Monitor settings.
     * @param parant  Parent process. Will share classifier and predictor with
     *                the parent.
     */
    Monitor(pid_t tid, const scarphase_monitor_attr *attr, Monitor *parent = 0);

    /**
     * @brief D-tor
     */
    ~Monitor();

    /**
     * @brief Start the monitoring process.
     */
    int Start();

    /**
     * @brief Start the monitoring process.
     */
    int Stop();

    /**
     * @brief Handle interval and process signature.
     */
    scarphase_event_info_t* HandleWindow();

    /**
     * @brief HandleSignal
     * @param signum
     * @param info
     * @param uc
     * @return
     */
    scarphase_event_info_t* HandleSignal(int signum, siginfo_t *info, void *uc);

private: //-------------------------------------------------------------------//

    /**
     * @brief Monitor attributes and configuration data.
     */
    struct _Conf : public scarphase_monitor_attr {

        /**
         * The thread id this monitor is watching.
         */
        int tid;

        /**
         *
         */
        bool debug;

    } conf;

    /**
     * @brief Settings and variables for dynamic sample rate.
     */
    struct _DynamicSampleRate {

        /**
         * @brief True/False if dsr is enabled.
         */
        bool enabled;

        /**
         * @brief The current sample period.
         */
        size_t current_sample_period;

        /**
         * @brief
         */
        size_t maximum_sample_period;

        /**
         * @brief
         */
        double similarity_threshold;

    } dsr;

    /**
     * @brief Extra hooks to add more functionallity.
     */
    struct _Hook {

        /**
         * @brief Hash function, i.e., BBV[ hash(ip) ].
         */
        uint64_t (*hash)(scarphase_handle_t monitor, uint64_t ip);

    } hook;

    struct _Debug {

        /**
         * Event info passed to the callbacks.
         */
        scarphase_debug_data_t _debug;

    } debug;

    /**
     * @brief Event info passed to the callbacks.
     */
    scarphase_event_info_t event_info;

    /**
     * @brief data_accessor
     */
    scarphase_data_accessor_t *data_accessor;

    /**
     * @brief The current signature.
     */
    scarphase::internal::util::Signature signature;

    /**
     * @brief Phase tracking buffer.
     */
    scarphase::internal::util::PhaseTrackingBuffer ptb;

    /**
     * @brief Classifier.
     */
    boost::shared_ptr<scarphase::internal::classifier::Classifier> classifier;

    /**
     * @brief Predictor.
     */
    boost::shared_ptr<scarphase::internal::predictor::Predictor> predictor;

};

} /* namespace internal */
} /* namespace scarphase */

#endif /* __SCARPHASE_INTERNAL_MONITOR_HPP */
