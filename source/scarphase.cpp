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

#include <cstring>

#include "scarphase.h"
#include "scarphase/internal/monitor.hpp"

//----------------------------------------------------------------------------//

// Return codes
#define SCARPHASE_FAILURE                          -1
#define SCARPHASE_SUCCESS                           0

//----------------------------------------------------------------------------//

extern "C"
{

//----------------------------------------------------------------------------//

void
scarphase_init_attr(scarphase_monitor_attr_t *attr)
{   
    BOOST_ASSERT(attr != NULL);

    memset(attr, 0, sizeof(scarphase_monitor_attr_t));

    attr->window_size = 100000000;
    attr->sample_period = 50000;

    attr->dsr.enabled = false;
    attr->dsr.maximum_sample_period = 15;
    attr->dsr.similarity_threshold = 0.5;

    attr->classifier.type = SCARPHASE_CLASSIFIER_TYPE_LEADER_FOLLOWER;
    attr->classifier.attr.leader_follower.type =
            SCARPHASE_CLASSIFIER_LEADER_FOLLOWER_TYPE_UNBOUNDED;
    attr->classifier.attr.leader_follower.similarity_threshold = 0.35;
    attr->classifier.attr.leader_follower.transition_threshold = 0;

    attr->predictor.type = SCARPHASE_PREDICTOR_TYPE_LAST_VALUE;
}


//----------------------------------------------------------------------------//

int
scarphase_init()
{
    return SCARPHASE_SUCCESS;
}

//----------------------------------------------------------------------------//

int
scarphase_shutdown()
{
    return SCARPHASE_SUCCESS;
}

//----------------------------------------------------------------------------//

scarphase_handle_t
scarphase_open_monitor(
    pid_t tid, scarphase_monitor_attr_t *attr, scarphase_handle_t parent)
{
    BOOST_ASSERT(attr != NULL);

    try
    {
        scarphase::internal::Monitor *monitor =
            new scarphase::internal::Monitor(
                tid, attr, static_cast<scarphase::internal::Monitor*>(parent)
            );

        // Return the monitor as a handle
        return static_cast<void*>(monitor);
    }
    catch(std::exception &e)
    {
        BOOST_ASSERT(0);
    }
}

//----------------------------------------------------------------------------//

int
scarphase_close_monitor(scarphase_handle_t handle)
{
    BOOST_ASSERT(handle != NULL);

    try
    {
        scarphase::internal::Monitor *monitor =
            static_cast<scarphase::internal::Monitor*>(handle);

        delete monitor;
    }
    catch(std::exception &e)
    {
        BOOST_ASSERT(0);
    }

    return SCARPHASE_SUCCESS;
}

//----------------------------------------------------------------------------//

int
scarphase_start_monitor(scarphase_handle_t handle)
{
    BOOST_ASSERT(handle != NULL);

//    try
    {
        scarphase::internal::Monitor *monitor =
            static_cast<scarphase::internal::Monitor*>(handle);

        return monitor->Start();
    }
//    catch(std::exception &e)
//    {
//        BOOST_ASSERT(0);
//    }

}

//----------------------------------------------------------------------------//

int
scarphase_stop_monitor(scarphase_handle_t handle)
{
    BOOST_ASSERT(handle != NULL);

    try
    {
        scarphase::internal::Monitor *monitor =
            static_cast<scarphase::internal::Monitor*>(handle);

        return monitor->Stop();
    }
    catch(std::exception &e)
    {
        BOOST_ASSERT(0);
    }
}

//----------------------------------------------------------------------------//

scarphase_event_info_t*
scarphase_handle_signal(scarphase_handle_t handle,
                        int signum, siginfo_t *info, void *uc)
{
    BOOST_ASSERT(handle != NULL);

    try
    {
        scarphase::internal::Monitor *monitor =
            static_cast<scarphase::internal::Monitor*>(handle);

        return monitor->HandleSignal(signum, info, uc);
    }
    catch(std::exception &e)
    {
        BOOST_ASSERT(0);
    }

}

//----------------------------------------------------------------------------//

scarphase_event_info_t*
scarphase_force_new_window(scarphase_handle_t handle)
{
    BOOST_ASSERT(handle != NULL);

    try
    {
        scarphase::internal::Monitor *monitor =
            static_cast<scarphase::internal::Monitor*>(handle);

        return monitor->HandleWindow();
    }
    catch(std::exception &e)
    {
        BOOST_ASSERT(0);
    }
}

//----------------------------------------------------------------------------//

} /* extern "C" */
