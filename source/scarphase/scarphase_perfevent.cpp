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

#include "scarphase.h"
#include "scarphase/scarphase_perfevent.h"

#include "scarphase/internal/util/perf_event_wrapper.hpp"

namespace spi = scarphase::internal;

//----------------------------------------------------------------------------//

struct AccessorWrapper
{

    scarphase_data_accessor_t accessor;

    /**
     * @brief The two perfomance counters.
     */
    struct _Event {

        /**
         * @brief Event to sample branch intructions.
         */
        boost::shared_ptr<spi::util::perf_event_wrapper::Event> sample;

        /**
         * @brief Event to count number of instruction.
         */
        boost::shared_ptr<spi::util::perf_event_wrapper::Event> window;

    } event;

    /**
     * @brief samples
     */
    std::vector<uint64_t> samples;

    /**
     * @brief samples
     */
    bool configured;

};

//----------------------------------------------------------------------------//

static void
_process_event(spi::util::perf_event_wrapper::Event *e, void *v, int status)
{
    std::vector<uint64_t>  *samples =
        static_cast<std::vector<uint64_t> *>(v);

    if (status) { return; }

    samples->push_back(e->Read<uint64_t>());
}

//----------------------------------------------------------------------------//

#ifdef __cplusplus
extern "C" {
#endif

static int
_start(void* _assessor)
{
    AccessorWrapper *accessor = static_cast<AccessorWrapper*>(_assessor);

    try
    {
    if (not accessor->configured)
    {
        accessor->event.sample->Configure();
        accessor->event.window->Configure();
        accessor->configured = true;
    }


    accessor->event.sample->Start();
    accessor->event.window->Start();
    }
    catch(...)
    {
        std::cout << boost::current_exception_diagnostic_information() << std::endl;
        throw;
    }

    return 1;
}


static int
_stop(void* _assessor)
{
    AccessorWrapper *accessor = static_cast<AccessorWrapper*>(_assessor);

    accessor->event.sample->Stop();
    accessor->event.window->Stop();

    return 1;
}

static int
_handle_signal(void* _assessor, int signum, siginfo_t *info, void *uc)
{
    AccessorWrapper *accessor = static_cast<AccessorWrapper*>(_assessor);
    return accessor->event.window->CheckSignal(signum, info, uc);
}

static int
_get_samples(void* _accessor, const uint64_t **samples, size_t *size)
{
    AccessorWrapper *accessor = static_cast<AccessorWrapper*>(_accessor);

    // Clear array for new samples
    accessor->samples.clear();

    // Collect all samples
    accessor->event.sample->ProcessEvents(
        _process_event, static_cast<void*> (&accessor->samples)
    );

    // Process events, but don't do anything
    accessor->event.window->ProcessEvents(0, 0);

    *samples = accessor->samples.data();
    *size    = accessor->samples.size();

    // Refresh to trigger next window
    accessor->event.window->Refresh();

    return 1;
}

static uint64_t
_get_sample_period(void* _assessor)
{
    AccessorWrapper *accessor = static_cast<AccessorWrapper*>(_assessor);
    return accessor->event.sample->GetSamplePeriod();
}

static int
_set_sample_period(void* _assessor, uint64_t sample_period)
{
    AccessorWrapper *accessor = static_cast<AccessorWrapper*>(_assessor);
    accessor->event.sample->SetSamplePeriod(sample_period);
    return 1;
}

static uint64_t
_get_window_size(void* _assessor)
{
    AccessorWrapper *accessor = static_cast<AccessorWrapper*>(_assessor);
    return accessor->event.window->GetSamplePeriod();
}

static int
_set_window_size(void* _assessor, uint64_t sample_period)
{
    AccessorWrapper *accessor = static_cast<AccessorWrapper*>(_assessor);

    // The updating is delayed with one period,
    // Reconfigure to force correct sample period
    accessor->event.window->SetSamplePeriod(sample_period);

    if (accessor->configured)
    {
        accessor->event.window->Reconfigure();
        accessor->event.window->Refresh();
    }

    return 1;
}

//----------------------------------------------------------------------------//

scarphase_data_accessor_t*
scarphase_create_perfevent_data_accessor(
    pid_t tid, scarphase_perfevent_attr_t *attr)
{
    AccessorWrapper *accessor = new AccessorWrapper();

    accessor->accessor.start = _start;
    accessor->accessor.stop = _stop;
    accessor->accessor.handle_signal = _handle_signal;
    accessor->accessor.get_samples = _get_samples;
    accessor->accessor.get_sample_period = _get_sample_period;
    accessor->accessor.set_sample_period = _set_sample_period;
    accessor->accessor.get_window_size = _get_window_size;
    accessor->accessor.set_window_size = _set_window_size;

    //------------------------------------------------------------------------//
    // Performance counters                                                   //
    //------------------------------------------------------------------------//

    accessor->event.sample =
        boost::shared_ptr<spi::util::perf_event_wrapper::Event>(
            new spi::util::perf_event_wrapper::Event(
                tid,
                attr->sample.counter,
                attr->sample.type_id,
                0,
                PERF_SAMPLE_IP
            )
        );

    accessor->event.window =
        boost::shared_ptr<spi::util::perf_event_wrapper::Event>(
            new spi::util::perf_event_wrapper::Event(
                tid,
                attr->window.counter,
                attr->window.type_id,
                0,
                PERF_SAMPLE_READ
            )
        );

    // Don't wakeup on branch events. Let the kernel buffer all the data,
    // and read it in go when we get a interval event.
    accessor->event.sample->SetWakeup(0);

    // Dont send a signal, the interval event will read from the
    // sample event
    accessor->event.window->EnableSigIO();

    // 0 == No PEBS, 1,2 == PEBS
    accessor->event.sample->SetPreciseIp(
        attr->sample.precise_ip);

    accessor->event.sample->SetKernalBufferSize(
        attr->sample.kernal_buffer_size);

    accessor->event.sample->SetPinned();
    accessor->event.window->SetPinned();

    accessor->configured = false;

    return &accessor->accessor;
}

void
scarphase_destroy_perfevent_data_accessor(scarphase_data_accessor_t* _accessor)
{
    AccessorWrapper *accessor =
        static_cast<AccessorWrapper*>(static_cast<void*>(_accessor));

    delete accessor;
}

#ifdef __cplusplus
}
#endif
