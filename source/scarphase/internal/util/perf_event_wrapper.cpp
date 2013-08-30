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

#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <iostream>

#include "scarphase/internal/util/perf_event_wrapper.hpp"

namespace scarphase {
namespace internal {
namespace util {
namespace perf_event_wrapper {

//----------------------------------------------------------------------------//

ChildEvent::ChildEvent(uint64_t config, uint32_t type)
    : id(0)  // 0 == Undefined
{

    // Zero the whole attribute struct
    std::memset(&attr, 0, sizeof(attr));

    attr.disabled       = false; // Have to be false, kernel panic otherwise
    attr.inherit        = false; // children inherit it

    attr.pinned         = false; // must always be on PMU
    attr.exclusive      = false; // only group on PMU

    attr.exclude_user   = false; // don't count user
    attr.exclude_kernel = true;  // ditto kernel
    attr.exclude_hv     = true;  // ditto hypervisor
    attr.exclude_idle   = true;  // don't count when idle

    attr.mmap           = false; // include mmap data
    attr.comm           = false; // include comm data
    attr.freq           = false; // use freq, not period
    attr.inherit_stat   = false; // per task counts
    attr.enable_on_exec = false; // next exec enables
    attr.task           = false; // trace fork/exit
    attr.watermark      = false; // wakeup_watermark

    // Just the size of the struct.
    attr.size           = sizeof(struct perf_event_attr);

    attr.type           = type;
    attr.config         = config;

    attr.precise_ip     = 0; // Leader decide
    attr.wakeup_events  = 0; // Leader decide

    attr.sample_type    = 0; // Leader decide
    attr.sample_period  = 0; // Leader decide
    attr.read_format    = 0; // Leader decide

}

ChildEvent::~ChildEvent()
{
    close(fd);
}

//----------------------------------------------------------------------------//

// Alloc mem for events
std::vector<Event*> *Event::event_lookup = NULL;

//----------------------------------------------------------------------------//

Event::Event(pid_t tid,
             uint64_t ctr_config, uint32_t ctr_type,
             uint64_t sample_period, uint64_t sample_type)
    :   enable_sigio(false),
        kernal_buffer_size(1),
        page_size(sysconf(_SC_PAGESIZE)),
        page_mask((kernal_buffer_size * page_size) - 1),
        fd(0), id(0), tid(tid), state(UNCONFIGURED),
        hdr(0), data(0), data_head(0), data_tail(0)
{

    // Zero the whole attribute struct
    std::memset(&attr, 0, sizeof(attr));

    attr.disabled       = true;
    attr.inherit        = false; // children inherit it

    attr.pinned         = true;  // must always be on PMU
    attr.exclusive      = false; // only group on PMU

    attr.exclude_user   = false; // don't count user
    attr.exclude_kernel = true;  // ditto kernel
    attr.exclude_hv     = true;  // ditto hypervisor
    attr.exclude_idle   = true;  // don't count when idle

    attr.mmap           = false; // include mmap data
    attr.comm           = false; // include comm data
    attr.freq           = false; // use freq, not period
    attr.inherit_stat   = false; // per task counts
    attr.enable_on_exec = false; // next exec enables
    attr.task           = false; // trace fork/exit
    attr.watermark      = false; // wakeup_watermark

    // Just the size of the struct.
    attr.size           = sizeof(struct perf_event_attr);

    attr.type           = ctr_type;
    attr.config         = ctr_config;

    attr.precise_ip     = 0;
    attr.wakeup_events  = 1; // wakeup every n events

    attr.sample_type    = sample_type;
    attr.sample_period  = sample_period;

    attr.read_format    = 0;

    if (Event::event_lookup == NULL)
    {
        Event::event_lookup = new std::vector<Event*>();
    }

    Event::event_lookup->push_back(this);

}

Event::~Event()
{

    // Close the fd if it has been opened
    if (fd)
    {       

        // Deallocate buffer
        if (hdr)
        {
            if (munmap(hdr, (kernal_buffer_size + 1) * page_size) == -1)
            {
                BOOST_THROW_EXCEPTION(PerfEventError() <<
                                      boost::errinfo_errno(errno) <<
                                      boost::errinfo_api_function("munmap"));
            }
        }

        // Close event
        if (close(fd) == -1)
        {
            BOOST_THROW_EXCEPTION(PerfEventError() <<
                                  boost::errinfo_errno(errno) <<
                                  boost::errinfo_api_function("close"));
        }
    }

    // Remove event from vector
    std::vector<Event*>::iterator e =
        std::find(Event::event_lookup->begin(),
                  Event::event_lookup->end(),
                  this);

    if (e != Event::event_lookup->end())
    {
        Event::event_lookup->erase(e);
    }

    if (Event::event_lookup->empty())
    {
        delete Event::event_lookup;
    }

}

//----------------------------------------------------------------------------//

bool
Event::CheckSignal(int signum, siginfo_t *info, void*)
{
    BOOST_ASSERT(info != NULL);

    // Check if correct signal
    return (signum == SIGIO && fd == info->si_fd);
}

//----------------------------------------------------------------------------//

Event*
Event::FindEvent(int signum, siginfo_t *info, void *uc)
{
    BOOST_ASSERT(Event::event_lookup != 0);

    // Find corresponding event
    for (size_t i = 0; i < Event::event_lookup->size(); i++)
    {
        if ((*Event::event_lookup)[i]->CheckSignal(signum, info, uc))
        {
            return (*Event::event_lookup)[i];
        }
    }

    // The signal did not belong to us
    return NULL;

}

//----------------------------------------------------------------------------//

void
Event::ProcessEvents(ProcessEventCallback event_callback,
                     void *event_callback_data)
{

    // Save head and tail in case the kernel updated them
    data_head = hdr->data_head;
    data_tail = hdr->data_tail;

    // Check if we have any data in the buffer
    if (data_tail == data_head)
    {
        return;
    }

    // Read as many bytes as possible
    for (size_t available = GetAvailableBytes();
         available > sizeof(struct perf_event_header);
         available = GetAvailableBytes())
    {

        // Get the hdr
        struct perf_event_header ehdr = Peak<struct perf_event_header>();

        // Not enbough data in buffer
        if (ehdr.size > available)
        {
            break;
        }

        // Skip the header we already read
        Skip<struct perf_event_header>();

        if (ehdr.type != PERF_RECORD_SAMPLE)
        {

            Skip(ehdr.size - sizeof(struct perf_event_header));

            switch(ehdr.type)
            {
            case PERF_RECORD_MMAP:
            case PERF_RECORD_READ:
            case PERF_RECORD_THROTTLE:
            case PERF_RECORD_UNTHROTTLE:
            case PERF_RECORD_COMM:
            case PERF_RECORD_EXIT:
            case PERF_RECORD_FORK:
            case PERF_RECORD_LOST:
            case PERF_RECORD_MAX:

            // Call the userfunction
            if (event_callback)
            {
                event_callback(this, event_callback_data, ehdr.type);
            }

            default:
                BOOST_ASSERT(1);
            }

        }
        // If PERF_RECORD_SAMPLE
        else
        {
            // Call the userfunction
            if (event_callback)
            {
                // The callback function is required to read the message from
                // the buffer
                event_callback(this, event_callback_data, 0);
            }
            else
            {
                Skip(ehdr.size - sizeof(struct perf_event_header));
            }

        } // endif PERF_RECORD_SAMPLE

    } // End while loop

    hdr->data_tail = data_tail;
}

//----------------------------------------------------------------------------//

static int
open(struct perf_event_attr &attr,
     pid_t pid,
     int cpu,
     int group_fd,
     unsigned long flags)
{

    #ifdef __powerpc__
    #define __NR_perf_event_open  319
    #elif defined(__x86_64__)
    #define __NR_perf_event_open  298
    #elif defined(__i386__)
    #define __NR_perf_event_open  336
    #endif

    int fd = syscall(__NR_perf_event_open, &attr, pid, cpu, group_fd, flags);

    if (fd == -1)
    {
        std::cout << strerror(errno) << std::endl;
        BOOST_THROW_EXCEPTION(PerfEventError() <<
                              boost::errinfo_errno(errno) <<
                              boost::errinfo_api_function("syscall"));
    }

    return fd;
}

void
Event::Configure()
{

    // Open the event.
    fd = open(attr, tid, -1, -1, 0);

    // Open all child events
    for (size_t i = 0; i < child_events.size(); i++)
    {
         child_events[i]->fd = open(child_events[i]->attr, tid, -1, fd, 0);
    }

    if (attr.read_format & PERF_FORMAT_ID)
    {

        if (attr.read_format & PERF_FORMAT_GROUP)
        {

            // Add for nr
            int skip = 1;

            // Add for time_enabled
            if (attr.read_format & PERF_FORMAT_TOTAL_TIME_ENABLED) skip++;

            // Add for time_running
            if (attr.read_format & PERF_FORMAT_TOTAL_TIME_RUNNING) skip++;

            // Size to read
            size_t size = (skip + 2 * (child_events.size() + 1));
            uint64_t values[size];

            // On overflow, the non lead events are stored in the sample.
            // However we need some key to figure the order in which they
            // were laid out in the buffer. The file descriptor does not
            // work for this. Instead, we extract a unique ID for each event.
            // That id will be part of the sample for each event value.
            // Therefore we will be able to match value to events
            //
            // PERF_FORMAT_ID: returns unique 64-bit identifier in addition
            // to event value.
            if (read(fd, &values, size * sizeof(uint64_t)) == -1)
            {
                BOOST_THROW_EXCEPTION(PerfEventError() <<
                                      boost::errinfo_errno(errno) <<
                                      boost::errinfo_api_function("read"));
            }

            // we are using PERF_FORMAT_GROUP, therefore the structure
            // of val is as follows:
            //
            //      { u64           nr;
            //        { u64         time_enabled; } && PERF_FORMAT_ENABLED
            //        { u64         time_running; } && PERF_FORMAT_RUNNING
            //        { u64         value;
            //          { u64       id;           } && PERF_FORMAT_ID
            //        }             cntr[nr];
            //
            // We are skipping the first 3 values
            //      (nr, time_enabled, time_running)
            // and then for each event we get a pair of values.
            // 3 = nr+time_enabled+time_running, 1 = to skid value
            id = values[(2 * 0) + 1 + skip];
            for(size_t i = 0; i < child_events.size(); i++)
            {
                child_events[i]->id = values[(2 * (i + 1)) + 1 + skip];
            }

        }
        else
        {

            // Add for value
            int skip = 1;

            // Add for time_enabled
            if (attr.read_format & PERF_FORMAT_TOTAL_TIME_ENABLED) skip++;

            // Add for time_running
            if (attr.read_format & PERF_FORMAT_TOTAL_TIME_RUNNING) skip++;

            // Size to read
            size_t size = (skip + 1);
            uint64_t values[size];

            if (read(fd, &values, size * sizeof(uint64_t)) == -1)
            {
                BOOST_THROW_EXCEPTION(PerfEventError() <<
                                      boost::errinfo_errno(errno) <<
                                      boost::errinfo_api_function("read"));
            }

            //
            //	{ __u64		value;
            //	  { __u64		time_enabled; } && PERF_FORMAT_ENABLED
            //	  { __u64		time_running; } && PERF_FORMAT_RUNNING
            //	  { __u64		id;           } && PERF_FORMAT_ID
            //	} && !PERF_FORMAT_GROUP
            id = values[skip];

        }

    }

    // Allocate buffer
    uint8_t *buffer = (uint8_t*) mmap(
        NULL,
        (kernal_buffer_size + 1) * page_size,
        PROT_READ|PROT_WRITE,
        MAP_SHARED,
        fd,
        0);

    if (buffer == MAP_FAILED)
    {
        BOOST_THROW_EXCEPTION(PerfEventError() <<
                              boost::errinfo_errno(errno) <<
                              boost::errinfo_api_function("mmap"));
    }

    // Data points to beginning of buffer payload
    data = buffer + page_size;

    // The start of the buffer points to the header
    hdr = (struct perf_event_mmap_page*) buffer;

    if (enable_sigio)
    {

        // Setup asynchronous notification on the file descriptor
        if (fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_ASYNC) == -1)
        {
            BOOST_THROW_EXCEPTION(PerfEventError() <<
                                  boost::errinfo_errno(errno) <<
                                  boost::errinfo_api_function("fcntl"));
        }

        // Set F_SETSIG to allow SIGIO with siginfo->si_fd
        if (fcntl(fd, F_SETSIG, SIGIO) == -1)
        {
            BOOST_THROW_EXCEPTION(PerfEventError() <<
                                  boost::errinfo_errno(errno) <<
                                  boost::errinfo_api_function("fcntl"));
        }

        f_owner_ex owner;
        owner.type = F_OWNER_TID;
        owner.pid  = tid;

        // Change it so the pid get the signal
        if (fcntl(fd, F_SETOWN_EX, &owner) == -1)
        {
            BOOST_THROW_EXCEPTION(PerfEventError() <<
                                  boost::errinfo_errno(errno) <<
                                  boost::errinfo_api_function("fcntl"));
        }

    }

    state = CONFIGURED;

}

void
Event::Reconfigure()
{
    close(fd);

    if (munmap(hdr, (kernal_buffer_size + 1) * page_size) == -1)
    {
        BOOST_THROW_EXCEPTION(PerfEventError() <<
                              boost::errinfo_errno(errno) <<
                              boost::errinfo_api_function("munmap"));
    }

    Configure();
}

//----------------------------------------------------------------------------//

} /* namespace perf_event_wrapper */
} /* namespace util */
} /* namespace internal */
} /* namespace scarphase */
