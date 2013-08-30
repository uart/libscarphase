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

#ifndef __SCARPHASE_PERFEVENT_EVENT_HPP
#define __SCARPHASE_PERFEVENT_EVENT_HPP

#include <vector>
#include <signal.h>
#include <stdint.h>
#include <algorithm>

#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>
#include <boost/exception/all.hpp>

#include <linux/perf_event.h>

//----------------------------------------------------------------------------//

namespace scarphase {
namespace internal {
namespace util {
namespace perf_event_wrapper {

/**
 * @brief The PerfEventError struct
 */
struct PerfEventError : virtual std::exception, virtual boost::exception { };

/**
 * @brief Group perf events together.
 */
class ChildEvent: public boost::noncopyable //--------------------------------//
{

    /**
     * @brief Set Event as a friend class.
     */
    friend class Event;

public: //--------------------------------------------------------------------//

    /**
     * @brief C-tor for child events.
     * @param[in]  config           The counter config, what to count.
     * @param[in]  type             The perf_event type, HW, RAW, etc
     */
    ChildEvent(uint64_t config, uint32_t type);

    /**
     * @brief D-tor
     */
    ~ChildEvent();

    /**
     * @brief Get the unique kernel identifier.
     */
    uint64_t GetId() const;

    /**
     * @brief Read from the file descriptor.
     * @returns The value read.
     */
    template<typename T>
    T ReadNow();

    /**
     * @brief Read from the file descriptor.
     * @param[out]  value variable to save the read data.
     */
    template<typename T>
    void ReadNow(T &value);

    /**
     * @brief Conviniance function.
     */
    void Start();
    void Stop();

    /**
     * @brief Refresh for another sample.
     */
    void Refresh();

    /**
     * @brief Enable
     */
    void Enable();

    /**
     * @brief Disable
     */
    void Disable();

    /**
     * @brief Reset
     */
    void Reset();

private: //-------------------------------------------------------------------//

    /**
     * Perf event attributes.
     */
    perf_event_attr attr;

    /**
     * The event file descriptor.
     */
    int fd;

    /**
     * Kernel event id.
     */
    uint64_t id;

};

/**
 * @brief Class to handle perf_event.
 */
class Event: public boost::noncopyable //-------------------------------------//
{

public: //--------------------------------------------------------------------//

    enum State {
        UNCONFIGURED,
        CONFIGURED,
        STARTED,
        STOPPED
    };

    /**
     * @brief A callback when an event occur.
     * @param[in]   e The event.
     * @param[in]   v User defind data.
     */
    typedef void (*ProcessEventCallback)(Event *e, void *v, int status);

    /**
     * @brief Shared ptr for the child event.
     */
    typedef boost::shared_ptr<ChildEvent> ChildEventPtr;

    //------------------------------------------------------------------------//
    //

    /**
     * C-tor for leader events
     * @param[in]  tid              Thread id.
     * @param[in]  ctr_config       The counter config, what to count.
     * @param[in]  ctr_type         The perf_event type, HW, RAW, etc
     * @param[in]  sample_period    The sample period to sample with.
     * @param[in]  sample_type      The sample type. sample_type
     */
    Event(pid_t    tid,
          uint64_t ctr_config,
          uint32_t ctr_type,
          uint64_t sample_period,
          uint64_t sample_type);

    /**
     * @brief D-tor.
     */
    ~Event();

    //------------------------------------------------------------------------//
    // Settings

    /**
     * @brief Add an child event to the group.
     */
    void AttachEvent(ChildEventPtr event);

    /**
     * @brief Get the unique kernel identifier.
     */
    uint64_t GetId() const;

    /**
     * @brief Get the unique kernel identifier.
     */
    int GetFd() const;

    /**
     * @brief Get the unique kernel identifier.
     */
    uint64_t GetSamplePeriod() const;

    /**
     * @brief Set frequency instead of sample period.
     */
    void SetSamplePeriod(uint64_t sample_period);

    /**
     * @brief Set pid to monitor.
     * @warning Must be called before configure or it will not have any effects.
     */
    void SetTID(pid_t tid);

    /**
     * @brief Set number of event to wake up after. When SIGIO will be sent.
     */
    void SetWakeup(uint32_t wakeup);

    /**
     * @brief The counter must always be on PMU.
     */
    void SetPinned();

    /**
     * @brief Set frequency instead of sample period.
     */
    void SetFrequency(uint64_t frequency);

    /**
     * @brief Set precise ip.
     *
     * precise_ip:
     *
     *  0 - SAMPLE_IP can have arbitrary skid
     *  1 - SAMPLE_IP must have constant skid
     *  2 - SAMPLE_IP requested to have 0 skid
     *  3 - SAMPLE_IP must have 0 skid
     *
     *  See also PERF_RECORD_MISC_EXACT_IP
     */
    void SetPreciseIp(int precise_ip);

    /**
     * @brief Set number of pages to allocate.
     */
    void SetKernalBufferSize(size_t size);

    /**
     * @brief
     */
    void SetExcludeUser(bool exclude);

    /**
     * @brief
     */
    void SetExcludeKernel(bool exclude);

    /**
     * @brief
     */
    void SetExcludeHyperVisor(bool exclude);

    /**
     * @brief
     */
    void SetExcludeIdle(bool exclude);

    /**
     * @brief Enable sigio so a signal will be generated when event triggers.
     */
    void EnableSigIO();

    /**
     * @brief Enable sigio so a signal will be generated when event triggers.
     */
    void DisableSigIO();

    //------------------------------------------------------------------------//
    // Open and configure event

    /**
     * @brief Start the performance event.
     */
    void Configure();

    /**
     * @brief Reconfigure event.
     */
    void Reconfigure();

    //------------------------------------------------------------------------//
    // Read data from ProcessEvents callback

    /**
     * @brief Get number of bytes that are available in the buffer.
     */
    size_t GetAvailableBytes() const;

    /**
     * @brief Read from the buffer when an event has occured.
     * @returns The value that was read.
     * @throws std::runtime_error if buffer underflow.
     */
    template<typename T>
    T Peak();

    /**
     * @brief Read from the buffer when an event has occured.
     * @param[out]  value variable to save the read data.
     * @throws std::runtime_error if buffer underflow.
     */
    template<typename T>
    void Peak(T &value);

    /**
     * @brief Read from the buffer when an event has occured.
     * @returns The value that was read.
     * @throws std::runtime_error if buffer underflow.
     */
    template<typename T>
    T Read();

    /**
     * @brief Read from the buffer when an event has occured.
     * @param[out]  value variable to save the read data.
     * @throws std::runtime_error if buffer underflow.
     */
    template<typename T>
    void Read(T &value);

    /**
     * @brief Skip n bytes from buffer.
     */
    template<typename T>
    void Skip();

    /**
     * @brief Skip n bytes from buffer.
     * @param[in]   n    Number of bytes to skip.
     */
    void Skip(size_t n);

    //------------------------------------------------------------------------//
    // Read data

    /**
     * @brief Read from the file descriptor.
     * @returns The value read.
     */
    template<typename T>
    T ReadNow();

    /**
     * @brief Read from the file descriptor.
     * @param[out]  value variable to save the read data.
     */
    template<typename T>
    void ReadNow(T &value);

    //------------------------------------------------------------------------//
    // Runtime management

    /**
     * @brief Conviniace function.
     */
    void Start();
    void Stop();

    /**
     * @brief Refresh for attr.wakeup_events samples.
     */
    void Refresh();

    /**
     * @brief Enable
     */
    void Enable();

    /**
     * @brief Disable
     */
    void Disable();

    /**
     * @brief Reset
     */
    void Reset();

    /**
     * @brief Process events in the buffer.
     */
    void ProcessEvents(ProcessEventCallback event_callback,
                       void *event_callback_data);

    //------------------------------------------------------------------------//
    // Settings

    /**
     * @brief Check if signal belongs to event
     * @returns
     */
    bool CheckSignal(int signum, siginfo_t *info, void *uc);

    /**
     * @brief Find the event the signal belongs to.
     */
    static Event* FindEvent(int signum, siginfo_t *info, void *uc);

private: //-------------------------------------------------------------------//


    //------------------------------------------------------------------------//
    // Static

    /**
     * @brief Lookup events from fd.
     */
    static std::vector<Event*> *event_lookup;

    //------------------------------------------------------------------------//
    // Settings

    /**
     * @brief Perf event attributes.
     */
    perf_event_attr attr;

    /**
     * @brief If enable, a signal will be sent when event occures.
     */
    bool enable_sigio;

    /**
     * @brief Number of pages to allocate.
     */
    size_t kernal_buffer_size;

    /**
     * @brief The page size and size.
     */
    size_t page_size, page_mask;

    //------------------------------------------------------------------------//
    // Properties

    /**
     * @brief The event file descriptor.
     */
    int fd;

    /**
     * @brief Kernel event id.
     */
    uint64_t id;

    /**
     * @brief The process/thread id to monitor.
     */
    int tid;

    /**
     * @brief state
     */
    State state;

    /**
     * @brief The child events.
     */
    std::vector<ChildEventPtr> child_events;

    //------------------------------------------------------------------------//
    // Buffer state

    /**
     * @brief Pointer to the header of the buffer.
     */
    struct perf_event_mmap_page *hdr;

    /**
     * @brief Memory buffer.
     */
    uint8_t *data;

    /**
     * @brief The head in the buffer.
     */
    uint64_t data_head, data_tail;

};

} /* namespace perf_event_wrapper */
} /* namespace util */
} /* namespace internal */
} /* namespace scarphase */


//---------------------------------------------------------------------------//
// Inline Definitions                                                        //
//---------------------------------------------------------------------------//

#include <cstring>
#include <errno.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

namespace scarphase {
namespace internal {
namespace util {
namespace perf_event_wrapper {

inline uint64_t
ChildEvent::GetId() const
{
    return id;
}

//----------------------------------------------------------------------------//

template<typename T>
inline T
ChildEvent::ReadNow()
{
    T value;
    ReadNow(value);
    return value;
}

template<typename T>
inline void
ChildEvent::ReadNow(T &value)
{
    // Read value
    if (read(fd, &value, sizeof(T)) == -1)
    {
        BOOST_THROW_EXCEPTION(PerfEventError() <<
                              boost::errinfo_errno(errno) <<
                              boost::errinfo_api_function("read"));
    }

}

//----------------------------------------------------------------------------//

inline void
ChildEvent::Start()
{
    Refresh();
}

inline void
ChildEvent::Stop()
{
    Disable();
}

inline void
ChildEvent::Refresh()
{
    if (ioctl(fd, PERF_EVENT_IOC_REFRESH, 1) == -1)
    {
        BOOST_THROW_EXCEPTION(PerfEventError() <<
                              boost::errinfo_errno(errno) <<
                              boost::errinfo_api_function("ioctl"));
    }
}

inline void
ChildEvent::Enable()
{
    if (ioctl(fd, PERF_EVENT_IOC_ENABLE, 1) == -1)
    {
        BOOST_THROW_EXCEPTION(PerfEventError() <<
                              boost::errinfo_errno(errno) <<
                              boost::errinfo_api_function("ioctl"));
    }
}

inline void
ChildEvent::Disable()
{
    if (ioctl(fd, PERF_EVENT_IOC_DISABLE, 1) == -1)
    {
        BOOST_THROW_EXCEPTION(PerfEventError() <<
                              boost::errinfo_errno(errno) <<
                              boost::errinfo_api_function("ioctl"));
    }
}

inline void
ChildEvent::Reset()
{
    if (ioctl(fd, PERF_EVENT_IOC_RESET, 1) == -1)
    {
        BOOST_THROW_EXCEPTION(PerfEventError() <<
                              boost::errinfo_errno(errno) <<
                              boost::errinfo_api_function("ioctl"));
    }
}



//----------------------------------------------------------------------------//



inline void
Event::AttachEvent(ChildEventPtr event)
{
    // Add the child event
    child_events.push_back(event);

    // Set the attribute to group
    attr.read_format = attr.read_format | PERF_FORMAT_GROUP;
}

//----------------------------------------------------------------------------//

inline uint64_t
Event::GetId() const
{
    return id;
}

inline int
Event::GetFd() const
{
    return fd;
}

inline size_t
Event::GetAvailableBytes() const
{
    return (data_head - data_tail);
}

inline void
Event::SetTID(int pid)
{
    this->tid = pid;
}

inline void
Event::SetWakeup(uint32_t wakeup)
{
    attr.wakeup_events = wakeup;
}

inline void
Event::SetPinned()
{
    attr.pinned = true;
}

inline void
Event::SetFrequency(uint64_t frequency)
{
    attr.freq = true;
    attr.sample_freq = frequency;
}

inline uint64_t
Event::GetSamplePeriod() const
{
    return attr.sample_period;
}

inline void
Event::SetSamplePeriod(uint64_t sample_period)
{
    attr.sample_period = sample_period;

    if (state == UNCONFIGURED)
    {
        attr.freq = false;
    }
    else
    {
        if (ioctl(fd, PERF_EVENT_IOC_PERIOD, &attr.sample_period) == -1)
        {
            BOOST_THROW_EXCEPTION(PerfEventError() <<
                                  boost::errinfo_errno(errno) <<
                                  boost::errinfo_api_function("ioctl"));
        }
    }
}

inline void
Event::SetPreciseIp(int precise_ip = 0)
{
    attr.precise_ip = precise_ip;
}

inline void
Event::SetKernalBufferSize(size_t size)
{
    kernal_buffer_size = size;
    page_mask = (kernal_buffer_size * page_size) - 1;
}

inline void
Event::SetExcludeUser(bool exclude)
{
    attr.exclude_user = exclude;
}

inline void
Event::SetExcludeKernel(bool exclude)
{
    attr.exclude_kernel = exclude;
}

inline void
Event::SetExcludeHyperVisor(bool exclude)
{
    attr.exclude_hv = exclude;
}

inline void
Event::SetExcludeIdle(bool exclude)
{
    attr.exclude_idle = exclude;
}

inline void
Event::EnableSigIO()
{
    enable_sigio = true;
}

inline void
Event::DisableSigIO()
{
    enable_sigio = false;
}

//--------------------------------------------------------------------------//

template<typename T>
inline T
Event::Peak()
{
    T value;
    Peak(value);
    return value;
}

template<typename T>
inline void
Event::Peak(T &value)
{

    // Position of tail within the buffer payload
    uint64_t tail = data_tail & page_mask;

    // Size of what is available
    // data_head, data_tail never wrap around  
    BOOST_ASSERT (sizeof(T) <= data_head - data_tail);

    // sz <= avail_sz, we can satisfy the request

    // c = size till end of buffer
    // buffer payload size is necessarily
    // a power of two, so we can do:
    // |           buffer             |
    //                      ^ tail     ^ page_mack + 1
    //                      |     c    |
    size_t c = page_mask + 1 - tail;

    // Min with requested size
    size_t m = std::min(c, sizeof(T));

    // Copy beginning
    std::memcpy((uint8_t*) &value, data + tail, m);

    // Copy wrapped around leftover
    if ((sizeof(T) - m) > 0)
    {
        std::memcpy(((uint8_t*) &value) + m, data, sizeof(T) - m);
    }

}

template<typename T>
inline T
Event::Read()
{
    T value;
    Read(value);
    return value;
}

template<typename T>
inline void
Event::Read(T &value)
{
    // Read value
    value = Peak<T>();

    // Update tail to tell kernel we have read from the buffer.
    data_tail += sizeof(T);
}

//--------------------------------------------------------------------------//

template<typename T>
inline void
Event::Skip()
{
    Skip(sizeof(T));
}

inline void
Event::Skip(size_t n)
{

    if ((data_tail + n) > data_head)
    {
        n = data_head - data_tail;
    }

    data_tail += n;
}

//--------------------------------------------------------------------------//

template<typename T>
inline T
Event::ReadNow()
{
    T value;
    ReadNow(value);
    return value;
}

template<typename T>
inline void
Event::ReadNow(T &value)
{
    // Read value
    if (read(fd, &value, sizeof(T)) == -1)
    {
        BOOST_THROW_EXCEPTION(PerfEventError() <<
                              boost::errinfo_errno(errno) <<
                              boost::errinfo_api_function("read"));
    }

}

//--------------------------------------------------------------------------//

inline void
Event::Start()
{
    (enable_sigio) ? Refresh() : Enable();
}

inline void
Event::Stop()
{
    Disable();
}

inline void
Event::Refresh()
{
    if (ioctl(fd, PERF_EVENT_IOC_REFRESH, attr.wakeup_events) == -1)
    {
        BOOST_THROW_EXCEPTION(PerfEventError() <<
                              boost::errinfo_errno(errno) <<
                              boost::errinfo_api_function("ioctl"));
    }
    state = STARTED;
}

inline void
Event::Enable()
{
    if (ioctl(fd, PERF_EVENT_IOC_ENABLE, 1) == -1)
    {
        BOOST_THROW_EXCEPTION(PerfEventError() <<
                              boost::errinfo_errno(errno) <<
                              boost::errinfo_api_function("ioctl"));
    }
    state = STARTED;
}

inline void
Event::Disable()
{
    if (ioctl(fd, PERF_EVENT_IOC_DISABLE, 1) == -1)
    {
        BOOST_THROW_EXCEPTION(PerfEventError() <<
                              boost::errinfo_errno(errno) <<
                              boost::errinfo_api_function("ioctl"));
    }
    state = STOPPED;
}

inline void
Event::Reset()
{
    if (ioctl(fd, PERF_EVENT_IOC_RESET, 1) == -1)
    {
        BOOST_THROW_EXCEPTION(PerfEventError() <<
                              boost::errinfo_errno(errno) <<
                              boost::errinfo_api_function("ioctl"));
    }
}

//----------------------------------------------------------------------------//

} /* namespace perf_event_wrapper */
} /* namespace util */
} /* namespace internal */
} /* namespace scarphase */

#endif /* __SCARPHASE_PERFEVENT_EVENT_HPP */
