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

#ifndef __SCARPHASE_INTERNAL_UTIL_PHASE_TRACKING_BUFFER_HPP
#define __SCARPHASE_INTERNAL_UTIL_PHASE_TRACKING_BUFFER_HPP

#include <boost/circular_buffer.hpp>

#include <scarphase/internal/util/phase.hpp>
#include <scarphase/internal/util/run_length.hpp>

/**
 * @brief The size of the buffer. Should be large enough to hold all the
 *        pattern the predictor might look for.
 */
#define BUFFER_SIZE             20

namespace scarphase {
namespace internal {
namespace util {

namespace spi = scarphase::internal;

/**
 * @brief Keep track of the phases.
 */
class PhaseTrackingBuffer //--------------------------------------------------//
{

public: //--------------------------------------------------------------------//

    typedef boost::circular_buffer<spi::util::RunLength> buffer_t;

    /**
     * @brief C-tor.
     */
    PhaseTrackingBuffer(size_t size = BUFFER_SIZE);

    /**
     * @brief Find phase time intervals ago.
     */
    pid_t phase(int time) const;

    /**
     * @brief Get buffer size.
     */
    size_t size() const;

    /**
     * @brief Get length in number of windows.
     */
    size_t length() const;

    /**
     * @brief Get buffer.
     */
    const buffer_t& buffer() const;

    /**
     * @brief Update the buffer with the phase id of the previous interval.
     */
    void push(pid_t phase);

    /**
     * @brief Get run length at index.
     */
    spi::util::RunLength operator[](size_t index) const;

    /**
     * @brief Compare two buffers.
     */
    bool operator==(const PhaseTrackingBuffer &other) const;


private: //-------------------------------------------------------------------//

    /**
     * @brief Phase buffer.
     */
    buffer_t buffer_;

};

} /* namespace util */
} /* namespace internal */
} /* namespace scarphase */

//----------------------------------------------------------------------------//
// Inline Definitions                                                         //
//----------------------------------------------------------------------------//

namespace scarphase {
namespace internal {
namespace util {

inline
PhaseTrackingBuffer::PhaseTrackingBuffer(size_t size)
    : buffer_(size)
{
    BOOST_ASSERT_MSG(size > 0, "Invalid size.");
}

//----------------------------------------------------------------------------//

inline int
PhaseTrackingBuffer::phase(int time) const
{
    for (size_t i = 0; i < buffer_.size(); i++)
    {
        time -= buffer_[i].length();

        // If we arrived at right run length
        if (time < 0)
        {
            return buffer_[i].phase();
        }
    }

    return UNKNOWN_PHASE_ID;
}

inline size_t
PhaseTrackingBuffer::size() const
{
    return buffer_.size();
}

inline size_t
PhaseTrackingBuffer::length() const
{
    size_t length = 0;
    for (size_t i = 0; i < buffer_.size(); i++)
    {
        length += buffer_[i].length();
    }
    return length;
}

//----------------------------------------------------------------------------//

inline void
PhaseTrackingBuffer::push(int phase)
{
    if (not buffer_.empty() && buffer_[0].phase() == phase)
    {
        buffer_[0]++;
    }
    // If we had a phase change, reset the run length
    else
    {
        buffer_.push_front(spi::util::RunLength(phase, 1));
    }
}

//----------------------------------------------------------------------------//

inline spi::util::RunLength
PhaseTrackingBuffer::operator[](size_t index) const
{
    return buffer_[index];
}

inline bool
PhaseTrackingBuffer::operator==(const PhaseTrackingBuffer &other) const
{
    if (size() != other.size()) return false;

    for (size_t i = 0; i < buffer_.size(); i++)
    {
        if (buffer_[i] != other.buffer_[i])
        {
            return false;
        }
    }

    return true;
}

//----------------------------------------------------------------------------//

inline const PhaseTrackingBuffer::buffer_t&
PhaseTrackingBuffer::buffer() const
{
    return buffer_;
}

//----------------------------------------------------------------------------//

} /* namespace util */
} /* namespace internal */
} /* namespace scarphase */


#endif /* __SCARPHASE_INTERNAL_UTIL_PHASE_TRACKING_BUFFER_HPP */
