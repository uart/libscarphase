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

#ifndef __SCARPHASE_INTERNAL_UTIL_RUN_LENGTH_HPP
#define __SCARPHASE_INTERNAL_UTIL_RUN_LENGTH_HPP

#include <scarphase/internal/util/phase.hpp>

namespace scarphase {
namespace internal {
namespace util {

/**
 * @brief Class to describe a run length. How many intervals a phase have
 *        been in.
 */
class RunLength //------------------------------------------------------------//
{

public: //--------------------------------------------------------------------//

    /**
     * @brief C-tor.
     */
    RunLength(pid_t phase = UNINITIALIZED_PHASE_ID, int length = 1);

    /**
     * @brief Get phase.
     * @return phase
     */
    pid_t phase() const;

    /**
     * @brief Get length.
     * @return length
     */
    int length() const;

    /**
     * @brief Update the runlength.
     */
    void Update(pid_t phase, int length);

    /**
     * @brief operator ++
     * @return
     */
    RunLength& operator++();

    /**
     * @brief operator ++
     * @return
     */
    RunLength operator++(int);

    /**
     * @brief Compare two runlengths.
     */
    bool operator==(const RunLength &rl) const;

    /**
     * @brief Compare two runlengths.
     */
    bool operator!=(const RunLength &rl) const;

private: //-------------------------------------------------------------------//

    /**
     * @brief The "current" phase.
     */
    pid_t phase_;

    /**
     * @brief The number of intervals the phase has been in.
     */
    int length_;

}; /* class RunLength */

} /* namespace util */
} /* namespace internal */
} /* namespace scarphase */

//----------------------------------------------------------------------------//
// Inline Definitions                                                         //
//----------------------------------------------------------------------------//

#include <boost/assert.hpp>

namespace scarphase {
namespace internal {
namespace util {

inline
RunLength::RunLength(pid_t phase, int length)
    : phase_(phase),
      length_(length)
{
    BOOST_ASSERT(length > 0);
}

//----------------------------------------------------------------------------//

inline int
RunLength::phase() const
{
    return phase_;
}

inline int
RunLength::length() const
{
    return length_;
}

inline void
RunLength::Update(pid_t phase, int length)
{
    BOOST_ASSERT(length > 0);

    this->phase_  = phase;
    this->length_ = length;
}

//----------------------------------------------------------------------------//

inline RunLength&
RunLength::operator++()
{
    length_++;
    return *this;
}

inline RunLength
RunLength::operator++(int)
{
    RunLength t = *this;
    ++*this;
    return t;
}

inline bool
RunLength::operator==(const RunLength &rl) const
{
    return (phase_ == rl.phase_ && length_ == rl.length_);
}

inline bool
RunLength::operator!=(const RunLength &rl) const
{
    return !(*this == rl);
}

//----------------------------------------------------------------------------//

} /* namespace util */
} /* namespace internal */
} /* namespace scarphase */

#endif /* __SCARPHASE_INTERNAL_UTIL_RUN_LENGTH_HPP */
