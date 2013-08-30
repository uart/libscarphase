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

#ifndef __SCARPHASE_INTERNAL_UTIL_SATURATION_COUNTER_HPP
#define __SCARPHASE_INTERNAL_UTIL_SATURATION_COUNTER_HPP

#include <stdint.h>

namespace scarphase {
namespace internal {
namespace util {

/**
 * @brief Confidence counter.
 */
template<typename counter_type = uint8_t, int MIN = 0, int MAX = 100>
class SaturationCounter //--------------------------------------------------//
{

public: //------------------------------------------------------------------//

    /**
     * @brief C-tor.
     */
    SaturationCounter();

    /**
     * @brief Increment counter.
     */
    counter_type operator++(int);

    /**
     * @brief Decrement counter.
     */
    counter_type operator--(int);

    /**
     * @brief Reset the counter.
     */
    void Reset();

    /**
     * @brief Get minumim value.
     * @return Minimum possible value.
     */
    static int GetMinValue() { return MIN; }

    /**
     * @brief Get maximum value.
     * @return Maximum possible value.
     */
    static int GetMaxValue() { return MAX; }

    /**
     * @brief Get counter value.
     */
    const counter_type& Value() const;

private: //-----------------------------------------------------------------//

    counter_type counter;

};

} /* namespace util */
} /* namespace internal */
} /* namespace scarphase */

//---------------------------------------------------------------------------//
// Inline Definitions                                                        //
//---------------------------------------------------------------------------//

#include <algorithm>
#include <boost/assert.hpp>
#include <boost/static_assert.hpp>

namespace scarphase {
namespace internal {
namespace util {

template<typename counter_type, int MIN, int MAX>
SaturationCounter<counter_type, MIN, MAX>::SaturationCounter()
    : counter(MIN)
{
    BOOST_STATIC_ASSERT_MSG((MIN < MAX), "Invalid range.");

    // Do nothing
}

template<typename counter_type, int MIN, int MAX>
counter_type SaturationCounter<counter_type, MIN, MAX>::operator++(int)
{
    counter = std::min(counter + 1, MAX);
    return counter;
}

template<typename counter_type, int MIN, int MAX>
inline counter_type
SaturationCounter<counter_type, MIN, MAX>::operator--(int)
{
    counter = std::max(counter - 1, MIN);
    return counter;
}

template<typename counter_type, int MIN, int MAX>
inline void
SaturationCounter<counter_type, MIN, MAX>::Reset()
{
    counter = MIN;
}

template<typename counter_type, int MIN, int MAX>
inline const counter_type&
SaturationCounter<counter_type, MIN, MAX>::Value() const
{
    return counter;
}

} /* namespace util */
} /* namespace internal */
} /* namespace scarphase */


#endif /* __SCARPHASE_INTERNAL_UTIL_SATURATION_COUNTER_HPP */
