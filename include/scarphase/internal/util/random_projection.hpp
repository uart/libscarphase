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

#ifndef __SCARPHASE_INTERNAL_UTIL_RANDOM_PROJECTION_HPP
#define __SCARPHASE_INTERNAL_UTIL_RANDOM_PROJECTION_HPP

#include <stdint.h>
#include <boost/array.hpp>

namespace scarphase {
namespace internal {
namespace util {

/**
 * @brief Random projection to reduce the dimentioality of a large vector
 *        space.
 *
 *  V[1,n] * M[n,m] = V[1,m]
 *
 */
template<typename value_type,
         int      OUTPUT_SIZE>
class RandomProjection //-----------------------------------------------------//
{

public: //--------------------------------------------------------------------//

    /**
     * @brief C-tor.
     */
    RandomProjection();

    /**
     * @brief Hash a value using random projection.
     *        y = x * M, where y << x
     *
     * @param[in]  x Value to hash.
     * @returns A hash of x.
     *
     */
    value_type Hash(value_type x);

private: //-------------------------------------------------------------------//

    /**
     * @brief The size of the input variable.
     */
    #define INPUT_SIZE int(sizeof(value_type) * 8)

    /**
     * @brief Random projection matrix.
     */
    boost::array< boost::array< uint8_t, INPUT_SIZE >, OUTPUT_SIZE > m;

};

} /* namespace util */
} /* namespace internal */
} /* namespace scarphase */

//----------------------------------------------------------------------------//
// Inline Definitions                                                         //
//----------------------------------------------------------------------------//

#include <cstdlib>
#include <boost/random.hpp>
#include <boost/static_assert.hpp>

namespace scarphase {
namespace internal {
namespace util {

template<typename value_type,
         int      OUTPUT_SIZE>
RandomProjection<value_type, OUTPUT_SIZE>::RandomProjection()
{
    BOOST_STATIC_ASSERT(OUTPUT_SIZE > 0);

    boost::mt19937 rng;
    boost::uniform_int<> bin(0,1);
    boost::variate_generator< boost::mt19937&,
                              boost::uniform_int<> > die(rng, bin);

    for (int i = 0; i < OUTPUT_SIZE; i++)
    {
        for (int j = 0; j < INPUT_SIZE; j++)
        {
            m[i][j] = die(); //std::rand() % 2;
        }
    }

}

template<typename value_type,
         int      OUTPUT_SIZE>
inline value_type
RandomProjection<value_type, OUTPUT_SIZE>::Hash(value_type x)
{
    value_type result = 0;
    for (int i = 0; i < OUTPUT_SIZE; i++)
    {
        int t = 0;
        for (int j = 0; j < INPUT_SIZE; j++)
        {
            t += m[i][j] & ((x >> j) & 0x01);
        }
        result |= (t % 2) << i;
    }
    return result;
}

} /* namespace util */
} /* namespace internal */
} /* namespace scarphase */

#endif /* __SCARPHASE_INTERNAL_UTIL_RANDOM_PROJECTION_HPP */
