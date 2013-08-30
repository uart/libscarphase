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

#ifndef __SCARPHASE_UTIL_SIGNATURE_HPP
#define __SCARPHASE_UTIL_SIGNATURE_HPP

#include <vector>
#include <stdint.h>
#include <boost/array.hpp>

/**
 * @brief The Sherwood et al. showed that 32 entry big frequnce vector
 *        works well.
 */
#define SIGNATURE_SIZE          32

namespace scarphase {
namespace internal {
namespace util {

/**
 * @brief Signature class.
 */
class Signature //------------------------------------------------------------//
{

public: //--------------------------------------------------------------------//

    /**
     * @brief
     */
    typedef double value_type;

    /**
     * @brief The underlying vector.
     */
    typedef boost::array<value_type, SIGNATURE_SIZE> frequency_vector_t;

    /**
     * @brief The underlying vector.
     */
    typedef std::vector<value_type> user_vector_t;

    /**
     * @brief C-tor
     */
    Signature(size_t uv_size = 0, value_type uv_weight = 0);

    /**
     * @brief C-tor
     */
    Signature(const value_type *fv, size_t fv_size,
              const value_type *uv = NULL, size_t uv_size = 0,
              value_type uv_weight = 0);

    /**
     * @brief D-tor
     */
    ~Signature();

    /**
     * @brief Get signature size.
     * @return size
     */
    size_t size() const;

    /**
     * @brief Get signature size.
     * @return size
     */
    size_t fv_size() const;

    /**
     * @brief Get signature size.
     * @return size
     */
    size_t uv_size() const;

    /**
     * @brief Get the underlying vector.
     */
    const value_type *fv_data() const;

    /**
     * @brief Get the underlying vector.
     */
    const value_type *uv_data() const;

    /**
     * @brief
     */
    value_type& operator[] (const uint64_t index);

    /**
     * @brief
     */
    const value_type& operator[] (const uint64_t index) const;

    /**
     * @brief Reset the signature for a new interval.
     */
    void Reset();

    /**
     * @brief Normalize the vector.
     * @warning Should only be called once.
     */
    void Normalize();

    /**
     * @brief The manhattan distance between two signatures.
     */
    value_type ManhattanDistance(const Signature &signature) const;

    /**
     * @brief Merge together.
     */
    void Merge(const Signature &other, double weight = 0.5);

private: //-------------------------------------------------------------------//

    /**
     * @brief Frequency vector.
     */
    frequency_vector_t fv;

    /**
     * @brief User vector.
     */
    user_vector_t uv;

    /**
     * @brief The weight of the user vector.
     */
    double uv_weight;

}; /* class Signature */

} /* namespace util */
} /* namespace internal */
} /* namespace scarphase */


//----------------------------------------------------------------------------//
// Inline Definitions                                                         //
//----------------------------------------------------------------------------//

#include <cmath>
#include <numeric>
#include <algorithm>

#include <boost/assert.hpp>
#include <boost/static_assert.hpp>

namespace scarphase {
namespace internal {
namespace util {

//----------------------------------------------------------------------------//

inline size_t
Signature::size() const
{
    return fv_size() + uv_size();
}

inline size_t
Signature::fv_size() const
{
    return SIGNATURE_SIZE;
}

inline size_t
Signature::uv_size() const
{
    return uv.size();
}

//----------------------------------------------------------------------------//

/**
 * @brief Get the underlying vector.
 */
inline const Signature::value_type*
Signature::fv_data() const
{
    return fv.data();
}

/**
 * @brief Get the underlying vector.
 */
inline const Signature::value_type*
Signature::uv_data() const
{
    return uv.data();
}

//----------------------------------------------------------------------------//

inline Signature::value_type&
Signature::operator[] (const uint64_t index)
{
    BOOST_ASSERT(index < SIGNATURE_SIZE);

    return fv[index];
}


inline const Signature::value_type&
Signature::operator[] (const uint64_t index) const
{
    BOOST_ASSERT(index < SIGNATURE_SIZE);

    return fv[index];
}

//----------------------------------------------------------------------------//

} /* namespace util */
} /* namespace internal */
} /* namespace scarphase */

#endif /* __SCARPHASE_UTIL_SIGNATURE_HPP */
