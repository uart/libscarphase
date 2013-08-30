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

#include "scarphase/internal/util/signature.hpp"

namespace scarphase {
namespace internal {
namespace util {

//----------------------------------------------------------------------------//

Signature::Signature(size_t uv_size, value_type uv_weight)
    : fv(),
      uv(uv_size, value_type(0)),
      uv_weight(uv_weight)
{
    Reset();
}

Signature::Signature(const value_type *fv, size_t fv_size,
                     const value_type *uv, size_t uv_size,
                     value_type uv_weight)
    : uv(uv_size, value_type(0)),
      uv_weight(uv_weight)
{
    BOOST_STATIC_ASSERT(SIGNATURE_SIZE > 0);

    BOOST_ASSERT(fv != NULL);
    BOOST_ASSERT(fv_size == SIGNATURE_SIZE);

    std::copy(fv, fv + SIGNATURE_SIZE, this->fv.begin());

    if (uv != NULL && uv_size > 0)
    {
        std::copy(uv, uv + uv_size, this->uv.begin());
    }

}

Signature::~Signature()
{
    // Do nothing
}

//----------------------------------------------------------------------------//

void
Signature::Reset()
{
    std::fill(fv.begin(), fv.end(), value_type(0));
    std::fill(uv.begin(), uv.end(), value_type(0));
}

//----------------------------------------------------------------------------//

template<typename vector_t>
Signature::value_type
ManhattenDistance(const vector_t &v0, const vector_t &v1)
{
    BOOST_ASSERT(v0.size() == v1.size());

    Signature::value_type distance = 0;

    // e.g. d(p,q) = |p1 - q1| + |p2 - q2|
    for (size_t i = 0; i < v0.size(); i++)
    {
        distance += std::abs(v0[i] - v1[i]);
    }

    return distance;
}

Signature::value_type
Signature::ManhattanDistance(const Signature &signature) const
{
    Signature::value_type fvd = ManhattenDistance(fv, signature.fv);
    Signature::value_type uvd = ManhattenDistance(uv, signature.uv);

    return (1.0 - uv_weight) * fvd + uv_weight * uvd;
}

//----------------------------------------------------------------------------//

template<typename vector_t>
void
NormalizeVector(vector_t &v)
{
    // Calculate the magnitude
    Signature::value_type m = std::accumulate(
        v.begin(), v.end(),
        Signature::value_type(0),
        std::plus<Signature::value_type>());

    // If zero, just make sure the manitude is 1.
    if (m < 1.0)
    {
        std::fill(v.begin(), v.end(), 1.0 / SIGNATURE_SIZE);
    }
    // Normalize the vector
    else
    {
        for (size_t i = 0; i < SIGNATURE_SIZE; i++)
        {
            v[i] /= m;
        }
    }
}

void
Signature::Normalize()
{
    NormalizeVector(fv);
    NormalizeVector(uv);
}

//----------------------------------------------------------------------------//

template<typename vector_t>
void MergeVector(vector_t &v0, const vector_t &v1, double weight)
{
    BOOST_ASSERT(v0.size() == v1.size());

    // c = c + (1/count) * (x - c)
    for (size_t i = 0; i < v0.size(); i++)
    {
        v0[i] = v0[i] + (v1[i] - v0[i]) * weight;
    }

}

void
Signature::Merge(const Signature &other, double weight)
{
    MergeVector(fv, other.fv, weight);
    MergeVector(uv, other.uv, weight);
}

//----------------------------------------------------------------------------//

} /* namespace util */
} /* namespace internal */
} /* namespace scarphase */
