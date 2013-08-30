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

#include "scarphase/internal/predictor/last_value_predictor.hpp"

namespace scarphase {
namespace internal {
namespace predictor {

//----------------------------------------------------------------------------//

LastValuePredictor::LastValuePredictor()
    : last_phase(UNINITIALIZED_PHASE_ID),
      cache(CacheEntry(UNINITIALIZED_PHASE_ID))
{
    // Do nothing
}

LastValuePredictor::~LastValuePredictor()
{
    // Do nothing
}

LastValuePredictor::CacheEntry::CacheEntry(int phase)
    : phase(phase)
{
    // Do nothing
}

//----------------------------------------------------------------------------//

void
LastValuePredictor::Update(pid_t phase)
{
    CacheEntry *e;

    // Get index
    pid_t index = last_phase;

    // Find the phase in the cache
    if ((e = cache.Find(index, last_phase)) != NULL)
    {
        // Decreament if phase change, otherwise
        (last_phase != phase) ? e->confidence-- : e->confidence++;
    }
    // The phase was not in the cache. Add it
    else
    {
        e = cache.Invalidate(last_phase);
        e->phase = last_phase;
        e->confidence.Reset();
    }

    // Remember last phase/value
    last_phase = phase;

}

//----------------------------------------------------------------------------//

spi::util::Prediction
LastValuePredictor::Predict()
{
    CacheEntry *e;

    // Get index
    pid_t index = last_phase;

    // Find the phase in the cache
    if ((e = cache.Find(index, last_phase)) == NULL)
    {
        e = cache.Invalidate(last_phase);
        e->phase = last_phase;
        e->confidence.Reset();
    }

    // Pick the last phase
    return spi::util::Prediction(e->phase, e->confidence.Value());
}

//----------------------------------------------------------------------------//

} /* namespace predictor */
} /* namespace internal */
} /* namespace scarphase */
