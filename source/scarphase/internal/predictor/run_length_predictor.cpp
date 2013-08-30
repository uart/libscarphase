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

#include <iostream>
#include "scarphase/internal/predictor/run_length_predictor.hpp"

namespace scarphase {
namespace internal {
namespace predictor {


//----------------------------------------------------------------------------//

RunLengthPredictor::RunLengthPredictor(size_t table_size,
                                       size_t pattern_size,
                                       size_t confidence_threshold)
    : cache(table_size, CacheEntry(pattern_size)),
      pattern(pattern_size),
      confidence_threshold(confidence_threshold)
{

}

RunLengthPredictor::~RunLengthPredictor()
{
    // Do nothing
}

RunLengthPredictor::CacheEntry::CacheEntry(size_t pattern_size)
    : pattern(pattern_size),
      prediction(UNINITIALIZED_PHASE_ID)
{
    // Do nothing
}

//----------------------------------------------------------------------------//

size_t
RunLengthPredictor::HashPattern(const spi::util::PhaseTrackingBuffer &pattern)
{
    size_t hash = 0;
    for (size_t i = 0; i < pattern.size(); i++)
    {
        boost::hash_combine(hash, pattern[i].phase());
        boost::hash_combine(hash, pattern[i].length());
    }
    return hash;
}

//----------------------------------------------------------------------------//

void
RunLengthPredictor::Update(pid_t phase)
{
    //
    CacheEntry *e;

    // Get index
    size_t index = HashPattern(pattern);

    // Find the phase in the cache
    if ((e = cache.Find(index, pattern)) != NULL)
    {
        // Increament if we predicted correctly
        if (e->prediction == phase)
        {
            e->confidence++;
        }
        else
        {
            e->confidence--;
            e->prediction = phase;
        }
    }
    // The phase was not in the cache. Add it
    else
    {
        // Invalidate an old entry if we have a phase change
        if (pattern.phase(0) != phase)
        {
            e = cache.Invalidate(index);
            e->pattern = pattern;
            e->prediction = phase;
            e->confidence.Reset();
        }
    }

    // Add to current pattern
    pattern.push(phase);

    // Update
    last_value_predictor.Update(phase);
}

//----------------------------------------------------------------------------//

spi::util::Prediction
RunLengthPredictor::Predict()
{
    //
    CacheEntry *e;

    // Get index
    size_t index = HashPattern(pattern);

    if ((e = cache.Find(index, pattern)) != NULL)
    {
        if (e->prediction >= 0 &&
            e->confidence.Value() >= confidence_threshold)
        {
            return spi::util::Prediction(e->prediction, e->confidence.Value());
        }
    }

    return last_value_predictor.Predict();

}

//----------------------------------------------------------------------------//

} /* namespace predictor */
} /* namespace internal */
} /* namespace scarphase */
