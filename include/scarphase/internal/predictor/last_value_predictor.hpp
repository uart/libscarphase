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

#ifndef __SCARPHASE_PREDICTOR_LAST_VALUE_PREDICTOR_HPP
#define __SCARPHASE_PREDICTOR_LAST_VALUE_PREDICTOR_HPP

#include <scarphase/internal/util/cache.hpp>
#include <scarphase/internal/util/phase_tracking_buffer.hpp>

#include <scarphase/internal/predictor/abstract_predictor.hpp>

namespace scarphase {
namespace internal {
namespace predictor {

/**
 * @brief Last value predictor. It will always predict that the next interval
 *        will belong to the same phase as the previous interval.
 */
class LastValuePredictor: public virtual AbstractPredictor //-----------------//
{

public: //--------------------------------------------------------------------//

    /**
     * @brief C-tor
     */
    LastValuePredictor();

    /**
     * @brief D-tor
     */
    virtual ~LastValuePredictor();

    /**
     * @brief Predict the next phase.
     */
    virtual void Update(pid_t phase);

    /**
     * @brief Predict the next phase.
     */
    virtual spi::util::Prediction Predict();

protected: //-----------------------------------------------------------------//

    struct CacheEntry
    {

        /**
         * @brief C-tor
         */
        CacheEntry(pid_t phase = UNINITIALIZED_PHASE_ID);

        /**
         * @brief Compare a cache entry.
         */
        bool operator==(pid_t phase) const;

        /**
         * @brief The phase id.
         */
        int phase;

        /**
         * @brief How confident we are.
         */
        confidence_counter_t confidence;

    };

    /**
     * @brief The last value/phase.
     */
    pid_t last_phase;

    /**
     * @brief The cache
     */
    spi::util::StaticCache<CacheEntry, 128> cache;

};

} /* namespace predictor */
} /* namespace internal */
} /* namespace scarphase */

//----------------------------------------------------------------------------//
// Inline Definitions                                                         //
//----------------------------------------------------------------------------//

namespace scarphase {
namespace internal {
namespace predictor {

inline bool
LastValuePredictor::CacheEntry::operator==(int phase) const
{
    return (this->phase == phase);
}

} /* namespace predictor */
} /* namespace internal */
} /* namespace scarphase */

//----------------------------------------------------------------------------//

#endif /* __SCARPHASE_PREDICTOR_LAST_VALUE_PREDICTOR_HPP */
