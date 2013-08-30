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

#ifndef __SCARPHASE_PREDICTOR_RUN_LENGTH_PREDICTOR_HPP
#define __SCARPHASE_PREDICTOR_RUN_LENGTH_PREDICTOR_HPP

#include <boost/functional/hash.hpp>

#include <scarphase/internal/util/phase_tracking_buffer.hpp>

#include <scarphase/internal/predictor/abstract_predictor.hpp>
#include <scarphase/internal/predictor/last_value_predictor.hpp>

namespace scarphase {
namespace internal {
namespace predictor {

/**
 * @brief
 */
class RunLengthPredictor: public virtual AbstractPredictor //-----------------//
{

public: //--------------------------------------------------------------------//

    /**
     * @brief C-tor
     */
    RunLengthPredictor(size_t table_size,
                       size_t pattern_size,
                       size_t confidence_threshold = 0);

    /**
     * @brief D-tor
     */
    virtual ~RunLengthPredictor();

    /**
     * @brief Predict the next phase.
     */
    virtual void Update(pid_t phase);

    /**
     * @brief Predict the next phase.
     */
    virtual spi::util::Prediction Predict();


protected: //-----------------------------------------------------------------//

    typedef spi::util::PhaseTrackingBuffer pattern_t;

    /**
     * @brief Entry in the loockup table.
     */
    struct CacheEntry
    {

        /**
         * @brief C-tor
         */
        CacheEntry(size_t pattern_size);

        /**
         * @brief Compare
         */
        bool operator==(const pattern_t &pattern) const;

        /**
         * @brief The pattern.
         */
        pattern_t pattern;

        /**
         * @brief The next phase prediction.
         */
        pid_t prediction;

        /**
         * @brief Confidence counter
         */
        confidence_counter_t confidence;

    };

    /**
     * @brief The lookup table.
     */
    spi::util::DynamicCache<CacheEntry> cache;

    /**
     * @brief The pattern
     */
    pattern_t pattern;

    /**
     * @brief Confidence threshold.
     */
    size_t confidence_threshold;

    /**
     * @brief Last value predictor for when markov failes.
     */
    LastValuePredictor last_value_predictor;

protected: //-----------------------------------------------------------------//

    /**
     * @brief Create a hash of the pattern.
     */
    static size_t HashPattern(const spi::util::PhaseTrackingBuffer& pattern);

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
RunLengthPredictor::CacheEntry::operator==(const pattern_t &pattern) const
{
    return (this->pattern == pattern);
}

} /* namespace predictor */
} /* namespace internal */
} /* namespace scarphase */

//----------------------------------------------------------------------------//


#endif /* __SCARPHASE_PREDICTOR_RUN_LENGTH_PREDICTOR_HPP */
