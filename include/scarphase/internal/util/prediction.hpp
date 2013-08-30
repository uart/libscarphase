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


#ifndef __SCARPHASE_INTERNAL_UTIL_PREDICTION_HPP
#define __SCARPHASE_INTERNAL_UTIL_PREDICTION_HPP

#include <cstddef>
#include <scarphase/internal/util/phase.hpp>

namespace scarphase {
namespace internal {
namespace util {

/**
 *
 */
class Prediction //-----------------------------------------------------------//
{

public: //--------------------------------------------------------------------//

    /**
     * @brief C-tor.
     */
    Prediction(pid_t phase, size_t confidence = 0);

    /**
     * @brief
     */
    pid_t phase() const;

    /**
     * @brief
     */
    size_t confidence() const;

private: //-------------------------------------------------------------------//

    /**
     * @brief The next phase.
     */
    pid_t phase_;

    /**
     * @brief How confident we are in our prediction.
     */
    size_t confidence_;

};

} /* namespace predictor */
} /* namespace internal */
} /* namespace scarphase */

//----------------------------------------------------------------------------//
// Inline Definitions                                                         //
//----------------------------------------------------------------------------//

namespace scarphase {
namespace internal {
namespace util {

//----------------------------------------------------------------------------//

inline
Prediction::Prediction(pid_t phase, size_t confidence)
    : phase_(phase),
      confidence_(confidence)
{
    // Do nothing
}

//----------------------------------------------------------------------------//

inline int
Prediction::phase() const
{
    return phase_;
}

inline size_t
Prediction::confidence() const
{
    return confidence_;
}

//----------------------------------------------------------------------------//

} /* namespace util */
} /* namespace internal */
} /* namespace scarphase */

//---------------------------------------------------------------------------//



#endif /* __SCARPHASE_INTERNAL_UTIL_PREDICTION_HPP */
