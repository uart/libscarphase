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


#ifndef __SCARPHASE_INTERNAL_CLASSIFIER_CLASSIFIER_HPP
#define __SCARPHASE_INTERNAL_CLASSIFIER_CLASSIFIER_HPP

#include <scarphase/internal/util/phase.hpp>
#include <scarphase/internal/util/signature.hpp>

namespace scarphase {
namespace internal {
namespace classifier {

namespace spi = scarphase::internal;

/**
 *
 */
struct Classifier //----------------------------------------------------------//
{

    /**
     * Classify signature.
     * @param[in]   signature The signature to classify.
     * @param[in]   sample    Performance feedback.
     * @returns The phase id.
     */
    virtual pid_t
    ClassifySignature(const spi::util::Signature &signature) = 0;

    /**
     * Find closest phase/cluster.
     * @param[in]   signature The signature to search for.
     * @param[in]   threshold The maximum distance to the phase.
     * @returns The closest phase if the distance is below the threshold
     *          otherwise -1.
     */
    virtual pid_t
    FindPhase(const spi::util::Signature &signature, double threshold) = 0;

};

} /* namespace classifier */
} /* namespace internal */
} /* namespace scarphase */

#endif /* __SCARPHASE_INTERNAL_CLASSIFIER_CLASSIFIER_HPP */
