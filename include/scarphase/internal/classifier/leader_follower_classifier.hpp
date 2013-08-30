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


#ifndef __SCARPHASE_INTERNAL_CLASSIFIER_LF_CLASSIFIER_HPP
#define __SCARPHASE_INTERNAL_CLASSIFIER_LF_CLASSIFIER_HPP

#include <stdint.h>
#include <scarphase/internal/classifier/classifier.hpp>

namespace scarphase {
namespace internal {
namespace classifier {

/**
 * @brief Online LF classifier that all other LF classifiers inherit.
 */
class LeaderFollowerClassifier : public virtual Classifier //-----------------//
{  

protected: //-----------------------------------------------------------------//

    /**
     * @brief C-tor
     */
    LeaderFollowerClassifier(double similarity_threshold,
                             unsigned int transition_threshold);

    /**
     * @see Classifier
     */
    virtual pid_t
    ClassifySignature(const spi::util::Signature &signature) = 0;

    /**
     * @see Classifier
     */
    virtual pid_t
    FindPhase(const spi::util::Signature &signature, double threshold) = 0;

protected: //-----------------------------------------------------------------//

    /**
     * @brief
     */
    struct Cluster
    {

        /**
         * @brief C-tor
         */
        Cluster(const spi::util::Signature &signature);

        /**
         * @brief The phase id.
         */
        pid_t phase;

        /**
         * @brief The number of interval this siganture has been in.
         */
        size_t count;

        /**
         * @brief The signature/center.
         */
        spi::util::Signature signature;

    };

protected: //-----------------------------------------------------------------//

    /**
     * @brief The next phase id. Incrementet for each new phase.
     */
    pid_t next_phase_id;

    /**
     * @brief Similarity threshold. Used to match two signatures.
     */
    double similarity_threshold;

    /**
     * @brief Transition threshold.
     */
    unsigned int transition_threshold;

protected: //-----------------------------------------------------------------//

    /**
     * @brief
     */
    template<typename cluster_list_t>
    int FindNearestCluster(
        const cluster_list_t &clusters,
        const spi::util::Signature &signature,
        double similarity_threshold);

};

} /* namespace classifier */
} /* namespace internal */
} /* namespace scarphase */

//----------------------------------------------------------------------------//
// Inline Definitions                                                         //
//----------------------------------------------------------------------------//

#include <limits>

namespace scarphase {
namespace internal {
namespace classifier {

template<typename cluster_list_t>
int
LeaderFollowerClassifier::FindNearestCluster(
    const cluster_list_t &clusters,
    const spi::util::Signature &signature,
    double similarity_threshold)
{
    // [0..1] => [0..2]
    similarity_threshold = similarity_threshold * 2.0;

    //
    int min_i = 0;

    //
    spi::util::Signature::value_type d;
    spi::util::Signature::value_type min_d =
        std::numeric_limits<spi::util::Signature::value_type>::max();

    // Check for similar signatures
    for (size_t i = 0; i < clusters.size(); i++)
    {
        // Select the one with the closest similarity
        d = signature.ManhattanDistance(clusters[i].signature);
        if (d < min_d)
        {
            min_d = d;
            min_i = i;
        }
    }

    // If we found an interval that is similar enough
    return (min_d < similarity_threshold) ? min_i : -1;
}

} /* namespace classifier */
} /* namespace internal */
} /* namespace scarphase */

//----------------------------------------------------------------------------//

#endif /* __SCARPHASE_INTERNAL_CLASSIFIER_LF_CLASSIFIER_HPP */
