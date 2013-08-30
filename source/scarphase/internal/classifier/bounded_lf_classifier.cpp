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

#include <limits>

#include "scarphase/internal/classifier/bounded_lf_classifier.hpp"

namespace scarphase {
namespace internal {
namespace classifier {

//----------------------------------------------------------------------------//

BoundedLeaderFollowerClassifier::BoundedLeaderFollowerClassifier(
        double similarity_threshold, unsigned int transition_threshold)
    : LeaderFollowerClassifier(similarity_threshold, transition_threshold),
      next_age(std::numeric_limits<unsigned int>::max())
{
    // Do nothing
}

BoundedLeaderFollowerClassifier::~BoundedLeaderFollowerClassifier()
{
    // Do nothing
}

BoundedLeaderFollowerClassifier::Cluster::Cluster()
    : LeaderFollowerClassifier::Cluster(spi::util::Signature()),
      age(std::numeric_limits<unsigned int>::max())
{
    // Do nothing
}

//----------------------------------------------------------------------------//

int
BoundedLeaderFollowerClassifier::ClassifySignature(
    const spi::util::Signature& signature)
{

    //
    int min_i = -1, oldest_i = 0;

    spi::util::Signature::value_type d;
    spi::util::Signature::value_type min_d =
        std::numeric_limits<spi::util::Signature::value_type>::max();

    // Check for similar signatures
    for (size_t i = 0; i < clusters.size(); i++)
    {
        // Select the one with the closest similarity
        d = signature.ManhattanDistance(clusters[i].signature);
        if (clusters[i].count && d < min_d)
        {
            min_d = d;
            min_i = i;
        }

        // Find the least recently used entry
        if (clusters[oldest_i].age < clusters[i].age)
        {
            oldest_i = i;
        }
    }

    // If we found an interval that is similar enough
    if (min_i != -1 && min_d < similarity_threshold)
    {

        // Update cluster signature/center
        clusters[min_i].signature.Merge(signature, 1.0 / clusters[min_i].count);

        // Increament the count
        clusters[min_i].count++;

        // Update the age
        clusters[min_i].age = --next_age;

        // If the count is above the treshold, change it to a stable
        // signature and give it a new phaseid
        if (clusters[min_i].phase == TRANSITION_PHASE_ID &&
            clusters[min_i].count >= transition_threshold)
        {
            clusters[min_i].phase = next_phase_id;

            // Generate the next phase id
            next_phase_id++;
        }

        return clusters[min_i].phase;

    }
    // Else replase the oldest entry with the new signature
    else
    {
        // Copy the signature
        clusters[oldest_i].signature = signature;

        // Increament the count
        clusters[oldest_i].count = 1;

        // Update age
        clusters[oldest_i].age   = --next_age;

        // If the count is above the treshold, change it to a stable
        // signature and give it a new phaseid
        if (clusters[oldest_i].count >= transition_threshold)
        {
            clusters[oldest_i].phase = next_phase_id;

            // Generate the next phase id
            next_phase_id++;
        }
        else
        {
            clusters[oldest_i].phase = TRANSITION_PHASE_ID;
        }

        return clusters[oldest_i].phase;

    }

}

//----------------------------------------------------------------------------//

int
BoundedLeaderFollowerClassifier::FindPhase(
    const spi::util::Signature &signature, double threshold)
{
    // Find nearest cluster
    int idx = FindNearestCluster(clusters, signature, threshold);

    // Return phase id
    return (idx >= 0) ? clusters[idx].phase : -1;
}

} /* namespace classifier */
} /* namespace internal */
} /* namespace scarphase */
