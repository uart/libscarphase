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


#include "scarphase/internal/classifier/unbounded_lf_classifier.hpp"

namespace scarphase {
namespace internal {
namespace classifier {

//----------------------------------------------------------------------------//

UnboundedLeaderFollowerClassifier::UnboundedLeaderFollowerClassifier(
        double similarity_threshold, unsigned int transition_threshold)
    : LeaderFollowerClassifier(similarity_threshold, transition_threshold)
{
    // Do nothing
}

UnboundedLeaderFollowerClassifier::~UnboundedLeaderFollowerClassifier()
{
    // Do nothing
}

//----------------------------------------------------------------------------//

int
UnboundedLeaderFollowerClassifier::ClassifySignature(
    const spi::util::Signature& signature)
{

    //
    int idx = FindNearestCluster(clusters, signature, similarity_threshold);

    // If we found an interval that is similar enough
    if (idx != -1)
    {

        // Update cluster signature/center
        clusters[idx].signature.Merge(signature, 1.0 / clusters[idx].count);

        // Increment count
        clusters[idx].count++;

        // If the count is above the treshold, change it to a stable
        // signature and give it a new phaseid
        if (clusters[idx].phase == TRANSITION_PHASE_ID &&
            clusters[idx].count >= transition_threshold)
        {
            clusters[idx].phase = next_phase_id;

            // Generate the next phase id
            next_phase_id++;
        }

        return clusters[idx].phase;

    }
    // Else add a new cluster with the new signature
    else
    {
        // Copy the signature
        Cluster new_cluster(signature);

        // If the count is above the treshold, change it to a stable
        // signature and give it a new phaseid
        if (new_cluster.count >= transition_threshold)
        {
            new_cluster.phase = next_phase_id;

            // Generate the next phase id
            next_phase_id++;
        }
        else
        {
            new_cluster.phase = TRANSITION_PHASE_ID;
        }

        // Add new cluster
        clusters.push_back(new_cluster);

        return new_cluster.phase;

    }

}

//----------------------------------------------------------------------------//

int
UnboundedLeaderFollowerClassifier::FindPhase(
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
