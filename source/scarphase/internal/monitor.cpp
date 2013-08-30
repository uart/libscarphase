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

#include <cmath>
#include <cstring>

#include "scarphase/internal/util/random_projection.hpp"

#include "scarphase/internal/classifier/bounded_lf_classifier.hpp"
#include "scarphase/internal/classifier/unbounded_lf_classifier.hpp"

#include "scarphase/internal/predictor/last_value_predictor.hpp"
#include "scarphase/internal/predictor/run_length_predictor.hpp"

#include "scarphase/internal/monitor.hpp"

namespace scarphase {
namespace internal {

namespace spi = scarphase::internal;

//----------------------------------------------------------------------------//

static uint64_t
default_hash_function(scarphase_handle_t, uint64_t ip)
{
    static spi::util::RandomProjection<uint64_t, 5> rp;
    return rp.Hash(ip);
}

//----------------------------------------------------------------------------//

Monitor::Monitor(pid_t tid, const scarphase_monitor_attr *attr, Monitor *parent)
{
    BOOST_ASSERT(attr != NULL);
    BOOST_ASSERT(attr->data_accessor != NULL);

    memcpy(dynamic_cast<scarphase_monitor_attr*> (&conf),
           attr,
           sizeof(scarphase_monitor_attr));

    conf.tid  = tid;
    conf.debug = true;


    hook.hash = (attr->hooks.hash_function) ? attr->hooks.hash_function
                                            : default_hash_function;

    //------------------------------------------------------------------------//
    // Data accessor                                                          //
    //------------------------------------------------------------------------//
    {
        data_accessor = conf.data_accessor;
        data_accessor->set_sample_period(data_accessor, conf.sample_period);
        data_accessor->set_window_size(data_accessor, conf.window_size);
    }

    //------------------------------------------------------------------------//
    // Dynamic Sample Rate                                                    //
    //------------------------------------------------------------------------//
    {
        dsr.enabled                = attr->dsr.enabled;
        dsr.maximum_sample_period  = attr->dsr.maximum_sample_period;
        dsr.similarity_threshold   = attr->dsr.similarity_threshold;
    }

    //------------------------------------------------------------------------//
    // Classification                                                         //
    //------------------------------------------------------------------------//
    if (parent)
    {
        this->classifier = parent->classifier;
    }
    else if (attr->classifier.type == SCARPHASE_CLASSIFIER_TYPE_LEADER_FOLLOWER)
    {
        if (attr->classifier.attr.leader_follower.type ==
            SCARPHASE_CLASSIFIER_LEADER_FOLLOWER_TYPE_BOUNDED)
        {
            classifier = boost::shared_ptr<spi::classifier::Classifier> (
                new spi::classifier::BoundedLeaderFollowerClassifier(
                    attr->classifier.attr.leader_follower.similarity_threshold,
                    attr->classifier.attr.leader_follower.transition_threshold
                )
            );
        }
        else // if (attr->classifier.attr.leader_follower.type ==
             // SCARPHASE_CLASSIFIER_LEADER_FOLLOWER_TYPE_BOUNDED)
        {
            classifier = boost::shared_ptr<spi::classifier::Classifier> (
                new spi::classifier::UnboundedLeaderFollowerClassifier(
                    attr->classifier.attr.leader_follower.similarity_threshold,
                    attr->classifier.attr.leader_follower.transition_threshold
                )
            );
        }
    }
    else
    {
        BOOST_ASSERT(1);
    }

    //------------------------------------------------------------------------//
    // Prediction                                                             //
    //------------------------------------------------------------------------//
    if (parent)
    {
        this->predictor = parent->predictor;
    }
    else if (attr->predictor.type == SCARPHASE_PREDICTOR_TYPE_LAST_VALUE)
    {
        predictor = boost::shared_ptr<spi::predictor::Predictor> (
            new spi::predictor::LastValuePredictor());

    }
    else if (attr->predictor.type == SCARPHASE_PREDICTOR_TYPE_RUN_LENGTH)
    {
        predictor = boost::shared_ptr<spi::predictor::Predictor> (
            new spi::predictor::RunLengthPredictor(
                attr->predictor.attr.run_length.cache_size,
                attr->predictor.attr.run_length.pattern_length,
                attr->predictor.attr.run_length.confidence_threshold
            )
        );
    }
    else
    {
        BOOST_ASSERT(1);
    }

    //------------------------------------------------------------------------//

    event_info.handle = static_cast<void*>(this);
    event_info.tid    = conf.tid;

    event_info._debug  = static_cast<void*>(&debug._debug);

    debug._debug.signature.fv_size   = signature.fv_size();
    debug._debug.signature.fv_values = signature.fv_data();

    debug._debug.signature.uv_size   = signature.uv_size();
    debug._debug.signature.uv_values = signature.uv_data();

    dsr.current_sample_period = 1;

}

Monitor::~Monitor()
{

}

//----------------------------------------------------------------------------//

int
Monitor::Start()
{
    return data_accessor->start(data_accessor);
}

int
Monitor::Stop()
{
    return data_accessor->stop(data_accessor);
}

//----------------------------------------------------------------------------//

scarphase_event_info_t*
Monitor::HandleWindow()
{

    // Get samples from accessor
    const uint64_t *samples; size_t size;

    data_accessor->get_samples(data_accessor, &samples, &size);

    // Fake copy to debug info
    debug._debug.samples.values = samples;
    debug._debug.samples.size = size;

    // Reset the bbv with 0's
    signature.Reset();

    for (size_t i = 0; i < size; i++)
    {
        signature[ hook.hash(this, samples[i]) ] += 1.0;
    }

    // Normalize the signature
    signature.Normalize();

    pid_t phase;

    if (not dsr.enabled)
    {
        // Classify the signature
         phase = classifier->ClassifySignature(signature);
    }
    else //
    {
        if (dsr.current_sample_period == 1)
        {
            // Classify the signature
            phase = classifier->ClassifySignature(signature);
        }
        else
        {
            phase = classifier->FindPhase(signature, dsr.similarity_threshold);
            phase = (phase == -1) ? TRANSITION_PHASE_ID : phase;
        }
    }

    event_info.phase = phase;

    // Update predictors
    predictor->Update(event_info.phase);

    // Predict the next phase
    spi::util::Prediction prediction = predictor->Predict();

    // Update event info
    event_info.prediction.phase      = prediction.phase();
    event_info.prediction.confidence = prediction.confidence();

    if (dsr.enabled && ptb.size())
    {
        // If phase change
        if (phase != ptb[0].phase())
        {
            goto reset_sample_period;
        }

        // If we predict that we are still in the same phase
        if (prediction.phase()        == phase &&
            prediction.confidence()   >= 1)
        {

            // Only grow if phase is stable
            if (phase != TRANSITION_PHASE_ID)
            {
                dsr.current_sample_period =
                    std::min(dsr.current_sample_period * 2,
                             dsr.maximum_sample_period);

                // Update accessor
                data_accessor->set_sample_period(data_accessor,
                    conf.sample_period * dsr.current_sample_period);
            }

            goto done;
        }
        else
        {
            goto reset_sample_period;
        }

        reset_sample_period:
        {
            if (dsr.current_sample_period > 1)
            {
                // Reset sample period
                dsr.current_sample_period = 1;

                // Update accessor
                data_accessor->set_sample_period(data_accessor,
                    conf.sample_period * dsr.current_sample_period);
            }
        }

    done:
        ;

    }

    // Add phase to phase tracking buffer
    ptb.push(phase);

    // Event info
    return &event_info;

}

//----------------------------------------------------------------------------//

scarphase_event_info_t*
Monitor::HandleSignal(int signum, siginfo_t *info, void *uc)
{

    // Forward and check if data accessor has any available
    if (data_accessor->handle_signal(data_accessor, signum, info, uc) == 0)
    {
        return NULL;
    }

    // Handle window
    return HandleWindow();

}

//----------------------------------------------------------------------------//

} /* namespace internal */
} /* namespace scarphase */

