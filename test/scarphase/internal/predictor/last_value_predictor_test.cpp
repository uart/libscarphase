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

#include <gtest/gtest.h>
#include "scarphase/internal/predictor/last_value_predictor.hpp"

namespace spi = scarphase::internal;
namespace spi_pred = scarphase::internal::predictor;

static pid_t PHASE_SEQUENCE[] = { 1, 1, 2, 1, 2, 1 };
static size_t CONF_SEQUENCE[] = { 0, 1, 0, 0, 0, 0 };

#define PHASE_SEQUENCE_SIZE (sizeof(PHASE_SEQUENCE) / sizeof(int))

TEST(LastValuePredictorTest, CTor)
{
    EXPECT_NO_THROW(spi_pred::LastValuePredictor());
}

TEST(LastValuePredictorTest, Predict)
{
    spi_pred::LastValuePredictor lvp;

    for (size_t i = 0; i < PHASE_SEQUENCE_SIZE; i++)
    {
        // Update
        lvp.Update(PHASE_SEQUENCE[i]);

        // Predict
        spi::util::Prediction p = lvp.Predict();

        EXPECT_EQ(PHASE_SEQUENCE[i], p.phase());
        EXPECT_EQ(CONF_SEQUENCE[i], p.confidence());
    }
}



