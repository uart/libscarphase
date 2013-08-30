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
#include "scarphase/internal/util/saturation_counter.hpp"

namespace spi = scarphase::internal;

#define MIN 0
#define MAX 10

struct SaturationCounterTest : public testing::Test
{
  spi::util::SaturationCounter<int, MIN, MAX> s0;
};

TEST_F(SaturationCounterTest, CTor)
{
    EXPECT_EQ(s0.Value(), MIN);
}

TEST_F(SaturationCounterTest, Overflow)
{
    for (int i = s0.GetMinValue(); i < s0.GetMaxValue(); i++)
    {
        EXPECT_EQ(i, s0.Value());
        s0++;
    }

    s0++;
    EXPECT_EQ(s0.Value(), MAX);
}

TEST_F(SaturationCounterTest, Underflow)
{
    s0--;
    EXPECT_EQ(s0.Value(), MIN);
}

TEST_F(SaturationCounterTest, IncrementAndDecrement)
{
    int v = s0.Value();
    s0++; s0--;
    EXPECT_EQ(s0.Value(), v);
}
