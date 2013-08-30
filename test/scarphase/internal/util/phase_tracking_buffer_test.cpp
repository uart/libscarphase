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
#include "scarphase/internal/util/phase_tracking_buffer.hpp"

namespace spi = scarphase::internal;

static int PHASE_SEQUENCE[] = { 1, 1, 2, 1, 2, 1 };

#define PHASE_SEQUENCE_SIZE (sizeof(PHASE_SEQUENCE) / sizeof(int))

struct PhaseTrackingBufferTest : public testing::Test
{

  spi::util::PhaseTrackingBuffer ptb0;

  virtual void SetUp()
  {
      for (size_t i = 0; i < PHASE_SEQUENCE_SIZE; i++)
      {
          ptb0.push(PHASE_SEQUENCE[i]);
      }
  }

};

TEST_F(PhaseTrackingBufferTest, CTor)
{
    EXPECT_DEATH(spi::util::PhaseTrackingBuffer(0) , "");
}

TEST_F(PhaseTrackingBufferTest, Push)
{
    spi::util::PhaseTrackingBuffer ptb(2);

    ptb.push(0);
    EXPECT_EQ(0, ptb.buffer()[0].phase());
    EXPECT_EQ(1, ptb.buffer()[0].length());

    ptb.push(1);
    EXPECT_EQ(1, ptb.buffer()[0].phase());
    EXPECT_EQ(1, ptb.buffer()[0].length());
    EXPECT_EQ(0, ptb.buffer()[1].phase());
    EXPECT_EQ(1, ptb.buffer()[1].length());

    ptb.push(1);
    EXPECT_EQ(1, ptb.buffer()[0].phase());
    EXPECT_EQ(2, ptb.buffer()[0].length());
    EXPECT_EQ(0, ptb.buffer()[1].phase());
    EXPECT_EQ(1, ptb.buffer()[1].length());

    ptb.push(2);
    EXPECT_EQ(2, ptb.buffer()[0].phase());
    EXPECT_EQ(1, ptb.buffer()[0].length());
    EXPECT_EQ(1, ptb.buffer()[1].phase());
    EXPECT_EQ(2, ptb.buffer()[1].length());
}

TEST_F(PhaseTrackingBufferTest, phase)
{
    for (size_t i = 0; i < PHASE_SEQUENCE_SIZE; i++)
    {
        EXPECT_EQ(PHASE_SEQUENCE[PHASE_SEQUENCE_SIZE - i - 1],
                  ptb0.phase(i));
    }

    EXPECT_EQ(UNKNOWN_PHASE_ID, ptb0.phase(PHASE_SEQUENCE_SIZE));
}

