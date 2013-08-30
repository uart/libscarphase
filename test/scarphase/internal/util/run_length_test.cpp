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
#include "scarphase/internal/util/run_length.hpp"

namespace spi = scarphase::internal;

#define MIN 0
#define MAX 10

struct RunLengthTest : public testing::Test
{

  spi::util::RunLength rl0;
  spi::util::RunLength rl1;

  virtual void SetUp()
  {
    rl0.Update(0, 1);
    rl1.Update(0, 2);
  }

};

TEST_F(RunLengthTest, CTor)
{
    EXPECT_NO_THROW(spi::util::RunLength(0, 1));
}

TEST_F(RunLengthTest, Getters)
{
    EXPECT_EQ(0, rl0.phase());
    EXPECT_EQ(1, rl0.length());
}

TEST_F(RunLengthTest, Equal)
{
    EXPECT_TRUE(rl0 == rl0);
    EXPECT_FALSE(rl0 == rl1);
}

TEST_F(RunLengthTest, NotEqual)
{
    EXPECT_FALSE(rl0 != rl0);
    EXPECT_TRUE(rl0 != rl1);
}

TEST_F(RunLengthTest, PostIncrement)
{
    int t = rl0.length();
    rl0++;
    EXPECT_EQ(t + 1, rl0.length());
}

TEST_F(RunLengthTest, PreIncrement)
{
    int t = rl0.length();
    ++rl0;
    EXPECT_EQ(t + 1, rl0.length());
}
