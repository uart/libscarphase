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

#include <numeric>
#include <gtest/gtest.h>
#include "scarphase/internal/util/signature.hpp"

namespace spi = scarphase::internal;

struct SignatureTest : public testing::Test
{

  spi::util::Signature s0;
  spi::util::Signature s1;

  virtual void SetUp()
  {
      for (size_t i = 0; i < SIGNATURE_SIZE; i++)
      {
          s0[i] = 0;
          s1[i] = 1;
      }
  }

};

TEST_F(SignatureTest, CTor)
{
    EXPECT_NO_THROW(spi::util::Signature());

    spi::util::Signature s;

    for (size_t i = 0; i < s0.size(); i++)
    {
      EXPECT_FLOAT_EQ(0.0, s[i]);
    }
}

TEST_F(SignatureTest, AllZeros)
{
    for (size_t i = 0; i < s0.size(); i++)
    {
      EXPECT_FLOAT_EQ(0.0, s0[i]);
    }
}

TEST_F(SignatureTest, AllOnes)
{
    for (size_t i = 0; i < s1.size(); i++)
    {
      EXPECT_FLOAT_EQ(1.0, s1[i]);
    }
}

TEST_F(SignatureTest, Reset)
{
    s1.Reset();
    EXPECT_FLOAT_EQ(0.0, std::accumulate(s1.fv_data(),
                                         s1.fv_data() + s1.fv_size(), 0.0));
}

TEST_F(SignatureTest, Normalize)
{
    s1.Normalize();
    for (size_t i = 0; i < s1.size(); i++)
    {
      EXPECT_FLOAT_EQ(1.0 / SIGNATURE_SIZE, s1[i]);
    }
}

TEST_F(SignatureTest, ManhattanDistance)
{
    EXPECT_FLOAT_EQ(s0.ManhattanDistance(s1), s1.ManhattanDistance(s0));
    EXPECT_FLOAT_EQ(s0.size(), s0.ManhattanDistance(s1));
}
