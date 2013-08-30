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
#include "scarphase/internal/classifier/bounded_lf_classifier.hpp"

namespace spi = scarphase::internal;

struct BoundedLeaderFollowerClassifierTest : public testing::Test
{

  spi::util::Signature s0a, s0b;
  spi::util::Signature s1a, s1b;

  virtual void SetUp()
  {
      s0a[0] = 1.0;
      s0b[0] = 0.9;

      s1a[1] = 1.0;
      s1b[1] = 0.8;
  }

};

TEST_F(BoundedLeaderFollowerClassifierTest, CTor)
{
    EXPECT_NO_THROW(spi::classifier::BoundedLeaderFollowerClassifier(0.25, 0));
}

TEST_F(BoundedLeaderFollowerClassifierTest, ClassifySignature)
{
    spi::classifier::BoundedLeaderFollowerClassifier classifier(0.25, 0);

    EXPECT_EQ(1, classifier.ClassifySignature(s0a));
    EXPECT_EQ(1, classifier.ClassifySignature(s0b));
    EXPECT_EQ(2, classifier.ClassifySignature(s1a));
    EXPECT_EQ(1, classifier.ClassifySignature(s0a));
    EXPECT_EQ(2, classifier.ClassifySignature(s1b));
}

TEST_F(BoundedLeaderFollowerClassifierTest, FindPhase)
{
    spi::classifier::BoundedLeaderFollowerClassifier classifier(0.25, 0);

    pid_t phase = classifier.ClassifySignature(s0a);

    // Find s0 phase
    EXPECT_EQ(phase, classifier.FindPhase(s0b, 0.25));

    // Find s1 phase
    EXPECT_EQ(-1, classifier.FindPhase(s1a, 0.25));
}

