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
#include "scarphase/internal/util/cache.hpp"

namespace spi = scarphase::internal;

typedef int test_type;

#define SIZE 10

typedef spi::util::StaticCache<test_type, SIZE> _SCache;
typedef spi::util::DynamicCache<test_type> _DCache;

static test_type init_value = 42;

TEST(CacheTest, CTor)
{
    _SCache scache(init_value);
    _DCache dcache(SIZE, init_value);

    for (int i = 0; i < SIZE; i++)
    {
      EXPECT_EQ(init_value, *scache.Invalidate(i));
      EXPECT_EQ(init_value, *dcache.Invalidate(i));
    }
}

TEST(CacheTest, FindAndInvalidate)
{
    _SCache scache(init_value);
    _DCache dcache(SIZE, init_value);

    // Check if we can find init_value at index 0
    EXPECT_TRUE(scache.Find(0, init_value) != NULL);
    EXPECT_TRUE(dcache.Find(0, init_value) != NULL);

    // Check that it contains init_value
    EXPECT_EQ(init_value, *scache.Find(0, init_value));
    EXPECT_EQ(init_value, *dcache.Find(0, init_value));

    test_type v = 21;

    // Check that index 0 has value 0 - False
    EXPECT_FALSE(scache.Find(0, v) != NULL);
    EXPECT_FALSE(dcache.Find(0, v) != NULL);

    // Update the cache at index 0
    *scache.Invalidate(0) = v;
    *dcache.Invalidate(0) = v;

    // Check update
    EXPECT_TRUE(scache.Find(0, v) != NULL);
    EXPECT_TRUE(dcache.Find(0, v) != NULL);
}

