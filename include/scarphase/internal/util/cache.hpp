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

#ifndef __SCARPHASE_INTERNAL_UTIL_CACHE_HPP
#define __SCARPHASE_INTERNAL_UTIL_CACHE_HPP

#include <vector>
#include <boost/array.hpp>

namespace scarphase {
namespace internal {
namespace util {

/**
 * @brief A direct mapped cache to hold phase data.
 */
template<class value_type,
         class storage_type>
class Cache //----------------------------------------------------------------//
{

public: //--------------------------------------------------------------------//

    /**
     * @brief
     */
    template<class index_t>
    value_type *Invalidate(index_t index);

    /**
     * @brief
     */
    template<class index_t, class tag_t>
    value_type *Find(index_t index, tag_t tag);

protected: //-----------------------------------------------------------------//

    /**
     * @brief
     */
    storage_type cache;

};

//----------------------------------------------------------------------------//

template<class value_type,
         size_t SIZE>
class       StaticCache
    : public virtual Cache< value_type, boost::array<value_type, SIZE> >
{

public: //--------------------------------------------------------------------//

    /**
     * @brief
     */
    StaticCache(const value_type &v);

};

//----------------------------------------------------------------------------//

template<class value_type>
class DynamicCache
    : public virtual Cache<value_type, std::vector<value_type> >
{

public: //--------------------------------------------------------------------//

    /**
     * @brief
     */
    DynamicCache(size_t size, const value_type &v);

};

} /* namespace util */
} /* namespace internal */
} /* namespace scarphase */

//----------------------------------------------------------------------------//
// Inline Definitions                                                         //
//----------------------------------------------------------------------------//

namespace scarphase {
namespace internal {
namespace util {

//----------------------------------------------------------------------------//

template<class value_type,
         class storage_type>
template<class index_t>
inline value_type*
Cache<value_type, storage_type>::Invalidate(index_t index)
{
    return &cache[index % cache.size()];
}

template<class value_type,
         class storage_type>
template<class index_t, class tag_t>
inline value_type*
Cache<value_type, storage_type>::Find(index_t index, tag_t tag)
{
    // Get cache entry
    value_type &entry = cache[index % cache.size()];

    // Return entry if valid match
    return (entry == tag) ? &entry : NULL;

}

//----------------------------------------------------------------------------//

template<class value_type,
         size_t SIZE>
StaticCache<value_type, SIZE>::StaticCache(const value_type &v)
{
    for (size_t i = 0; i < SIZE; i++)
    {
        Cache< value_type, boost::array<value_type, SIZE> >::cache[i] = v;
    }
}


//----------------------------------------------------------------------------//

template<class value_type>
DynamicCache<value_type>::DynamicCache(size_t size, const value_type &v)
{
    for (size_t i = 0; i < size; i++)
    {
        Cache< value_type, std::vector<value_type> >::cache.push_back(v);
    }
}

//----------------------------------------------------------------------------//

} /* namespace util */
} /* namespace internal */
} /* namespace scarphase */

#endif /* __SCARPHASE_INTERNAL_UTIL_CACHE_HPP */
