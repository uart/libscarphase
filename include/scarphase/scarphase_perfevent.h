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

#ifndef __SCARPHASE_PERFEVENT_H
#define __SCARPHASE_PERFEVENT_H

#include <scarphase.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Linux perf_events settings.
 */
struct scarphase_perfevent_attr
{

    struct sample
    {

        /**
         * @brief The branch counter id.
         */
        int counter;

        /**
         * @brief The branch counter type id.
         * @see enum perf_type_id in perf_events.h;
         */
        int type_id;

        /**
         * @brief Randomize the sample period.
         * @warning The kernel must have been patch for this to work.
         */
        int randomize;

        /**
         * @brief Enable/disable PEBS.
         * @warning The kernel must have been patch for this to work with
         *          branch instructions.
         */
        int precise_ip;

        /**
         * @brief The size of the kernal buffer in number of bytes.
         */
        int kernal_buffer_size;

    } sample;

    struct window
    {

        /**
         * @brief The branch counter id.
         */
        int counter;

        /**
         * @brief The branch counter type id.
         * @see enum perf_type_id in perf_events.h;
         */
        int type_id;

    } window;

};

/**
 * Typedef for the monitor attr struct.
 */
typedef struct scarphase_perfevent_attr scarphase_perfevent_attr_t;

/**
 * @brief
 */
scarphase_data_accessor_t* scarphase_create_perfevent_data_accessor(
    pid_t tid, scarphase_perfevent_attr_t *attr);

/**
 * @brief
 */
void scarphase_destroy_perfevent_data_accessor(
    scarphase_data_accessor_t *accessor);

#ifdef __cplusplus
}
#endif


#endif /* __SCARPHASE_PERFEVENT_H */
