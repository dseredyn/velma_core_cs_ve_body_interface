/*
 Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of the Warsaw University of Technology nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef SHM_COMM_H_
#define SHM_COMM_H_

#ifdef __cplusplus
#include <atomic>
using namespace std;
#else
#include <stdatomic.h>
#endif

#define FALSE 0
#define TRUE 1

#define CHANNEL_DATA_SIZE(S, R) (sizeof(channel_hdr_t) + (R) * sizeof(int) + (R) * sizeof(int) + ((R) + 2) * (S))

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    atomic_int latest; // index of latest written buffer
    atomic_int readers; // number of connected readers
    int max_readers; // number of allocated readers
    unsigned int size; // size of buffer element
} channel_hdr_t;

typedef struct {
    channel_hdr_t *hdr;
    atomic_int *reader_ids; // contain information which reader ids are in use
    atomic_int *reading; // readers state array
    char **buffer; // data buffers (readers + 2)
} channel_t;

typedef struct {
    unsigned int index; // index of currently used write buffer
    int *inuse; // pointer to inuse array (readers)
    channel_t *channel; // pointer to corresponding channel structure
} writer_t;

typedef struct {
    unsigned int id; // reader id
    channel_t *channel; // pointer to corresponding channel structure
} reader_t;

int init_channel_hdr(unsigned int, int, channel_hdr_t *);

int init_channel(channel_hdr_t *, channel_t *);

int create_writer(channel_t *, writer_t *);

void release_writer(writer_t *wr);

void *writer_buffer_get(writer_t *wr);

int writer_buffer_write(writer_t *wr);

int create_reader(channel_t *, reader_t *);

void release_reader(reader_t *);

void *reader_buffer_get(reader_t *);

#ifdef __cplusplus
};  // extern "C"
#endif

#endif  // SHM_COMM_H_

