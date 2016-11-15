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

#include "shm_comm_locking.h"

#include <stdlib.h>
#include <string.h>
//#include <stdio.h>
#include <time.h>
#include <errno.h>

int init_channel_hdr (int size, int readers, int flags, channel_hdr_t *shdata)
{
  if (shdata == NULL) {
    return SHM_INVAL;
  }

  if (size < 1)
  {
    return SHM_INVAL;
  }

  if (readers < 1)
  {
    return SHM_INVAL;
  }

  // set mutex shared between processes
  pthread_mutexattr_t mutex_attr;
  pthread_mutexattr_init(&mutex_attr);
  if ((flags & SHM_SHARED) == SHM_SHARED)
  {
    pthread_mutexattr_setpshared (&mutex_attr, PTHREAD_PROCESS_SHARED);
  }
  pthread_mutex_init (&shdata->mtx, &mutex_attr);

  // set condition shared between processes
  pthread_condattr_t cond_attr;
  pthread_condattr_init(&cond_attr);
  if ((flags & SHM_SHARED) == SHM_SHARED)
  {
    pthread_condattr_setpshared (&cond_attr, PTHREAD_PROCESS_SHARED);
  }
  pthread_cond_init (&shdata->cond, &cond_attr);

  shdata->latest = 0;
  shdata->readers = 0;
  shdata->max_readers = readers;
  shdata->size = size;
  pthread_mutex_t* reader_ids = (pthread_mutex_t*) ((char *) shdata + sizeof(channel_hdr_t));
  int* reading = (int *) ((char *) shdata + sizeof(channel_hdr_t) + readers * sizeof(pthread_mutex_t));

  for (size_t i = 0; i < readers; i++)
  {
    pthread_mutexattr_t mutexAttr;
    pthread_mutexattr_init(&mutexAttr);
    pthread_mutexattr_setpshared(&mutexAttr, PTHREAD_PROCESS_SHARED);
    pthread_mutexattr_setrobust(&mutexAttr, PTHREAD_MUTEX_ROBUST);
    pthread_mutex_init(&reader_ids[i], &mutexAttr);
    reading[i] = 0;
  }

  return 0;
}

int init_channel (channel_hdr_t* hdr, const void* data, channel_t* chan)
{
  if (hdr == NULL)
  {
    return SHM_INVAL;
  }

  if (data == NULL)
  {
    return SHM_INVAL;
  }

  if (chan == NULL)
  {
    return SHM_INVAL;
  }

  chan->hdr = hdr;
  chan->reader_ids = (int *) ((char*) hdr + sizeof(channel_hdr_t));
  chan->reading = (int *) ((char *) hdr + sizeof(channel_hdr_t) + hdr->max_readers * sizeof(pthread_mutex_t));
  chan->buffer = malloc ((hdr->max_readers + 2) * sizeof(char*));
  if (chan->buffer == NULL)
  {
    return -1;
  }

  for (size_t i = 0; i < (hdr->max_readers + 2); i++)
  {
    chan->buffer[i] = ((char *) data) + hdr->max_readers * sizeof(pthread_mutex_t) + i * hdr->size;
  }
  return 0;
}

int create_writer (channel_t* chan, writer_t* wr)
{
  if (chan == NULL)
  {
    return SHM_INVAL;
  }

  if (wr == NULL)
  {
    return SHM_INVAL;
  }

  wr->inuse = malloc ((chan->hdr->max_readers + 2) * sizeof(int));

  if (wr->inuse == NULL)
  {
    return -1;
  }

  wr->channel = chan;

  return 0;
}
;

void release_writer (writer_t* wr)
{
  if (wr != NULL)
  {
    if (wr->inuse != NULL)
    {
      free (wr->inuse);
    }
    wr->inuse = NULL;
    wr->channel = NULL;
    wr->index = -1;
  }
}

int writer_buffer_get (writer_t* wr, void** buf)
{
  if (wr == NULL)
  {
    return SHM_INVAL;
  }

  if (buf == NULL)
  {
    return SHM_INVAL;
  }

  for (size_t i = 0; i < (wr->channel->hdr->max_readers + 2); i++)
  {
    wr->inuse[i] = FALSE;
  }

//printf("writer_buffer_get hdr->latest %d\n", wr->channel->hdr->latest);

  wr->inuse[wr->channel->hdr->latest] = TRUE;

  for (size_t i = 0; i < wr->channel->hdr->max_readers; i++)
  {
    wr->inuse[wr->channel->reading[i]] = TRUE;
  }

  wr->index = -1;

  for (size_t i = 0; i < (wr->channel->hdr->max_readers + 2); i++)
  {
    if (wr->inuse[i] == FALSE)
    {
      wr->index = i;
    }
  }

  if (wr->index < 0)
  {
    return -2;
  }
  else
  {
    *buf = wr->channel->buffer[wr->index];
    return 0;
  }
}

int writer_buffer_write (writer_t* wr)
{
  if (wr == NULL)
  {
    return SHM_INVAL;
  }

  if (wr->index < 0)
  {
    return -1;
  }

  pthread_mutex_lock (&wr->channel->hdr->mtx);
  wr->channel->hdr->latest = wr->index;
  pthread_cond_broadcast (&wr->channel->hdr->cond);
  pthread_mutex_unlock (&wr->channel->hdr->mtx);

  wr->index = -1;

  return 0;
}

int create_reader (channel_t* chan, reader_t* reader)
{
  if (chan == NULL)
  {
    return SHM_INVAL;
  }

  if (reader == NULL)
  {
    return SHM_INVAL;
  }
  // cheack reader slot avalibility

  int err = 0;

  reader->id = -1;

  pthread_mutex_lock (&chan->hdr->mtx);

  if (chan->hdr->readers < chan->hdr->max_readers)
  {
    reader->channel = chan;

    for (int i = 0; i < chan->hdr->max_readers; i++)
    {
      int lock_status = pthread_mutex_trylock(&chan->reader_ids[i]);
      int acquired = FALSE;
      switch (lock_status)
      {
      case 0:
        acquired = TRUE;
        break;
      case EBUSY:
        // do nothing:
        // other reader acquired the mutex
        // the loop should be continued
        break;
      case EINVAL:
        err = -5;
        break;
      case EAGAIN:
        err = -6;
        break;
      case EDEADLK:
        err = -7;
        break;
      case EOWNERDEAD:
        // the reader that acquired the mutex is dead
        //TODO: make the state consistent

        // recover the mutex
        if (pthread_mutex_consistent(&chan->reader_ids[i]) == EINVAL) {
          err = -8;
          break;
        }
        if (pthread_mutex_unlock(&chan->reader_ids[i]) != 0) {
          err = -9;
          break;
        }
        if (pthread_mutex_trylock(&chan->reader_ids[i]) != 0) {
          err = -10;
          break;
        }
        chan->hdr->readers--;
        acquired = TRUE;
        break;
      default:
        // other error
        err = -11;
        break;
      }

      if (err != 0) {   // an error occured in current iteration
        break;
      }

      if (acquired)
      {
        reader->id = i;
        chan->hdr->readers++;
        break;
      }
    }

    if (reader->id < 0 && err == 0)
    {
      err = -3;
    }
  }
  else
  {
    err = -2;
  }

  pthread_mutex_unlock (&chan->hdr->mtx);

  return err;
}

void release_reader (reader_t* re)
{
  if (re == NULL)
  {
    return;
  }

  pthread_mutex_lock (&re->channel->hdr->mtx);

  pthread_mutex_unlock(&re->channel->reader_ids[re->id]);
  re->channel->hdr->readers--;
  re->id = -1;

  pthread_mutex_unlock (&re->channel->hdr->mtx);
}

void* reader_buffer_get (reader_t* re)
{
  if (re == NULL)
  {
    return NULL;
  }

  void* ret = NULL;

  pthread_mutex_lock (&re->channel->hdr->mtx);
  if (-1 < re->channel->hdr->latest && re->channel->hdr->latest < (re->channel->hdr->max_readers + 2))
  {
    re->channel->reading[re->id] = re->channel->hdr->latest;
    ret = re->channel->buffer[re->channel->reading[re->id]];
  }
  pthread_mutex_unlock (&re->channel->hdr->mtx);

  return ret;
}

void* reader_buffer_wait (reader_t* re)
{
  if (re == NULL)
  {
    return NULL;
  }

  pthread_mutex_lock (&re->channel->hdr->mtx);

  while (re->channel->reading[re->id] == re->channel->hdr->latest)
  {
    pthread_cond_wait (&re->channel->hdr->cond, &re->channel->hdr->mtx);
  }

  re->channel->reading[re->id] = re->channel->hdr->latest;
  pthread_mutex_unlock (&re->channel->hdr->mtx);

  return re->channel->buffer[re->channel->reading[re->id]];
}

void* reader_buffer_timedwait (reader_t* re, time_t sec, long int nsec)
{
  if (re == NULL)
  {
    return NULL;
  }

  if (nsec < 0 || nsec >= 1000000000)
  {
    return NULL;
  }

  struct timespec abstime;
  clock_gettime(CLOCK_REALTIME, &abstime);
  abstime.tv_sec += sec;
  abstime.tv_nsec += nsec;
  if (abstime.tv_nsec >= 1000000000) {
    abstime.tv_nsec -= 1000000000;
    ++abstime.tv_sec;
  }

  pthread_mutex_lock (&re->channel->hdr->mtx);

  while (re->channel->reading[re->id] == re->channel->hdr->latest)
  {
    int result = pthread_cond_timedwait (&re->channel->hdr->cond, &re->channel->hdr->mtx, &abstime);
    if (result == ETIMEDOUT)
    {
      pthread_mutex_unlock (&re->channel->hdr->mtx);
      return NULL;
    }
  }

  re->channel->reading[re->id] = re->channel->hdr->latest;
  pthread_mutex_unlock (&re->channel->hdr->mtx);

  return re->channel->buffer[re->channel->reading[re->id]];
}

