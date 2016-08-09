#ifndef __SHM_COMM_API__
#define __SHM_COMM_API__

#include "shm_comm_locking.h"

#ifdef __cplusplus
extern "C" {
#endif

int create_shm_object(const char *shm_name, int size, int readers);
int delete_shm_object(const char *shm_name);
int connect_channel(const char *name, channel_t *chan);
int disconnect_channel(channel_t *chan);

#ifdef __cplusplus
};  // extern "C"
#endif

#endif  // __SHM_COMM_API__

