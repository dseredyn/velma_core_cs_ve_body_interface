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

#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <getopt.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>

#include "shm_comm_api.h"

int create_shm_object(const char *shm_name, int size, int readers) {
    int shm_hdr_fd;
    int shm_data_fd;
    char shm_name_hdr[128];
    char shm_name_data[128];

    channel_hdr_t *shm_hdr;

    channel_t channel;

    strcpy(shm_name_hdr, shm_name);
    strcat(shm_name_hdr, "_hdr");

    shm_hdr_fd = shm_open(shm_name_hdr, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
//    shm_hdr_fd = shm_open(shm_name_hdr, O_RDWR | O_CREAT | O_EXCL, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
    if (shm_hdr_fd < 0) {
        fprintf(stderr, "shm_open failed\n");
        perror(NULL);
        return -1;
    }

    if (ftruncate(shm_hdr_fd, CHANNEL_HDR_SIZE(size, readers)) != 0) {
        fprintf(stderr, "ftruncate failed\n");
        shm_unlink(shm_name_hdr);
        return -1;
    }

    shm_hdr = mmap(NULL, CHANNEL_HDR_SIZE(size, readers), PROT_READ | PROT_WRITE, MAP_SHARED, shm_hdr_fd, 0);

    if (shm_hdr == MAP_FAILED) {
        fprintf(stderr, "mmap failed\n");
        shm_unlink(shm_name_hdr);
        return -1;
    }

    strcpy(shm_name_data, shm_name);
    strcat(shm_name_data, "_data");

//    shm_data_fd = shm_open(shm_name_data, O_RDWR | O_CREAT | O_EXCL, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
    shm_data_fd = shm_open(shm_name_data, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
    if (shm_data_fd < 0) {
        fprintf(stderr, "shm_open failed\n");
        perror(NULL);
        return -1;
    }

    if (ftruncate(shm_data_fd, CHANNEL_DATA_SIZE(size, readers)) != 0) {
        fprintf(stderr, "ftruncate failed\n");
        shm_unlink(shm_name_data);
        return -1;
    }

    init_channel_hdr(size, readers, SHM_SHARED, shm_hdr);

    munmap(shm_hdr, CHANNEL_HDR_SIZE(size, readers));
    close(shm_hdr_fd);
    close(shm_data_fd);

    return 0;
}

int delete_shm_object(const char *shm_name) {
    char shm_name_hdr[128];
    char shm_name_data[128];

    strcpy(shm_name_hdr, shm_name);
    strcat(shm_name_hdr, "_hdr");

    strcpy(shm_name_data, shm_name);
    strcat(shm_name_data, "_data");

    shm_unlink(shm_name_data);
    shm_unlink(shm_name_hdr);

    return 0;
}

int connect_channel(const char *name, channel_t *chan) {
    int shm_hdr_fd;
    int shm_data_fd;
    channel_hdr_t *shm_hdr;
    void *shm_data;
    char shm_name[128];

    strcpy(shm_name, name);
    strcat(shm_name, "_hdr");

    shm_hdr_fd = shm_open(shm_name, O_RDWR, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
    if (shm_hdr_fd < 0) {
        fprintf(stderr, "shm_open failed\n");
        perror(NULL);
        return -1;
    }

    struct stat sb;
    fstat(shm_hdr_fd, &sb);

    shm_hdr = mmap(NULL, sb.st_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_hdr_fd, 0);

    if (shm_hdr == MAP_FAILED) {
        perror("mmap failed\n");
        return -1;
    }

    strcpy(shm_name, name);
    strcat(shm_name, "_data");

    shm_data_fd = shm_open(shm_name, O_RDWR, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
    if (shm_data_fd < 0) {
        fprintf(stderr, "shm_open failed\n");
        perror(NULL);
        return -1;
    }

    fstat(shm_data_fd, &sb);

    shm_data = mmap(NULL, sb.st_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_data_fd, 0);

    if (shm_data == MAP_FAILED) {
        perror("mmap failed\n");
        return -1;
    }

    init_channel(shm_hdr, shm_data, chan);

    return 0;
}

int disconnect_channel(channel_t *chan) {
    size_t size = CHANNEL_DATA_SIZE(chan->hdr->size, chan->hdr->max_readers);
    munmap(chan->hdr, size);
    chan->reader_ids = NULL;
    chan->reading = NULL;
    free(chan->buffer);
    chan->buffer = NULL;
    chan->hdr = NULL;
    return 0;
}

