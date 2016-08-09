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

