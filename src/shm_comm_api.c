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
/*
int main(int argc, char **argv) {
    int size = 100;
    int readers = 10;
    int sleep_time = 1;

    int type; // 0 - channel, 1 - reader, 2 - writer

    char shm_name[200];
    char shm_name_tmp[128];
    int shm_hdr_fd;
    int shm_data_fd;
    channel_hdr_t *shm_hdr;

    channel_t channel;
    reader_t re;
    writer_t wr;

    opterr = 0;

    int c;
    while ((c = getopt(argc, argv, "r:s:n:")) != -1) {
        switch (c) {
        case 'r':
            readers = atoi(optarg);
            break;
        case 's':
            size = atoi(optarg);
            break;
        case 'n':
            sleep_time = atoi(optarg);
            break;
        case '?':
            if (optopt == 'r' || optopt == 's' || optopt == 'n')
                fprintf(stderr, "Option -%c requires an argument.\n", optopt);
            else if (isprint(optopt))
                fprintf(stderr, "Unknown option `-%c'.\n", optopt);
            else
                fprintf(stderr, "Unknown option character `\\x%x'.\n", optopt);
            return 1;
        default:
            abort();
        }
    }

    if ((optind + 2) == argc) {
        if (!strcmp("channel", argv[optind])) {
            type = 0;
        } else if (!strcmp("reader", argv[optind])) {
            type = 1;
        } else if (!strcmp("writer", argv[optind])) {
            type = 2;
        } else {
            return 0;
        }

        if (!strcpy(shm_name, argv[optind+1])) {
            return 0;
        }
    } else {
        printf("not enough arguments\n");
        return 0;
    }

    stop = 0;

    char wdata = 33;

    switch (type) {
    case 0:
        printf("creating channel [%s] size: %d readers: %d \n", shm_name, size, readers);

        create_channel()
        break;
    case 1:
        printf("creating reader on channel [%s]\n", shm_name);
        if (connect_channel(shm_name, &channel) != 0) {
            return -1;
        }

        int ret = create_reader(&channel, &re);

        if (ret != 0) {

            if (ret == -1)
                printf("invalid reader_t pointer\n");

            if (ret == -2)
                printf("no reader slots avalible\n");
            return 0;
        }

        while(stop == 0) {
            char *buf = (char*) reader_buffer_wait(&re);

            if (buf == NULL) {
                printf("reader get NULL buffer\n");
                return -1;
            }
            printf("reading [%c]\n", *buf);
            sleep(sleep_time);
        }

        release_reader(&re);
        disconnect_channel(&channel);
        break;
    case 2:
        printf("creating writer on channel [%s]\n", shm_name);
        if (connect_channel(shm_name, &channel) != 0) {
            printf("unable to open channel\n");
            return -1;
        }

        ret = create_writer(&channel, &wr);

        if (ret != 0) {

            if (ret == -1) {
                printf("invalid writer_t pointer\n");
            }

            if (ret == -2) {
                printf("no writers slots avalible\n");
            }
            return -1;
        }

        while(stop == 0) {
            char *buf = NULL;
            writer_buffer_get(&wr, (void**)&buf);

            if (buf == NULL) {
                printf("writer get NULL buffer\n");
                return -1;
            }

            memset(buf, wdata, wr.channel->hdr->size);

            writer_buffer_write(&wr);

            printf("writing [%c] readers: %d\n", wdata, wr.channel->hdr->readers);

            wdata++;
            sleep(sleep_time);
        }

        release_writer(&wr);
        disconnect_channel(&channel);
        break;
    }

    return 0;
}
*/

