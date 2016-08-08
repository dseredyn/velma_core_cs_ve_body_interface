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

#include <rtt/Component.hpp>
#include <rtt_rosclock/rtt_rosclock.h>

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>

#include "velma_lli_test_time.h"

VelmaTestTime::VelmaTestTime(const std::string &name) :
    RTT::TaskContext(name, PreOperational),
    ros_sec_(0),
    ros_nsec_(0)
{
}

bool VelmaTestTime::configureHook() {
/*    int shm_fd;
    channel_hdr_t *shm_hdr;
    const char *shm_name = "velma_lli_cmd";

    shm_fd = shm_open(shm_name, O_RDWR, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
    if (shm_fd < 0) {
        std::string err_str(strerror(errno));
        std::cout << "VelmaTestTime shm_open failed: " << err_str << std::endl;
        return false;
    }

    struct stat sb;
    fstat(shm_fd, &sb);

    shm_hdr = reinterpret_cast<channel_hdr_t*>( mmap(NULL, sb.st_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0) );

    if (shm_hdr == MAP_FAILED) {
        std::string err_str(strerror(errno));
        std::cout << "VelmaTestTime mmap failed failed: " << err_str << std::endl;
        return false;
    }

    if (CHANNEL_DATA_SIZE(shm_hdr->size, shm_hdr->max_readers) != sb.st_size) {
        std::cout << "VelmaTestTime error data" << std::endl;
        return false;
    }

    init_channel(shm_hdr, &chan_);

    int ret = create_reader(&chan_, &re_);

    if (ret != 0) {

        if (ret == -1)
            printf("invalid reader_t pointer\n");

        if (ret == -2)
            printf("no reader slots avalible\n");
        return 0;
    }
*/
    // Initialize and enable the simulation clock
    rtt_rosclock::use_manual_clock();
    rtt_rosclock::enable_sim();

    return true;
}

void VelmaTestTime::cleanupHook() {
/*    const size_t size = CHANNEL_DATA_SIZE(chan_.hdr->size, chan_.hdr->max_readers);

    release_reader(&re_);

    munmap(chan_.hdr, size);
    chan_.reader_ids = NULL;
    chan_.reading = NULL;
    free(chan_.buffer);
    chan_.buffer = NULL;
    chan_.hdr = NULL;
*/
}

bool VelmaTestTime::startHook() {
    prev_time_ = ros::Time::now();
    lost_comm_ = false;
    return true;
}

void VelmaTestTime::stopHook() {
}

void VelmaTestTime::increaseTime() {
    ros_nsec_ += 1000000;
    if (ros_nsec_ == 1000000000) {
        ros_nsec_ = 0;
        ++ros_sec_;
    }
}

void VelmaTestTime::updateHook() {

//    void *buf = reader_buffer_get(&re_);

//    ros::Time time = ros::Time::now();
    ros::Duration(0.001).sleep();

    uint32_t sec = ros_sec_;
    increaseTime();
    if (sec != ros_sec_) {
        std::cout << "time: " << ros_sec_ << std::endl;
    }
    rtt_rosclock::update_sim_clock(ros::Time(ros_sec_, ros_nsec_));
}
