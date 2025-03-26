/**
 * @file sdcard.hpp
 * @author Lorenzo Thomas Contessa <lorenzocontessa.dev@gmail.com>
 * @date 29/04/2023
 * @copyright 2023 Copyright (c) | Sapienza Space Team, all rights reserved.
 */

#ifndef PAYLOAD_SDCARD_HPP
#define PAYLOAD_SDCARD_HPP

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/fs/fs.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/smf.h>

#include <ff.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <iostream>

namespace drivers::sdcard {
    class SDCard {
        public:
            explicit SDCard();
            ~SDCard();

            std::vector<std::string> lsdir(std::string path);
            int writeTelemetry(std::string telemetryStr);
        private:
            const device *_device;

            FATFS       fat_fs_ {};
            fs_mount_t  mp_ {.type = FS_FATFS, .fs_data = &fat_fs_};
            std::string disk_mount_point_ {"/SD:"};
    };
}

#endif