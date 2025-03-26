/**
 * @file sdcard.cpp
 * @author Lorenzo Thomas Contessa <lorenzocontessa.dev@gmail.com>
 * @date 29/04/2023
 * @copyright 2023 Copyright (c) | Sapienza Space Team, all rights reserved.
 */

#include "sdcard.hpp"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(SDCARD, LOG_LEVEL_DBG);

namespace drivers::sdcard
{
    SDCard::SDCard(){
        std::string disk_pdrv = "SD";
        uint64_t    memory_size_mb;
        uint32_t    block_count;
        uint32_t    block_size;

        if (disk_access_init(disk_pdrv.c_str()) != 0) {
            printk("%d\n",disk_access_init(disk_pdrv.c_str()));
            LOG_ERR("Storage init ERROR");
            return;
        }

        if (disk_access_ioctl(disk_pdrv.c_str(), DISK_IOCTL_GET_SECTOR_COUNT, &block_count)) {
            LOG_ERR("Unable to get sector count");
            return;
        }
        LOG_INF("Block Count: %u", block_count);

        if (disk_access_ioctl(disk_pdrv.c_str(), DISK_IOCTL_GET_SECTOR_SIZE, &block_size)) {
            LOG_ERR("Unable to get sector size");
            return;
        }
        LOG_INF("Sector size: %u", block_size);

        memory_size_mb = (uint64_t) block_count * block_size;
        LOG_INF("Memory Size(MB): %u", (uint32_t) (memory_size_mb >> 20));

        mp_.mnt_point = disk_mount_point_.c_str();

        int res = fs_mount(&mp_);
        if (res != FR_OK) {
            LOG_ERR("Error mounting disk");
            return;
        }

        return;
    }
    
    std::vector<std::string> SDCard::lsdir(std::string path) {
        fs_dir_t  dirp;
        fs_dirent entry;

        fs_dir_t_init(&dirp);

        int res = fs_opendir(&dirp, path.c_str());
        if (res) {
            LOG_ERR("Error opening dir %s [%d]", path.c_str(), res);
            return {};
        }

	    LOG_INF("\nListing dir %s ...\n", path.c_str());
        for (;;) {
            res = fs_readdir(&dirp, &entry);

            if (res || entry.name[0] == 0) break;

            if (entry.type == FS_DIR_ENTRY_DIR) {
                LOG_DBG("[DIR ] %s", entry.name);
            } else {
                LOG_DBG("[FILE] %s (size = %zu)", entry.name, entry.size);
            }
        }

        fs_closedir(&dirp);

        // TODO: return list
        return {};
    }

    int SDCard::writeTelemetry(std::string telemetryStr) {
        char path[128];
        struct fs_file_t file;
        int base = strlen(disk_mount_point_.c_str());

        fs_file_t_init(&file);

        if (base >= (sizeof(path) - sizeof("flight.csv"))) {
            LOG_ERR("Not enough concatenation buffer to create file paths");
            return false;
        }

        // LOG_INF("Creating some dir entries in %s", disk_mount_point_.c_str());
        strncpy(path, disk_mount_point_.c_str(), sizeof(path));

        path[base++] = '/';
        path[base] = 0;
        strcat(&path[base], "flight.csv");

        if (fs_open(&file, path, FS_O_CREATE | FS_O_WRITE | FS_O_APPEND) != 0) {
            LOG_ERR("Failed to create file %s", path);
            return -1;
        }

        if(fs_seek(&file, 0, FS_SEEK_SET)) {
            LOG_ERR("Can't seek on file %s", path);
            return -1;   
        }

        int res = fs_write(&file, telemetryStr.c_str(), ((telemetryStr.length() + 1) * sizeof(char)));

        if (res == EBADF) {
            LOG_ERR("EBADF: Can't write on file %s", path);
            return -1;
        }

        if(res == ENOTSUP) {
            LOG_ERR("ENOTSUP: Can't write on file %s", path);
            return -1;
        }

        if(res < 0) {
            LOG_ERR("Can't write on file %s", path);
            return res;
        }

        fs_close(&file);
        
        LOG_DBG("Telemetry written on file");
        return 0;
    }
}