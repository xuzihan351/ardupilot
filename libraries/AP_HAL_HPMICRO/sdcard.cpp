/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "sdcard.h"



#include <sys/stat.h>
#include <sys/unistd.h>
#include <sys/types.h>
#include "SPIDevice.h"
#include "hpm_sdmmc_sd.h"
#include "ff.h"
#include "diskio.h"

#ifdef HAL_HPMICRO_SDCARD

FATFS s_sd_disk;
FIL s_file;
BYTE work[FF_MAX_SS];
const TCHAR driver_num_buf[4] = { DEV_SD + '0', ':', '/', '\0' };
#define TEST_DIR_NAME "hpmicro_sd_test_dir0"

static const AP_HAL::HAL& hpmhal = AP_HAL::get_HAL();

static HAL_Semaphore sem;

void update_fw()
{
}

const char *show_error_string(FRESULT fresult)
{
    const char *result_str;

    switch (fresult) {
    case FR_OK:
        result_str = "succeeded";
        break;
    case FR_DISK_ERR:
        result_str = "A hard error occurred in the low level disk I/O level";
        break;
    case FR_INT_ERR:
        result_str = "Assertion failed";
        break;
    case FR_NOT_READY:
        result_str = "The physical drive cannot work";
        break;
    case FR_NO_FILE:
        result_str = "Could not find the file";
        break;
    case FR_NO_PATH:
        result_str = "Could not find the path";
        break;
    case FR_INVALID_NAME:
        result_str = "Tha path name format is invalid";
        break;
    case FR_DENIED:
        result_str = "Access denied due to prohibited access or directory full";
        break;
    case FR_EXIST:
        result_str = "Access denied due to prohibited access";
        break;
    case FR_INVALID_OBJECT:
        result_str = "The file/directory object is invalid";
        break;
    case FR_WRITE_PROTECTED:
        result_str = "The physical drive is write protected";
        break;
    case FR_INVALID_DRIVE:
        result_str = "The logical driver number is invalid";
        break;
    case FR_NOT_ENABLED:
        result_str = "The volume has no work area";
        break;
    case FR_NO_FILESYSTEM:
        result_str = "There is no valid FAT volume";
        break;
    case FR_MKFS_ABORTED:
        result_str = "THe f_mkfs() aborted due to any problem";
        break;
    case FR_TIMEOUT:
        result_str = "Could not get a grant to access the volume within defined period";
        break;
    case FR_LOCKED:
        result_str = "The operation is rejected according to the file sharing policy";
        break;
    case FR_NOT_ENOUGH_CORE:
        result_str = "LFN working buffer could not be allocated";
        break;
    case FR_TOO_MANY_OPEN_FILES:
        result_str = "Number of open files > FF_FS_LOCK";
        break;
    case FR_INVALID_PARAMETER:
        result_str = "Given parameter is invalid";
        break;
    default:
        result_str = "Unknown error";
        break;
    }
    return result_str;
}

static FRESULT sd_mount_fs(void)
{
    FRESULT fresult = f_mount(&s_sd_disk, driver_num_buf, 1);
    if (fresult == FR_OK) {
        hpmhal.console->printf("SD card has been mounted successfully\n");
    } else {
        hpmhal.console->printf("Failed to mount SD card, cause: %s\n", show_error_string(fresult));
    }

    fresult = f_chdrive(driver_num_buf);
    return fresult;
}

static FRESULT sd_mkfs(void)
{
    hpmhal.console->printf("Formatting the SD card, depending on the SD card capacity, the formatting process may take a long time\n");
    FRESULT fresult = f_mkfs(driver_num_buf, NULL, work, sizeof(work));
    if (fresult != FR_OK) {
        hpmhal.console->printf("Making File system failed, cause: %s\n", show_error_string(fresult));
    } else {
        hpmhal.console->printf("Making file system is successful\n");
    }

    return fresult;
}

void mount_sdcard()
{
    FRESULT fatfs_result;
    hpmhal.console->printf("Mounting sd \n");
    WITH_SEMAPHORE(sem);
    /* Before doing FATFS operation, ensure the SD card is present */
    DSTATUS dstatus = disk_status(DEV_SD);
    if (dstatus == STA_NODISK) {
        hpmhal.console->printf("No disk in the SD slot, please insert an SD card...\n");
        do {
            dstatus = disk_status(DEV_SD);
        } while (dstatus == STA_NODISK);
        hpmhal.scheduler->delay(100);
        hpmhal.console->printf("Detected SD card, re-initialize the filesystem...\n");
    }
    dstatus = disk_initialize(DEV_SD);
    if (dstatus != RES_OK) {
        hpmhal.console->printf("Failed to initialize SD disk\n");
    }
    fatfs_result = sd_mount_fs();
    if (fatfs_result == FR_NO_FILESYSTEM) {
        hpmhal.console->printf("There is no File system available, making file system...\n");
        fatfs_result = sd_mkfs();
        if (fatfs_result != FR_OK) {
            hpmhal.console->printf("Failed to make filesystem, cause:%s\n", show_error_string(fatfs_result));
        }
    }
}


bool sdcard_retry(void)
{
    return false;
}

void unmount_sdcard()
{
    f_unmount(driver_num_buf);
}
void sdcard_stop(void)
{
}

#else
// empty impl's
void mount_sdcard()
{
}
void unmount_sdcard()
{
}
bool sdcard_retry(void)
{
    return true;
}
#endif




