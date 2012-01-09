/*
* Copyright 2010 by Adam Mayer <adam@makerbot.com>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/


#ifndef SDCARD_HH_
#define SDCARD_HH_

#include <stdint.h>

#include "fat.h"

namespace sdcard {

/**
* This enumeration lists all the SD card call error/success codes.
* Any non-zero value is an error condition.
*/
typedef enum {
	SD_SUCCESS              = 0,  // Operation succeeded
	SD_ERR_NO_CARD_PRESENT  = 1,  // No SD card is inserted in the slot
	SD_ERR_INIT_FAILED      = 2,  // SD card initialization failed
	SD_ERR_PARTITION_READ   = 3,  // Couldn't read the card's partition table
	SD_ERR_OPEN_FILESYSTEM  = 4,  // Couldn't open the FAT16 filesystem --
																//  check that it's real FAT16
	SD_ERR_NO_ROOT          = 5,  // No root directory found
	SD_ERR_CARD_LOCKED      = 6,  // Card is locked, writing forbidden
	SD_ERR_FILE_NOT_FOUND   = 7,  // Could not find specific file
	SD_ERR_GENERIC          = 8   // General error
} SdErrorCode;

extern struct fat_file_struct* file;

/**
* Reset the SD card subsystem.
*/
void reset();

/**
* Start a directory scan.
*/
SdErrorCode directoryReset();
/**
* Get the next filename in a directory scan.
*/
SdErrorCode directoryNextEntry(char* buffer, uint8_t bufsize);

/* support for sjfw functionality */

// Checks for existence of 'sjfwauto.gcode' and executes it if present.
bool autorun();

// Called from mainloop to allow spooling reads
void update();

bool openFile(const char* name, struct fat_file_struct** file);

// Updates 'current file' and returns it
char const* getNextfile();

// Just get current file
char const* getCurrentfile();

// print currentfile
bool printcurrent();

// returns the current file position, null if not sd printing
uint32_t  getFilePos();

// returns the current file size, null if not sd printing
uint32_t  getFileSize();

// pause printing, call printcurrent() to resume
bool pause();

// get current position
uint32_t getCurrentPos();

// get file size
uint32_t getCurrentSize();

/**************************/
/** Read File             */
/**************************/

SdErrorCode startRead(char const* filename);
bool readHasNext();
uint8_t readNext();
void readRewind(uint8_t bytes);
void finishRead();
bool isReading();

} // namespace sdcard

#endif // SDCARD_HH_
