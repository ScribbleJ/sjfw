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

#include "SDCard.h"

#include <avr/io.h>
#include <string.h>
#include "sd-reader_config.h"
#include "fat.h"
#include "sd_raw.h"
#include "partition.h"
#include "GcodeQueue.h"
#include "config.h"
#include "Host.h"

#ifndef USE_DYNAMIC_MEMORY
#error Dynamic memory should be explicitly disabled in the G3 mobo.
#endif

#if (USE_DYNAMIC_MEMORY == 1)
#error Dynamic memory should be explicitly disabled in the G3 mobo.
#endif

#define SD_MAX_FN 16
namespace sdcard {

struct partition_struct* partition = 0;
struct fat_fs_struct* fs = 0;
struct fat_dir_struct* dd = 0;
struct fat_file_struct* file = 0;

char currentfile[SD_MAX_FN] = {0};

bool playing = false;
bool paused = false;
uint8_t next_byte;
bool has_more;

bool openPartition()
{
  /* open first partition */
  partition = partition_open(sd_raw_read,
                             sd_raw_read_interval,
                             0,
                             0,
                             0);

  if(!partition)
  {
    /* If the partition did not open, assume the storage device
    * is a "superfloppy", i.e. has no MBR.
    */
    partition = partition_open(sd_raw_read,
                               sd_raw_read_interval,
                               0,
                               0,
                               -1);
  }
  if(!partition)
    return false;
  return true;
}

bool openFilesys()
{
  /* open file system */
  fs = fat_open(partition);
  return fs != 0;
}

bool openRoot()
{
  // Open root directory
  struct fat_dir_entry_struct rootdirectory;
  fat_get_dir_entry_of_path(fs, "/", &rootdirectory);
  dd = fat_open_dir(fs, &rootdirectory);
  return dd != 0;
}

SdErrorCode initCard() {
	if (!sd_raw_init()) {
		if (!sd_raw_available()) {
			reset();
			return SD_ERR_NO_CARD_PRESENT;
		} else {
			reset();
			return SD_ERR_INIT_FAILED;
		}
	} else if (!openPartition()) {
		reset();
		return SD_ERR_PARTITION_READ;
	} else if (!openFilesys()) {
		reset();
		return SD_ERR_OPEN_FILESYSTEM;
	} else if (!openRoot()) {
		reset();
		return SD_ERR_NO_ROOT;
		
	/* we need to keep locked as the last check */
	} else if (sd_raw_locked()) {
		return SD_ERR_CARD_LOCKED;
	}
    playing = false;
    paused = false;
	return SD_SUCCESS;
}

SdErrorCode directoryReset() {
  reset();
  SdErrorCode rsp = initCard();
  if (rsp != SD_SUCCESS && rsp != SD_ERR_CARD_LOCKED) {
    return rsp;
  }
  fat_reset_dir(dd);
  return SD_SUCCESS;
}

SdErrorCode directoryNextEntry(char* buffer, uint8_t bufsize) {
	struct fat_dir_entry_struct entry;
	// This is a bit of a hack.  For whatever reason, some filesystems return
	// files with nulls as the first character of their name.  This isn't
	// necessarily broken in of itself, but a null name is also our way
	// of signalling we've gone through the directory, so we discard these
	// entries.  We have an upper limit on the number of entries to cycle
	// through, so we don't potentially lock up here.
	uint8_t tries = 5;
	while (tries) {
		if (fat_read_dir(dd, &entry)) {
			if ((entry.attributes & FAT_ATTRIB_VOLUME) == 0) {
				int i;
				for (i = 0; (i < bufsize-1) && entry.long_name[i] != 0; i++) {
					buffer[i] = entry.long_name[i];
				}
				buffer[i] = 0;
				if (i > 0) {
					break;
				} else {
					tries--;
				}
			}
		} else {
			buffer[0] = 0;
			break;
		}
	}
	return SD_SUCCESS;
}

bool findFileInDir(const char* name, struct fat_dir_entry_struct* dir_entry)
{
  while(fat_read_dir(dd, dir_entry))
  {
    if(strcmp(dir_entry->long_name, name) == 0)
    {
      fat_reset_dir(dd);
      return true;
    }
  }
  return false;
}

bool openFile(const char* name, struct fat_file_struct** file)
{
  struct fat_dir_entry_struct fileEntry;
  if(!findFileInDir(name, &fileEntry))
  {
    return false;
  }

  *file = fat_open_file(fs, &fileEntry);
  return true;
}

bool isReading() {
	return playing;
}

void fetchNextByte() {
  int16_t read = fat_read_file(file, &next_byte, 1);
  has_more = read > 0;
}

bool readHasNext() {
  return has_more;
}

uint8_t readNext() {
  uint8_t rv = next_byte;
  fetchNextByte();
  return rv;
}

SdErrorCode startRead(char const* filename) {
  reset();
  SdErrorCode result = initCard();
  /* for playback it's ok if the card is locked */
  if (result != SD_SUCCESS && result != SD_ERR_CARD_LOCKED) {
    return result;
  }
  file = 0;
  if (!openFile(filename, &file) || file == 0) {
    return SD_ERR_FILE_NOT_FOUND;
  }
  playing = true;
  fetchNextByte();
  return SD_SUCCESS;
}

void readRewind(uint8_t bytes) {
  int32_t offset = -((int32_t)bytes);
  fat_seek_file(file, &offset, FAT_SEEK_CUR);
}

void finishRead() {
  playing = false;
  paused = false;
  if (file != 0) {
	  fat_close_file(file);
	  //sd_raw_sync();
  }
  file = 0;
}


void reset() {
	if (playing)
		finishRead();
	if (dd != 0) {
		fat_close_dir(dd);
		dd = 0;
	}
	if (fs != 0) {
		fat_close(fs);
		fs = 0;
	}
	if (partition != 0) {
		partition_close(partition);
		partition = 0;
	}
}

bool autorun() {
  if (playing)
    return false;

  SdErrorCode e = startRead("sjfwauto.gcd");
  if(e != SD_SUCCESS)
  {
    return false;
  }

  return true;
}

void update() {
  if(!playing || paused)
    return;
  
  if(GCODES.isFull())
  {
    return;
  }
  
  if(!readHasNext())
  {
    finishRead();
    return;
  }

  char buf[MAX_GCODE_FRAG_SIZE] = {0};
  int x=0;
  for(;x < MAX_GCODE_FRAG_SIZE && readHasNext(); x++)
  {
    buf[x] = readNext();
    if(buf[x] <= 32)
      break;
  }
  if(buf[x] > 32)
  {
    HOST.write("SRO\n");
    // TODO: Buffer overrun... error out
    return;
  }

  GCODES.parsebytes(buf, x, SD_SOURCE);
}

char const* getCurrentfile()
{
  if (file)
    return file->dir_entry.long_name;
  else
    return currentfile;
}

char const* getNextfile()
{
  if (playing)
    finishRead();

  sdcard::SdErrorCode e;
  if(currentfile[0] == 0)
  {
    e = sdcard::directoryReset();
    //if(e != sdcard::SD_SUCCESS)  HOST.labelnum("dr: ", e, true);
  }

  do {
    e = sdcard::directoryNextEntry(currentfile,SD_MAX_FN);
  } while (e == sdcard::SD_SUCCESS && currentfile[0] == '.');
  //if(e != sdcard::SD_SUCCESS) HOST.labelnum("dne: ", e, true);
  return currentfile;
}

bool pause() {
  if (!playing)
    return false;

  paused = true;
  return paused;
}

bool printcurrent() {
  if (playing)
  {
    if (paused)
    {
      paused = false;
      return true;
    }
    finishRead();
  }

  SdErrorCode e = startRead(currentfile);
  if(e != SD_SUCCESS)
  {
    // HOST.labelnum("pc: ", e, true);
    return false;
  }
  return true;
}

uint32_t getCurrentPos() {
  if (file)
    return file->pos;
  return 0;
}

uint32_t getCurrentSize() {
  if (file)
    return file->dir_entry.file_size;
  return 0;
}

} // namespace sdcard
