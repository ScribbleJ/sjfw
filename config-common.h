/* YOU ARE PROBABLY LOOKING FOR THE BOARD CONFIG FILES UNDER ramps/ AND gen4/
   AND NOT THIS FILE, which is just defining some things peculiar to the sourcecode. */

// Maximum length of a single 'fragment' of Gcode; characters in-between spaces.
#define MAX_GCODE_FRAG_SIZE 32
// Gcode is a big structure here, 10 is a lot of ram.
#define GCODE_BUFSIZE 10
// Each source eats anough ram for 1 addtl gcode
#define GCODE_SOURCES 2
#define HOST_SOURCE 0
#define SD_SOURCE 1



// RECV buffer must be large enough to hold at least 1 full line of gcode.  Anything
// larger is overkill - limited to 255 chars by RingBuffer addresstype presently
#define HOST_RECV_BUFSIZE 250
// SEND buffer must be large enough to hold as much text as we might spew in a loop; it's a lot.
#define HOST_SEND_BUFSIZE 250
#define HOST_BAUD 57600

