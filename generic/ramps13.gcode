M402         ;Write this configuration to EEPROM

; Standard RAMPS 1.3/1.4 Pinsettings.
M207 P13 S10 ;Set hotend pins, P=thermistor, S=FET
M208 P14 S8  ;Set platform pins, P=thermistor, S=FET
M309 P1 S1 ;Set endstop inversion and pullups; must happen before M304/M305 OR ELSE!!!!!!
M300 X54 Y60 Z46 E26 ;Set axis STEP pins
M301 X55 Y61 Z48 E28 ;Set axis DIR pins
M302 X38 Y56 Z62 E24 ;Set axis ENABLE pins
M304 X3 Y14 Z18      ;Set axis MIN pins
M305 X2 Y15 Z19      ;Set axis MAX pins
M307 X0 Y0 Z1 E0     ;Set axis inversion; 0=no, 1=yes
M308 X0 Y0 Z1 E0     ;Set axis disable-after-move

; These axis speeds and steps-per-mm are the ones I'm using and almost certainly need 
; to be different for you!
M200 X62.745 Y62.745 Z566.93 E729.99 ;set axis steps per mm
;M200 X62.745 Y62.745 Z2267.718 E729.99 ;set axis steps per mm
M201 X2000 Y2000 Z75 E1500 ;set axis start speeds
M202 X6000 Y6000 Z300 E2000 ;set axis max speeds
M206 X1500 Y1500 Z100 E2000 ;set accel mm/s/s

; LCD setup as per wiki.
M250 P63     ;set LCD RS pin
M251 P42     ;set LCD RW pin
M252 P65     ;set LCD E pin
M253 S4 P59  ;set LCD Data 4 pin
M253 S5 P64  ;set LCD Data 5 pin
M253 S6 P44  ;set LCD Data 6 pin
M253 S7 P66  ;set LCD Data 7 pin - always set last of all LCD pins OR ELSE!
M254 S0 P35  ;set Keypad Row 1 pin
M254 S1 P33  ;set Keypad Row 2 pin
M254 S2 P31  ;set Keypad Row 3 pin
M254 S3 P29  ;set Keypad Row 4 pin
M255 S3 P37  ;set Keypad Col 4 pin
M255 S2 P23  ;set Keypad Col 3 pin
M255 S1 P25  ;set Keypad Col 2 pin
M255 S0 P27  ;set Keypad Col 1 pin - always set last of all Keypad pins OR ELSE!

M216 P9      ;set Fan pin

M211 P5000   ;Autoreport temperatures every 5 seconds

M104 S0      ;Be sure Extruder is off - helpful with EC
M140 S0      ;Be sure Bed is off - helpful with EC
M84          ;Disable all motors
M106         ;Fan on

M350 P1      ;Enable lookahead

M400         ;End EEPROM write/read
