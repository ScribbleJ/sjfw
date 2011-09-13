M402         ;Write this configuration to EEPROM

; Standard Gen6 Pin Settings
M207 P5 S14 ;Set hotend pins, P=thermistor, S=FET
;M208 P14 S8  ;Set platform pins, P=thermistor, S=FET
M309 P1 S1 ;Set endstop inversion and pullups; must happen before M304/M305 OR ELSE!!!!!!
M300 X15 Y23 Z27 E4 ;Set axis STEP pins
M301 X18 Y22 Z28 E2 ;Set axis DIR pins
M302 X19 Y24 Z29 E3 ;Set axis ENABLE pins
M304 X20 Y25 Z30    ;Set axis MIN pins
;M305 X Y Z          ;Set axis MAX pins
M307 X0 Y0 Z0 E0    ;Set axis inversion; 0=no, 1=yes
M308 X0 Y0 Z1 E0    ;Set axis disable-after-move

; These axis speeds and steps-per-mm are the ones I'm using and almost certainly need 
; to be different for you!
M200 X62.745 Y62.745 Z566.93 E729.99 ;set axis steps per mm; must happen before m201/202/203 OR ELSE!
;M200 X62.745 Y62.745 Z2267.718 E729.99 ;set axis steps per mm; must happen before m201/202/203 OR ELSE!
M201 X2000 Y2000 Z75 E1500 ;set axis start speeds
M202 X9000 Y9000 Z300 E2000 ;set axis max speeds
M206 X1000 Y1000 Z100 E2000 ;set accel mm/s/s

M211 P5000   ;Autoreport temperatures every 5 seconds

M104 S0      ;Be sure Extruder is off - helpful with EC
M140 S0      ;Be sure Bed is off - helpful with EC
M84          ;Disable all motors

M350 P1      ;Enable lookahead

M400         ;End EEPROM write/read
