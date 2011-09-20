M402                             ; Write this config to EEPROM
M309 P1 S1                       ; Endstops
M300 X28 Y25 Z22 E15             ; STEP pins
M301 X27 Y24 Z17 E14             ; DIR pins
M302 X26 Y23 Z16 E3              ; ENABLE pins
M304 X12 Y10                     ; MIN pins
M305 Z7                          ; MAX pins
M307 X1 Y1 Z0 E1                 ; Axis Inversion
M308 X0 Y0 Z0 E0                 ; Disable After Move
M200 X47.07 Y47.07 Z200 E44.169  ; Steps per MM
M201 X1000 Y1000 Z100 E4000      ; Start Speed
M202 X5000 Y5000 Z1000 E6000     ; Max speed
M206 X1000 Y1000 Z500 E2000      ; Acceleration
; LCD setup as per sjfw page on reprap wiki.
M250 P47     ;set LCD RS pin
M251 P43     ;set LCD RW pin
M252 P41     ;set LCD E pin
M253 S4 P39  ;set LCD Data 4 pin
M253 S5 P37  ;set LCD Data 5 pin
M253 S6 P35  ;set LCD Data 6 pin
M253 S7 P33  ;set LCD Data 7 pin - always set last of all LCD pins OR ELSE!
M255 S2 P48  ;set Keypad Col 3 pin
M255 S1 P46  ;set Keypad Col 2 pin
M254 S3 P42  ;set Keypad Row 4 pin
M254 S2 P40  ;set Keypad Row 3 pin
M254 S1 P38  ;set Keypad Row 2 pin
M254 S0 P36  ;set Keypad Row 1 pin
M255 S3 P34  ;set Keypad Col 4 pin
M255 S0 P44  ;set Keypad Col 1 pin - always set last of all Keypad pins OR ELSE!
M104 S0                          ; Turn off hotend heat (Important with EC!)
M140 S0                          ; Turn off platform heat (Important with EC!)
M350 P1                          ; Enable lookahead
M211 P5000                       ; Report temperatures every 5 seconds
M84                              ; Disable all Motors
M400                             ; End EEPROM write
