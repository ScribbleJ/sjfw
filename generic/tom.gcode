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
M202 X4800 Y4800 Z1000 E6000     ; Max speed
M206 X300 Y300 Z500 E2000        ; Acceleration
M104 S0                          ; Turn off hotend heat (Important with EC!)
M140 S0                          ; Turn off platform heat (Important with EC!)
M350 P1                          ; Enable lookahead
M211 P5000                       ; Report temperatures every 5 seconds
M84                              ; Disable all Motors
M400                             ; End EEPROM write
