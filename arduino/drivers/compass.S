;HM55B compass module driver
;compile with avr-gcc for ATmega32 @ 16Mhz
;Maximum clock rate 5Mhz, target 4Mhz

; new pins
;SCK:  PG2  , pin 3
;MISO: PG1  , pin 2
;MOSI: PG0  , pin 1
;EN:   PD7  , pin 4

; old pins
;SCK:  PB7  , pin 3
;MIso: PB6  , pin 2
;MOSI: PB5  , pin 1
;EN:   PB4  , pin 4

.text

.global compass_init
   .type compass_init, @function

compass_init:
;  ldi r18,0b10110000  ;load r18
;  in r18,0x17        ; read current direction register
;  sbr r18,0b10110000 ; set up output pins
;  cbr r18,0b01000000 ; set up input pins
;  out 0x17,r18       ; set output register 

   ; set up direction registers
   sbi 0x0A,7         ; PD7 as output

   in r18,0x13        ; read DDRG
   sbr r18,0b00000101 ; output pins PG0 and PG2
   cbr r18,0b00000010 ; input pin PG1
   out 0x13,r18       ; write DDRG

   sbi 0x0B,7         ; set enable high

   in r18,0x14        ; read current input state
   cbr r18,0b00000111 ; set clock and out low, no pull-up on input
   out 0x14,r18       ; write to output/pullup register
   
   mov r19,r18        ; copy
   sbr r19,0b00000100 ; set clock high
   ;sbr r18,0b00000000 ; set nothing
   cbi 0x0B,7         ; set enable low
   ldi r20,4          ; set to 4 for use as loop counter

loop1:                ; enable low, toggle clock 4 times ; initialization pulse
   out 0x14,r19       ; clock high
   dec r20            ; decrement loop counter
   nop                ; synchronize
   out 0x14,r18       ; clock low
   brne loop1         ; repeat loop if(r20!=0)
   
   cbr r18,0b00000111 ; set clock and out low, no pull-up on input
   out 0x14,r18       ; set output
   sbi 0x0B,7         ; set enable high
   
   ret                ; end of function, return

.global compass
   .type compass, @function
compass:
   ;setup direction register
   sbi 0x0A,7         ; PD7 as output

   in r18,0x13        ; read DDRG
   sbr r18,0b00000101 ; output pins PG0 and PG2
   cbr r18,0b00000010 ; input pin PG1
   out 0x13,r18       ; write DDRG

   
   ;start of get compass routine
   in r18,0x12        ; read state of input register
   cbr r18,0b00000111 ; clear all signal bits
   ;sbr r18,0b00000000 ; set enable high
   ;mov r0,r18         ; copy to r0
   ;cbr r18,0b00010000 ; set enable low
   mov r19,r18        ; copy to r19
   mov r20,r18        ; copy to r20
   mov r21,r18        ; copy to r21
   cbi 0x0B,7         ; set enable high
   sbr r18,0b00000101 ; set clock and out high

start_measure:
   ;this and loop2 send start measurement command, 1000
   out 0x14,r18       ; output high, clock high
   sbr r19,0b00000001 ; set output high
   sbr r20,0b00000100 ; set clock high
   out 0x14,r19       ; output high, clock low
   
   ldi r22, 3         ; set counter to 3
   nop
loop2:
   out 0x14,r20       ; clock high, output low
   dec r22            ;  decrement loop counter
   nop                ;  synchronize
   out 0x14,r21       ; clock low, output low
   brne loop2         ;  repeat loop if(r22!=0)   two cycles
   

   ret
   
   ldi r22,255
   ldi r26,255
debug1:
   ;long ass-delay for debugging
   nop
   nop
   nop
   nop
   dec r26
   brne debug1
   ldi r26, 255
   dec r22
   brne debug1
   ;end debug delay

   nop                ;
   nop                ;
   nop                ;
   ldi r26,0          ;
   
.global compass_poll
   .type compass_poll, @function
compass_poll:

   in r18,0x12        ; read state of input register
   cbr r18,0b00000111 ; clear all signal bits
   mov r19,r18        ; copy to r19
   mov r20,r18        ; copy to r20
   mov r21,r18        ; copy to r21
   sbr r18,0b00000101 ; set clock and out high
   sbr r19,0b00000001 ; set output high
   sbr r20,0b00000100 ; set clock high

   clr r23            ; set r23 to 0 , x msb
   clr r24            ; set r24 to 0 , y lsb
   clr r25            ; set r25 to 0 , y msb
   clr r26            ; set r26 to 0; tmp for x lsb

start_read:
   ;loops 3 and 4 send the read sequence 1100
   ;start by sending pulse to EN
   out 0x14,r21       ;output low, clock low
   cbi 0x0B,7         ; set enable low
   nop
   nop
   nop
   sbi 0x0B,7         ; set enable high
   nop
   nop
   nop
   cbi 0x0B,7         ; set enable low
   nop
   nop
   nop
   ldi r22,2          ;
loop3:
   out 0x14,r18       ;output high, clock high
   dec r22            ;
   nop                ;
   out 0x14,r19       ;output high, clock low
   brne loop3         ;
   
   ldi r22,2          ;
loop4:
   out 0x14,r20       ;output low, clock high
   dec r22            ;
   nop                ;
   out 0x14,r21       ;output low, clock low
   brne loop4         ;
   
wait:
   ldi r22,8          ;
   out 0x14,r20       ;clock high
   ldi r27,0          ;
   nop                ;
   nop                ;
read:
   out 0x14,r21       ;clock low
   sbic 0x12,1        ;two or three cycles?
   add r27,r22        ;
   out 0x14,r20       ;clock high
   lsr r22            ;
   brne read          ;
   
   cpi r27,0b00001100 ;
   nop                ;
   nop                ;
;   brne start_read
   brne notready
   
   ldi r22,4          ;
input1:
   out 0x14,r21       ;clock low
   sbic 0x12,1        ;
   ;add r23,r22        ;
   or r23,r22        ;
   out 0x14,r20       ;clock high
   lsr r22            ;
   brne input1        ;
   
   ldi r22,128        ;
input2:
   out 0x14,r21       ;clock low
   sbic 0x12,1        ;
   ;add r26,r22        ;
   or r26,r22        ;
   out 0x14,r20       ;clock high
   lsr r22            ;
   brne input2        ;

   nop
   nop
   nop
   nop
   
   ldi r22,4          ;
input3:
   out 0x14,r21       ;clock low
   sbic 0x12,1        ;
   add r25,r22        ;
   out 0x14,r20       ;clock high
   lsr r22            ;
   brne input3        ;
   
   ldi r22,128        ;
input4:
   out 0x14,r21       ;clock low
   sbic 0x12,1        ;
   add r24,r22        ;
   out 0x14,r20       ;clock high
   lsr r22            ;
   brne input4        ;
   
   mov r22,r26        ;
   out 0x14,r21       ;clock low
   sbi 0x0B,7         ; set enable high
   
   ;fix sign bits
   sbrc r23,2
   ori r23,0b11111000
   
   sbrc r25,2
   ori r25,0b11111000

   ret

notready:
   clr r22
   clr r23
   clr r24
   clr r25
   ret

