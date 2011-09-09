;Operating system for ATMega2560 on Arduino Mega 2560
;compile with avr-gcc for ATmega2560 @ 16Mhz

.text

.global system
.global system_init
.global yeild

;register usage:
; saved: r2-r17, r28-r29
; local: r18-r27, r30-r31
; function parameters: r25-r8, left-to-right.
;  example foo( (u16) 1025, (u08) 64); r25=2, r24=1, r23=0(empty), r22=64
; function return: 8-bit in r24, 16-bit in r25:r24, 32-bit in r25:r24:r23:r22
; X = r27:r26
; Y = r29:r28
; Z = r31:r30

;assumptions about the code is being run:
; a call stack shouldn't be more than 4 functions deep.
;  this means no recursion!
; a function should not need parameters on the stack (more than 18 bytes)
; a function should not need to use the stack to return (more than 8 bytes)
; largest observed frame size for current code is 23 bytes
;  most functions have a frame size of 0 bytes
;  biggest function might need 32 bytes total (frame and return address)
;  this means a program should only need 4*32=128 bytes of stack space!
;  perhaps allocate a little less, say 64 bytes
;   this affords a depth of about 30 calls

;per-process data structure
; store stack pointer: 2 bytes
; run frequency: 1 byte (every N cycles, 0 for idle)
; last run: 1 byte (N cycles ago)
; priority: 1 byte (lower is better)
;  TOTAL: 5 bytes
;
; push registers and SREG onto the stack

#define P_SZ 5
#define P_CNT 8
#define STACK_SZ 128

#define PORTB 0x05

;total space allocated per-process: 98 bytes
; not bad!

;timer usage
; using timer 0 for preemption
;  servo driver uses timer 1
;  PWM driver uses timer 5

; TODO: process slot allocation/tracking
;  rather than store the number of running processes, store a bitmap of which
;  process slots are in use at any given time.
;
;  process termination just clears the associated bit in the bitmap
;  - the scheduler won't run a process that isn't marked in the bitmap
;  - we should probably give the scheduler an idle process to run if it can't 
;    find anything else to run
;
;  process startup picks the first available slot, clears the associated data
;   structures and starts the process

;Drivers
; should deal with input and output peripheral control
; interrupt routines should be as short as possible
;  make setup routines longer instead, if possible
; most drivers probably ought to be written in assembly

;CODE STARTS HERE

;process preemption timer interrupt (TIMER0 OVF)
.global __vector_23
.global yeild
   .type yeild, @function
__vector_23: 
   push r31
   push r30
   push r29
   push r28
   ; grab SREG before we change it
   in r28, 0x3f ; SREG

   ; get increment the last run counts for all processes
   ldi r31, hi8(process_table+3)
   ldi r30, lo8(process_table+3)

   ld r29, Z
   inc r29
   st Z, r29

   adiw r30, P_SZ
   ld r29, Z
   inc r29
   st Z, r29

   adiw r30, P_SZ
   ld r29, Z
   inc r29
   st Z, r29

   adiw r30, P_SZ
   ld r29, Z
   inc r29
   st Z, r29

   adiw r30, P_SZ
   ld r29, Z
   inc r29
   st Z, r29

   adiw r30, P_SZ
   ld r29, Z
   inc r29
   st Z, r29

   adiw r30, P_SZ
   ld r29, Z
   inc r29
   st Z, r29

   adiw r30, P_SZ
   ld r29, Z
   inc r29
   st Z, r29

   ; increment ticks
   lds r30, ticks
   lds r21, ticks+1
   adiw r30, 1
   sts ticks, r30
   sts ticks+1, r31

   clr r31
   lds r30, ticks+2
   adc r30, r31
   sts ticks+2, r30

   lds r30, ticks+3
   adc r30, r31
   sts ticks+3, r30
   ; done incrementing ticks

   ; restore sreg
   out 0x3f, r28
   pop r28
   pop r29
   pop r30
   pop r31

yeild:
   ;sei  ; enable nested interrupts

;   sbi PORTB, 7
   ; push all of the process registers onto the stack
   push r0
   push r1
   push r2
   push r3
   push r4
   push r5
   push r6
   push r7
   push r8
   push r9
   push r10
   push r11
   push r12
   push r13
   push r14
   push r15
   push r16
   push r17
   push r18
   push r19
   push r20
   push r21
   push r22
   push r23
   push r24
   push r25
   push r26
   push r27
   push r28
   push r29
   push r30
   push r31

   ; grab SREG and push it onto the stack too
   in r29, 0x3f ; SREG
   push r29

   ; grab stack pointer at this point and store
   lds r31, current_pid
   ldi r30, P_SZ ; size of process info structure
   mul r31, r30
   ldi r30, lo8(process_table)
   ldi r31, hi8(process_table)
   add r30, r0
   adc r31, r1 ; index into process table now in Z
   ; store stack pointer
   in r0, 0x3d ; SPL
   in r1, 0x3e ; SPH
   st Z+, r0
   st Z+, r1


   ; SCHEDULER
L2:
   ; priority scheduler
   ; l[]: last run
   ; s[]: scheduled frequency
   ; p[]: priority; lower is favored
;   next = 0;
;   priority = 255;
;   for( i=0; i<num_pids; i++ ) {
;      if( l[i] >= s[i] )
;         if( p[i] < priority ) {
;            priority = p[i];
;            next = i;
;         }
;   }
;   l[next] = 0;
;  
   lds r31, num_pids
   ldi r30, P_SZ

   ; load address of process_table into Y
   ldi r29, hi8(process_table)
   ldi r28, lo8(process_table)

   ser r18   ; priority
   clr r17   ; next
L3:
   mov r27, r31
   dec r27
   ; load s[i]
   mul r30, r27 ; reslult into r1:r0
   add r0, r28
   adc r1, r29
   mov r26, r0
   mov r27, r1
   ; X now has process base
   adiw r26, 2 ; skip past stack pointer
   ld r23, X+ ; schedule
   ld r24, X+ ; last run
   ld r25, X+ ; priority

   cp r24, r23 ; do r24 - r23, set SREG
   brlo L4 ; r24 < r23
   cp r25, r18 ; do r25 - r18
   brsh L4 ; r25 >= r18
   
   ; actually do stuff here!
   mov r18, r25
   mov r17, r31
   dec r17


L4:
   dec r31
   brne L3 ; loop if r31 != 0

   ; next pid is r17
   mul r17, r30
   add r0, r28
   adc r1, r29
   mov r31, r1
   mov r30, r0
   adiw r30, 3 ; offset for last run
   clr r0
   st Z, r0 ; store 0 at last run-time

   mov r30, r17
   ; r30 is now next PID to run
   ; END OF SCHEDULER

   sts current_pid, r30 ; save to SRAM
   ldi r31, P_SZ
   mul r30, r31
   ldi r30, lo8(process_table)
   ldi r31, hi8(process_table)
   add r30, r0
   adc r31, r1 ; Z now has address of next process to run

   ; restore stack pointer.
   ld r28, Z+  ; SPL from SRAM
   ld r29, Z+  ; SPH from SRAM

   cli ; disable interrupts before writing stack pointer
   out 0x3d, r28 ; SPL
   out 0x3e, r29 ; SPH
   ;sei ; enable interrupts now that we're done writing the stack pointer

   pop r28 ; get SREG off stack
   out 0x3f, r28 ; set SREG
   ; restore registers from stack
   pop r31
   pop r30
   pop r29
   pop r28
   pop r27
   pop r26
   pop r25
   pop r24
   pop r23
   pop r22
   pop r21
   pop r20
   pop r19
   pop r18
   pop r17
   pop r16
   pop r15
   pop r14
   pop r13
   pop r12
   pop r11
   pop r10
   pop r9
   pop r8
   pop r7
   pop r6
   pop r5
   pop r4
   pop r3
   pop r2
   pop r1
   pop r0

   ;cbi PORTB, 7

   reti ; return and enable interrupts
   ; measured run-time of <15 uS (about 13.5uS)

.global system_init
   .type system_init, @function
system_init:

   ; for now, I think I only need to initialize the globals, and set up timer
   clr r30
   sts current_pid, r30
   ldi r30, 1
   sts num_pids, r30
   ; timing done by overflow timer of timer0
   ; to enable:
   ;  page 131:
   ;   set WGM 2:0 to 7; fast PWM mode with OCR0A as TOP, interrupt on TOP
   ;
   ;  processor clock: 16MHz
   ; target frequency: 1000Hz (1mS)
   ;  need divisor: 16000
   ;  available prescalers: 1, 8, 64, 256, 1024
   ;  scale by: 250 * 64 ( prescaler 011, page 133 )
   ; Final timer frequency of 1000 Hz
   ;   1.000mS per interrupt
   ;    overhead of < 1.5% (interrupt time of 15uS/1000uS)
   ;
   cli ; disable interrupts for a little bit

   ; TCCR0A (WGM1 = 1; WGM0 = 1)
   ldi r30, 0b00000011
   out 0x24, r30

   ; TCCR0B (WGM2 = 1; CS = 011)
   ldi r30, 0b00001011
   out 0x25, r30

   ; TIMSK0 (TOIE0 = 1)
   ldi r30, 0b00000001
   sts 0x6E, r30 ; use STS instruction; see page 21

   ; OCR0A, set to desired TOP
   ldi r30, 249 ; (250 steps total)
   out 0x27, r30

   ret

.global system ; u08 system( void (*task)(void), u08 sched, u08 pri );
   .type system, @function
; take a function pointer and run it as a new task, assuming there is a free
;  spot in the pid table
; if it returns, it should jump to a cleanup function
; it should start with an empty set of registers
; CHECKING: check if max number of rocesses has been reached, but not for 
;  stack-related problems
system:

   ; I think this means moving the register and sreg initialization from 
   ;  memory to the stack

   mov r18, r24 ; low byte of function pointer 
   mov r19, r25 ; high byte of function pointer

   ; sched in r22
   ; pri in r20

   ; DEBUG
   ;movw r30, r18
   ; function pointer now in Z 
   ;icall
   ; END DEBUG

   ; r24: return value
   ldi r24, 1 ; set error value (makes ruturning easy)
   ; SUMMARY:
   ; we need to zero the register entries in the process table, set the 
   ; stack pointer, set the SREG, and set the return address
   ; we also need to increment the number of running processes
   ; then we can call yeild, and let the task switcher take over

   ; disable interrupts while we increment process count.
   cli
   lds r27, num_pids
   cpi r27, P_CNT      ; check that we're not running too many processes
   brge system_end ; return with error if we are running too many

   ; increment number of running processes 
   mov r25, r27
   inc r25
   sts num_pids, r25
   sei ; done incrementing process count, enable interrupts

   ; set up stack first, then initialize data structure

   ; (using Z to write to new stack)
   ; multiply by STACK_SZ
   ldi r24, STACK_SZ
   mul r24, r27 ; stack pointer offset into r1:r0
   ldi r30, 0xff ; higest ram address (low byte)
   ldi r31, 0x21 ; highet ram address (high byte)
   sub r30, r0 ; compute low byte of new stack pointer
   sbc r31, r1 ; compute high byte of new stack pointer
   clr r1
   clr r0
   ; set return address
   ; apparently, return address is pushed low-byte first onto the stack
   ;
   ; since ret and reti do (PC <- stack), no offset should be needed
   ldi r24, lo8(system_loop)
   st -Z, r24
   ldi r24, hi8(system_loop)
   st -Z, r24
   ; TODO: figure out if this will cause problems and come up with a fix
   st -Z, r1 ; set highest byte to 0. gcc uses 2 bytes for function pointers

   ; store function pointer
   st -Z, r18 ; store low byte of function pointer
   st -Z, r19 ; store high byte of function pointer
   st -Z, r1  ; store 0, since gcc uses 16-bit function pointers. see above
   
   ; store 0 to register locations; next 32 bytes on stack
   ldi r25, 32 ; use r25 for counting
L1:
   st -Z, r1
   dec r25
   brne L1

   ; TODO: figure out why this makes it work...
   st -Z, r1

   ; store SREG
   ldi r25, 0b10000000 ; only I (interrupts) enabled
   st -Z, r25


   ldi r26, P_SZ
   mul r26, r27
   ldi r26, lo8(process_table)
   ldi r27, hi8(process_table)
   add r26, r0
   adc r27, r1 ; now address into process_table is in X
   clr r1 ; reset gcc's zero register
   clr r0
   
   ; store new stack pointer
   st X+, r30
   st X+, r31

   ; I realize that this makes no sense. at this point, I don't care, as long
   ; as I can keep them straight
   st X+, r22 ; schedule
   st X+, r0  ; last run (0)
   st X+, r20 ; priority

   ;looks like we are ready to start our new process!
   rcall yeild

   clr r24
system_end:
   clr r1 ; a litle cleanup
   sei ; enable interrupts
   ret

; void schedule(unsigned char sched);
.global schedule
   .type schedule, @function
schedule:
   lds r31, current_pid
   ldi r30, P_SZ
   mul r30, r31
   ldi r30, lo8(process_table)
   ldi r31, hi8(process_table)
   add r30, r0
   adc r31, r1
   adiw r30, 2 ; offset to schedule
   st Z, r24 ; parameter in r24
   ret

; void priority(unsigned char pri)
.global priority
   .type priority, @function
priority:
   lds r31, current_pid
   ldi r30, P_SZ
   mul r30, r31
   ldi r30, lo8(process_table)
   ldi r31, hi8(process_table)
   add r30, r0
   adc r31, r1
   adiw r30, 4 ; offset to priority
   st Z, r24
   ret

; placeholder for system cleanup routine
system_loop:
   rcall yeild
   rjmp system_loop

   .section .bss
; global OS variables
.global process_table
   .type process_table, @object
   .size process_table, P_SZ*P_CNT ; space for P_CNT processes @P_SZ bytes each
process_table:
   .skip P_SZ*P_CNT,0 ; layout: SPL, SPH, schedule, last, priority
.global current_pid
   .type current_pid, @object
   .size current_pid, 1
current_pid:
   .skip 1,0
.global num_pids
   .type num_pids, @object
   .size num_pids, 1
num_pids:
   .skip 1,0
.global ticks
   .type ticks, @object
   .size ticks, 4
ticks:
   .skip 4,0
