#define SREG 0x3f
#define UDR0 0xC6

; 138 cycles per bit at 115200 baud

; original serial RX interrupt takes 61 cycles (including reti)
; original serial TX interrupt takes 68 cycles (including reti)

.text

interrupt:
   ; prologue
   push r29
   push r30
   push r31
   in r30, SREG
   push r30
   ; prologue: 9 cycles

   lds r30, rx_ptr
   lds r31, (rx_ptr+1)
   lds r29, UDR0 
   st Z+, r29
   adiw r30, 1
   andi r30, 0x7
   sts rx_ptr, r30
   sts (rx_ptr+1), r31
   ; body: 15 cycles

   ; footer
   pop r30
   out SREG, r30
   pop r31
   pop r30
   pop r29
   ; footer: 9 cycles

   reti ; 5 cycles
   ; total 9 + 9 + 15 + 5 = 38 cycles
   ; 40% faster
   

.section .bss
.global tx_buffer
   .type tx_buffer, @object
   .size tx_buffer, 4*8
tx_buffer:
   .skip 4*8

.global rx_buffer
   .type rx_buffer, @object
   .size rx_buffer, 4*8
rx_buffer:
   .skip 4*8

.global rx_head
   .type rx_head, @object
   .size rx_head, 1
rx_head:
   .skip 1

.global rx_size
   .type rx_size, @object
   .size rx_size, 1
rx_size:
   .skip 1

.global rx_ptr
   .type rx_ptr, @object
   .size rx_ptr, 2
rx_ptr:
   .skip 2

.global tx_head
   .type tx_head, @object
   .size tx_head, 1
tx_head:
   .skip 1

.global tx_size
   .type tx_size, @object
   .size tx_size, 1
tx_size:
   .skip 1

