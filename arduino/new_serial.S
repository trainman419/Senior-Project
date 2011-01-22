#define SREG 0x3f
#define UDR0 0xC6

; 138 cycles per bit at 115200 baud

; original serial RX interrupt takes 61 cycles (including reti)
; original serial TX interrupt takes 68 cycles (including reti)

.text

rx_interrupt:
   ; prologue
   push r29
   push r30
   push r31
   in r30, SREG
   push r30
   ; prologue: 9 cycles

   lds r30, rx_ptr_head
   lds r31, (rx_ptr_head+1)
   lds r29, UDR0 
   st Z+, r29
   andi r30, 0x7
   sts rx_ptr_head, r30
   sts (rx_ptr_head+1), r31
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
   
tx_interrupt:
   push r29
   push r30
   push r31
   in r30, SREG
   push r30
   ; prologue: 4*2 + 1 = 9

   lds r30, tx_ptr_tail
   lds r31, (tx_ptr_tail+1) 
   lds r29, tx_ptr_head
   ; if( r29 == r31 )
   ; if( tx_ptr_head == tx_ptr_tail ) 
   ;    return
   cpse r29, r31
   ; TODO: disable data register empty interrupt
   rjmp TX_1

   ld r29, Z+
   sts UDR0, r29
   andi r30, 0xF8
   sts tx_ptr_tail, r30
   sts (tx_ptr_tail+1), r31
   ; body: 7*2 + 1*3 + 1 = 18

TX_1:
   ; footer
   pop r30
   out SREG, r30
   pop r31
   pop r30
   pop r29
   reti
   ; footer: 4*2 + 1 + 5 = 14
   ; total: 9 + 14 + 18 = 41
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

.global rx_ptr_head
   .type rx_ptr_head, @object
   .size rx_ptr_head, 2
rx_ptr_head:
   .skip 2

.global rx_ptr_tail
   .type rx_ptr_tail, @object
   .size rx_ptr_tail, 2
rx_ptr_tail:
   .skip 2

.global tx_ptr_head
   .type tx_ptr_head, @object
   .size tx_ptr_head, 2
tx_ptr_head:
   .skip 2

.global tx_ptr_tail
   .type tx_ptr_tail, @object
   .size tx_ptr_tail, 2
tx_ptr_tail:
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

