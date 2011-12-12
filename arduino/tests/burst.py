
import serial
import test

if __name__ == '__main__':
   test.load('burst_tx.hex')

   ser = serial.Serial(test.port, test.baud, timeout=1)

   rx_size = 5
   rx_old_size = 0
   rx_old_size_cnt = 0

   while rx_old_size_cnt < 5:
      input = ser.read(rx_size)
      rx_size = len(input)

      byte tx = [rx_size & 0xff, (rx_size >> 8) & 0xff]
      ser.write(tx)
      print "Got burst of size %d"%rx_size

      if rx_size == rx_old_size:
         rx_old_size_cnt++
      else:
         rx_old_size = 0
      rx_old_size = rx_size


   print "Maximum tx burst size: %d"%rx_size
   ser.close()


   test.load('burst_rx.hex')

   ser.open()

   tx_size = 1
   tx_old_size = 0
   tx_old_size_cnt = 0

   while tx_old_size_cnt < 5:
      byte tx = range(tx_size)
      ser.write(tx)

      input = ser.read(2)
      tx_size = input[0] | (input[1] << 8)

      print "Sent burst of %d and got %d"%(tx_old_size + 1, tx_size)

      if tx_size == tx_old_size:
         tx_old_size_cnt++
      else:
         tx_old_size_cnt = 0

      tx_old_size = tx_size
      tx_size++

   print "Maximum rx burst size: %d"%tx_old_size
