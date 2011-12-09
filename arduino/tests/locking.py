
import serial
import test

if __name__ == '__main__':
   test.load('locking.hex')

   ser = serial.Serial(test.port, test.baud)

   while:
      input = read(26)
      if input != 'abcdefghijklmnopqrstuvwxyz' and 
         input != 'ABCDEFGHIJKLMNOPQRSTUVWXYZ':
        print "Input mismatch %s"%input

