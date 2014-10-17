#!/usr/bin/python
import serial

hora = '9'
minutos = '25'
ser = serial.Serial('/dev/ttyACM0',57600,timeout=1)
print ser.readline()
read = raw_input('Presiona Enter y la opcion')
ser.write('\r')
if read == '1':
   hora = raw_input('Dame la hora a escribir: ')
   ser.write('1')
   ser.write(hora)
   ser.write('\n')
   minutos = raw_input('Dame los minutos a escribir: ')
   ser.write('2')
   ser.write(minutos)
   ser.write('\n')
elif read == '2':
   ser.write('3')
   archivo = open("imagen.raw","r")
   ser.write(archivo.read())
   print ser.readline()
else:
   print('opcion no valida')

   

print('Saliendo del programa BzzzZZZZ')
