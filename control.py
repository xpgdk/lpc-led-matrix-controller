#!/usr/bin/python

import serial
import threading
import time

class SerialLedControl:
	def __init__(self, device):
		self.running = True
		self.serial = serial.serial_for_url(device, baudrate=9600, timeout=3)
		#serial.Serial(device, baudrate=9600, timeout=1)
		#self.reader_thread = threading.Thread(target=self.reader_thread)
		#self.reader_thread.setDaemon(1)
		#self.reader_thread.start()

		self.read()
		self.send("\r")
		self.waitFor("SPI> ")

	def reader_thread(self):
		while self.running:
			data = self.serial.read(self.serial.inWaiting())
			if len(data) > 0:
				print data,

	def shutdown(self):
		self.running = False
		#self.reader_thread.join()

	def send(self, msg):
		for m in msg:
			self.serial.write(m)
			time.sleep(0.001)
			s = self.serial.read(1)
			if m != s:
				print str(ord(m)) + " != " + str(ord(s))

	def read(self):
		return self.serial.read(self.serial.inWaiting())

	def expect(self, expected):
		actual = self.serial.read(len(expected))
		if expected != actual:
			raise Exception("Expected '" + expected + "', got '" + actual + "'")

	def waitFor(self, expected):
		got = ""
		while not got.endswith(expected):
			got = got + self.serial.read(1)

	def set_message(self, msg):
		print "LEN: "+ str(len(msg))
		cmd = "[2 " + str(len(msg)) + " "
		for c in msg:
			cmd = cmd + str(ord(c)) + " "
		cmd = cmd + "0 r:1 ]\r"
		print "CMD: " + cmd
		self.send(cmd)
		self.waitFor("SPI> ")

	def clear_message(self):
		self.send("[3 0 r:1]\r")
		self.waitFor("SPI> ")

control = SerialLedControl("/dev/ttyACM0")
control.send('\r')
control.expect("\nSPI> ")
control.send('\r')
control.expect("\nSPI> ")
#control.send("[1 1 1 1\r\n]")
#control.send('[2 3 66 67 65 0 r:1]\r\n')
control.clear_message()
control.set_message("#F00Hej ")
control.set_message("#0F0m#F00ed ")
control.set_message("#FF0d#0F0ig ")
control.set_message("Marie! ")
control.shutdown()
