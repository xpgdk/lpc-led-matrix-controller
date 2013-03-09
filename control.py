#!/usr/bin/python

import serial
import threading
import time

from PIL import Image

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

	def sendW(self, str):
		self.send(str + "\r")
		self.waitFor("SPI> ")

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

	def flip(self):
		self.send("[5 0 r:1]\r")
		self.waitFor("SPI> ")

	def put_pixels(self, startX, startY, endX, endY):
		startXhigh = str((startX >> 8) & 0xFF)
		startXlow = str(startX & 0xFF)
		startYhigh = str((startY >> 8) & 0xFF)
		startYlow = str(startY & 0xFF)

		endXhigh = str((endX >> 8) & 0xFF)
		endXlow = str(endX & 0xFF)
		endYhigh = str((endY >> 8) & 0xFF)
		endYlow = str(endY & 0xFF)

		self.sendW("[ 4 ")
		self.sendW(startXhigh + " " + startXlow + " ")
		self.sendW(startYhigh + " " + startYlow + " ")
		self.sendW(endXhigh + " " + endXlow + " ")
		self.sendW(endYhigh + " " + endYlow + " ")

		pixelCount = (endX-startX)*(endY-startY)

		for c in range(0, pixelCount):
			print c
			self.sendW("32 0 ")
		self.sendW("]")

	def send_image(self, filename, startX, startY, endX, endY):
		image = Image.open(filename)
		px = image.load()

		startXhigh = str((startX >> 8) & 0xFF)
		startXlow = str(startX & 0xFF)
		startYhigh = str((startY >> 8) & 0xFF)
		startYlow = str(startY & 0xFF)

		endXhigh = str((endX >> 8) & 0xFF)
		endXlow = str(endX & 0xFF)
		endYhigh = str((endY >> 8) & 0xFF)
		endYlow = str(endY & 0xFF)

		self.sendW("[ 4 ")
		self.sendW(startXhigh + " " + startXlow + " ")
		self.sendW(startYhigh + " " + startYlow + " ")
		self.sendW(endXhigh + " " + endXlow + " ")
		self.sendW(endYhigh + " " + endYlow + " ")

		for y in range(startY, endY):
			for x in range(startX, endX):
				pixel = px[x,y]
				r = pixel[0]
				g = pixel[1]
				r = int(r/255.0 * 32)
				g = int(g/255.0 * 32)
				#print "Red: ",pixel[0],"=",r
				self.sendW(str(r) + " " + str(g) + " ")
		self.sendW("]")


control = SerialLedControl("/dev/ttyACM0")
control.send('\r')
control.expect("\nSPI> ")
control.send('\r')
control.expect("\nSPI> ")
control.clear_message()
#control.put_pixels(0, 0, 20, 5)
control.send_image("test.png", 0, 0, 16, 16) 
control.flip()
#control.set_message("#2000Hej ")
#control.set_message("med ")
#control.set_message("dig ")
control.shutdown()
