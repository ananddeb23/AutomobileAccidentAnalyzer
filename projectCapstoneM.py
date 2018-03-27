import serial
import RPi.GPIO as GPIO      
import os, time
import datetime
from picamera import PiCamera
from time import sleep
from subprocess import call
from __future__ import print_function
from imutils.object_detection import non_max_suppression
from imutils import paths
import numpy as np
import argparse
import imutils
import cv2

# Initialize date time
now = datetime.datetime.now()
# Used to check change in orientation of the car
prevorien = 0
# ALL REFERENCE Information stored in environment variables to be updated securely and on authorisation from RTO
familynumber = os.environ['FAMILYNUM']
policenumber = os.environ['POLNUM']
servernumber = os.environ['SERVERNUM']
hospitalnumber = os.environ['HOSNUM']
limpactthrehold = os.environ['LIMPACT']
uimpactthreshold = os.environ['UIMPACT']
alcoholthreshold = os.environ['ALCOHOLTHRESH']
# Loop counter for image turn. We currently store last 5 image frames
imgTrn = 0
imagelist = ['img0.jpg','img1.jpg', 'img2.jpg', 'img3.jpg', 'img4.jpg']

GPIO.setmode(GPIO.BOARD)    

# Initialized image descriptor to be used for human classification
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

# Enable Serial Communication
port = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=1)
 
# Transmitting AT Commands to the Modem
# '\r\n' indicates the Enter key
 
port.write('AT'+'\r\n')
rcv = port.read(10)
print rcv
time.sleep(1)

# Disable the Echo
port.write('ATE0'+'\r\n')     
rcv = port.read(10)
print rcv
time.sleep(1)

# Select Message format as Text mode 
port.write('AT+CMGF=1'+'\r\n')  
rcv = port.read(10)
print rcv
time.sleep(1)

# New SMS Message Indications 
port.write('AT+CNMI=2,1,0,0,0'+'\r\n')  
rcv = port.read(10)
print rcv
time.sleep(1)
 
# Sending a message to a particular Number
 
def sendsms(number, message):
    port.write('AT+CMGS="'+number+"'+'\r\n')
    rcv = port.read(10)
    print rcv
    time.sleep(1)
 
    port.write(message'+'\r\n')  # Message
    rcv = port.read(10)
    print rcv
  # Enable to send SMS
    port.write("\x1A")
    for i in range(10):
    rcv = port.read(10)
    print rcv

def checkhit():
    hitcount = 0
    for imagePath in imagelist:
	# load the image and resize it to (1) reduce detection time
	# and (2) improve detection accuracy
	image = cv2.imread(imagePath)
	image = imutils.resize(image, width=min(400, image.shape[1]))
	orig = image.copy()
 
	# detect people in the image
	(rects, weights) = hog.detectMultiScale(image, winStride=(4, 4),
		padding=(8, 8), scale=1.05)
 
	# draw the original bounding boxes
	for (x, y, w, h) in rects:
		cv2.rectangle(orig, (x, y), (x + w, y + h), (0, 0, 255), 2)
 
	# apply non-maxima suppression to the bounding boxes using a
	# fairly large overlap threshold to try to maintain overlapping
	# boxes that are still people
	rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
	pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)
 
	# the number of bounding boxes if more than zero means some human part detected
        if(len(rects) > 0):
               hitcount = hitcount + 1
    if(hitcount >0):
        # Send probability human hit      
        return (hitcount/5)
    return 0
               
# Stop execution of program               
def halt():
    exit()
# Stop execution of program after time in minues
def haltafter(timeval):
    time.sleep(timeval*60)
    exit()
               
while(1):
               # Periodic input from Arduino(Microcontroller)
               read_ser = serial.readline()
               input_list = read_ser.split(';')
               force = float(input_list[0][input_list[0].index('=') +1 :]);
               lat = float(input_list[1][input_list[1].index('=') +1 :]);
               lon = float(input_list[2][input_list[2].index('=') +1 :]);
               orien = int(input_list[3][input_list[3].index('=') +1 :]);
               alcohol = float(input_list[4][input_list[4].index('=') +1 :]);
               trn = "img"+ str(imgTrn)+".jpg)
               imgTrn = imgTrn + 1
               imgTrn = imgTrn%5

               # Periodic capture image frame. This camera is placed in front of the car
               camera.capture(trn)

               # Check change in the orientation of the car( Car overturn)
               if(orien != prevorien && prevorien!=0):
                   curtime = now.strftime("%Y-%m-%d %H:%M")
                   # contruct message
                   message = carid+" has overturned at latitude "+lat+ " longtitude "+lng+" at "+curtime
                   sendsms(familynumber, message)
                   sendsms(servernumber, message)
                   # Halt car
                   halt()
               prevorien = orien
               
                # Driver has alcohol level over allowed levels
               if(alcohol > alcoholthreshold):
                   curtime = now.strftime("%Y-%m-%d %H:%M")
                   # contruct message
                   message = carid+" has crossed alcohol limit at latitude "+lat+ " longtitude "+lng+" alcohol level "+alcohol+ "at "+curtime
                   sendsms(policenumber, message)
                   sendsms(servernumber, message)
                   # Halt car after beffer time to park the car
                   haltafter(3)
               
               if(force > limpactthreshold AND force < uimpactthreshold):
                    # Impact recieved is enough to be substantial for object hit but not severe for people inside the car
                    # Take immediate snap of driver. 
                    call(["fswebcam", "-d", "/dev/video1", "-r", "1280x720", "--no-banner", "./%d.jpg" % i])
                    # Check if human has been hit
                    human_hit = checkhit()
                    if(human_hit >0):
                                  curtime = now.strftime("%Y-%m-%d %H:%M")
                                  message = carid+" has hit a human with probability "+human_hit+" at  latitude "+lat+ " longtitude "+lng+" alcohol level "+alcohol+ "at "+curtime
                                  sendsms(policenumber, message)
                                  sendsms(servernumber, message)
               if(force >= uimpactthreshold):
                    # Impact severe. People inside the car need medical assistance
                    curtime = now.strftime("%Y-%m-%d %H:%M")
                    # contruct message
                    message = carid+" has met with an accident at  latitude" +lat+ " longtitude "+lng+ "at "+curtime
                    sendsms(policenumber, message)
                    sendsms(servernumber, message)
                    sendsms(hospitalnumber, message)
                    sendsms(familynumber, message)
                    call(["fswebcam", "-d", "/dev/video1", "-r", "1280x720", "--no-banner", "./%d.jpg" % i])
                    # Also check if human has been hit too
                    human_hit = checkhit()
                    if(human_hit >0):
                                  # contruct message
                                  message = carid+" has hit a human with probability "+human_hit+" at  latitude "+lat+ " longtitude "+lng+" alcohol level "+alcohol+ "at "+curtime
                                  sendsms(policenumber, message)
                                  sendsms(servernumber, message)

                   
                       
               
               
