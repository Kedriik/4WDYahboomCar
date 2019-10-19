import socket
import struct
import time
from threading import Thread
from threading import Lock
import cv2
import urllib 
import numpy as np


TCP_IP = "192.168.50.1"
TCP_PORT = 8888
BUFFER_SIZE = 1024
MESSAGE = "d\n"
response = ""
enFRONTSERVOLEFT = 1
enFRONTSERVORIGHT= 2
enSERVOUP = 3
enSERVODOWN = 4
enSERVOUPDOWNINIT = 5
enSERVOLEFT = 6
enSERVORIGHT = 7
enSERVOSTOP = 8
enSERVOFRONTINIT = 9

enSTOP = 0
enRUN = 1
enBACK = 2
enLEFT = 3
enRIGHT = 4
enTLEFT = 5
enTRIGHT = 6

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))
#response  = s.recv(BUFFER_SIZE)
#print(response.decode('utf-8'))    
MESSAGE = '$4WD{}{}0000{}00000000000#\n'
#4WDControl = 0;
#cameraControl = 0
def turnLeft(relax = 3):
    MESSAGE = '$4WD00000{}00000000000#\n'.format(enSERVOLEFT)
    s.send(MESSAGE.encode())
    
def turnRight(relax = 3):
    MESSAGE = '$4WD00000{}00000000000#\n'.format(enSERVORIGHT)
    s.send(MESSAGE.encode())
    
def turnUp(relax = 3):
    MESSAGE = '$4WD00000{}00000000000#\n'.format(enSERVOUP)
    s.send(MESSAGE.encode())
    
def init(relax = 3):
    MESSAGE = '$4WD00000{}00000000000#\n'.format(enSERVOFRONTINIT)
    s.send(MESSAGE.encode())
    
def turnDown(relax = 3):
    MESSAGE = '$4WD00000{}00000000000#\n'.format(enSERVODOWN)
    s.send(MESSAGE.encode())

def faster():
    MESSAGE = '$4WD{}{}0000{}00000000000#\n'.format(1,1,0)
    s.send(MESSAGE.encode())
    
def slower():
    MESSAGE = '$4WD{}{}0000{}00000000000#\n'.format(1,2,0)
    s.send(MESSAGE.encode())
def run():
    MESSAGE = '$4WD{}{}0000{}00000000000#\n'.format(enRUN,0,0)
    s.send(MESSAGE.encode())
def back():
    MESSAGE = '$4WD{}{}0000{}00000000000#\n'.format(enBACK,0,0)
    s.send(MESSAGE.encode())
def stop():
    MESSAGE = '$4WD{}{}0000{}00000000000#\n'.format(enSTOP,0,0)
    s.send(MESSAGE.encode())
def left():
    MESSAGE = '$4WD{}{}0000{}00000000000#\n'.format(enLEFT,0,0)
    s.send(MESSAGE.encode())
def right():
    MESSAGE = '$4WD{}{}0000{}00000000000#\n'.format(enRIGHT,0,0)
    s.send(MESSAGE.encode())
def tright():
    MESSAGE = '$4WD{}{}0000{}00000000000#\n'.format(enTRIGHT,0,0)
    s.send(MESSAGE.encode())
def tleft():
    MESSAGE = '$4WD{}{}0000{}00000000000#\n'.format(enTLEFT,0,0)
    s.send(MESSAGE.encode())
g_gather_response = True

def get_response():
    while(g_gather_response):
        current_response  = s.recv(BUFFER_SIZE)
        #if(current_response != response):
            #response = current_response
            #print(response.decode('utf-8'))
        
g_gather_image = True
def get_image():
    while(g_gather_image):
        stream = urllib.request.urlopen('http://192.168.50.1:8080/?action=snapshot')
        
        nparr = np.fromstring(stream.read(), np.uint8)
        img_np = cv2.imdecode(nparr,cv2.IMREAD_COLOR)
        img_np = cv2.flip(img_np, 0)
        img_np = cv2.flip(img_np, 1)
        cv2.imshow('i', img_np)
        
        if cv2.waitKey(25) & 0xFF==ord('q'):    
            cv2.destroyAllWindows()
            break
        
    cv2.destroyAllWindows()

response_thread = Thread(target = get_response)
image_thread = Thread(target = get_image)
response_thread.start()
image_thread.start()


    
while(1):
    print("type a command")
    command = input();
    if command == 'exit':
        g_gather_response = False;
        g_gather_image = False
        break
    elif command == 'cleft':
        turnLeft()
    elif command == 'cinit':
        init()
    elif command == 'cright':
        turnRight()
    elif command == 'cup':
        turnUp()
    elif command == 'cdown':
        turnDown()
    elif command == 'slower':
        slower()
    elif command == 'faster':
        faster()
    elif command == 'run':
        run()
    elif command == 'back':
        back()
    elif command == 'stop':
        stop()
    elif command == 'right':
        right()
    elif command == 'left':
        left()
    elif command == 'tright':
        tright()
    elif command == 'tleft':
        tleft()        
    else:
        MESSAGE = command + '\n'
        s.send(MESSAGE.encode())

response_thread.join();
image_thread.join();
s.shutdown(1);

#$4WD{4WDMOVE}{speed}000{camMove}#

#printf("Checkpoint3\n");
#$00000000600000000000#

