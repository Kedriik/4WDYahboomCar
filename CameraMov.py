import socket
import struct
import time
from threading import Thread
from threading import Lock
import cv2
import urllib 
import numpy as np
from getkeys import key_check
from time import sleep


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

g_CarSpeed = 3;
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
    global g_CarSpeed
    g_CarSpeed = g_CarSpeed + 1;
    if(g_CarSpeed > 10):
        g_CarSpeed = 10;
    
def slower():
    global g_CarSpeed
    g_CarSpeed = g_CarSpeed - 1;
    if(g_CarSpeed < 1):
        g_CarSpeed = 1
def setCarSpeed(carSpeed):
    global g_CarSpeed
    g_CarSpeed = carSpeed;
    if(g_CarSpeed < 1):
        g_CarSpeed = 1
    if(g_CarSpeed > 10):
        g_CarSpeed = 10;
        
def run():
    global g_CarSpeed
    MESSAGE = '$4WD{}{}0000{}00000000000#\n'.format(enRUN,g_CarSpeed,0)
    s.send(MESSAGE.encode())
def back():
    global g_CarSpeed
    MESSAGE = '$4WD{}{}0000{}00000000000#\n'.format(enBACK,g_CarSpeed,0)
    s.send(MESSAGE.encode())
def stop():
    global g_CarSpeed
    MESSAGE = '$4WD{}{}0000{}00000000000#\n'.format(enSTOP,g_CarSpeed,0)
    s.send(MESSAGE.encode())
def left():
    global g_CarSpeed
    MESSAGE = '$4WD{}{}0000{}00000000000#\n'.format(enLEFT,g_CarSpeed,0)
    s.send(MESSAGE.encode())
def right():
    global g_CarSpeed
    MESSAGE = '$4WD{}{}0000{}00000000000#\n'.format(enRIGHT,g_CarSpeed,0)
    s.send(MESSAGE.encode())
def tright():
    global g_CarSpeed
    MESSAGE = '$4WD{}{}0000{}00000000000#\n'.format(enTRIGHT,g_CarSpeed,0)
    s.send(MESSAGE.encode())
def tleft():
    global g_CarSpeed
    MESSAGE = '$4WD{}{}0000{}00000000000#\n'.format(enTLEFT,g_CarSpeed,0)
    s.send(MESSAGE.encode())
g_gather_response = True

def get_response():
    while(g_gather_response):
        current_response  = s.recv(BUFFER_SIZE)
        print(current_response)
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
        sleep(0.1)
    cv2.destroyAllWindows()

#response_thread = Thread(target = get_response)
image_thread = Thread(target = get_image)
#response_thread.start()
image_thread.start()


def keys_to_output(keys):
    #[A,W,D,S]
    output=[0,0,0,0]    
    if 'A' in keys:
        output[0] = 1
    if 'W' in keys:
        output[1] = 1
    if 'D' in keys:
        output[2] = 1
    if 'S' in keys:
        output[3] = 1        
    return output

while(1):
    keys   = key_check();
    if 'A' in keys:
        tleft()
        stop();
    elif 'W' in keys:
        run()
        stop()
    elif 'D' in keys:
        tright()
        stop()
    elif'S' in keys:
        back()
        stop()
    else:
        stop()
    sleep(0.1)
    current_response  = s.recv(BUFFER_SIZE)
    print(current_response)
#response_thread.join();
image_thread.join();
s.shutdown(1);

#$4WD{4WDMOVE}{speed}000{camMove}#

#printf("Checkpoint3\n");
#$00000000600000000000#

