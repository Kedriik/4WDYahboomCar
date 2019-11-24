import socket
import struct
import time
from threading import Thread
from threading import Lock
import win32api
import cv2
import urllib 
import numpy as np
from getkeys import key_check
from time import sleep
import tensorflow as tf
import cv2
import label_map_util
import visualization_utils as vis_util

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
MESSAGE = '$4WD{}{}0000{}00000000000#\n'

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
image_state = None

def get_image():
    global image_state
    stream = urllib.request.urlopen('http://192.168.50.1:8080/?action=stream')
    sbytes = bytearray()
    start = '\xff\xd8'.encode('ISO-8859-1')
    end   = '\xff\xd9'.encode('ISO-8859-1')
    while(g_gather_image):
        sbytes += stream.read(1024)
        a = sbytes.find(start)
        b = sbytes.find(end)
        if a != -1 and b != -1:
            jpg = sbytes[a:b+2]
            sbytes = sbytes[b+2:]
            
            img = np.frombuffer(jpg, dtype=np.uint8)
            i = cv2.imdecode(img, cv2.IMREAD_COLOR)
            img_np = cv2.flip(i, 0)
            image_state = cv2.flip(img_np, 1)

def get_image1():
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
    
keys   = []
g_gather_keys = True
g_b_graph_initialized = False
def get_keys_t():
    global keys
    while(g_gather_keys):
        keys = key_check()
        sleep(0.1)

PATH_TO_CKPT =  'frozen_inference_graph.pb'
PATH_TO_LABELS = 'mscoco_label_map.pbtxt'
NUM_CLASSES = 90
label_map = label_map_util.load_labelmap( PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

detection_graph = tf.Graph()
if g_b_graph_initialized == False:
    with detection_graph.as_default():
      od_graph_def = tf.GraphDef()
      with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')
    g_b_graph_initialized = True
g_analyze_image = True
g_detected_items = []
g_running_trackers = []
g_tracked_ids = []
tracker_ID = 0
def bboxToPixels(bbox):
    xmin = int(bbox[1]*float(image_state.shape[1]))
    ymin = int(bbox[0]*float(image_state.shape[0]))
    xmax = int(bbox[3]*float(image_state.shape[1]))
    ymax = int(bbox[2]*float(image_state.shape[0]))
    return (xmin,ymin,xmax-xmin,ymax-ymin)

def isPointInsideBbox(point,bbox):
    xmin = bbox[1]
    ymin = bbox[0]
    xmax = bbox[3]
    ymax = bbox[2]
    if point[0] > xmin and point[0] < xmax and point[1] > ymin and point[1] < ymax:
        return True
    else:
        return False

def showTrackedItems(frame,bbox):
    crop_img = frame[bbox[1]:bbox[1]+bbox[3],bbox[0]:bbox[0]+bbox[2]]
    cv2.imshow("cropped", crop_img)
    
def add_tracker(frame,bbox):
    global tracker_ID
    bbox = bboxToPixels(bbox)
    #TLD the best so far
    #tracker = cv2.TracxkerTLD_create()
    tracker = cv2. cv2.TrackerGOTURN_create()
    tracker.init(frame, bbox)
    print("tracker added")
    g_running_trackers.append(tracker)
    g_tracked_ids.append(tracker_ID)
    tracker_ID =  tracker_ID + 1
    showTrackedItems(frame,bbox)


def track_selected(event,x,y,flags,param):
    global g_detected_items
    global image_np
    if event == cv2.EVENT_LBUTTONDBLCLK:
        print("Chck2")
        click_point = []
        click_point.append(x/image_state.shape[1]);
        click_point.append(y/image_state.shape[0]);
        print(click_point)
        for i in range(len(g_detected_items)):
            if isPointInsideBbox(click_point, g_detected_items[i]) == True:
                add_tracker(image_state,g_detected_items[i])
                 
#
def analyze_image():
    cv2.namedWindow('Vision')
    cv2.setMouseCallback('Vision',track_selected) 
    
    with detection_graph.as_default():
        with tf.Session(graph=detection_graph) as sess:
            while g_analyze_image:
                state_right = win32api.GetKeyState(0x02)  # Right button down = 0 or 1. Button up = -127 or -128
                if state_right == 0 or state_right == 1:
                #screen = cv2.resize(grab_screen(region=(capturing_coordinates)), (capturing_resize))
                    screen = image_state
                #image_np = cv2.cvtColor(screen, cv2.COLOR_BGR2RGB)
                image_np = screen
                g_detected_items.clear();
                if len(g_running_trackers) == 0:
                    # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
                    image_np_expanded = np.expand_dims(image_np, axis=0)
                    image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
                    # Each box represents a part of the image where a particular object was detected.
                    boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
                    # Each score represent how level of confidence for each of the objects.
                    # Score is shown on the result image, together with the class label.
                    scores = detection_graph.get_tensor_by_name('detection_scores:0')
                    classes = detection_graph.get_tensor_by_name('detection_classes:0')
                    num_detections = detection_graph.get_tensor_by_name('num_detections:0')
                    # Actual detection.
                    (boxes, scores, classes, num_detections) = sess.run(
                    [boxes, scores, classes, num_detections],
                    feed_dict={image_tensor: image_np_expanded})
                    ##### boxes: a 2 dimensional numpy array of [N, 4]: (ymin, xmin, ymax, xmax)
                    
                    # Visualization of the results of a detection.
                    vis_util.visualize_boxes_and_labels_on_image_array(
                    image_np,
                    np.squeeze(boxes),
                    np.squeeze(classes).astype(np.int32),
                    np.squeeze(scores),
                    category_index,
                    use_normalized_coordinates=True,
                    line_thickness=1,
                    min_score_thresh=.50)
                    
                    
                    for i,b in enumerate(boxes[0]):
                        if scores[0][i] > 0.50:
                            g_detected_items.append(b.tolist())
                
                for i in range(len(g_running_trackers)):
                    # Update tracker
                    ok, bbox = g_running_trackers[i].update(image_np)
                    # Draw bounding box
                    if ok:
                      # Tracking success
                        p1 = (int(bbox[0]), int(bbox[1]))
                        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                        cv2.rectangle(image_np, p1, p2, (255,255,255), 3, 1)
                    else :
                      # Tracking failure
                      cv2.putText(image_np, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
                cv2.imshow('Vision', image_np)
                if cv2.waitKey(25) & 0xFF==ord('q'):    
                    cv2.destroyAllWindows()
                    break
          
    
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

image_thread = Thread(target = get_image1)
analyze_thread = Thread(target = analyze_image)
keys_thread = Thread(target = get_keys_t)
analyze_thread.start()
image_thread.start()
keys_thread.start();

#wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwssssx
#
while(1):
    if 'X' in keys:
        g_analyze_image = False
        g_gather_image = False
        g_gather_keys = False
        break
        
    if 'J' in keys:
        turnLeft()
    elif 'I' in keys:
        turnUp()
    elif 'L' in keys:
        turnRight()
    elif'K' in keys:
        turnDown()
    elif 'U' in keys:
        init()
        
    if 'A' in keys:
        tleft()
        stop();
    elif 'W' in keys:
        run()
        #stop()
    elif 'D' in keys:
        tright()
        stop()
    elif'S' in keys:
        back()
        stop()
    else:
        stop()
    sleep(0.01)
    current_response  = s.recv(BUFFER_SIZE)
#response_thread.join();
image_thread.join()
keys_thread.join()
analyze_thread.join()
s.shutdown(1);

#$4WD{4WDMOVE}{speed}000{camMove}#

#printf("Checkpoint3\n");
#$00000000600000000000#

