# Need to find a max number of frames we want the fifo buffer to hold
import cv2
import numpy as np
import socket
import time
import threading
import Queue
import struct
from timeit import default_timer as timer
import os

print cv2. __version__
##lower_range = np.array([160,200,119])
##upper_range = np.array([180,255,255])
#lower_range = np.array([47,150, 37])
#upper_range = np.array([85,255,255])

#widthMin = 100
#heightMin = 0
#widthmax = 10000
#heightmax = 10000
#tempString = "0000"

UDP_IP = "10.8.42.2"
UDP_PORT = 8420

sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # UDP

# define range of green color in HSV
lower_range = np.array([47,15,110])
upper_range = np.array([85,255,255])

class frameThread(threading.Thread):
    def __init__(self, threadID, name, multiImageQ):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.multiImageQ = multiImageQ

    def run(self):
        print "Starting " + self.name
        grabFrames()
        print "Ending " + self.name

class filterThread (threading.Thread):
    def __init__(self, threadID, name, multiImageQ, multiImageQ2):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.multiImageQ = multiImageQ
        self.multiImageQ2 = multiImageQ2

    def run(self):
        print "Starting " + self.name
        filterFrame()
        print "Ending " + self.name

class contourThread(threading.Thread):
    def __init__(self, threadID, name, multiImageQ2):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.multiImageQ2 = multiImageQ

    def run(self):
        print "Starting " + self.name
        #filterContours()
        print "Ending " + self.name

class MultiImage(object):
    hsv = None
    time = None
    frame = None

def grabFrames():
    while True:
        try:
            print "test"
            capIndex = 0
            while True:
                try:
                    cap = cv2.VideoCapture(capIndex)
                    cap.set(3, 1920)
                    cap.set(4, 1080)
                    cap.set(5, 30)
                    if cap.isOpened():
                        break
                    cap.release()
                    capIndex = (capIndex + 1) % 5
                    time.sleep(0.5)
                except:
                    pass
            #cap.set(3, 1920)
            #cap.set(4, 1080)
            #cap.set(5, 30)

            #time.sleep(2)
            #cap.set(15, -3.0)
            try:
                os.system("v4l2-ctl --set-ctrl=exposure_auto=0")
                os.system("v4l2-ctl --set-ctrl=exposure_absolute=5")
            except:
                pass
            failCount = 0
            savecount = 0
            while(True):
                multiImage = MultiImage()
                time3 = time.time()
                ret, frame = cap.read()
                if ret == True:
                    failCount = 0
                    #int(round(time.time() * 1000))
                    multiImage.time = time3
                    multiImage.frame = frame
                    multiImageQ.put(multiImage)
                    #cv2.imwrite('/run/image.jpg',frame)
                    savecount += 1
                    if savecount > 25:
                        cv2.imwrite('/run/image.jpg',frame)
                        savecount = 0
                else:
                    failCount += 1
                    if failCount > 5:
                        break
            #cap.release()
            print "Camera Failed. Trying Again\n"
        except:
            pass

def filterFrame():
    #Wait for camera and vision mode
    camera = 1
    vision_mode = 1

    #Assign undistort coeficients based on which camera odroid was assigned to use
    if camera == 1:
        #skew=1.9667
        mtx = np.array([[1282.2859, 0.0, 915.5393],
                    [0.0, 1278.0055, 493.5490],
                    [0.0, 0.0, 1.0]], dtype=np.float32)
        dist = np.array([[-0.4313, 0.2558, 0.0012, 0.0008, -0.0939]], dtype=np.float32)
    elif camera == 2:
        #skew=13.8924
        mtx = np.array([[1361.6363, 0.0, 894.7961],
                    [0.0, 1364.1334, 489.0682],
                    [0.0, 0.0, 1.0]], dtype=np.float32)
        dist = np.array([[-0.4742, 0.3069, 0.0016, 0.0032, -0.1129]], dtype=np.float32)
    elif camera == 3:
        #skew=6.2909
        mtx = np.array([[1283.0860, 0.0, 978.3822],
                    [0.0, 1279.5974, 475.5994],
                    [0.0, 0.0, 1.0]], dtype=np.float32)
        dist = np.array([[-0.4942, 0.4647, 0.0053, -0.0041, -0.2870]], dtype=np.float32)
    elif camera == 4:
        #skew=4.8317
        mtx = np.array([[1277.0419, 0.0, 949.1053],
                    [0.0, 1276.2433, 581.0923],
                    [0.0, 0.0, 1.0]], dtype=np.float32)
        dist = np.array([[-0.4205, 0.2484, -0.0045, 0.0000, -0.0924]], dtype=np.float32)
    elif camera == 5:
        #skew=4.9563
        mtx = np.array([[1275.5074, 0.0, 970.0742],
                    [0.0, 1267.5058, 567.1792],
                    [0.0, 0.0, 1.0]], dtype=np.float32)
        dist = np.array([[-0.4151, 0.2550, -0.0001, 0.0042, -0.1151]], dtype=np.float32)
    elif camera == 6:
        #skew=-9.4773
        mtx = np.array([[1517.7481, 0.0, 961.2901],
                    [0.0, 1505.2944, 509.1859],
                    [0.0, 0.0, 1.0]], dtype=np.float32)
        dist = np.array([[-0.6328, 0.5593, 0.0204, 0.0145, -0.0694]], dtype=np.float32)
    delay = timer()
    delay_time = timer()

    while True:
        try:
            if not multiImageQ.empty():
                while (multiImageQ.qsize() > 0): #grab most recent frame
                    multiImage = multiImageQ.get()
                    print "Got new image\n"

                frame = multiImage.frame

                delay = timer()
                newTime = (delay - delay_time) * 1000
                print str(newTime)

                if vision_mode == 9:
                    #Shutdown odroid
                    vision_mode = vision_mode

                if vision_mode == 1 or vision_mode == 3:
                    widthMin = 50
                    heightMin = 10
                    widthmax = 10000
                    heightmax = 10000
                    cx = 240
                    cy = 135
                elif vision_mode == 5:
                    widthMin = 10
                    heightMin = 20
                    widthmax = 10000
                    heightmax = 10000
                    cx = 240
                    cy = 135

                scale = 960.0/cx
                scaley = 540.0/cy

                mode = 6
                count = 0
                heights = []
                widths = []
                x1 = []
                y1 = []
                centroidInfo = []
                fcenterX = 1.0
                fY = 1.0

                start = timer()
                frame = cv2.resize(frame,(1920,1080), interpolation=cv2.INTER_NEAREST)
                frame2 = cv2.resize(frame.copy(),(cx*2,cy*2), interpolation=cv2.INTER_NEAREST)
                height, width, channels = frame.shape

                centerCal = 960, 540
                centerPoints = np.array([[centerCal]], dtype=np.float32)
                centerOff = cv2.undistortPoints(centerPoints, mtx, dist)

                #frame = cv2.pyrUp(frame)
                hsv = cv2.cvtColor(frame2.copy(), cv2.COLOR_BGR2HSV) #Allows filtering in hsv
                mask = cv2.inRange(hsv, lower_range, upper_range) #Filters

                contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                #Process contours
                for cnt in contours:
                    x,y,w,h = cv2.boundingRect(cnt)
                    width = w
                    height = h

                    if (width<widthmax/scale) and (height <heightmax/scaley) and (width >= widthMin/scale) and (height > heightMin/scaley):

                        pt1 = x, y
                        pt2 = x+width, y+height

                        centerX = x + (width/2)
                        centerY = y + (height/2)

                        #Undistort Points
                        #points = np.array([[pt1], [pt2]], dtype=np.float32)
                        #output = cv2.undistortPoints(points, mtx, dist)
                        #newX = output[0][0][0]
                        #newY = output[0][0][1]
                        #newX2 = output[1][0][0]
                        #newY2 = output[1][0][1]

                        x1.append(x*scale)
                        y1.append(y*scaley)
                        widths.append(width*scale)
                        heights.append(height*scaley)
                        centroidInfo.append(cnt)
                        #widths.append(abs(newX2 - newX))
                        #heights.append(abs(newY2 - newY))

                        #cv2.rectangle(frame2, pt1, pt2, (0,255,0), 4)
                        count += 1

                if vision_mode == 1 or vision_mode == 3:
                    if count == 2:
                        if vision_mode == 1:
                            mode = 1
                        else:
                            mode = 3
                        if y1[0] < y1[1]:
                            #contour 0 is higher
                            fwidth = (widths[0])
                            fcenterX = x1[0] + (fwidth / 2)
                            fheight = y1[1] + heights[1] - y1[0]
                            fY = y1[0] + (fheight / 2)

                        else:
                            #contour1 is higher
                            fwidth = fwidth = (widths[1])
                            fcenterX = x1[0] + (fwidth / 2)
                            fheight = y1[0] + heights[0] - y1[1]
                            fY = y1[1] + (fheight / 2)

                    elif count == 1:
                        #For now have count == 1 do nothing
                        if vision_mode == 1:
                            mode = 2
                        else:
                            mode = 4
                        fY = 0
                        fcenterX = 0
                    else:
                        if vision_mode == 1:
                            mode = 2
                        else:
                            mode = 4
                        fY = 0
                        fcenterX = 0
                elif vision_mode == 5:
                    #Handle different vision scenarios
                    if count == 4:
                        mode = 5
                        area1 = widths[0] * heights[0]
                        area2 = widths[1] * heights[1]
                        area3 = widths[2] * heights[2]
                        area4 = widths[3] * heights[3]
                        areas = [area1, area2, area3, area4]

                        min_pos = areas.index(min(areas))
                        areas.pop(min_pos)
                        widths.pop(min_pos)
                        heights.pop(min_pos)
                        x1.pop(min_pos)
                        y1.pop(min_pos)
                        min_pos = areas.index(min(areas))
                        areas.pop(min_pos)
                        widths.pop(min_pos)
                        heights.pop(min_pos)
                        x1.pop(min_pos)
                        y1.pop(min_pos)

                        M0 = cv2.moments(centroidInfo[0])
                        M1 = cv2.moments(centroidInfo[1])
                        fcenter1 = int(M0['m10']/M0['m00']) * scale
                        fcenter2 = int(M1['m10']/M1['m00']) * scale
                        fcenterX = (fcenter1 + fcenter2) / 2

                        if y1[0] + heights[0] < y1[1] + heights[1]:
                            fY = y1[1] + heights[1]
                        else:
                            fY = y1[0] + heights[0]

                        #Undistort Points
                        distPoint = fcenterX, fY
                        points = np.array([[distPoint]], dtype=np.float32)
                        output = cv2.undistortPoints(points, mtx, dist)
                        fcenterX = output[0][0][0]
                        fY = output[0][0][1]
                    elif count == 3:
                        mode = 5
                        area1 = widths[0] * heights[0]
                        area2 = widths[1] * heights[1]
                        area3 = widths[2] * heights[2]
                        areas = [area1, area2, area3]

                        min_pos = areas.index(min(areas))
                        areas.pop(min_pos)
                        widths.pop(min_pos)
                        heights.pop(min_pos)
                        x1.pop(min_pos)
                        y1.pop(min_pos)

                        M0 = cv2.moments(centroidInfo[0])
                        M1 = cv2.moments(centroidInfo[1])
                        fcenter1 = int(M0['m10']/M0['m00']) * scale
                        fcenter2 = int(M1['m10']/M1['m00']) * scale
                        fcenterX = (fcenter1 + fcenter2) / 2

                        if y1[0] + heights[0] < y1[1] + heights[1]:
                            fY = y1[1] + heights[1]
                        else:
                            fY = y1[0] + heights[0]

                        #Undistort Points
                        distPoint = fcenterX, fY
                        points = np.array([[distPoint]], dtype=np.float32)
                        output = cv2.undistortPoints(points, mtx, dist)
                        fcenterX = output[0][0][0]
                        fY = output[0][0][1]
                    elif count == 2:
                        mode = 5

                        M0 = cv2.moments(centroidInfo[0])
                        M1 = cv2.moments(centroidInfo[1])
                        fcenter1 = int(M0['m10']/M0['m00']) * scale
                        fcenter2 = int(M1['m10']/M1['m00']) * scale
                        fcenterX = (fcenter1 + fcenter2) / 2

                        if y1[0] + heights[0] < y1[1] + heights[1]:
                            fY = y1[1] + heights[1]
                        else:
                            fY = y1[0] + heights[0]

                        #Undistort Points
                        distPoint = fcenterX, fY
                        points = np.array([[distPoint]], dtype=np.float32)
                        output = cv2.undistortPoints(points, mtx, dist)
                        fcenterX = output[0][0][0]
                        fY = output[0][0][1]
                    else:
                        mode = 6
                        fcenterX = 0
                        fY = 0

                ######################
                if vision_mode == 1 or vision_mode == 3:
                    cx = 240
                    cy = 135
                    scale = 960.0/cx

                    print "Center " + str(fcenterX) + "," + str(fY)
                    if fcenterX < cx:
                        fcenterX = cx
                    if fcenterX > 1920-cx:
                        fcenterX = 1920-cx
                    if fY < cy:
                        fY = cy
                    if fY > 1080-cy:
                        fY = 1080-cy

                    x1 = []
                    y1 = []
                    widths = []
                    heights = []
                    centroidInfo = []
                    print y1
                    leftOffset = fcenterX - cx
                    topOffset = fY - cy
                    count = 0
                    frame = frame[fY - cy:fY + cy, fcenterX - cx:fcenterX + cx]
                    hsv2 = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2HSV) #Allows filtering in hsv
                    mask2 = cv2.inRange(hsv2, lower_range, upper_range) #Filters

                    contours2, hierarchy2 = cv2.findContours(mask2.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                    #Process contours
                    for cnt in contours2:
                        x,y,w,h = cv2.boundingRect(cnt)
                        width = w
                        height = h
                        if (width<widthmax) and (height <heightmax) and (width >= widthMin) and (height > heightMin):
                            pt1 = x, y
                            pt2 = x+width, y+height
                            cv2.rectangle(frame, pt1, pt2, (0,255,0), 4)

                            x = x #+ fcenterX - cx
                            y = y #+ fcenterX - cy
                            pt1x = x, y
                            pt2y = x+width, y+height
                            centerX = (x + (width/2))
                            centerY = (y + (height/2))

                            centroidInfo.append(cnt)
                            x1.append(x)
                            y1.append(y)
                            widths.append(width)
                            heights.append(height)
                            count += 1

                    if vision_mode == 1 or vision_mode == 3:
                        if count == 2:
                            if vision_mode == 1:
                                mode = 1
                            else:
                                mode = 3
                            if y1[0] < y1[1]:
                                #contour 0 is higher
                                M = cv2.moments(centroidInfo[0])
                                fcenterX = int(M['m10']/M['m00']) + leftOffset
                                fY = int(M['m01']/M['m00']) + topOffset
                                #Undistort Points
                                distPoint = fcenterX, fY
                                points = np.array([[distPoint]], dtype=np.float32)
                                output = cv2.undistortPoints(points, mtx, dist)
                                fcenterX = output[0][0][0]
                                fY = output[0][0][1]
                            else:
                                #contour1 is higher
                                M = cv2.moments(centroidInfo[1])
                                fcenterX = int(M['m10']/M['m00']) + leftOffset
                                fY = int(M['m01']/M['m00']) + topOffset
                                #Undistort Points
                                distPoint = fcenterX, fY
                                points = np.array([[distPoint]], dtype=np.float32)
                                output = cv2.undistortPoints(points, mtx, dist)
                                fcenterX = output[0][0][0]
                                fY = output[0][0][1]

                        elif count == 1:
                            #For now have count == 1 do nothing
                            if vision_mode == 1:
                                mode = 2
                            else:
                                mode = 4
                            fY = 0
                            fcenterX = 0
                        else:
                            if vision_mode == 1:
                                mode = 2
                            else:
                                mode = 4
                            fY = 0
                            fcenterX = 0
                    elif vision_mode == 5:
                        #Handle different vision scenarios
                        if count == 4:
                            mode = 5
                            area1 = widths[0] * heights[0]
                            area2 = widths[1] * heights[1]
                            area3 = widths[2] * heights[2]
                            area4 = widths[3] * heights[3]
                            areas = [area1, area2, area3, area4]

                            min_pos = areas.index(min(areas))
                            areas.pop(min_pos)
                            widths.pop(min_pos)
                            heights.pop(min_pos)
                            x1.pop(min_pos)
                            y1.pop(min_pos)
                            min_pos = areas.index(min(areas))
                            areas.pop(min_pos)
                            widths.pop(min_pos)
                            heights.pop(min_pos)
                            x1.pop(min_pos)
                            y1.pop(min_pos)

                            M0 = cv2.moments(centroidInfo[0])
                            M1 = cv2.moments(centroidInfo[1])
                            fcenter1 = int(M0['m10']/M0['m00']) + leftOffset
                            fcenter2 = int(M1['m10']/M1['m00']) + leftOffset
                            fcenterX = (fcenter1 + fcenter2) / 2

                            if y1[0] + heights[0] < y1[1] + heights[1]:
                                fY = y1[1] + heights[1]
                            else:
                                fY = y1[0] + heights[0]
                            fY += + topOffset
                            #Undistort Points
                            distPoint = fcenterX, fY
                            points = np.array([[distPoint]], dtype=np.float32)
                            output = cv2.undistortPoints(points, mtx, dist)
                            fcenterX = output[0][0][0]
                            fY = output[0][0][1]
                        elif count == 3:
                            mode = 5
                            area1 = widths[0] * heights[0]
                            area2 = widths[1] * heights[1]
                            area3 = widths[2] * heights[2]
                            areas = [area1, area2, area3]

                            min_pos = areas.index(min(areas))
                            areas.pop(min_pos)
                            widths.pop(min_pos)
                            heights.pop(min_pos)
                            x1.pop(min_pos)
                            y1.pop(min_pos)

                            M0 = cv2.moments(centroidInfo[0])
                            M1 = cv2.moments(centroidInfo[1])
                            fcenter1 = int(M0['m10']/M0['m00']) + leftOffset
                            fcenter2 = int(M1['m10']/M1['m00']) + leftOffset
                            fcenterX = (fcenter1 + fcenter2) / 2

                            if y1[0] + heights[0] < y1[1] + heights[1]:
                                fY = y1[1] + heights[1]
                            else:
                                fY = y1[0] + heights[0]
                            fY += + topOffset
                            #Undistort Points
                            distPoint = fcenterX, fY
                            points = np.array([[distPoint]], dtype=np.float32)
                            output = cv2.undistortPoints(points, mtx, dist)
                            fcenterX = output[0][0][0]
                            fY = output[0][0][1]
                        elif count == 2:
                            mode = 5

                            M0 = cv2.moments(centroidInfo[0])
                            M1 = cv2.moments(centroidInfo[1])
                            fcenter1 = int(M0['m10']/M0['m00']) + leftOffset
                            fcenter2 = int(M1['m10']/M1['m00']) + leftOffset
                            fcenterX = (fcenter1 + fcenter2) / 2

                            if y1[0] + heights[0] < y1[1] + heights[1]:
                                fY = y1[1] + heights[1]
                            else:
                                fY = y1[0] + heights[0]
                            fY += + topOffset

                            #Undistort Points
                            distPoint = fcenterX, fY
                            points = np.array([[distPoint]], dtype=np.float32)
                            output = cv2.undistortPoints(points, mtx, dist)
                            fcenterX = output[0][0][0]
                            fY = output[0][0][1]

                        else:
                            mode = 6
                            fcenterX = 0
                            fY = 0
                else:
                    frame = cv2.resize(frame,(cx*2,cy*2), interpolation=cv2.INTER_NEAREST)

                #elapsed_time = timer() - start
                elapsed_time = time.time() - multiImage.time
                quality = 0
                fcenterX -= centerOff[0][0][0]
                fY -= centerOff[0][0][1]
                #Print the data that roborio will use
                print "Processing Time: " + str(elapsed_time)
                print "fcenterX: " + str(fcenterX)
                print "fbottomY: " + str(fY)
                print "Mode: " + str(mode)
                print "Cam: " + str(camera)
                print "Quality: " + str(quality)

                #Send the data in the form of a 17 byte packet
                packetTime = bytearray(struct.pack("f", elapsed_time))
                packetCenterX = bytearray(struct.pack("f", fcenterX))
                packetBottomY = bytearray(struct.pack("f", fY))
                packetMode = bytearray(struct.pack("b", mode))
                packetCam = bytearray(struct.pack("b", camera))
                packetQuality = bytearray(struct.pack("b", quality))
                finalPacket = packetTime+packetCenterX[:]+packetBottomY[:]+packetMode[:]+packetCam[:]+packetQuality[:]
                sock.sendto(finalPacket, (UDP_IP, UDP_PORT))
                #udp.sendPacket(finalPacket)
                delay_time = timer()

                #hsv = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2HSV) #Allows filtering in hsv
                #multiImage.hsv = hsv
                #multiImageQ2.put(multiImage)
        except:
            pass

multiImageQ = Queue.Queue(0)
multiImageQ2 = Queue.Queue(0)
lock = threading.Lock()
thread1 = frameThread(0, "Thread-1", 0)
thread2 = filterThread(0, "Thread-2", 0,0)
thread3 = contourThread(0, "Thread-3", 0)

thread1.daemon=True
thread2.daemon=True
thread3.daemon=True

#thread1.start()
thread2.start()
#thread3.start()

grabFrames()
