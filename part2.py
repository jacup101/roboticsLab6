import tellopy
import cv2
import os
import datetime
import time
import random
import numpy as np
pathToImg = "null"






def handlePictureReceived(event, sender, data):
    global pathToImg
    # Create a file in ~/Pictures/ to receive image data from the drone.
    path = '%s/Volumes/workplace/robotics/lab6/photos/tello-%s.jpeg' % (
        os.getenv('HOME'),
        datetime.datetime.now().strftime('%Y-%m-%d_%H%M%S'))
    with open(path, 'wb') as fd:
        fd.write(data)
    pathToImg = path


def generateRandRGBTuple():
    return (random.randint(0,255), random.randint(0, 255), random.randint(0, 255))


def handle_contours(pathToImg, searchingFor, show):
    img = cv2.imread(pathToImg)
    # Get a grayscale of the image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Get params to create canny image
    median = cv2.medianBlur(gray, 3)
    v = np.median(gray)
    sigma = .33
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    # Get edges using canny algo
    edged = cv2.Canny(median, lower, upper)
    # Attempt to close the image using morphology
    se = np.ones((10,10), dtype='uint8')
    image_close = cv2.morphologyEx(edged, cv2.MORPH_CLOSE, se)
    # Attempt to close any remaining gaps using a slight dilation
    kernel = np.ones((10,10),np.uint8)
    dilated= cv2.dilate(image_close, kernel, iterations = 2)

    # Set up parameters for CV decisions
    rectangleFound = False
    maxRectArea = 40000
    circleFound = False
    maxCircArea = 30000
    triangleFound = False
    maxTriArea = 20000
    # Finding Contours
    # Use a copy of the image e.g. edged.copy()
    # since findContours alters the image

    # RECTANGLES + TRIANGLES DETECTION
    contours, hierarchy = cv2.findContours(dilated,
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    for contour in contours:

        area = cv2.contourArea(contour)
        if area < 250.0:
            pass
        else:
            approx = cv2.approxPolyDP(contour, 0.01* cv2.arcLength(contour, True), True)

            randRGB = generateRandRGBTuple()

            x = approx.ravel()[0]
            y = approx.ravel()[1] - 5
            if len(approx) == 3:
                print("Triangle with area: {0} detected".format(area))
                cv2.putText( img, "Triangle", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, randRGB )
                cv2.drawContours(img, [approx], 0, randRGB, 5)
                if area > maxTriArea:
                    triangleFound = True

            elif len(approx) >= 4 and len(approx) < 8:
                # We have a rectangle
                print("Rectangle with area: {0} detected".format(area))
                cv2.putText(img, "rectangle", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, randRGB)
                cv2.drawContours(img, [approx], 0, randRGB, 5)
                if area > maxRectArea:
                    rectangleFound = True
            elif len(approx) > 5 and cv2.isContourConvex(approx):
                print("Circle with area: {0} detected".format(area))
                cv2.putText(img, "circle", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, randRGB)
                if area > maxCircArea:
                    circleFound = True
            cv2.drawContours(img, [approx], 0, randRGB, 5)
    if show:
        cv2.imshow('shapes', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    if searchingFor in "rectangle":
        return rectangleFound
    if searchingFor in "circle":
        return circleFound
    if searchingFor in "triangle":
        return triangleFound






def initDrone(drone, waitTime):
    drone.connect()
    print("Connecting..")
    drone.subscribe(drone.EVENT_FILE_RECEIVED, handlePictureReceived)
    time.sleep(5)

def takeoff(drone, down, waitTime):
    drone.takeoff()
    time.sleep(waitTime)
    drone.down(down)
    time.sleep(waitTime)

def land(drone, waitTime):
    drone.land()
    time.sleep(waitTime)

def flyForward(drone, distance, sleepTime):
    drone.forward(distance)
    time.sleep(sleepTime)

def flyUpward(drone, distance, sleepTime):
    drone.up(distance)
    time.sleep(sleepTime)

def stopEverything(drone, waitTime):
    drone.set_throttle(0)
    drone.set_pitch(0)
    drone.set_roll(0)

def rotateCCW(drone, angle, sleepTime):
    drone.counter_clockwise(angle)
    time.sleep(sleepTime)


def processCV(drone, waitTime, searchingFor):
    drone.take_picture()
    time.sleep(waitTime)
    return handle_contours(pathToImg, searchingFor, False)

useDrone = False

if useDrone:
    drone = tellopy.Tello()
    initDrone(drone, 10)

    takeoff(drone, 30, 5)
    #flyUpward(drone,10,5)
    time.sleep(3)
    numAllowedSteps = 7
    steps = 0
    moving = 'forward'
    while steps < numAllowedSteps:
        if moving in 'forward':
            flyForward(drone, 10, 5)
        if moving in 'up':
            flyUpward(drone, 10, 5)
        time.sleep(3)
        if moving in 'forward':
            searchResult = processCV(drone, 5, 'circle')
        else:
            searchResult = processCV(drone, 5, 'triangle')
        stopEverything(drone,2)
        if searchResult:
            if moving in 'forward':
                moving = 'up'
            else:
                break
            #rotateCCW(drone, 20, 3)
        steps += 1




    land(drone, 5)


    drone.quit()
    time.sleep(3)


    time.sleep(3)
else:
    print("Not using drone...")
    handle_contours("./photos/tello-2022-05-03_123436.jpeg", "rectangle", True)
