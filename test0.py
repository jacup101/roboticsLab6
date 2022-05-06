import tellopy
import cv2
import os
import datetime
import time

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





def show_contours(pathToImg):
    img = cv2.imread('testtt.png')
    # imgGry = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # ret , thrash = cv2.threshold(imgGry, 240 , 255, cv2.CHAIN_APPROX_NONE)
    # contours , hierarchy = cv2.findContours(thrash, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edged = cv2.Canny(gray, 30, 200)
    # cv2.waitKey(0)

    # Finding Contours
    # Use a copy of the image e.g. edged.copy()
    # since findContours alters the image
    contours, hierarchy = cv2.findContours(edged,
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    for contour in contours:

        area = cv2.contourArea(contour)
        if area < 100.0:
            pass
        else:
            approx = cv2.approxPolyDP(contour, 0.01* cv2.arcLength(contour, True), True)
            cv2.drawContours(img, [approx], 0, (0, 0, 0), 5)
            x = approx.ravel()[0]
            y = approx.ravel()[1] - 5
            if len(approx) == 3:
                cv2.putText( img, "Triangle", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0) )
            elif len(approx) == 4 :
                # We have a rectangle
                x, y , w, h = cv2.boundingRect(approx)

                area = cv2.contourArea(contour)
                print(area)
                aspectRatio = float(w)/h
                # print(aspectRatio)
                if aspectRatio >= 0.95 and aspectRatio < 1.05:
                    cv2.putText(img, "square", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))

                else:
                    cv2.putText(img, "rectangle", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))

            elif len(approx) == 5 :
                cv2.putText(img, "pentagon", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
            elif len(approx) == 10 :
                cv2.putText(img, "star", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
            else:
                cv2.putText(img, "circle", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))

    cv2.imshow('shapes', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()



drone = tellopy.Tello()


drone.connect()
print("hi")

drone.takeoff()

drone.forward(10)
drone.backward(10)
drone.land()

#drone.takeoff()

#drone.forward(10)
#drone.moveLeft(10)
#drone.land()
