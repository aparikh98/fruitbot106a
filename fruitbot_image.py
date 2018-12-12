#!/usr/bin/env python
import cv2, time, sys
import numpy as np
import argparse
import os
from numpy.linalg import *
# from std_msgs.msg import String
import matplotlib.pyplot as plt
import scipy.misc
# import skimage.filter

height = .42 # y distance of the table
width = .60  # x distance of the table


def check_homography(image, H, nx, ny, length=.06):
  # H should be a 3x3 numpy.array
  # nx is the number of tiles in the x direction
  # ny is the number of tiles in the y direction
  # length is the length of one side of a tile
  # image is an image array
  for i in range(nx+1):
    for j in range(ny+1):
      xbar = np.array([[i*length],[j*length],[1]])
      ubar = np.dot(H,xbar).T[0]
      u = np.int(ubar[0]/ubar[2])
      v = np.int(ubar[1]/ubar[2])
      print 'Dot location: ' + str((u,v))
      cv2.circle(image, (u,v), 50, 255, -1)
  plt.imshow(image)
  plt.show()

def getOrientation(img, Q):
    edges = cv2.Canny(img,100,200)
    y, x = np.nonzero(edges)
    x_cent = np.mean(x)
    y_cent = np.mean(y)
    x = x - x_cent
    y = y - y_cent
    coords = np.vstack([x, y])
    cov = np.cov(coords)
    evals, evecs = np.linalg.eig(cov)
    sort_indices = np.argsort(evals)[::-1]
    x_v1, y_v1 = evecs[:, sort_indices[0]]  # Eigenvector with largest eigenvalue


    theta = np.tanh((x_v1)/(y_v1))

    scale = 20
    x_v2, y_v2 = evecs[:, sort_indices[1]]

    plt.plot(x, y, 'k.')
    plt.plot([x_v1*-scale*10, x_v1*scale*10],
             [y_v1*-scale*10, y_v1*scale*10], color='red',linewidth = 4)
    plt.plot([x_v2*-scale, x_v2*scale],
             [0, y_v2*scale], color='blue',linewidth = 2)
    plt.plot([x_v1, x_v1],
        [y_v1*-scale*10, y_v1*scale*10], color='green', linewidth = 4)
    plt.axis('equal')
    plt.gca().invert_yaxis()  # Match the image system with origin at top left
    plt.title(theta)
    plt.show()

    return (x_cent,y_cent, theta)

def removeduplicates(boxes, classNames):
    dups = []
    n = len(boxes)
    for i in range(n):
        for j in range(i+1, n):
            if classNames[i] != classNames[j]:
                continue
            if (boxes[i][0] < boxes[j][0]):
                box1x = boxes[i]
                box2x = boxes[j]
            else:
                box1x = boxes[j]
                box2x = boxes[i]
            if (boxes[i][1] < boxes[j][1]):
                box1y = boxes[i]
                box2y = boxes[j]
            else:
                box1y = boxes[j]
                box2y = boxes[i]
            if (box1x[2] > box2x[0]) and (box1y[3]> box2y[1]):
                dups.append(j)
    boxes = [boxes[b] for b in range(n) if b not in dups]
    classNames = [classNames[b] for b in range(n) if b not in dups]
    return boxes, classNames

def transformcoordinates(boundingbox, classNames, Q, np_image):
  realworld = []
    #origin coordinates
  sawyer_x = 0.35
  sawyer_y = 0.78
  sawyer_z = -.23

  for i in range(len(boundingbox)):
    x = boundingbox[i][0]
    y = boundingbox[i][1]
    x2 = boundingbox[i][2]
    y2 = boundingbox[i][3]
    objClass = classNames[i]
    cropped_img = np_image[y:y2, x:x2]
    x_cent,y_cent, orientation = getOrientation(cropped_img, Q)
    x+= x_cent
    y+= y_cent
    uv = np.array([x,y,1])
    N = np.matmul(Q,uv)
    x_rw =  sawyer_x + (N[1])/ N[2]
    y_rw = sawyer_y - (N[0])/ N[2]

    realworld.append([x_rw, y_rw, sawyer_z, orientation, objClass])
  return realworld
#Need 3D Numpy Array
def getBoundingBoxAndClass(image_data):

    # load the COCO class labels our YOLO model was trained on
    labelsPath = os.path.sep.join(["./darknet/data", "coco.names"])
    LABELS = open(labelsPath).read().strip().split("\n")
    # initialize a list of colors to represent each possible class label
    np.random.seed(42)
    COLORS = np.random.randint(0, 255, size=(len(LABELS), 3), dtype="uint8")
    # derive the paths to the YOLO weights and model configuration
    weightsPath = os.path.sep.join(["./darknet", "yolov3.weights"])
    configPath = os.path.sep.join(["./darknet/cfg", "yolov3.cfg"])
    # load our YOLO object detector trained on COCO dataset (80 classes)
    print("[INFO] loading YOLO from disk...")
    net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)

    #3D Numpy array of image data from web-cam
    image = image_data
    (H, W) = image.shape[:2]

    # determine only the *output* layer names that we need from YOLO
    ln = net.getLayerNames()
    ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]

    # construct a blob from the input image and then perform a forward
    # pass of the YOLO object detector, giving us our bounding boxes and
    # associated probabilities
    blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416), swapRB=True, crop=False)
    net.setInput(blob)
    start = time.time()
    layerOutputs = net.forward(ln)
    end = time.time()

    # show timing information on YOLO
    print("[INFO] YOLO took {:.6f} seconds".format(end - start))

    # initialize our lists of detected bounding boxes, confidences, and
    # class IDs, respectively
    boxes = []
    confidences = []
    classIDs = []
    classNames = []

    # loop over each of the layer outputs
    for output in layerOutputs:
        # loop over each of the detections
        for detection in output:
            # extract the class ID and confidence (i.e., probability) of
            # the current object detection
            scores = detection[5:]
            classID = np.argmax(scores)
            confidence = scores[classID]

            # filter out weak predictions by ensuring the detected
            # probability is greater than the minimum probability
            if confidence > 0.5:
                # scale the bounding box coordinates back relative to the
                # size of the image, keeping in mind that YOLO actually
                # returns the center (x, y)-coordinates of the bounding
                # box followed by the boxes' width and height
                box = detection[0:4] * np.array([W, H, W, H])
                (centerX, centerY, width, height) = box.astype("int")

                # use the center (x, y)-coordinates to derive the top and
                # and left corner of the bounding box
                x = int(centerX - (width / 2))
                y = int(centerY - (height / 2))

                # update our list of bounding box coordinates, confidences,
                # and class IDs
                boxes.append([x, y, int(width), int(height)])
                confidences.append(float(confidence))
                classIDs.append(classID)
                classNames.append(LABELS[classID])

    # apply non-maxima suppression to suppress weak, overlapping bounding
    # boxes
    idxs = cv2.dnn.NMSBoxes(boxes, confidences, 0.5,
        0.3)

    # ensure at least one detection exists
    if len(idxs) > 0:
        # loop over the indexes we are keeping
        for i in idxs.flatten():
            # extract the bounding box coordinates
            (x, y) = (boxes[i][0], boxes[i][1])
            (w, h) = (boxes[i][2], boxes[i][3])

            # draw a bounding box rectangle and label on the image
            color = [int(c) for c in COLORS[classIDs[i]]]
            cv2.rectangle(image, (x, y), (x + w, y + h), color, 15)
            text = "{}: {:.4f}".format(LABELS[classIDs[i]], confidences[i])
            cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,
                3, color, 2)

    # show the output image
    plt.imshow(image)
    plt.show()
    boxes = [[b[0], b[1], b[0] + b[2], b[1]+b[3]] for b in boxes]
    boxes, classNames = removeduplicates(boxes, classNames)
    return (boxes, classNames)

def generate_homography(img):
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    kernel = np.ones((100,100),np.uint8)
    gray = cv2.morphologyEx(gray, cv2.MORPH_OPEN, kernel)
    plt.imshow(gray)
    plt.show()
    corners = cv2.goodFeaturesToTrack(gray,4,0.01,2000)
    corners = np.int0(corners)
    s = img.shape
    points = []
    num = 0
    print(corners)
    corners = [[p for p in corners if (p[0][0] < s[1]/2 and p[0][1] > s[0]/2)][0],[p for p in corners if (p[0][0] < s[1]/2 and p[0][1] < s[0]/2)][0], [p for p in corners if (p[0][0] > s[1]/2 and p[0][1] < s[0]/2)][0], [p for p in corners if (p[0][0] > s[1]/2 and p[0][1] > s[0]/2)][0]]
    for i in corners:
        num+=1
        x,y = i.ravel()
        points.append([x,y])
        cv2.circle(img,(x,y),30,255,-1)
        txt = str(num)
        font  = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 1
        fontColor = (255,255,255)
        lineType = 2
        cv2.putText(img,txt, (x,y), font, fontScale,fontColor,lineType)
    plt.imshow(img)
    plt.show()


    # Convert the Python list of points to a NumPy array of the form
    #   | u1 u2 u3 u4 |
    #   | v1 v2 v3 v4 |
    uv = np.array(points).T
    print(uv)
    b = uv.flatten('F')
    x = np.array([0,0, width, width])
    y = np.array([0, height, height,  0])
    A = np.array([[x[0], 0, x[1], 0, x[2], 0, x[3], 0], [y[0], 0, y[1], 0, y[2], 0, y[3], 0], [1, 0, 1, 0, 1, 0, 1, 0], [0, x[0], 0, x[1], 0, x[2], 0, x[3]], [0, y[0], 0, y[1], 0, y[2], 0, y[3]], [0, 1, 0, 1, 0, 1, 0, 1], [-1 * b[0]*x[0], -1*b[1]*x[0], -1 *b[2]*x[1], -1 * b[3] * x[1], -1 * b[4] * x[2],-1 * b[5] * x[2], -1 * b[6] * x[3], -1 * b[7] * x[3]], [-1 * b[0]*y[0], -1*b[1]*y[0], -1 *b[2]*y[1], -1 * b[3] * y[1], -1 * b[4] * y[2],-1 * b[5] * y[2], -1 * b[6] * y[3], -1 * b[7] * y[3]]])
    A = A.T
    H = np.append(np.linalg.solve(A, b),1).reshape((3,3))
    Q = np.linalg.inv(H)
    return (Q,H)





if __name__ == '__main__':
    np_image = cv2.imread("finaldesk.jpg")
    image_copy = np.copy(np_image)
    plt.imshow(np_image)
    plt.show()
    boundingBox, classNames = getBoundingBoxAndClass(image_copy)
    image_copy1 = np.copy(np_image)
    n = len(classNames)

    Q, H = generate_homography(image_copy1)
    image_copy2 = np.copy(np_image)

    check_homography(image_copy2, H, 10, 7)

      # continue

    realWorldObjects = transformcoordinates(boundingBox, classNames, Q, np_image)
    for a in realWorldObjects:
        print(a)
        plt.scatter(a[0], a[1])
    plt.show()
    n = len(realWorldObjects)
    print(n, 'objects detected')
    print(realWorldObjects)
    f= open("output.txt","w+")
    for obj in realWorldObjects:
        f.write(str(obj)[1:-1] + '\n')
    f.close()
