# !/usr/bin/env python
# coding:utf-8

import cv2
import numpy as np
import argparse
import sys

frame = None
draw = False
start_point = (-1, -1)

mode = None
cv2.namedWindow("windows", cv2.WINDOW_NORMAL)


def nothing(x):
    pass


def MouseCallBack(event, x, y, flags, param):
    global draw
    global start_point
    f = cv2.getTrackbarPos("free", "windows")
    o = cv2.getTrackbarPos("occupancied", "windows")
    u = cv2.getTrackbarPos("unknown", "windows")

    if event == cv2.EVENT_LBUTTONDOWN:
        draw = True
        start_point = (x, y)
    elif event == cv2.EVENT_MOUSEMOVE and flags == cv2.EVENT_FLAG_LBUTTON:
        if draw:
            if mode == "free":
                cv2.line(frame, start_point, (x, y), (255, 255, 255), thickness=f)
            elif mode == "occupancied":
                cv2.line(frame, start_point, (x, y), (0, 0, 0), thickness=o)
            elif mode == "unknown":
                cv2.line(frame, start_point, (x, y), (205, 205, 205), thickness=u)
            start_point = (x, y)
    elif event == cv2.EVENT_LBUTTONUP:
        draw = False


cv2.createTrackbar("free", "windows", 1, 20, nothing)
cv2.createTrackbar("occupancied", "windows", 1, 20, nothing)
cv2.createTrackbar("unknown", "windows", 1, 20, nothing)

cv2.setMouseCallback("windows", MouseCallBack)

if __name__ == '__main__':
    if len(sys.argv) <= 1:
        print("please enter the .pgm file.")
        exit(1)
    pgm_file = sys.argv[1]

    frame = cv2.imread(pgm_file)
    print frame.shape
    print np.unique(frame)  # np.unique(x) 输出x中所有不重复的元素

    while True:
        cv2.imshow("windows", frame)
        k = cv2.waitKey(25)
        if k == ord('q'):
            break
        elif k == ord('f'):
            print "Draw free space"
            mode = "free"
        elif k == ord('o'):
            print "Draw occupancied space"
            mode = "occupancied"
        elif k == ord("u"):
            print "Draw unknown space"
            mode = "unknown"
        elif k == ord('s'):
            cv2.imwrite("test.pgm", frame)

cv2.destroyAllWindows()
