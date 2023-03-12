#!/usr/bin/env python3

import rospy
import numpy as np
import sys
from sensor_msgs.msg import Image
from perception.msg import lineArray, line
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import cv2
import math
from sklearn.linear_model import HuberRegressor
from matplotlib.animation import FuncAnimation, PillowWriter
import time

class laneDetectionNODE():
    def __init__(self):
        self.debug = False
        self.show_final = self.debug or False

        # TODO docstring
        rospy.init_node('laneDetectionNODE', anonymous=False)
        rospy.Subscriber("/automobile/image_raw", Image, self._streams)
        self.lane_publisher = rospy.Publisher("/lane_info", lineArray, queue_size=1)

        cv2.startWindowThread()
        if self.show_final:
            cv2.namedWindow("Test")
        # if self.debug:
            # cv2.namedWindow("Debug")

    def run(self):
        rospy.loginfo('starting laneDetectionNODE')
        rospy.spin()    

    def imgmsg_to_cv2(self, img_msg):

        dtype = np.dtype("uint8") # Hardcode to 8 bits...
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                        dtype=dtype, buffer=img_msg.data)
        # If the byt order is different between the message and the system.
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            image_opencv = image_opencv.byteswap().newbyteorder()
        return image_opencv

    def _streams(self, msg):
        t1 = time.time()

        img = self.imgmsg_to_cv2(msg)

        if self.debug or self.show_final:
            plt.rcParams["figure.figsize"] = (10,7.5)

        if self.debug:
            plt.imshow(img)
            plt.show()

        gray_img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        if self.debug:
            plt.imshow(gray_img, cmap='gray')
            plt.show()

        kernel_size = 5
        bilateral = False
        dilate = True
        if bilateral:
            blur_img = cv2.bilateralFilter(gray_img, 10, 75, 75)
        else:
            blur_img = cv2.GaussianBlur(gray_img, (kernel_size, kernel_size), 0)

        if dilate:
            blur_img = cv2.dilate(blur_img, np.ones((5, 5), np.uint8), iterations=1)

        if self.debug:
            plt.imshow(blur_img, cmap='gray')
            plt.show()

        low_threshold, high_threshold = [200, 300]
        canny_img = cv2.Canny(blur_img, low_threshold, high_threshold)

        if self.debug:
            plt.imshow(canny_img, cmap='gray')
            plt.show()

        x_size = img.shape[1]
        y_size = img.shape[0]
        points = np.array([[0, y_size], [0, int(y_size//1.6)], [x_size//4, int(y_size//2.3)], [x_size//2, int(y_size//3)], [3*x_size//4, int(y_size//2.3)], 
                        [x_size-1, int(y_size//1.6)], [x_size-1, y_size]])#, [3*x_size//4, y_size], [x_size//2, y_size-140], [x_size//4, y_size]])
        mask = np.zeros_like(canny_img)
        cv2.fillPoly(mask, [points], 130)

        if self.debug:
            plt.imshow(mask, cmap='gray')
            plt.show()

        mask_img = np.bitwise_and(canny_img, mask)

        if self.debug:
            plt.imshow(mask_img, cmap='gray')
            plt.show()

        lines = cv2.HoughLinesP(mask_img, rho=1, theta=math.pi/180,
                                threshold=30, lines=None,
                                minLineLength=10,
                                maxLineGap=15)

        height_threshold = 130

        def line_length(l):
            return (l[2]-l[0])**2+(l[3]-l[0])**2

        '''
        for l in lines:
            slope = abs((l[3]-l[1])/(l[2]-l[0]+1e-10))
            length = line_length(l)
            midpoint = (l[3] + l[1]) / 2
            if slope < 0.1 and length > 10000 and midpoint > 300:
        '''

        if self.debug:
            for l in lines:
                l=l[0]
                plt.plot([l[0], l[2]], [l[1], l[3]], color='red')
        
            plt.imshow(img, cmap='gray')
            plt.show()

        lines_right = []
        lines_left = []

        cw_l = []

        for l in lines:
            l=l[0]
            
            if l[1]<0:#y_size-height_threshold:
                continue
            if l[3]<0:#y_size-height_threshold:
                continue
        
            slope = abs((l[3]-l[1])/(l[2]-l[0]+1e-10))
            if slope < 1/4:
                if abs(l[2]-l[0]) < 60:
                    av = (l[1]-l[3])/2
                    #print("***")
                    #print(l)
                    md=0
                    for ll in lines:
                        ll=ll[0]
                        md = (ll[0]-l[0])**2+(ll[1]-l[1])**2
                        md = min(md, (ll[0]-l[2])**2+(ll[1]-l[3])**2)
                        md = min(md, (ll[2]-l[2])**2+(ll[3]-l[3])**2)
                        md = min(md, (ll[2]-l[0])**2+(ll[3]-l[1])**2)
                        if md< 125:
                            cw_l.append(l)
                            break
                    #print(md)
                continue

            if l[0]<x_size/2 and l[2]>x_size/2:
                continue

            if l[0]>x_size/2 and l[2]<x_size/2:
                continue
            
            lsq = (l[2]-l[0])**2+(l[3]-l[1])**2
            if lsq<4900:
                continue

            if l[0]<x_size/2:
                lines_left.append(l)

            if l[0]>x_size/2:
                lines_right.append(l)

        if self.debug:
            for l in cw_l:
                plt.plot([l[0], l[2]], [l[1], l[3]], color='red')
        
            plt.imshow(img, cmap='gray')
            plt.show()

        if len(cw_l) > 2:
            #print(f'number of horizontal lines: {len(cw_l)}')
            av=0
            for l in cw_l:
                av+=(l[1]+l[3])/2
                #print((l[1]+l[3])/2)
            av /= len(cw_l)
            #print(f'avg: {av}')
            yu=[]
            yd=[]

            ys=[]
            for l in cw_l:
                a = (l[1]+l[3])/2
                a=a.item()
                ys.append(a)
            #print(max(ys)-min(ys))
            if max(ys)-min(ys)<22:
                yu=av
                yd=img.shape[0]
            else:
                for l in cw_l:
                    a = (l[1]+l[3])/2
                    a=a.item()
                    #print(type(a))
                    if a>av:
                        yd.append(a)
                    else:
                        yu.append(a)

                yu=sum(yu)/len(yu)
                yd=sum(yd)/len(yd)
            #print(yu)
            #print(yd)
            #print(f'yu: {yu}')
            #print(f'yd: {yd}')
            lll=[]
            llr=[]
            #print('crosswalk detected')
            for l in lines_left:
                #print('$$$$$$$$')
                #print(l)
                if not ((abs(l[1]-yu)<50 and abs(l[3]-yd)<50) or (abs(l[1]-yd)<50 and abs(l[3]-yu)<50)):
                    #print('in')
                    lll.append(l)
            
            for l in lines_right:
                #print('$$$$$$$$')
                #print(l)
                if not ((abs(l[1]-yu)<50 and abs(l[3]-yd)<50) or (abs(l[1]-yd)<50 and abs(l[3]-yu)<50)):
                    #print('in')
                    llr.append(l)

            lines_left=lll
            lines_right=llr

        if self.show_final:
            print("\nLL: ", len(lines_right))
            for l in lines_left:
                cv2.line(img, (l[0], l[1]), (l[2], l[3]), color=(104, 38, 192), thickness=3)
                # plt.plot([l[0], l[2]], [l[1], l[3]], color='red')
            # cv2.imshow('Debug', img)
            
            # plt.imshow(img, cmap='gray')
            # plt.show()

        sorted(lines_right, key=line_length)
        #print(lines_right)
        #lines_right=lines_right[:2]

        if self.show_final:
            print("LR: ", len(lines_right))
            for i, l in enumerate(lines_right):
                # plt.plot([l[0], l[2]], [l[1], l[3]], color='red')
                cv2.line(img, (l[0], l[1]), (l[2], l[3]), color=(104, 38, 192), thickness=3)

            # cv2.imshow('Debug', img)
            # plt.imshow(img, cmap='gray')
            # plt.show()

        if len(lines_right):

            Xr = np.array(lines_right)[:,0::2].reshape(-1,1)
            yr = np.array(lines_right)[:,1::2].reshape(-1)

            #print(Xr.shape)
            #print(Xr, yr)
            model_r = HuberRegressor(fit_intercept=True, alpha=0.0, max_iter=500,
                                                epsilon=10)
            model_r.fit(Xr, yr)

            Xr = np.array([x_size, x_size*2//3]).reshape(-1,1)
            yr = model_r.predict(Xr)
        else:
            Xr = np.array([x_size, x_size-height_threshold]).reshape(-1,1)
            yr = np.array([400.0, 400.0])


        if len(lines_left):
            Xl = np.array(lines_left)[:,0::2].reshape(-1,1)
            yl = np.array(lines_left)[:,1::2].reshape(-1)

            #print(Xl.shape)
            #print(Xl, yl)
            model_l = HuberRegressor(fit_intercept=True, alpha=0.0, max_iter=100,
                                                epsilon=10)
            model_l.fit(Xl, yl)

            Xl = np.array([0, x_size//3]).reshape(-1,1)
            yl = model_l.predict(Xl)
        else:
            Xl = np.array([x_size, x_size-height_threshold]).reshape(-1,1)
            yl = np.array([400.0, 400.0])

        #epsilon = 1e-10
        #y0 = y_size
        #x0 = (y0 - model.intercept_)/(model.coef_+epsilon)
        #y1 = y_size-height_threshold
        #x1 = (y1 - model.intercept_)/(model.coef_+epsilon)
        #print(x0, y0)
        #print(x1, y1)

        Xr[0] = x_size + (x_size // 3) / (yr[0] - yr[1]) * (y_size - yr[0] - 10)
        yr[0] = y_size - 10

        Xl[0] = (x_size // 3) / (yl[1] - yl[0]) * (y_size - yl[0] - 10)
        yl[0] = y_size - 10

        if self.show_final:
            # plt.imshow(img)
            # plt.plot(Xr.reshape(-1), yr, color='red', linewidth=7)
            # plt.plot(Xl.reshape(-1), yl, color='red', linewidth=7)
            # plt.show()
            cv2.line(img, (int(Xr.reshape(-1)[0]), int(yr[0])), (int(Xr.reshape(-1)[1]), int(yr[1])), color=(12,145,255), thickness=7)
            cv2.line(img, (int(Xl.reshape(-1)[0]), int(yl[0])), (int(Xl.reshape(-1)[1]), int(yl[1])), color=(12,145,255), thickness=7)
            cv2.imshow('Test', img)
            if cv2.waitKey(20) == 27:
                sys.exit(0)

        # print(f'Lane Detection, time to process one frame: {time.time()-t1}')

        right_line = line()
        right_line.x1 = int(Xr.reshape(-1)[0])
        right_line.y1 = int(yr[0])
        right_line.x2 = int(Xr.reshape(-1)[1])
        right_line.y2 = int(yr[1])

        left_line = line()
        left_line.x1 = Xl.reshape(-1)[0]
        left_line.y1 = int(yl[0])
        left_line.x2 = int(Xl.reshape(-1)[1])
        left_line.y2 = int(yl[1])

        message = lineArray()
        message.lines = [left_line, right_line]

        self.lane_publisher.publish(message)
        

if __name__ == "__main__":
    laneDetection = laneDetectionNODE()
    laneDetection.run()