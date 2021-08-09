import cv2
import numpy as np


def stationary_camera_transform(point, height):
    K = [215.6810060961547, 0.0, 376.5, 0.0, 215.6810060961547, 240.5, 0.0, 0.0, 1.0]

    land_height = 0.000009 # 0.5

    Z = height - land_height # Distancia do solo

    dX = (point[0] - K[2]) * Z / K[0]
    dY = (point[1] - K[5]) * Z / K[4]

    dist = (dX, dY)

    return dist

def algae_detector(img):
    img = img[:,:,[0,1,2]]

    img_r = img[:, :, 0]
    img_g = img[:, :, 1]
    img_b = img[:, :, 2]

    min_thr_r = 20
    min_thr_g = 120
    _, thr_g = cv2.threshold(img_r, min_thr_r, 1, cv2.THRESH_BINARY)
    _, thr_r = cv2.threshold(img_g, min_thr_g, 1, cv2.THRESH_BINARY)
    thr = np.multiply(thr_g, thr_r)*255

    #blur
    size_b = 5
    kernel_blur = np.ones((size_b, size_b),np.float32)/size_b*size_b
    dst = cv2.filter2D(thr, -1, kernel_blur)

    #closing
    size_c = 10
    kernel_closing = np.ones((size_c, size_c),np.float32)/size_c*size_c
    img_binary = cv2.morphologyEx(dst, cv2.MORPH_CLOSE, kernel_closing)

    mostTrue = thr.shape[0]*thr.shape[1]*255*0.98 # 98% of the image's pixels have the same color (255)
    mostFalse = thr.shape[0]*thr.shape[1]*255*0.02 #  2% of the image's pixels have the same color (255)
    img_pixels = thr.shape[0]*thr.shape[1]

    if (thr.sum() >= mostTrue) or (thr.sum() <= mostFalse):
        B_sum = img_b.sum()
        G_sum = img_g.sum()
        R_sum = img_r.sum()

        red_limit = 60
        blue_limit = 150
    
        if (R_sum < red_limit*thr.shape[0]*thr.shape[1]) and (B_sum > blue_limit*thr.shape[0]*thr.shape[1]):
            img_binary = np.zeros(thr.shape)
            return[]

    centres = []

    try:
        contours, hierarchy = cv2.findContours(img_binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for i in range(len(contours)):
            moments = cv2.moments(contours[i])
            centres.append((int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])))
    except:
        pass

    return centres
