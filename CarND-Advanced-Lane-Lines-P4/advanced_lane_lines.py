import numpy as np
import cv2
import glob
import matplotlib.pyplot as plt
import os

############################################################################################################

# prepare object points
objp = np.zeros((6*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

# arrays to store object points and image points from all the images
objpoints = [] # 3d points in real world space
imgpoints = [] # 2d points in image plane
img = []

# make a list of calibration images
images = glob.glob('./camera_cal/calibration*.jpg')

# step through the list and search for chessboard corners
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, (9,6), None)

    # if found, add object points, image points
    if ret == True:
        objpoints.append(objp)
        imgpoints.append(corners)

        # draw and display the corners
        #img = cv2.drawChessboardCorners(img, (9,6), corners, ret)
        #cv2.imshow('img', img)
        #cv2.waitKey(500)

#cv2.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img.shape[1::-1], None, None)

############################################################################################################

# Threshold functions

# identify pixels where x- or y-gradient falls within specified range
def abs_sobel_thresh(img, sobel_kernel=3, orient='x', thresh=(0, 255)):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if (orient == 'x'):
        sobel = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    elif (orient == 'y'):
        sobel = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    abs_sobel = np.absolute(sobel)
    scaled_sobel = np.uint8(255 * abs_sobel / np.max(abs_sobel))
    output = np.zeros_like(scaled_sobel)
    output[(scaled_sobel >= thresh[0]) & (scaled_sobel <= thresh[1])] = 1
    return output

# identify pixels where total gradient falls within specified range
def mag_thresh(img, sobel_kernel=3, thresh=(0, 255)):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    sobel = np.sqrt(sobelx**2 + sobely**2)
    abs_sobel = np.absolute(sobel)
    scaled_sobel = np.uint8(255 * abs_sobel / np.max(abs_sobel))
    output = np.zeros_like(scaled_sobel)
    output[(scaled_sobel >= thresh[0]) & (scaled_sobel <= thresh[1])] = 1
    return output

# identify pixels where gradient direction falls within specified range
def dir_threshold(img, sobel_kernel=3, thresh=(0, np.pi/2)):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    sobeldir = np.arctan2(np.absolute(sobely), np.absolute(sobelx))
    output = np.zeros_like(sobeldir)
    output[(sobeldir >= thresh[0]) & (sobeldir <= thresh[1])] = 1
    return output

# identify pixels where S-channel of HLS falls within specified range
def s_threshold(img, thresh=(0, 255)):
    hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
    s_channel = hls[:,:,2]
    output = np.zeros_like(s_channel)
    output[(s_channel >= thresh[0]) & (s_channel <= thresh[1])] = 1
    return output

# identify pixels where R-channel of BGR falls within specified range
def r_threshold(img, thresh=(0, 255)):
    r_channel = img[:,:,2]
    output = np.zeros_like(r_channel)
    output[(r_channel >= thresh[0]) & (r_channel <= thresh[1])] = 1
    return output

############################################################################################################

images = glob.glob('./test_images/*.jpg')
#images = glob.glob('./camera_cal/calibration1.jpg')

for fname in images:

    # apply a distortion correction
    img = cv2.imread(fname)
    img_undist = cv2.undistort(img, mtx, dist, None, mtx)

    # create thresholded binary image
    ksize = 3
    gradx_mask = abs_sobel_thresh(img_undist, orient='x', sobel_kernel=ksize, thresh=(40, 255))
    grady_mask = abs_sobel_thresh(img_undist, orient='y', sobel_kernel=ksize, thresh=(40, 255))
    #mag_mask = mag_thresh(img_undist, sobel_kernel=ksize, thresh=(20, 150))
    #dir_mask = dir_threshold(img_undist, sobel_kernel=ksize, thresh=(0.7, 1.3))
    r_mask = r_threshold(img_undist, thresh=(230, 255))
    s_mask = s_threshold(img_undist, thresh=(200, 255))

    combined = np.zeros_like(gradx_mask)
    combined[(s_mask == 1) | (r_mask == 1) | ((gradx_mask == 1) & (grady_mask == 1))] = 1

    #gradxy_mask = np.zeros_like(gradx_mask)
    #gradxy_mask[((gradx_mask == 1) & (grady_mask == 1))] = 1

    # apply a perspective transform to rectify binary image ("birds-eye view")
    src_coord = np.float32([[700, 440], [1120, 700], [200, 700], [580, 440]])
    offset = 100
    img_size = (img_undist.shape[1], img_undist.shape[0])
    #offset = 0
    #img_size = (300, 300)
    dst_coord = np.float32([[img_size[0] - offset, offset], [img_size[0] - offset, img_size[1] - offset],
                            [offset, img_size[1] - offset], [offset, offset]])
    M = cv2.getPerspectiveTransform(src_coord, dst_coord)
    #Minv = cv2.getPerspectiveTransform(dst_coord, src_coord)
    combined = cv2.warpPerspective(combined, M, img_size, flags=cv2.INTER_LINEAR)

    #plt.imshow(img_undist)
    #plt.plot(src_coord[0][0], src_coord[0][1], '.')
    #plt.plot(src_coord[1][0], src_coord[1][1], '.')
    #plt.plot(src_coord[2][0], src_coord[2][1], '.')
    #plt.plot(src_coord[3][0], src_coord[3][1], '.')

    # plt.imshow(s_mask, cmap='gray')
    # plt.axis('off')
    filename, file_extension = os.path.splitext(fname)
    # plt.savefig(filename + '_smask' + '.jpeg')
    # plt.imshow(r_mask, cmap='gray')
    # plt.axis('off')
    # plt.savefig(filename + '_rmask' + '.jpeg')
    # plt.imshow(gradx_mask, cmap='gray')
    # plt.axis('off')
    # plt.savefig(filename + '_gradx' + '.jpeg')
    # plt.imshow(grady_mask, cmap='gray')
    # plt.axis('off')
    # plt.savefig(filename + '_grady' + '.jpeg')
    # plt.imshow(gradxy_mask, cmap='gray')
    # plt.axis('off')
    # plt.savefig(filename + '_gradxy' + '.jpeg')
    plt.imshow(combined, cmap='gray')
    plt.axis('off')
    plt.savefig(filename + '_combined   ' + '.jpeg')