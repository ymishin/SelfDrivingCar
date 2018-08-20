import numpy as np
import cv2
import glob
import matplotlib.pyplot as plt
import os
from moviepy.editor import VideoFileClip

############################################################################################################

# Camera calibration

def calibrate():
    # prepare object points
    objp = np.zeros((6 * 9, 3), np.float32)
    objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)

    # arrays to store object points and image points from all the images
    objpoints = []  # 3d points in real world space
    imgpoints = []  # 2d points in image plane
    img = []

    # make a list of calibration images
    images = glob.glob('./camera_cal/calibration*.jpg')

    # step through the list and search for chessboard corners
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)

        # if found, add object points, image points
        if ret == True:
            objpoints.append(objp)
            imgpoints.append(corners)

            # draw and display the corners
            # img = cv2.drawChessboardCorners(img, (9,6), corners, ret)
            # cv2.imshow('img', img)
            # cv2.waitKey(500)

    # cv2.destroyAllWindows()
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img.shape[1::-1], None, None)

    images = glob.glob('./camera_cal/calibration*.jpg')
    for fname in images:
        img = cv2.imread(fname)
        img_undist = cv2.undistort(img, mtx, dist, None, mtx)
        plt.imshow(img_undist)
        head, tail = os.path.split(fname)
        plt.savefig('./output_images/' + 'undistorted_' + tail)

    return mtx, dist

mtx, dist = calibrate()

############################################################################################################

# Threshold functions

# identify pixels where x- or y-gradient falls within specified range
def sobel_threshold(img, sobel_kernel=3, orient='x', thresh=(0, 255)):
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
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
def mag_threshold(img, sobel_kernel=3, thresh=(0, 255)):
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
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
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    sobeldir = np.arctan2(np.absolute(sobely), np.absolute(sobelx))
    output = np.zeros_like(sobeldir)
    output[(sobeldir >= thresh[0]) & (sobeldir <= thresh[1])] = 1
    return output

# identify pixels where S-channel of HLS falls within specified range
def s_threshold(img, thresh=(0, 255)):
    hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
    s_channel = hls[:,:,2]
    output = np.zeros_like(s_channel)
    output[(s_channel >= thresh[0]) & (s_channel <= thresh[1])] = 1
    return output

# identify pixels where R-channel of RGB falls within specified range
def r_threshold(img, thresh=(0, 255)):
    r_channel = img[:,:,0]
    output = np.zeros_like(r_channel)
    output[(r_channel >= thresh[0]) & (r_channel <= thresh[1])] = 1
    return output

############################################################################################################

def find_lane_pixels(binary_warped):
    # Take a histogram of the bottom half of the image
    histogram = np.sum(binary_warped[binary_warped.shape[0] // 2:, :], axis=0)
    # Create an output image to draw on
    out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 200
    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines
    midpoint = np.int(histogram.shape[0] // 2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    # Choose the number of sliding windows
    nwindows = 9
    # Set the width of the windows +/- margin
    margin = 100
    # Set minimum number of pixels found to recenter window
    minpix = 50

    # Set height of windows - based on nwindows above and image shape
    window_height = np.int(binary_warped.shape[0] // nwindows)
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    # Current positions to be updated later for each window in nwindows
    leftx_current = leftx_base
    rightx_current = rightx_base

    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []

    # Step through the windows one by one
    for window in range(nwindows):
        # Identify window boundaries in x and y (and right and left)
        win_y_low = binary_warped.shape[0] - (window + 1) * window_height
        win_y_high = binary_warped.shape[0] - window * window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin

        # Draw the windows on the visualization image
        cv2.rectangle(out_img, (win_xleft_low, win_y_low),
                      (win_xleft_high, win_y_high), (0, 255, 0), 2)
        cv2.rectangle(out_img, (win_xright_low, win_y_low),
                      (win_xright_high, win_y_high), (0, 255, 0), 2)

        # Identify the nonzero pixels in x and y within the window
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                          (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                           (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

        # Append these indices to the lists
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)

        # Recenter next window if more than minpix pixels are found
        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

    # Concatenate the arrays of indices (previously was a list of lists of pixels)
    try:
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
    except ValueError:
        # Avoids an error if the above is not implemented fully
        pass

    # Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    return leftx, lefty, rightx, righty, out_img

############################################################################################################

def measure_curvature_pixels(binary_warped, lefty, leftx, righty, rightx):
    '''
    Calculates the curvature of polynomial functions in pixels.
    '''
    # Define y-value where we want radius of curvature
    # We'll choose the maximum y-value, corresponding to the bottom of the image
    y_eval = binary_warped.shape[0]

    # Define conversions in x and y from pixels space to meters
    margin = 100
    ym_per_pix = 30.0 / (binary_warped.shape[0] - 2 * margin) # meters per pixel in y dimension
    xm_per_pix = 3.7 / (binary_warped.shape[1] - 2 * margin) # meters per pixel in x dimension

    left_fit_cr = np.polyfit(lefty * ym_per_pix, leftx * xm_per_pix, 2)
    right_fit_cr = np.polyfit(righty * ym_per_pix, rightx * xm_per_pix, 2)

    left_curverad = ((1 + (2 * left_fit_cr[0] * y_eval * ym_per_pix + left_fit_cr[1]) ** 2) ** 1.5) / np.absolute(2 * left_fit_cr[0])
    right_curverad = ((1 + (2 * right_fit_cr[0] * y_eval * ym_per_pix + right_fit_cr[1]) ** 2) ** 1.5) / np.absolute(2 * right_fit_cr[0])

    return left_curverad, right_curverad

############################################################################################################

def find_lanes_and_fit_polynomials(binary_warped, output_mode=1, history=None):

    # Find lane pixels first
    leftx, lefty, rightx, righty, out_img = find_lane_pixels(binary_warped)

    # Fit a second order polynomial to each lane
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)

    left_curverad, right_curverad = measure_curvature_pixels(binary_warped, lefty, leftx, righty, rightx)

    if (history is not None):
        max_diff = 2.5
        if (history.left_curverad != 0 and (history.left_curverad / left_curverad > max_diff or left_curverad / history.left_curverad > max_diff)):
            left_curverad = history.left_curverad
            left_fit = history.left_fit
        else:
            history.left_curverad = left_curverad
            history.left_fit = left_fit
        if (history.right_curverad != 0 and (history.right_curverad / right_curverad > max_diff or right_curverad / history.right_curverad > max_diff)):
            right_curverad = history.right_curverad
            right_fit = history.right_fit
        else:
            history.right_curverad = right_curverad
            history.right_fit = right_fit

    # Offset
    margin = 100
    ym_per_pix = 30.0 / (binary_warped.shape[0] - 2 * margin)  # meters per pixel in y dimension
    xm_per_pix = 3.7 / (binary_warped.shape[1] - 2 * margin)  # meters per pixel in x dimension
    image_center = binary_warped.shape[1] / 2
    y_eval = binary_warped.shape[0]
    lane_center = ((left_fit[0] * y_eval ** 2 + left_fit[1] * y_eval + left_fit[2]) + (right_fit[0] * y_eval ** 2 + right_fit[1] * y_eval + right_fit[2])) / 2
    offset = xm_per_pix * (image_center - lane_center)

    # Generate x and y values for plotting
    ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
    try:
        left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
        right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
    except TypeError:
        # Avoids an error if `left` and `right_fit` are still none or incorrect
        print('The function failed to fit a line!')
        left_fitx = 1 * ploty ** 2 + 1 * ploty
        right_fitx = 1 * ploty ** 2 + 1 * ploty

    if (output_mode == 1):
        # Colors in the left and right lane regions
        out_img[lefty, leftx] = [255, 0, 0]
        out_img[righty, rightx] = [0, 0, 255]

        # Plots the left and right polynomials on the lane lines
        plt.imshow(out_img)
        plt.xlim(0, 1280)
        plt.ylim(720, 0)
        plt.plot(left_fitx, ploty, color='yellow')
        plt.plot(right_fitx, ploty, color='yellow')
    elif (output_mode == 2):
        # Create an image to draw the lines on
        warp_zero = np.zeros_like(binary_warped).astype(np.uint8)
        out_img = np.dstack((warp_zero, warp_zero, warp_zero))

        # Recast the x and y points into usable format for cv2.fillPoly()
        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))

        # Draw the lane onto the warped blank image
        cv2.fillPoly(out_img, np.int_([pts]), (0, 255, 0))

    return out_img, left_curverad, right_curverad, offset

############################################################################################################

def process_image(fname):

    img = cv2.imread(fname)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    img_undist = cv2.undistort(img, mtx, dist, None, mtx)

    plt.figure()
    plt.imshow(img_undist)
    head, tail = os.path.split(fname)
    plt.savefig('./output_images/' + 'undistorted_' + tail)

    # create thresholded binary image
    ksize = 3
    gradx_mask = sobel_threshold(img_undist, orient='x', sobel_kernel=ksize, thresh=(40, 255))
    grady_mask = sobel_threshold(img_undist, orient='y', sobel_kernel=ksize, thresh=(40, 255))
    # mag_mask = mag_thresh(img_undist, sobel_kernel=ksize, thresh=(20, 150))
    # dir_mask = dir_threshold(img_undist, sobel_kernel=ksize, thresh=(0.7, 1.3))
    r_mask = r_threshold(img_undist, thresh=(230, 255))
    s_mask = s_threshold(img_undist, thresh=(170, 255))
    combined = np.zeros_like(gradx_mask)
    combined[(s_mask == 1) | (r_mask == 1) | ((gradx_mask == 1) & (grady_mask == 1))] = 1

    plt.figure()
    plt.imshow(combined)
    plt.savefig('./output_images/' + 'binary_' + tail)

    # gradxy_mask = np.zeros_like(gradx_mask)
    # gradxy_mask[((gradx_mask == 1) & (grady_mask == 1))] = 1

    # apply a perspective transform to rectify binary image ("birds-eye view")
    src_coord = np.float32([[700, 450], [1110, 660], [190, 660], [580, 450]])
    margin = 100
    img_size = (img_undist.shape[1], img_undist.shape[0])
    dst_coord = np.float32([[img_size[0] - margin, margin], [img_size[0] - margin, img_size[1] - margin],
                            [margin, img_size[1] - margin], [margin, margin]])
    M = cv2.getPerspectiveTransform(src_coord, dst_coord)
    Minv = cv2.getPerspectiveTransform(dst_coord, src_coord)
    combined = cv2.warpPerspective(combined, M, img_size, flags=cv2.INTER_LINEAR)

    plt.figure()
    plt.imshow(combined)
    plt.savefig('./output_images/' + 'warped_' + tail)

    plt.figure()
    out_img, left_curverad, right_curverad, offset = find_lanes_and_fit_polynomials(combined, output_mode=1)
    plt.savefig('./output_images/' + 'poly_' + tail)
    print(tail, left_curverad, right_curverad, offset)

    out_img, left_curverad, right_curverad, offset = find_lanes_and_fit_polynomials(combined, output_mode=2)
    # Warp the blank back to original image space using inverse perspective matrix (Minv)
    out_img = cv2.warpPerspective(out_img, Minv, (img.shape[1], img.shape[0]))
    # Combine the result with the original image
    out_img = cv2.addWeighted(img_undist, 1, out_img, 0.3, 0)
    plt.figure()
    plt.imshow(out_img)
    plt.savefig('./output_images/' + 'lanelines_' + tail)

    return out_img

############################################################################################################

def process_video_frame(img, history):

    img_undist = cv2.undistort(img, mtx, dist, None, mtx)

    # create thresholded binary image
    ksize = 3
    gradx_mask = sobel_threshold(img_undist, orient='x', sobel_kernel=ksize, thresh=(40, 255))
    grady_mask = sobel_threshold(img_undist, orient='y', sobel_kernel=ksize, thresh=(40, 255))
    # mag_mask = mag_thresh(img_undist, sobel_kernel=ksize, thresh=(20, 150))
    # dir_mask = dir_threshold(img_undist, sobel_kernel=ksize, thresh=(0.7, 1.3))
    r_mask = r_threshold(img_undist, thresh=(230, 255))
    s_mask = s_threshold(img_undist, thresh=(170, 255))

    combined = np.zeros_like(gradx_mask)
    combined[(s_mask == 1) | (r_mask == 1) | ((gradx_mask == 1) & (grady_mask == 1))] = 1

    # gradxy_mask = np.zeros_like(gradx_mask)
    # gradxy_mask[((gradx_mask == 1) & (grady_mask == 1))] = 1

    # apply a perspective transform to rectify binary image ("birds-eye view")
    src_coord = np.float32([[700, 450], [1110, 660], [190, 660], [580, 450]])
    offset = 100
    img_size = (img_undist.shape[1], img_undist.shape[0])
    dst_coord = np.float32([[img_size[0] - offset, offset], [img_size[0] - offset, img_size[1] - offset],
                            [offset, img_size[1] - offset], [offset, offset]])
    M = cv2.getPerspectiveTransform(src_coord, dst_coord)
    Minv = cv2.getPerspectiveTransform(dst_coord, src_coord)
    combined = cv2.warpPerspective(combined, M, img_size, flags=cv2.INTER_LINEAR)

    out_img, left_curverad, right_curverad, offset = find_lanes_and_fit_polynomials(combined, 2, history)

    # Warp the blank back to original image space using inverse perspective matrix (Minv)
    out_img = cv2.warpPerspective(out_img, Minv, (img.shape[1], img.shape[0]))
    # Combine the result with the original image
    out_img = cv2.addWeighted(img_undist, 1, out_img, 0.3, 0)
    text = "left / right R: " + str(left_curverad) + " / " + str(right_curverad)
    cv2.putText(out_img, text, (10, 50), cv2.FONT_HERSHEY_PLAIN, 2.0, (255, 255, 255), 3)
    text = "car offset: " + str(offset)
    cv2.putText(out_img, text, (10, 100), cv2.FONT_HERSHEY_PLAIN, 2.0, (255, 255, 255), 3)

    return out_img

############################################################################################################

def process_images():
    images = glob.glob('./test_images/*.jpg')
    for fname in images:
        process_image(fname)

############################################################################################################

class History():
    def __init__(self):
        self.left_curverad = 0
        self.right_curverad = 0
        self.left_fit = []
        self.right_fit = []

def process_video():
    input_clip_file = "project_video.mp4"
    output_clip_file = "project_video_with_lines.mp4"
    history = History()
    input_clip = VideoFileClip(input_clip_file)
    output_clip = input_clip.fl_image(lambda img: process_video_frame(img, history))
    output_clip.write_videofile(output_clip_file, audio=False)

############################################################################################################

calibrate()
process_images()
process_video()