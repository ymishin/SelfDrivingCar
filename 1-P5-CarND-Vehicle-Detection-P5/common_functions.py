import os
import numpy as np
import cv2
import glob
import zipfile
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from random import shuffle
from skimage.feature import hog
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVC, LinearSVC
from sklearn.metrics import accuracy_score
from sklearn.model_selection import train_test_split
from scipy.ndimage.measurements import label
from moviepy.editor import VideoFileClip

############################################################################################################

def unzip_data():
    data_files = ['vehicles.zip', 'non-vehicles.zip']
    data_folder = './data/'
    for file in data_files:
        zip_ref = zipfile.ZipFile(data_folder + file, 'r')
        zip_ref.extractall(data_folder)
        zip_ref.close()

def read_data():
    data_folder = './data/'
    car_images = []
    noncar_images = []
    for fname in glob.glob(data_folder + 'vehicles/**/*.png', recursive=True):
        car_images.append(fname)
    for fname in glob.glob(data_folder + 'non-vehicles/**/*.png', recursive=True):
        noncar_images.append(fname)
    print('Car images: ', len(car_images))
    print('Non-car images: ', len(noncar_images))
    img = cv2.imread(car_images[0])
    print('Shape: ', img.shape)
    print('Type: ', img.dtype)
    return car_images, noncar_images

def bin_spatial(img, size=(32, 32)):
    color1 = cv2.resize(img[:, :, 0], size).ravel()
    color2 = cv2.resize(img[:, :, 1], size).ravel()
    color3 = cv2.resize(img[:, :, 2], size).ravel()
    return np.hstack((color1, color2, color3))

def color_hist(img, nbins=32):
    # Compute the histogram of the color channels separately
    channel1_hist = np.histogram(img[:, :, 0], bins=nbins)
    channel2_hist = np.histogram(img[:, :, 1], bins=nbins)
    channel3_hist = np.histogram(img[:, :, 2], bins=nbins)
    # Concatenate the histograms into a single feature vector
    hist_features = np.concatenate((channel1_hist[0], channel2_hist[0], channel3_hist[0]))
    return hist_features

def extract_hog(img, feature_vec=True):
    orient = 11
    pix_per_cell = 8
    cell_per_block = 2
    vis = False
    if vis:
        hog_features, hog_image = hog(img, orientations=orient,
                                      pixels_per_cell=(pix_per_cell, pix_per_cell),
                                      cells_per_block=(cell_per_block, cell_per_block),
                                      visualise=vis, feature_vector=feature_vec,
                                      block_norm='L2-Hys')
        return hog_features, hog_image
    else:
        hog_features = hog(img, orientations=orient,
                           pixels_per_cell=(pix_per_cell, pix_per_cell),
                           cells_per_block=(cell_per_block, cell_per_block),
                           visualise=vis, feature_vector=feature_vec,
                           block_norm='L2-Hys')
        return hog_features

def extract_features_single(img):
    cspace = 'YCrCb'
    if cspace != 'RGB':
        if cspace == 'HSV':
            feature_image = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        elif cspace == 'LUV':
            feature_image = cv2.cvtColor(img, cv2.COLOR_RGB2LUV)
        elif cspace == 'HLS':
            feature_image = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
        elif cspace == 'YUV':
            feature_image = cv2.cvtColor(img, cv2.COLOR_RGB2YUV)
        elif cspace == 'YCrCb':
            feature_image = cv2.cvtColor(img, cv2.COLOR_RGB2YCrCb)
    else:
        feature_image = np.copy(img)
    hog_features = []
    for channel in range(feature_image.shape[2]):
        hog_features.append(extract_hog(feature_image[:, :, channel]))
    hog_features = np.ravel(hog_features)
    all_features = False
    if (all_features):
        spatial_features = bin_spatial(feature_image)
        hist_features = color_hist(feature_image)
        features = np.concatenate((hog_features, spatial_features, hist_features))
    else:
        features = hog_features
    return features

def extract_features(images):
    features = []
    for fname in images:
        img = mpimg.imread(fname)
        features.append(extract_features_single(img))
    return features

def train_svm_classifier(car_features, noncar_features):
    # split the data into training and test sets
    X = np.vstack((car_features, noncar_features)).astype(np.float64)
    y = np.hstack((np.ones(len(car_features)), np.zeros(len(noncar_features))))
    rand_state = np.random.randint(0, 100)
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=rand_state)
    # fit a scaler
    X_scaler = StandardScaler().fit(X_train)
    X_train = X_scaler.transform(X_train)
    X_test = X_scaler.transform(X_test)
    # SVM classifier
    clf = LinearSVC()
    # fit the classifier
    clf.fit(X_train, y_train)
    # check accuracy
    pred = clf.predict(X_test)
    acc = accuracy_score(pred, y_test)
    print("SVM accuracy: ", acc)
    return clf, X_scaler

def process_images(clf, scaler):
    images = glob.glob('./test_images/*.jpg')
    for fname in images:
        img = mpimg.imread(fname)
        img = process_single_image(img, clf, scaler)
        plt.figure()
        plt.imshow(img)
        head, tail = os.path.split(fname)
        plt.savefig('./output_images/' + tail)

class History():
    def __init__(self, max_size=30):
        self.boxes = []
        self.max_size = max_size
    def add_boxes(self, boxes):
        self.boxes.extend(boxes)
        if (len(self.boxes) > self.max_size):
            self.boxes = self.boxes[-self.max_size:]

def process_video(clf, scaler):
    input_clip_file = "project_video.mp4"
    output_clip_file = "project_video_with_tracking.mp4"
    input_clip = VideoFileClip(input_clip_file)
    output_clip = input_clip.fl_image(lambda img: process_single_image(img, clf, scaler))
    output_clip.write_videofile(output_clip_file, audio=False)

def process_single_image(img, clf, scaler, history=None):
    boxes = []
    for scale in [0.75]:
        hot_windows = find_cars(img, int(img.shape[0] * 0.5), int(img.shape[0] * 0.7), scale, clf, scaler, 2)
        boxes.extend(hot_windows)
    for scale in [1.0, 1.5, 2.0, 2.5, 3.0, 3.5]:
        hot_windows = find_cars(img, int(img.shape[0] * 0.5), int(img.shape[0] * 1.0), scale, clf, scaler, 2)
        boxes.extend(hot_windows)
    heatmap = np.zeros_like(img[:, :, 0]).astype(np.float)
    if (history is None):
        add_heat(heatmap, boxes)
        apply_threshold(heatmap, 4)
    else:
        history.add_boxes(boxes)
        add_heat(heatmap, history.boxes)
        apply_threshold(heatmap, 5)
    labels = label(heatmap)
    img, boxes = draw_labeled_bboxes(np.copy(img), labels)
    return img

def add_heat(heatmap, bbox_list):
    # Iterate through list of bboxes
    for box in bbox_list:
        # Add += 1 for all pixels inside each bbox
        # Assuming each "box" takes the form ((x1, y1), (x2, y2))
        heatmap[box[0][1]:box[1][1], box[0][0]:box[1][0]] += 1
    # Return updated heatmap
    return heatmap

def apply_threshold(heatmap, threshold):
    # Zero out pixels below the threshold
    heatmap[heatmap <= threshold] = 0
    # Return thresholded map
    return heatmap

def draw_boxes(img, boxes):
    for box in boxes:
        cv2.rectangle(img, box[0], box[1], (0, 0, 255), 5)
    return img

def draw_labeled_bboxes(img, labels):
    boxes = []
    for car_number in range(1, labels[1]+1):
        # Find pixels with each car_number label value
        nonzero = (labels[0] == car_number).nonzero()
        # Identify x and y values of those pixels
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # Define a bounding box based on min/max x and y
        bbox = ((np.min(nonzerox), np.min(nonzeroy)), (np.max(nonzerox), np.max(nonzeroy)))
        # Draw the box on the image
        cv2.rectangle(img, bbox[0], bbox[1], (0,0,255), 6)
        boxes.append(bbox)
    # Return the image
    return img, boxes

def find_cars(img, ystart, ystop, scale, svc, scaler, cells_per_step=2):

    img = img.astype(np.float32) / 255
    cspace = 'YCrCb'
    if cspace != 'RGB':
        if cspace == 'HSV':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        elif cspace == 'LUV':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2LUV)
        elif cspace == 'HLS':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
        elif cspace == 'YUV':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2YUV)
        elif cspace == 'YCrCb':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2YCrCb)

    ctrans_tosearch = img[ystart:ystop, :, :]
    if scale != 1.0:
        imshape = ctrans_tosearch.shape
        ctrans_tosearch = cv2.resize(ctrans_tosearch, (np.int(imshape[1] / scale), np.int(imshape[0] / scale)))

    ch1 = ctrans_tosearch[:, :, 0]
    ch2 = ctrans_tosearch[:, :, 1]
    ch3 = ctrans_tosearch[:, :, 2]

    orient = 11
    pix_per_cell = 8
    cell_per_block = 2

    # Define blocks and steps as above
    nxblocks = (ch1.shape[1] // pix_per_cell) - cell_per_block + 1
    nyblocks = (ch1.shape[0] // pix_per_cell) - cell_per_block + 1
    nfeat_per_block = orient * cell_per_block ** 2

    # 64 was the orginal sampling rate, with 8 cells and 8 pix per cell
    window = 64
    nblocks_per_window = (window // pix_per_cell) - cell_per_block + 1
    nxsteps = (nxblocks - nblocks_per_window) // cells_per_step + 1
    nysteps = (nyblocks - nblocks_per_window) // cells_per_step + 1

    # Compute individual channel HOG features for the entire image
    hog1 = extract_hog(ch1, feature_vec=False)
    hog2 = extract_hog(ch2, feature_vec=False)
    hog3 = extract_hog(ch3, feature_vec=False)

    window_list = []
    for xb in range(nxsteps):
        for yb in range(nysteps):
            ypos = yb * cells_per_step
            xpos = xb * cells_per_step
            # Extract HOG for this patch
            hog_feat1 = hog1[ypos:ypos + nblocks_per_window, xpos:xpos + nblocks_per_window].ravel()
            hog_feat2 = hog2[ypos:ypos + nblocks_per_window, xpos:xpos + nblocks_per_window].ravel()
            hog_feat3 = hog3[ypos:ypos + nblocks_per_window, xpos:xpos + nblocks_per_window].ravel()
            hog_features = np.hstack((hog_feat1, hog_feat2, hog_feat3))

            xleft = xpos * pix_per_cell
            ytop = ypos * pix_per_cell

            all_features = False
            if (all_features):
                # Extract the image patch
                subimg = cv2.resize(ctrans_tosearch[ytop:ytop + window, xleft:xleft + window], (64, 64))
                spatial_features = bin_spatial(subimg)
                hist_features = color_hist(subimg)
                test_features = np.hstack((hog_features, spatial_features, hist_features)).reshape(1, -1)
            else:
                test_features = hog_features.reshape(1, -1)

            # Scale features and make a prediction
            test_features = scaler.transform(test_features)
            test_prediction = svc.predict(test_features)

            if test_prediction == 1:
                xbox_left = np.int(xleft * scale)
                ytop_draw = np.int(ytop * scale)
                win_draw = np.int(window * scale)
                window_list.append(((xbox_left, ytop_draw + ystart), (xbox_left + win_draw, ytop_draw + win_draw + ystart)))

    return window_list