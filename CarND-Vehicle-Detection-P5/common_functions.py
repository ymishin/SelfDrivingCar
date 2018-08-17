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
from sklearn.svm import SVC
from sklearn.metrics import accuracy_score
from sklearn.model_selection import train_test_split

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

def extract_hog(img):
    orient = 9
    pix_per_cell = 8
    cell_per_block = 2
    vis = False
    if vis:
        hog_features, hog_image = hog(img, orientations=orient,
                                      pixels_per_cell=(pix_per_cell, pix_per_cell),
                                      cells_per_block=(cell_per_block, cell_per_block),
                                      visualize=vis, feature_vector=True,
                                      block_norm='L2-Hys')
        return hog_features, hog_image
    else:
        hog_features = hog(img, orientations=orient,
                           pixels_per_cell=(pix_per_cell, pix_per_cell),
                           cells_per_block=(cell_per_block, cell_per_block),
                           visualize=vis, feature_vector=True,
                           block_norm='L2-Hys')
        return hog_features

def extract_features_single(img):
    cspace = 'YCrCb'  # ~98-99
    # cspace = 'BGR' # ~97
    if cspace != 'BGR':
        if cspace == 'HSV':
            feature_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        elif cspace == 'LUV':
            feature_image = cv2.cvtColor(img, cv2.COLOR_BGRLUV)
        elif cspace == 'HLS':
            feature_image = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        elif cspace == 'YUV':
            feature_image = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
        elif cspace == 'YCrCb':
            feature_image = cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)
    else:
        feature_image = np.copy(img)
        # COLOR_BGR2RGB
    hog_features = []
    for channel in range(feature_image.shape[2]):
        hog_features.append(extract_hog(feature_image[:, :, channel]))
    return np.ravel(hog_features)

def extract_features(images):
    features = []
    for fname in images:
        img = cv2.imread(fname)
        hog_features = extract_features_single(img)
        features.append(hog_features)
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
    #clf = SVC(kernel="rbf", gamma=5.0)
    clf = SVC(kernel="linear")
    # fit the classifier
    clf.fit(X_train, y_train)
    # check accuracy
    pred = clf.predict(X_test)
    acc = accuracy_score(pred, y_test)
    print("SVM accuracy: ", acc)
    return clf, X_scaler

def search_windows(img, window_list, clf, scaler):
    hot_windows = []
    for window in window_list:
        test_img = cv2.resize(img[window[0][1]:window[1][1], window[0][0]:window[1][0]], (64, 64))
        features = extract_features_single(test_img)
        test_features = scaler.transform(np.array(features).reshape(1, -1))
        if (clf.predict(test_features)) == 1:
            hot_windows.append(window)
    return hot_windows

def process_images(clf, scaler):
    images = glob.glob('./test_images/*.jpg')
    for fname in images:
        img = cv2.imread(fname)
        #img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        window_list = slide_window(img, y_start_stop=[int(img.shape[0]*0.5), None])
        hot_windows = search_windows(img, window_list, clf, scaler)
        for bbox in hot_windows:
            cv2.rectangle(img, bbox[0], bbox[1], (0, 0, 255), 5)
        plt.figure()
        plt.imshow(img)
        filename, file_extension = os.path.splitext(fname)
        plt.savefig(filename + '_out' + '.jpeg')

def slide_window(img, x_start_stop=[None, None], y_start_stop=[None, None],
                 xy_window=(64, 64), xy_overlap=(0.5, 0.5)):
    # If x and/or y start/stop positions not defined, set to image size
    if x_start_stop[0] == None:
        x_start_stop[0] = 0
    if x_start_stop[1] == None:
        x_start_stop[1] = img.shape[1]
    if y_start_stop[0] == None:
        y_start_stop[0] = 0
    if y_start_stop[1] == None:
        y_start_stop[1] = img.shape[0]
    # Compute the span of the region to be searched
    xspan = x_start_stop[1] - x_start_stop[0]
    yspan = y_start_stop[1] - y_start_stop[0]
    # Compute the number of pixels per step in x/y
    nx_pix_per_step = np.int(xy_window[0]*(1 - xy_overlap[0]))
    ny_pix_per_step = np.int(xy_window[1]*(1 - xy_overlap[1]))
    # Compute the number of windows in x/y
    nx_buffer = np.int(xy_window[0]*(xy_overlap[0]))
    ny_buffer = np.int(xy_window[1]*(xy_overlap[1]))
    nx_windows = np.int((xspan-nx_buffer)/nx_pix_per_step)
    ny_windows = np.int((yspan-ny_buffer)/ny_pix_per_step)
    # Initialize a list to append window positions to
    window_list = []
    # Loop through finding x and y window positions
    # Note: you could vectorize this step, but in practice
    # you'll be considering windows one by one with your
    # classifier, so looping makes sense
    for ys in range(ny_windows):
        for xs in range(nx_windows):
            # Calculate window position
            startx = xs*nx_pix_per_step + x_start_stop[0]
            endx = startx + xy_window[0]
            starty = ys*ny_pix_per_step + y_start_stop[0]
            endy = starty + xy_window[1]
            # Append window position to list
            window_list.append(((startx, starty), (endx, endy)))
    # Return the list of windows
    return window_list