import csv
import numpy as np
from PIL import Image
from keras.models import Sequential
from keras.layers.core import Dense, Activation, Flatten, Dropout, Lambda
from keras.layers.convolutional import Convolution2D, Cropping2D
from keras.layers.pooling import MaxPooling2D
import matplotlib.pyplot as plt

##### Load the data #####

#data_folder = 'd://sample_data/'
data_folder = 'd://train_data/'

images = []
angles = []
with open(data_folder + 'driving_log.csv') as csvfile:
    reader = csv.reader(csvfile, skipinitialspace=True)
    #next(reader)
    for line in reader:
        img_center = np.asarray(Image.open(line[0]))

        #img_center = np.asarray(Image.open(data_folder + line[0]))

        #img_left = np.asarray(Image.open(data_folder + line[1]))
        #img_right = np.asarray(Image.open(data_folder + line[2]))
        #images.extend([img_center, img_left, img_right])
        images.append(img_center)
        steering_center = float(line[3])
        #steering_left = steering_center + 0.1
        #steering_right = steering_center - 0.1
        #angles.extend([steering_center, steering_left, steering_right])
        angles.append(steering_center)

X_train = np.array(images)
y_train = np.array(angles)

##### Construct the network #####

# LeNet-5
#model = Sequential()
#model.add(Lambda(lambda x: x / 255.0 - 0.5, input_shape=(160,320,3)))
#model.add(Cropping2D(cropping=((60,20),(0,0))))
#model.add(Convolution2D(6, 5, 5))
#model.add(MaxPooling2D((2, 2)))
#model.add(Activation('relu'))
#model.add(Convolution2D(16, 5, 5))
#model.add(MaxPooling2D((2, 2)))
#model.add(Activation('relu'))
#model.add(Flatten())
#model.add(Dense(120))
#model.add(Dropout(0.5))
#model.add(Activation('relu'))
#model.add(Dense(84))
#model.add(Dropout(0.5))
#model.add(Activation('relu'))
#model.add(Dense(1))

# CNN from nVidia
model = Sequential()
model.add(Lambda(lambda x: x / 255.0 - 0.5, input_shape=(160,320,3)))
model.add(Cropping2D(cropping=((60,20),(0,0))))
model.add(Convolution2D(24, 5, 5, subsample=(2, 2)))
model.add(Activation('relu'))
model.add(Convolution2D(36, 5, 5, subsample=(2, 2)))
model.add(Activation('relu'))
model.add(Convolution2D(48, 5, 5, subsample=(2, 2)))
model.add(Activation('relu'))
model.add(Convolution2D(64, 3, 3))
model.add(Activation('relu'))
model.add(Convolution2D(64, 3, 3))
model.add(Activation('relu'))
model.add(Flatten())
model.add(Dense(100))
model.add(Dropout(0.5))
model.add(Dense(50))
model.add(Dropout(0.5))
model.add(Dense(10))
model.add(Dropout(0.5))
model.add(Dense(1))

model.compile(loss='mse', optimizer='adam')

##### Train the network #####

model.fit(X_train, y_train, validation_split=0.2, shuffle=True, nb_epoch=3, verbose=2)

#history_object = model.fit(X_train, y_train, validation_split=0.2, shuffle=True, nb_epoch=5, verbose=2)
#print(history_object.history.keys())
#plt.plot(history_object.history['loss'])
#plt.plot(history_object.history['val_loss'])
#plt.title('model mean squared error loss')
#plt.ylabel('mean squared error loss')
#plt.xlabel('epoch')
#plt.legend(['training set', 'validation set'], loc='upper right')
#plt.show()

##### Save the results #####

model.save(data_folder + 'model.h5')