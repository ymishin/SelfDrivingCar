import csv
import numpy as np
from PIL import Image
from keras import backend as K
from keras.models import Sequential
from keras.layers.core import Dense, Activation, Flatten, Dropout, Lambda
from keras.layers.convolutional import Convolution2D, Cropping2D
from keras.layers.pooling import MaxPooling2D
from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle
import matplotlib.pyplot as plt

##### Read data file #####

# images = []
# angles = []
# with open(data_folder + 'driving_log.csv') as csvfile:
#     reader = csv.reader(csvfile, skipinitialspace=True)
#     #next(reader)
#     for line in reader:
#
#         img_center = np.asarray(Image.open(line[0]))
#         #img_center = np.asarray(Image.open(data_folder + line[0]))
#
#         #img_left = np.asarray(Image.open(data_folder + line[1]))
#         #img_right = np.asarray(Image.open(data_folder + line[2]))
#         #images.extend([img_center, img_left, img_right])
#         images.append(img_center)
#         steering_center = float(line[3])
#         #steering_left = steering_center + 0.1
#         #steering_right = steering_center - 0.1
#         #angles.extend([steering_center, steering_left, steering_right])
#         angles.append(steering_center)
#
#X_train = np.array(images)
#y_train = np.array(angles)

data_folder = 'C://sandbox/sample_data/'
#data_folder = 'C://sandbox/train_data/'

samples = []
with open(data_folder + 'driving_log.csv') as csvfile:
    reader = csv.reader(csvfile, skipinitialspace=True)
    #next(reader)
    for line in reader:
        samples.append(line)

##### Split the data and construct generators #####

train_samples, validation_samples = train_test_split(samples, test_size=0.2, shuffle=True)

print('Training samples: ', len(train_samples))
print('Validation samples: ', len(validation_samples))

def generator(samples, batch_size=32):
    while True:
        shuffle(samples)
        for offset in range(0, len(samples), batch_size):
            images = []
            angles = []
            for batch_sample in samples[offset:offset+batch_size]:
                #image = np.asarray(Image.open(line[0]))
                image = np.asarray(Image.open(data_folder + batch_sample[0]))
                angle = float(batch_sample[3])
                images.append(image)
                angles.append(angle)
            X_train = np.array(images)
            y_train = np.array(angles)
            yield shuffle(X_train, y_train)

train_generator = generator(train_samples)
validation_generator = generator(validation_samples)

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

# Nvidia CNN
model = Sequential()
model.add(Cropping2D(cropping=((60,20),(0,0)), input_shape=(160,320,3)))
model.add(Lambda(lambda x: K.tf.image.resize_images(x, (66, 200))))
#model.add(Lambda(lambda x: x / 255.0 - 0.5))
model.add(Lambda(lambda x: x / 127.5 - 1.0))
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
#model.add(Dropout(0.5))
model.add(Dense(50))
#model.add(Dropout(0.5))
model.add(Dense(10))
#model.add(Dropout(0.5))
model.add(Dense(1))
model.compile(loss='mse', optimizer='adam')

##### Train the network #####

#history_object = model.fit(X_train, y_train, validation_split=0.2, shuffle=True, nb_epoch=3, verbose=2)
history_object = model.fit_generator(train_generator, samples_per_epoch=len(train_samples),
    validation_data=validation_generator, nb_val_samples=len(validation_samples), nb_epoch=3, verbose=2)

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