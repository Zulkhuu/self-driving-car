
from os import path
from PIL import Image as PImage
from matplotlib import pyplot as plt
import cv2
import csv
import numpy as np
import pandas as pd
import random
import sklearn
from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle
from keras.models import Sequential
from keras.layers import Dense, Lambda, Flatten, Cropping2D, Dropout
from keras.layers.convolutional import Convolution2D
from keras.optimizers import SGD
from keras.callbacks import ModelCheckpoint
from sklearn.metrics import mean_squared_error

# Hide Tensorflow deprecated messages
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'


def read_data(dir_name, steering_correction = 0.2):
    """Read driving data from given directory
    Input:
        dir_name: directory that contains saved simulator data including
                    'IMG' folder for images
                    driving_log.csv
        steering_correction: fixed steering angle correction value for 
                             using left and right camera image
    Output: 
        samples: list of pair of image filename and steering angle
    """
    csv_filename = dir_name + 'driving_log.csv'
    img_dir = dir_name + 'IMG/'
    
    samples = []
    with open(csv_filename) as csvfile:
        reader = csv.reader(csvfile)
        next(reader, None)  # skip the headers
        for line in reader:
            center_image = img_dir + line[0].split('/')[-1].strip()
            center_steering =  float(line[3])
            samples.append([center_image, center_steering])

            # Add left camera image, steering angle = center angle - correction
            left_image = img_dir + line[1].split('/')[-1].strip()
            samples.append([left_image, center_steering + steering_correction])
            
            # Add right camera image, steering angle = center angle - correction
            right_image = img_dir + line[2].split('/')[-1].strip()
            samples.append([right_image, center_steering - steering_correction])

    return samples

def print_data_summary(samples, title):
    """Print summary data for samples
    """
    print(f"{title}:")
    print(f"   Total Frame#: {len(samples)}")
    print(f"   Center Image: {samples[0][0]} Steering: {samples[0][1]}")
    print(f"   Left Image:   {samples[1][0]} Steering: {samples[1][1]}")
    print(f"   Right Image:  {samples[2][0]} Steering: {samples[2][1]}")


# Read recorded sample data
center_samples = read_data('data/center_driving/')
print_data_summary(center_samples, "Center driving data")

hard_turn_samples = read_data('data/hard_turn/')
print_data_summary(hard_turn_samples, "Hard turn data")

total_samples = center_samples + hard_turn_samples
print(f"\nTotal sample#: {len(total_samples)}")


def generator(samples, batch_size=32):
    """Create a generator from given sample data
    Input:
        samples: list of pairs of image filename and corresponding steering angle
        batch_size: batch size
    Output: Generator
    """
    num_samples = len(samples)
    while 1: # Loop forever so the generator never terminates
        shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]

            images = []
            angles = []
            for batch_sample in batch_samples:
                name = batch_sample[0]

                image_orig = cv2.imread(name)
                angle_orig = float(batch_sample[1])
                images.append(image_orig)
                angles.append(angle_orig)

                # Add flipped version also (batch_size is doubled!)
                image_flipped = cv2.flip(image_orig, 1)
                angle_flipped = -angle_orig
                images.append(image_flipped)
                angles.append(angle_flipped)

            # trim image to only see section with road
            X_train = np.array(images)
            y_train = np.array(angles)
            yield sklearn.utils.shuffle(X_train, y_train)


# Split data to training and validation set and create a generator for each
batch_size = 64
valid_split = 0.2

train_samples, validation_samples = train_test_split(total_samples, test_size=valid_split)

# compile and train the model using the generator function
train_generator = generator(train_samples, batch_size=batch_size)
validation_generator = generator(validation_samples, batch_size=batch_size)

# Sanity Check Generator
#xtrain_, ytrain_ = next(validation_generator)
#print(xtrain_)


# Build model based on Nvidia's End to End Learning for Self-Driving Cars
# Paper URL: https://arxiv.org/abs/1604.07316

model = Sequential()  
model.add(Lambda(lambda X:(X/127.5)-1, input_shape=(160, 320, 3)))
model.add(Cropping2D(cropping=( (70,25), (0,0) )))
model.add(Convolution2D(24,(5,5),activation='relu',strides=(2,2), padding='valid'))
model.add(Convolution2D(36,(5,5),activation='relu',strides=(2,2), padding='valid'))
model.add(Convolution2D(48,(5,5),activation='relu',strides=(2,2), padding='valid'))
model.add(Convolution2D(64,(3,3),activation='relu', padding='valid'))
model.add(Convolution2D(64,(3,3),activation='relu', padding='valid'))
model.add(Flatten())
model.add(Dropout(0.5))
model.add(Dense(100, activation='relu'))
model.add(Dropout(0.5))
model.add(Dense(50, activation='relu'))
model.add(Dropout(0.5))
model.add(Dense(10, activation='relu'))
model.add(Dense(1))

model.compile(optimizer='adam', loss='mse')

n_epoch = 10
history_object = model.fit_generator(train_generator, 
            steps_per_epoch=np.ceil(len(train_samples)/batch_size), 
            validation_data=validation_generator, 
            validation_steps=np.ceil(len(validation_samples)/batch_size), 
            epochs=n_epoch, verbose=1)
model.save(f"model_{n_epoch}epoch_{history_object.history['val_loss'][-1]:.4f}loss.h5")

### Plot the training and validation loss for each epoch
plt.plot(history_object.history['loss'])
plt.plot(history_object.history['val_loss'])
plt.title('model mean squared error loss')
plt.ylabel('mean squared error loss')
plt.xlabel('epoch')
plt.legend(['training set', 'validation set'], loc='upper right')
plt.show()


