import os
import cv2
import numpy as np
import random
import matplotlib.pyplot as plt

from tensorflow.keras.utils import to_categorical

def load(dataset_path,subset):
    rows = open(dataset_path+subset).read().strip().split("\n")[1:]
    random.shuffle(rows)
    X, y = [],[]
    for row in rows:
        (label, img_path) = row.strip().split(",")[-2:]
        img = cv2.imread(dataset_path+img_path,cv2.IMREAD_UNCHANGED)

        X.append(img)
        y.append(label)
    X, y = np.array(X), np.array(y)

    return X,y

def load_dataset(dataset_path):
    # load dataset
    X_train, y_train = load(dataset_path,"Train.csv")
    X_test, y_test = load(dataset_path,"Test.csv")
    # print size
    print('Loaded dataset with size:')
    print('      train(X,y): ',(X_train.shape,y_train.shape))
    print('      test(X,y): ',(X_test.shape,y_test.shape))

    labels_list = ['Speed limit (20km/h)','Speed limit (30km/h)','Speed limit (50km/h)','Speed limit (60km/h)',
                'Speed limit (70km/h)','Speed limit (80km/h)','Speed limit (100km/h)','Speed limit (120km/h)',
                'Yield','Stop','No entry','Road work','Pedestrians','Turn right ahead','Turn left ahead',
                'Ahead only']
                
    return X_train,y_train,X_test,y_test,labels_list



def plot_dataset(X_train,y_train):

    # plot example images
    idx = [15,2500,9999]
    fig,axes = plt.subplots(3)
    axes[0].imshow(X_train[idx[0]])
    axes[1].imshow(X_train[idx[1]])
    axes[2].imshow(X_train[idx[2]])
    plt.show()

    print('Labels: ',y_train[idx[0]],y_train[idx[1]],y_train[idx[2]])


def resize_imgs(X):
    X_out = []
    for img in X:
        img = cv2.resize(img,(32,32))
        X_out.append(img)
    X_out = np.array(X_out)
    return X_out

def preprocess_dataset(X_train,y_train,X_test,y_test):
    # convert labels to categorical
    n_labels = len(np.unique(y_train))
    y_train = to_categorical(y_train,n_labels)
    y_test = to_categorical(y_test,n_labels)
    # resize images to 32 x 32 x 3
    X_train = resize_imgs(X_train)
    X_test = resize_imgs(X_test)
    # convert pixel values to floats
    X_train = X_train.astype('float32')
    X_test = X_test.astype('float32')
    # normalize pixel values
    X_train = X_train / 255.0
    X_test = X_test / 255.0

    return X_train,y_train,X_test,y_test