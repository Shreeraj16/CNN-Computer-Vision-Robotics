import numpy as np
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv2D, MaxPooling2D, Flatten, Dense
from tensorflow.keras.layers import Dropout
from tensorflow.keras.regularizers import l2

def build_model(n_labels):

    # initialize model
    model = Sequential()

    # feature detector
    model.add(Conv2D(32, (3, 3), activation='relu',  padding='same', input_shape=(32, 32, 3)))
    model.add(MaxPooling2D((2, 2)))

    model.add(Conv2D(64, (3, 3), activation='relu', padding='same'))
    model.add(Conv2D(64, (3, 3), activation='relu', padding='same'))
    model.add(MaxPooling2D((2, 2)))

    model.add(Conv2D(128, (3, 3), activation='relu', padding='same'))
    model.add(Conv2D(128, (3, 3), activation='relu', padding='same'))
    model.add(MaxPooling2D((2, 2)))

    # classifier
    model.add(Flatten())
    model.add(Dense(128, activation='relu'))
    model.add(Dropout(0.2))

    model.add(Dense(128, activation='relu'))
    model.add(Dropout(0.2))

    model.add(Dense(n_labels, activation='softmax'))

    # compiler
    model.compile(optimizer='adam',
             loss='categorical_crossentropy',
             metrics=['accuracy'])

    return model


def train_model(model,X_train,y_train,X_test,y_test,n_epochs,batch_size):

    h = model.fit(X_train,y_train, epochs=n_epochs, batch_size=batch_size, validation_data=(X_test,y_test))#, verbose=0)

    return h


def save_model(model,path,name):
    model.save(path+name)