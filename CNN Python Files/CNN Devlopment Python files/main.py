import os

from dataset import load_dataset, plot_dataset, preprocess_dataset
from model import build_model, train_model, save_model
from evaluate import calc_accuracy, plot_history


# Load TRAFFIC SIGN dataset
path = "/home/user/catkin_ws/src/dlrepo/traffic_sign_data_set/"
X_train,y_train,X_test,y_test,labels_list = load_dataset(path)

# Plot images from dataset
plot_dataset(X_train,y_train)

# Preprocess dataset to enter NN
X_train,y_train,X_test,y_test = preprocess_dataset(X_train,y_train,X_test,y_test)

# Build CNN model as a classifier
model = build_model(len(labels_list))

# Train the model
n_epochs = 2
batch_size = 32
h = train_model(model,X_train,y_train,X_test,y_test,n_epochs,batch_size)

# Evaluate accuracy obtained by model
calc_accuracy(model,X_test,y_test)

# Plot history from training step
plot_history(h)

# Save the model
save_model(model, path, "cnn_traffic_sign.h5")