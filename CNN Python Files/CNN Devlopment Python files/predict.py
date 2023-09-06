from tensorflow.keras.models import load_model
from dataset import load_dataset
import matplotlib.pyplot as plt
import cv2

def predict_imgs(path,model_name,X_test, labels_list):
    # Load model
    model = load_model(path+model_name)

    # Load image
    img = cv2.resize(X_test[0], (32,32))
    img_to_pred = img.reshape(1,32,32,3)

    # Plot image
    plt.imshow(img)
    plt.show()

    # Predict
    y_pred = model.predict_classes(img_to_pred)
    label_pred = labels_list[y_pred[0]]

    return label_pred

path = "/catkin_ws/src/dlrepo/traffic_sign_data_set/"
X_train,y_train,X_test,y_test,labels_list = load_dataset(path)
model_name = "cnn_traffic_sign.h5"

print(predict_imgs(path,model_name,X_test, labels_list))