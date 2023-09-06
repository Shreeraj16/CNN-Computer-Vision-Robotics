import matplotlib.pyplot as plt


def calc_accuracy(model,X,y):

    _, accuracy = model.evaluate(X,y)
    print('Accuracy value: %f ' % (accuracy*100) )


def plot_history(h):

    fig = plt.subplots(1)
    plt.plot(h.history['accuracy'], label='train accuracy')
    plt.plot(h.history['val_accuracy'], label='test accuracy')
    plt.title('Classifier CNN evaluation')
    plt.legend()
    plt.show()