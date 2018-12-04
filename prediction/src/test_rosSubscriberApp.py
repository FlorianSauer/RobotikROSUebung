# coding=utf-8
import os
from unittest import TestCase

import keras
import numpy
import numpy as np
from keras import backend as k
from keras.datasets import mnist

from excercise_1.src.MySubscriber import RosPredictionApp


# teste die loadMakeModel darauf, ob sie aus eine Stringeingabe für einen Pfad eine Prediction abliefern kann
class TestRosSubscriberApp(TestCase):
    def getPath(self):
        return os.path.abspath(os.path.join(os.path.dirname(__file__), 'testResources/weights-best.hdf5'))

    def getRandomOrigMnistImage(self, seed=None):
        (_, _), (x_test, y_test) = mnist.load_data()
        if seed:
            rand_int = seed
        else:
            rand_int = numpy.random.randint(0, len(y_test), dtype='int')
        return x_test[rand_int]

    def getRandomReformedMnistImage(self, seed=None):
        # the data, split between train and test sets
        img_rows, img_cols = 28, 28

        (_, _), (x_test, y_test) = mnist.load_data()
        if seed:
            rand_int = seed
        else:
            rand_int = numpy.random.randint(0, len(y_test), dtype='int')

        if k.image_data_format() == 'channels_first':
            x_test = x_test.reshape(x_test.shape[0], 1, img_rows, img_cols)
            input_shape = (1, img_rows, img_cols)
        else:
            x_test = x_test.reshape(x_test.shape[0], img_rows, img_cols, 1)
            input_shape = (img_rows, img_cols, 1)

        x_test = x_test.astype('float32')
        x_test /= 255.
        return x_test[rand_int]

    def test_reshapeImage(self):
        seed = 6
        orig_mnist = self.getRandomOrigMnistImage(seed)
        reshaped_mnist = self.getRandomReformedMnistImage(seed)
        orig_mnist = orig_mnist.astype('float32')
        orig_mnist /= 255.
        orig_mnist = orig_mnist.reshape(28, 28, 1)
        print repr(orig_mnist)
        print '#'*25
        print repr(reshaped_mnist)
        print reshaped_mnist == orig_mnist
        print reshaped_mnist.shape
        print orig_mnist.shape
        # self.assertEquals(orig_mnist, reshaped_mnist)

    # teste ob das Model erfolgereiche Vorhersagen trifft
    def test_loadMakeModel(self):

        # params
        batch_size = 128
        num_classes = 10
        epochs = 12
        verbose_train = 1
        verbose_eval = 0

        # input image dimensions
        img_rows, img_cols = 28, 28

        # the data, split between train and test sets
        (x_train, y_train), (x_test, y_test) = mnist.load_data()

        if k.image_data_format() == 'channels_first':
            x_train = x_train.reshape(x_train.shape[0], 1, img_rows, img_cols)
            x_test = x_test.reshape(x_test.shape[0], 1, img_rows, img_cols)
            input_shape = (1, img_rows, img_cols)
        else:
            x_train = x_train.reshape(x_train.shape[0], img_rows, img_cols, 1)
            x_test = x_test.reshape(x_test.shape[0], img_rows, img_cols, 1)
            input_shape = (img_rows, img_cols, 1)

        x_train = x_train.astype('float32')
        x_test = x_test.astype('float32')
        x_train /= 255.
        x_test /= 255.
        print('x_train shape:', x_train.shape)
        print(x_train.shape[0], 'train samples')
        print(x_test.shape[0], 'test samples')

        # convert class vectors to binary class matrices
        y_train = keras.utils.to_categorical(y_train, num_classes)
        y_test = keras.utils.to_categorical(y_test, num_classes)

        # data to choose

        model = RosPredictionApp.loadMakeModel(self.getPath())
        for index in xrange(100):
            # expand dimension for batch
            input_data = np.expand_dims(x_test[index], axis=0)  # tensorflow
            input_label = y_test[index]

            # example prediction call
            prediction = model.predict(input_data)

            # revert from one-hot encoding
            prediction = np.argmax(prediction, axis=None, out=None)
            input_label = np.argmax(input_label, axis=None, out=None)
            self.assertEquals(input_label, prediction)

#
