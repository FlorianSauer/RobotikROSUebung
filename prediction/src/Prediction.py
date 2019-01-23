#!/usr/bin/env python
import os
import traceback

import cv2
import keras
import numpy
import rospy
from cv_bridge import CvBridge
from genpy import Message
from keras import Model, Sequential
from keras.layers import Conv2D, MaxPooling2D, Dropout, Flatten, Dense
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32, Bool
from typing import TypeVar, Type, Generic, Any
from wrapt import synchronized

from Libs.PythonLibs.Callback import Callback

# from SoundPlayer import SoundPlayer

T = TypeVar('T', Message, Message)
PUBLISH_RATE = 3  # hz


class RosSubscriber(Generic[T]):
    """
    Wrapper for rospy.Subscriber
    """

    def __init__(self, topic, datatype):
        # type: (str, Type[T]) -> None
        super(RosSubscriber, self).__init__()
        self.topic = topic  # type: str
        self.datatype = datatype  # type: Type[T]
        self._subscriber = rospy.Subscriber(self.topic, self.datatype, self.handle)

    def handle(self, message):
        # type: (T) -> None
        raise NotImplementedError


class MessagePrinterSubscriber(RosSubscriber):
    """print whether or not the prediction we made was correct"""

    def handle(self, message):
        # type: (Bool) -> None
        print "received Bool"
        print "data", message.data


class CompressedImageSubscriber(RosSubscriber):
    """receive compressed image and print image information"""

    def __init__(self, topic):
        # type: (str) -> None
        super(CompressedImageSubscriber, self).__init__(topic, CompressedImage)

    def handle(self, message):
        # type: (CompressedImage) -> None
        print "received CompressedImage"
        print "header", message.header
        print "format", message.format
        print "data", len(message.data), "bytes"


class PredictionCISubscriber(CompressedImageSubscriber):
    """class to predict what number was sent based on trained model"""

    def __init__(self, topic, model, prediction_callback):
        # type: (str, Model, Callback[int, Any]) -> None
        """
        :param topic:
        :param model: should be already trained model
        :param prediction_callback:
        """
        super(PredictionCISubscriber, self).__init__(topic)

        self.model = model
        self.prediction_callback = prediction_callback
        self.cv_bridge = CvBridge()

    @synchronized
    def handle(self, message):
        # type: (CompressedImage) -> None
        """make prediction based on received image message"""

        input_data = numpy.expand_dims(self.unpackMessage(message), axis=0)  # tensorflow
        prediction = self.model.predict(input_data)
        # noinspection PyUnresolvedReferences
        prediction = numpy.argmax(prediction, axis=None, out=None)

        # Todo: hotencoded -> real class number

        print "Predicted", prediction

        if self.prediction_callback.callable(prediction):
            self.prediction_callback.call(prediction)
        else:
            print "YOUR CALLBACK FAILED, maybe you should rework this ;)"

    # noinspection PyUnresolvedReferences
    def unpackMessage(self, message):
        # type: (CompressedImage) -> numpy.array
        """adjust the image message to necessary format for prediction"""
        return self.unpackMessageStatic(self.cv_bridge, message)

    # noinspection PyUnresolvedReferences
    @staticmethod
    def unpackMessageStatic(bridge, message):
        # type: (CvBridge, CompressedImage) -> numpy.array
        """adjust the image message to necessary format for prediction"""
        a = bridge.compressed_imgmsg_to_cv2(message)
        # print a.shape
        a = cv2.resize(a, (28, 28))
        a = a.reshape(28, 28, 1)
        a = a.astype('float32')
        a /= 255.
        return a


class RosPredictionApp(object):
    resource_path = os.path.abspath(os.path.join(os.path.dirname(__file__), 'Resources'))
    modelpath = os.path.join(resource_path, 'weights-best.hdf5')

    def __init__(self):
        print "__init__", self.__class__.__name__
        # self.sound_player = SoundPlayer(self.resource_path)
        # declare+register prediction result topic
        self.prediction_publisher = rospy.Publisher('/camera/input/specific/number',
                                                    Int32,
                                                    queue_size=1)  # publish given data as 32bit integer to topic
        # -> wrap .publish() in Callback
        # self.prediction_publish_callback = Callback(lambda i: self.sound_player.playSound(i), single=True)
        self.prediction_publish_callback = Callback(lambda i: self.prediction_publisher.publish(i), single=True)

        # -> pass Callback to self.subscriber
        # register subscriber @roscore node with given topic
        self.subscriber = PredictionCISubscriber('/camera/output/webcam/compressed_img_msgs',
                                                 self.loadMakeModel(self.modelpath),
                                                 self.prediction_publish_callback)
        pass

    def run(self):
        print "run", self.__class__.__name__
        rate = rospy.Rate(PUBLISH_RATE)

        while not rospy.is_shutdown():
            rate.sleep()
        pass

    @staticmethod
    def loadMakeModel(path):
        """" loads and initializes model according to mnist example"""
        num_classes = 10
        # input image dimensions
        img_rows, img_cols = 28, 28

        if keras.backend.image_data_format() == 'channels_first':
            input_shape = (1, img_rows, img_cols)
        else:
            input_shape = (img_rows, img_cols, 1)

        model = Sequential()
        model.add(Conv2D(filters=32,
                         kernel_size=(3, 3),
                         activation='relu',
                         input_shape=input_shape))
        model.add(Conv2D(filters=64,
                         kernel_size=(3, 3),
                         activation='relu'))
        model.add(MaxPooling2D(pool_size=(2, 2)))
        model.add(Dropout(rate=0.25))
        model.add(Flatten())
        model.add(Dense(units=128,
                        activation='relu'))
        model.add(Dropout(rate=0.5))
        model.add(Dense(units=num_classes,
                        activation='softmax'))

        model.compile(loss=keras.losses.categorical_crossentropy,
                      optimizer=keras.optimizers.Adadelta(),
                      metrics=['accuracy'])

        model.load_weights(filepath=path)
        # noinspection PyProtectedMember
        model._make_predict_function()
        return model


if __name__ == "__main__":
    verbose = 0  # use 1 for debug

    try:
        # register node
        rospy.init_node('prediction', anonymous=False)

        app = RosPredictionApp()
        app.run()

    except rospy.ROSInterruptException as e:
        print __file__, "crashed", repr(e)
        traceback.print_exc()
