#!/usr/bin/env python

import traceback

import keras
import numpy
import rospy
from cv_bridge import CvBridge
from genpy import Message
from keras import Model, Sequential
from keras.layers import Conv2D, MaxPooling2D, Dropout, Flatten, Dense
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32
from typing import TypeVar, Type, Generic, Any
from wrapt import synchronized

from Libs.PythonLibs.Callback import Callback

T = TypeVar('T', Message, Message)
PUBLISH_RATE = 3  # hz


class RosSubscriber(Generic[T]):
    def __init__(self, topic, datatype):
        # type: (str, Type[T]) -> None
        super(RosSubscriber, self).__init__()
        self.topic = topic  # type: str
        self.datatype = datatype  # type: Type[T]
        self._subscriber = rospy.Subscriber(self.topic, self.datatype, self.handle)

    def handle(self, message):
        # type: (T) -> None
        raise NotImplementedError


class CompressedImageSubscriber(RosSubscriber):

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
        super(PredictionCISubscriber, self).handle(message)
        prediction = self.model.predict(self.unpackMessage(message))

        # Todo: hotencoded -> real class number

        if self.prediction_callback.callable(prediction):
            self.prediction_callback.call(prediction)
        else:
            print "YOUR CALLBACK FAILED, maybe you should rework this ;)"

    # noinspection PyUnresolvedReferences
    def unpackMessage(self, message):
        # type: (CompressedImage) -> numpy.array
        self.unpackMessageStatic(self.cv_bridge, message)

    # noinspection PyUnresolvedReferences
    @staticmethod
    def unpackMessageStatic(bridge, message):
        # type: (CvBridge, CompressedImage) -> numpy.array
        return bridge.compressed_imgmsg_to_cv2(message)


#wrapper fuer rospy subscriber
class RosSubscriberApp(object):
    modelpath = 'todo'

    def __init__(self):
        print "__init__", self.__class__.__name__
        self.prediction_publisher = rospy.Publisher('/camera/input/specific/number',
                                                    Int32,
                                                    queue_size=1)  # publish given data to topic
        # -> wrap .publish() in Callback
        self.prediction_publish_callback = Callback(lambda i: self.prediction_publisher.publish(i), single=True)
        #wir melden den Subscriber am Rosnode an bzw. an der compressed image Topic

        # -> pass Callback to self.subscriber
        self.subscriber = PredictionCISubscriber('/camera/output/specific/compressed_img_msgs',
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
        return model


if __name__ == "__main__":
    verbose = 0  # use 1 for debug

    try:
        # register node
        rospy.init_node('mysubscriber', anonymous=False)

        app = RosSubscriberApp()
        app.run()

    except rospy.ROSInterruptException as e:
        print "MySubscriber crashed", repr(e)
        traceback.print_exc()
