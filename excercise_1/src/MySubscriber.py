#!/usr/bin/env python

import traceback

import numpy
import rospy
from genpy import Message
from keras import Model
from sensor_msgs.msg import CompressedImage
from typing import TypeVar, Type, Generic, Any

from Libs.PythonLibs.Callback import Callback

T = TypeVar('T', Message, Message)
PUBLISH_RATE = 3  # hz


class RosSubscriber(Generic[T]):
    def __init__(self, topic, datatype):
        # type: (str, Type[T]) -> None
        super(RosSubscriber, self).__init__()
        self.topic = topic
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

    def handle(self, message):
        # type: (CompressedImage) -> None
        super(PredictionCISubscriber, self).handle(message)
        prediction = self.model.predict(self.unpackMessage(message))

        if self.prediction_callback.callable(prediction):
            self.prediction_callback.call(prediction)

    def unpackMessage(self, message):
        # type: (CompressedImage) -> numpy.array
        raise NotImplementedError


class RosSubscriberApp(object):
    def __init__(self):
        print "__init__", self.__class__.__name__
        self.subscriber = CompressedImageSubscriber('/camera/output/specific/compressed_img_msgs')

        pass

    def run(self):
        print "run", self.__class__.__name__
        rate = rospy.Rate(PUBLISH_RATE)

        while not rospy.is_shutdown():
            rate.sleep()
        pass


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
