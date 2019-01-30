#!/usr/bin/env python
import os
import traceback

import rospy
# import rosgraph
from genpy import Message
from std_msgs.msg import Int32, Bool
from typing import TypeVar, Type, Generic

from SoundPlayer import SoundPlayer

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


class SoundPlayerSubscriber(RosSubscriber):

    def __init__(self, topic, soundpath):
        super(SoundPlayerSubscriber, self).__init__(topic, Int32)
        self.soundplayer = SoundPlayer(soundpath)

    def handle(self, message):
        # type: (Int32) -> None
        print "play", message.data
        self.soundplayer.playSound(message.data)


class RosSoundApp(object):
    resource_path = os.path.abspath(os.path.join(os.path.dirname(__file__), 'Resources'))

    def __init__(self):
        print "__init__", self.__class__.__name__
        self.subscriber = SoundPlayerSubscriber('number', self.resource_path)

        pass

    def run(self):
        print "run", self.__class__.__name__
        rate = rospy.Rate(PUBLISH_RATE)

        while not rospy.is_shutdown():
            rate.sleep()
            # print "Master online:", rosgraph.is_master_online()
        pass


if __name__ == "__main__":
    verbose = 0  # use 1 for debug

    try:
        # register node
        rospy.init_node('sound', anonymous=False)

        app = RosSoundApp()
        app.run()

    except rospy.ROSInterruptException as e:
        print __file__, "crashed", repr(e)
        traceback.print_exc()
