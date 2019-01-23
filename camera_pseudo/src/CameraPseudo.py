#!/usr/bin/env python
import cPickle
import socket
import struct
from Queue import Queue
from threading import Thread

import numpy
import rospy
from cv_bridge import CvBridge
from keras.datasets import mnist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Int32
from typing import Tuple

SPECIFIC_VALUE = 6  # value can be changed for different test-cases. This is not the labeled value but an index
PUBLISH_RATE = 3  # hz
USE_WEBCAM = True


class RemoteWebcamReader(object):
    def __init__(self):
        self.server_sock = socket.socket()
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind(('', 42069))
        self.server_sock.listen(5)

        self._isopened = False
        self.image_queue = Queue(1000)
        self.int_struct = struct.Struct('!Q')

    def start(self):
        server_thread = Thread(target=self.server_listener)
        server_thread.daemon = True
        server_thread.start()
        self._isopened = True

    def server_listener(self):
        while True:
            client_sock, client_addr = self.server_sock.accept()
            print client_addr, "connected on", self.server_sock.getsockname()
            client_thread = Thread(target=self.client_handler, args=(client_sock, client_addr))
            client_thread.daemon = True
            client_thread.start()

    def client_handler(self, client_sock, addr):
        # type: (socket.socket, Tuple[str, int]) -> None
        while True:
            try:
                pickle_len = self.int_struct.unpack(self._sock_recv_helper(client_sock, 8))[0]
                pickle_data = self._sock_recv_helper(client_sock, pickle_len)
            except socket.error as e:
                print "connection from", addr, "broke", repr(e)
                break
            print "client sent image with size", len(pickle_data)
            frame = cPickle.loads(pickle_data)  # type:
            self.image_queue.put(frame, block=False)

    def isOpened(self):
        return self._isopened

    def read(self):
        return True, self.image_queue.get()

    @classmethod
    def _sock_recv_helper(cls, sock, buffersize):
        # type: (socket.socket, int) -> str
        # Helper function to recv 'size' bytes or return None if EOF is hit
        # bypasses the problem, where a .sendall() -> read() would not transmit / receive all bytes.
        # ensures, that the given buffersize is received
        if buffersize == 0:
            return ''
        data = ''
        datalength = 0
        while datalength < buffersize:
            try:
                packet = sock.recv(buffersize - datalength)
                # print "received packet", len(packet) if packet else 0
            except (MemoryError, OverflowError):
                print 'buffersize', buffersize
                raise
            if not packet:
                raise socket.error("peer closed Connection without sending all expected data (expected: " + str(
                    buffersize) + ", received: " + str(len(data)) + ")")
            data += packet
            datalength = len(data)
        return data


class CameraPseudo:
    def __init__(self):
        # converts between ROS Image messages and OpenCV images.
        self.cv_bridge = CvBridge()

        # create a handle called publisher_webcam_comprs
        # to publish messages to a topic called /camera/output/webcam/compressed_img_msgs
        self.publisher_webcam_comprs = rospy.Publisher("/camera/output/webcam/compressed_img_msgs",
                                                       CompressedImage,
                                                       queue_size=1)
        # code to execute if webcam is connecected and
        # publishing images. Not the case yet
        if USE_WEBCAM:
            self.input_stream = RemoteWebcamReader()
            self.input_stream.start()
            if not self.input_stream.isOpened():
                raise Exception('Camera stream did not open\n')

        # publish specific: create another handle this time
        # for topic "specific". This topic will be used to send specific images
        # and will also be tweeked to publish random images
        self.publisher_specific_comprs = rospy.Publisher("/camera/output/specific/compressed_img_msgs",
                                                         CompressedImage,
                                                         queue_size=1)
        # publish specific: create another handle this time to evaluate the predictions made by the Prediction topic
        self.publisher_specific_check = rospy.Publisher("/camera/output/specific/check",
                                                        Bool,
                                                        queue_size=1)

        # # subscriber specific: receive prediction made by Prediction node as an integer. Send this
        # # to callback (camera_specific_callback) to evaluate the correctness of the prediction made
        # rospy.Subscriber('/camera/input/specific/number',
        #                  Int32,
        #                  self.camera_specific_callback)

        # publisher random: not relevant for our solution
        self.publisher_random_comprs = rospy.Publisher("/camera/output/random/compressed_img_msgs",
                                                       CompressedImage,
                                                       queue_size=1)
        # not relevant for our solution
        self.publisher_random_number = rospy.Publisher("/camera/output/random/number",
                                                       Int32,
                                                       queue_size=1)

        # use mnist data as pseudo webcam images
        (_, _), (self.images, self.labels) = mnist.load_data()

        rospy.loginfo("Publishing data...")

    def camera_specific_callback(self, msg):
        """check if input message is same as value in SPECIFIC_VALUE index"""
        result = True if msg.data == self.labels[SPECIFIC_VALUE] else False
        print "msg.data", msg.data
        print "self.labels[SPECIFIC_VALUE=" + str(SPECIFIC_VALUE) + "]", self.labels[SPECIFIC_VALUE]

        # publish result
        self.publisher_specific_check.publish(result)

    def publish_data(self, verbose=0):
        """set rate for publishing data and what to publish """
        rate = rospy.Rate(PUBLISH_RATE)

        while not rospy.is_shutdown():
            self.publish_webcam(verbose)

            rate.sleep()

    def publish_specific(self, verbose=0):
        """get an image from SPECIFIC_VALUE convert to image message and publish
        it according to publisher_specific_comprs.publish"""
        image = self.images[SPECIFIC_VALUE]

        # convert to msg
        compressed_imgmsg = self.cv_bridge.cv2_to_compressed_imgmsg(image)

        # publish data
        self.publisher_specific_comprs.publish(compressed_imgmsg)

        if verbose:
            rospy.loginfo(compressed_imgmsg.header.seq)
            rospy.loginfo(compressed_imgmsg.format)

    def publish_random(self, verbose=0):
        """ publish a randomly selected image """
        # get random number
        rand_int = numpy.random.randint(0, len(self.labels), dtype='int')

        # get image and number based on random value
        image = self.images[rand_int]
        number = self.labels[rand_int]

        # convert to msg
        compressed_imgmsg = self.cv_bridge.cv2_to_compressed_imgmsg(image)

        # publish data
        self.publisher_random_comprs.publish(compressed_imgmsg)
        self.publisher_random_number.publish(number)

        if verbose:
            rospy.loginfo(compressed_imgmsg.header.seq)
            rospy.loginfo(compressed_imgmsg.format)
            rospy.loginfo(number)

    def publish_webcam(self, verbose=0):
        """publish webcam output """
        if self.input_stream.isOpened():
            success, frame = self.input_stream.read()
            msg_frame = self.cv_bridge.cv2_to_compressed_imgmsg(frame)
            self.publisher_webcam_comprs.publish(msg_frame.header, msg_frame.format, msg_frame.data)

            if verbose:
                rospy.loginfo(msg_frame.header.seq)
                rospy.loginfo(msg_frame.format)


def main():
    verbose = 0  # use 1 for debug

    try:
        # register node
        rospy.init_node('camera_pseudo', anonymous=False)

        # init CameraPseudo
        cam = CameraPseudo()

        # start publishing data
        cam.publish_data(verbose)

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
