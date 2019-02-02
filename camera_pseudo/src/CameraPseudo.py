#!/usr/bin/env python
import cPickle
import socket
import struct
from Queue import Queue
from threading import Thread

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from typing import Tuple

PUBLISH_RATE = 3  # hz
USE_WEBCAM = True


class RemoteWebcamReader(object):
    """
    Object to replace/mimic the cv2 VideoCapture object.

    Opens a tcp server on port 42069. A corresponding WebcamTransmitter can connect to the server and transmit image
    data (data is packed with Pickle)
    """
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
        self.publisher_webcam_comprs = rospy.Publisher("/image",
                                                       CompressedImage,
                                                       queue_size=1)

        self.input_stream = RemoteWebcamReader()
        self.input_stream.start()
        if not self.input_stream.isOpened():
            raise Exception('Camera stream did not open\n')

        rospy.loginfo("Publishing data...")

    def publish_data(self, verbose=0):
        """set rate for publishing data and what to publish """
        rate = rospy.Rate(PUBLISH_RATE)

        while not rospy.is_shutdown():
            self.publish_webcam(verbose)

            rate.sleep()

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
