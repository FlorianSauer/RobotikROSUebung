import cPickle
import socket
import struct
import time

import cv2


class WindowsWebcamTransmitterApp(object):
    def __init__(self):
        self.input_stream = cv2.VideoCapture(0)
        if not self.input_stream.isOpened():
            raise Exception('inputstream not opened')
        self.client_sock = socket.socket()
        self.client_sock.connect(('127.0.0.1', 42069))
        self.int_struct = struct.Struct('!Q')

        pass

    def run(self):
        while True:
            time.sleep(1.0 / 60.0)  # 60 fps?
            success, frame = self.input_stream.read()
            if not success:
                print "unsuccessful read operation"
                break
            frame_pickled = cPickle.dumps(frame)
            frame_pickled_len = len(frame_pickled)
            send_buf = bytearray()
            send_buf.extend(self.int_struct.pack(frame_pickled_len))
            send_buf.extend(frame_pickled)
            self.client_sock.sendall(send_buf)
            print "sent", len(send_buf), "bytes"
        pass


if __name__ == "__main__":
    app = WindowsWebcamTransmitterApp()
    app.run()
