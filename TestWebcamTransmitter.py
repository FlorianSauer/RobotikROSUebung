import argparse
import cPickle
import socket
import struct
import sys
import time

import cv2


class WebcamTransmitterApp(object):
    argparser = argparse.ArgumentParser()
    argparser.add_argument('--ip', help="The ip of the ROS-Core", type=str, default='127.0.0.1',
                           dest='ip')  # Todo: better naming
    argparser.add_argument('--port', help="The port", default=42069, type=int, dest='port')
    namespace = argparser.parse_args(sys.argv[1:])

    def __init__(self):
        self.input_stream = cv2.VideoCapture(0)
        if not self.input_stream.isOpened():
            raise Exception('inputstream not opened')
        self.client_sock = socket.socket()
        try:
            self.client_sock.connect((self.namespace.ip, self.namespace.port))
        except socket.error as e:
            print "unable to open socket to target", repr(self.namespace.ip)
            print "Socket Error", "[" + str(e.errno) + "] " + e.strerror
            self.input_stream.release()
            self.client_sock.close()
            cv2.destroyAllWindows()
            exit(1)
        self.int_struct = struct.Struct('!Q')

    def run(self):
        try:
            while True:
                raw_input("press something")
                time.sleep(2.0)  # 60 fps?
                success, frame = self.input_stream.read()
                if not success:
                    print "Camera: unsuccessful read operation"
                    break
                frame = cv2.resize(frame, (100, 100))
                # frame = frame[:][0]
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                frame_pickled = cPickle.dumps(frame)
                frame_pickled_len = len(frame_pickled)
                send_buf = bytearray()
                send_buf.extend(self.int_struct.pack(frame_pickled_len))
                send_buf.extend(frame_pickled)
                self.client_sock.sendall(send_buf)
                print "sent", len(send_buf), "bytes"
        except socket.error as e:
            print "Unable to send image data to Target"
            print "Socket Error", "[" + str(e.errno) + "] " + e.strerror
        finally:
            self.input_stream.release()
            self.client_sock.close()
        pass


if __name__ == "__main__":
    app = WebcamTransmitterApp()
    app.run()
