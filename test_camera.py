# Stream video from capture card
import cv2
import numpy as np
import time


def stream_video():
    # USB 002 003
    
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    # set resolution and fps
    w, h = 1920, 1080
    fps = 60
    buf_size = 1
    bit_rate = 3000*1000
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    # set fps
    cap.set(cv2.CAP_PROP_BUFFERSIZE, buf_size)
    cap.set(cv2.CAP_PROP_FPS, fps)
    while True:
        ret = cap.grab()
        assert ret, ret
        ret, frame = cap.read()
        #ret, frame = cap.retrieve(frame)
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
stream_video()