import cv2
import numpy as np
import RPi.GPIO as GPIO
from threading import Thread
from queue import Queue
import time
from picamera.array import PiRGBArray
from picamera import PiCamera

class PiVideoStream:
    def __init__(self, resolution=(640, 480), framerate=32):
        # initialize the camera and stream
        self.camera = PiCamera()
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        self.rawCapture = PiRGBArray(self.camera, size=resolution)
        self.stream = self.camera.capture_continuous(self.rawCapture,
            format="bgr", use_video_port=True)
        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.frame = None
        self.stopped = False


    def start(self):
        # start the thread to read frames from the video stream
        Thread(target=self.update, args=()).start()
        return self
    

    def update(self):
        # keep looping infinitely until the thread is stopped
        for f in self.stream:
            # grab the frame from the stream and clear the stream in
            # preparation for the next frame
            self.frame = f.array
            self.rawCapture.truncate(0)
            # if the thread indicator variable is set, stop the thread
            # and resource camera resources
            if self.stopped:
                self.stream.close()
                self.rawCapture.close()
                self.camera.close()
                return

    def read(self):
        # return the frame most recently read
        return self.frame


    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True


print("[INFO] sampling THREADED frames from `picamera` module...")
vs = PiVideoStream(resolution=(640,480)).start()
time.sleep(2.0)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

GPIO.setup(7, GPIO.OUT)
pi_pwm = GPIO.PWM(7, 50)
pi_pwm.start(0)

def output_result(dot_count):
    global pi_pwm
    duty_cycles = range(0, 97, 8)
    try:
        pi_pwm.ChangeDutyCycle(duty_cycles[dot_count])
    except:
        pi_pwm.ChangeDutyCycle(0)

bdp_white = cv2.SimpleBlobDetector_Params()
''' UPDATE THESE PARAMETERS FOR YOUR WHITE DIE BLOB DETECTION '''
bdp_white.filterByColor = False

detector_white = cv2.SimpleBlobDetector_create(bdp_white)

# def white_dice(img):
#     ''' YOUR FILTERS GO HERE '''
#     bdp = cv2.SimpleBlobDetector_Params()
#     bdp.filterByArea = False
#     bdp.filterByConvexity = False
#     bdp.filterByCircularity = True
#     bdp.filterByInertia = False
#     bdp.filterByColor = True
#     bdp.blobColor = 255
#     bdp.minCircularity = 0.5
#     bdp.maxCircularity = 1
#     detector = cv2.SimpleBlobDetector_create(bdp)
#     points = detector.detect(mask)

#     return 0

bdp_color = cv2.SimpleBlobDetector_Params()
''' UPDATE THESE PARAMETERS FOR YOUR COLORED DICE BLOB DETECTION '''
bdp_color.filterByColor = False

detector_color = cv2.SimpleBlobDetector_create(bdp_color)

def colored_dice(img):
    ''' YOUR FILTERS GO HERE '''
    return 0

frame_count = 0
try:
    while True:
        result = vs.read()
        frame_count += 1
        img = cv2.rotate(result, cv2.ROTATE_180)

        ''' PART 1 '''
        new_dims = (int(img.shape[1] * 0.5), int(img.shape[0] * 0.5))
        downscale = cv2.resize(img, new_dims)

        new_img = cv2.cvtColor(downscale, cv2.COLOR_BGR2GRAY)

        # Gaussian Blur 
        blur = cv2.GaussianBlur(new_img,ksize=(11,11),sigmaX=0)

        # Lowering the block size worked 
        mask = cv2.adaptiveThreshold(blur, maxValue=255, adaptiveMethod=cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                    thresholdType=cv2.THRESH_BINARY_INV, blockSize=5, C=3)
        ''' GENERAL FILTERING '''

        res = cv2.erode(mask, np.ones((5,5), np.uint8), iterations=1)
        res = cv2.dilate(res, np.ones((3,3), np.uint8), iterations=2)

        img = res
        ''' WHITE DIE '''
        bdp = cv2.SimpleBlobDetector_Params()
        bdp.filterByArea = True
        bdp.filterByConvexity = False
        bdp.filterByCircularity = True
        bdp.filterByInertia = False
        bdp.filterByColor = True
        bdp.blobColor = 255
        bdp.minCircularity = 0.5
        bdp.maxCircularity = 1
        detector = cv2.SimpleBlobDetector_create(bdp)
        points = detector.detect(img)

        img_with_keypoints = cv2.drawKeypoints(img, points, np.array([]), (0, 0, 255), 
                    cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # shows the image with the keypoints highlighted in red 
        cv2.imshow("keypoints", img_with_keypoints)
        ''' COLORED DICE '''

        # outputs the number of points every 5 seconds
        if frame_count % 5 == 0:
            output_result(len(points))
            print(f"Frame {frame_count}: Detected dots = {len(points)}")

        k = cv2.waitKey(3)
        if k == ord('q'):
            # If you press 'q' in the OpenCV window, the program will stop running.
            break
        elif k == ord('p'):
            # If you press 'p', the camera feed will be paused until you press
            # <Enter> in the terminal.
            input()
except KeyboardInterrupt:
    pass

# Clean-up: stop running the camera and close any OpenCV windows
cv2.destroyAllWindows()
vs.stop()
