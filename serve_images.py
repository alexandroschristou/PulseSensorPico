import socket
import threading
import pickle
import struct
import time
import serial
import evdev

import cv2
import numpy as np
import heartpy as hp
import math


# FILES NEEDED
#
# /home/pi/mouse -> symlink do /dev/input/by-id/XXX, with XXX the file that represents an USB joystick
# /home/pi/joystick -> symlink to /dev/ttyUSB0 or /dev/ttyACM0, the file that appears when the Arduino is connected to the RaspberryPi

WIDTH = 320
HEIGHT = 240
BAUDRATE = 9600

def send(s, data):
    data = pickle.dumps(data)
    s.sendall(struct.pack('>i', len(data)))
    s.sendall(data)

def recv(s):
    data = s.recv(4, socket.MSG_WAITALL)
    data_len = struct.unpack('>i', data)[0]
    data = s.recv(data_len, socket.MSG_WAITALL)
    return pickle.loads(data)

class WebcamThread(threading.Thread):
    """ Class that reads images from the webcam
    """
    def __init__(self, filename):
        super().__init__()

        self._lock = threading.Lock()
        self._webcam = cv2.VideoCapture(filename, cv2.CAP_V4L2)
        self._webcam.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        self._webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        self._frame = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

    def run(self):
        while True:
            ret, f = self._webcam.read()

            if ret:
                with self._lock:
                    self._frame = f

    def frame(self):
        with self._lock:
            return self._frame

class MouseThread(threading.Thread):
    """ Thread that reads instructions from an evdev device (a joystick in this case)
    """
    def __init__(self, path):
        super().__init__()

        # Initial state
        self._x = 0.5
        self._y = 0.5
        self._overcome = 1.0
        self._button = 0
        self._lock = threading.Lock()
        self._device = None

        # Initialize device, gently fail if not available
        try:
            self._device = evdev.InputDevice(path)

            # Get max X and max Y for the joystick
            for code, absinfo in self._device.capabilities()[3]:
                if code == 0:
                    self._max_x = absinfo.max
                elif code == 1:
                    self._max_y = absinfo.max
                elif code == 6:
                    self._max_overcome = absinfo.max
        except Exception as e:
            print('Could not initialize JOYSTICK, not using it')
            print(e)

    def run(self):
        if self._device is None:
            return

        for event in self._device.read_loop():
            with self._lock:
                if event.code == 0 and event.type == 3:
                    # X position
                    self._x = event.value / self._max_x
                elif event.code == 1 and event.type == 3:
                    # Y position
                    self._y = event.value / self._max_y
                elif event.code == 6 and event.type == 3:
                    # Configuration wheel
                    self._overcome = event.value / self._max_overcome
                elif event.code == 288 and event.value == 1:
                    # Buttons
                    self._button = 1
                elif event.code == 289 and event.value == 1:
                    self._button = 2
                elif event.code == 290 and event.value == 1:
                    self._button = 3
                elif event.code == 291 and event.value == 1:
                    self._button = 4
                elif event.value == 0 and event.code in [288, 289, 290, 291]:
                    # A button has been released
                    self._button = 0

    def get_observations(self):
        with self._lock:
            return self._x, self._y, self._overcome, self._button

class ControlThread(threading.Thread):
    def __init__(self, filename, mouse):
        super().__init__()

        self._lock = threading.Lock()
        self._action = 'S'
        self._joystick = serial.Serial(filename, baudrate=BAUDRATE, timeout=0.1)
        self._mouse = mouse

        # Give some time for the USB CDC layer to initialize
        time.sleep(2.0)

    def run(self):
        while True:
            with self._lock:
                action_char = self._action

            # If the joystick is set in overcome, use it to produce X-Y coordinates
            mx, my, movercome, button = self._mouse.get_observations()

            if movercome > 0.5:
                if my < 0.3:
                    action_char = 'F'
                elif mx < 0.4:
                    action_char = 'L'
                elif mx > 0.6:
                    action_char = 'R'
                else:
                    action_char = 'S'

            # Send the action to the joystick
            self._joystick.write(bytes(action_char, 'ascii') + b'\n')
            self._joystick.flush()
            self._joystick.readline()

    def set_action(self, action_char):
        with self._lock:
            self._action = action_char

class PulseSensorThread(threading.Thread):
    def __init__(self):
        super().__init__()
        # # Windows
        # self.ser = serial.Serial('COM3', 115200, timeout=1)
        # Linux
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        # # Define window size and maximum amount
        self.sample_size = 100 # Number of data points to read per window
        self.window_size = 2000 # Maximum number of data points to store
        self.sample_rate = 125
        #Threshold for BPM change
        self._lock = threading.Lock()
        self.BPM_change_threshold = 20
        self.last_BPM = 0
        self.data = np.zeros(self.window_size)
        self.i = 0

    def run(self):
        self.init()
        while True:
            start_time = time.time()

            # Read the next window_size values from the pulse sensor
            new_data = self.read_data(self.sample_size)
            print("--- %s seconds ---" % (time.time() - start_time))
            self.data = np.concatenate((self.data, new_data), axis=None)

            if self.data.size > self.window_size:
                self.data = self.data[self.sample_size:]
            #print(self.data)
            self.data = hp.smooth_signal(self.data, sample_rate=self.sample_rate)

            # Plot raw data
            # plt.figure(figsize=(12,4))
            #plot_object = plt.plot(self.data)
            #plt.savefig('raw_plot'+ str(self.i) + '.jpg')

            #run the analysis
            if np.count_nonzero(self.data) == self.window_size:
                #Analyze data using HeartPy
                wd, m = hp.process(self.data, sample_rate=self.sample_rate, interp_threshold=65500)
                current_BPM = m['bpm']
                if self.last_BPM == 0:
                    self.last_BPM = current_BPM
                # Plot analyzed data
                #plot_object = hp.plotter(wd, m, show=False)
                #plot_object.savefig('plot_analyzed'+ str(self.i) + '.jpg')
                self.i = self.i + 1
                # display measures computed
                if abs(current_BPM-self.last_BPM) > self.BPM_change_threshold or math.isnan(current_BPM):
                    print('current heartbeat(last): ', self.last_BPM)
                else:
                    print('current heartbeat: ', current_BPM)

                with self._lock:
                    self.last_BPM = current_BPM

    def read_data(self, amount):
        samples = np.zeros(amount)
        for i in range(amount):
            response = self.ser.readline().decode('utf-8').rstrip()
            response = int(response)
            samples[i] = response
        return samples

    def init(self):
        self.read_data(self.sample_size*2)

    def get_bpm(self):
        with self._lock:
            return self.last_BPM



class ClientThread(threading.Thread):
    """ Thread that sends frames to a client
    """
    def __init__(self, conn, mouse, webcam, control, bpm):
        super().__init__()

        # Send without delay
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        self._conn = conn
        self._mouse = mouse
        self._webcam = webcam
        self._control = control
        self.bpm = bpm

    def run(self):
        while True:
            frame = self._webcam.frame()
            mx, my, movercome, button = self._mouse.get_observations()
            current_bpm = self.bpm.get_bpm()
	    #code to get the heartbeat

            # Send image
            jpg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 95])[1]
            send(self._conn, (jpg, mx, my, button, current_bpm))

            # Receive discrete action as character (F, L, R, s)
            action_char = recv(self._conn)

            # Send the action to the joystick
            self._control.set_action(action_char)

if _name_ == '_main_':
    # Open the webcam
    webcam = WebcamThread('/home/pi/webcam_left')
    webcam.start()

    # Open the joystick (the USB one, for user input)
    mouse = MouseThread('/home/pi/mouse')
    mouse.start()

    # Control thread that talks to the serial joystick
    control = ControlThread('/home/pi/joystick', mouse)
    control.start()

    #Pulsesensor thread 
    pulse_sensor_thread = PulseSensorThread()
    pulse_sensor_thread.start()

    # Create server
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('0.0.0.0', 9395))
    s.listen(10)
    print('Now accepting connections')

    while True:
        # Accept a client
        conn, addr = s.accept()

        c = ClientThread(conn, mouse, webcam, control, pulse_sensor_thread)
        c.start()