# import serial
# import numpy as np
# import heartpy as hp
# import matplotlib.pyplot as plt
# import time
# import math

# # Windows
# # ser = serial.Serial('COM3', 115200, timeout=1)
# # Linux
# ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

# # Define window size and maximum amount
# sample_size = 100  # Number of data points to read per window
# window_size = 2000  # Maximum number of data points to store
# sample_rate =  125
# BPM_change_threshold = 20
# last_BPM = 0

# # Initialize an empty numpy array to store the data
# data = np.zeros(window_size)

# # Define a function to read data from the source
# def read_data(amount):
#     samples = np.zeros(amount)
#     for i in range(amount):
#         response = ser.readline().decode('utf-8').rstrip()
#         response = int(response)
#         samples[i] = response
#     # samples = hp.scale_data(samples, lower=1, upper=1024)

#     return samples

# def init():
#     read_data(sample_size*2)


# # Loop infinitely
# i = 0

# init()
# while True:
#     # Read the next window_size values from the pulse sensor
#     start_time = time.time()
#     new_data = read_data(sample_size)

#     print("--- %s seconds ---" % (time.time() - start_time))
#     # print(new_data)

#     data = np.concatenate((data, new_data), axis=None)
#     if data.size > window_size:
#         data = data[sample_size:]
    
#     data = hp.smooth_signal(data, sample_rate = 125)

#     # Plot raw data
#     plt.figure(figsize=(12,4))
#     plot_object = plt.plot(data)
#     plt.savefig('raw_plot'+ str(i) + '.jpg')
#     #run the analysis
#     if np.count_nonzero(data) == window_size:

#         #Analyze data using HeartPy
#         wd, m = hp.process(data, sample_rate = sample_rate, interp_threshold=65500)
    
#         current_BPM = m['bpm']
#         if last_BPM == 0:
#             last_BPM = current_BPM

#         # Plot analyzed data
#         plot_object = hp.plotter(wd, m, show=False)
#         plot_object.savefig('plot_analyzed'+ str(i) + '.jpg')
#         i = i + 1 

#         # display measures computed
#         if abs(current_BPM-last_BPM) > BPM_change_threshold or math.isnan(current_BPM):
#             print('current heartbeat(last): ', last_BPM)
#         else: 
#             print('current heartbeat: ', current_BPM)




import serial
import numpy as np
import heartpy as hp
import matplotlib.pyplot as plt
import time
import math
import threading

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

# Create and start the thread
pulse_sensor_thread = PulseSensorThread()
pulse_sensor_thread.start()
