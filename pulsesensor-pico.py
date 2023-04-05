import machine
import utime
import uos
from machine import Pin

led = Pin(25, Pin.OUT)
pulse_sensor = machine.ADC(28)
# Set up sampling frequency
sampling_frequency = 125 # Hz
sampling_period = 1 / sampling_frequency * 1000 # ms
print(sampling_period)
while True:
    value = pulse_sensor.read_u16()
    print(str(value))
# Wait for the next sample
    utime.sleep_ms(int(sampling_period))

