import serial, struct, math
import numpy as np
import pyqtgraph as pg 
from PyQt5.QtWidgets import QApplication

# Serial port contiguration.
serialPort = "/dev/tty.usbmodem2081347756501"
serialBaud = 115200
ser = serial.Serial(serialPort, serialBaud)
ser.reset_input_buffer()

app = QApplication([])
win = pg.GraphicsLayoutWidget(show=True)
win.setWindowTitle("Realtime Accelerometer Analysis")

time_plot = win.addPlot(title = "Time Domain Acceleration")
x_curve = time_plot.plot(pen='r')
y_curve = time_plot.plot(pen='b')
z_curve = time_plot.plot(pen='g')
accel_x = []
accel_y = []
accel_z = []

# x_plot = win.addPlot(title='X Time Domain')
# x_curve = x_plot.plot(pen='r')
# accel_x = []

# y_plot = win.addPlot(title="Y Time Domain Acceleration")
# y_curve = y_plot.plot(pen='b')
# accel_y = []

# z_plot = win.addPlot(title="Z Time Domain")
# z_curve = z_plot.plot(pen='w')
# accel_z = []

def update():
    global accel_x, accel_y, accel_z
    accel_data = ser.read(12)
    fmt = "<%df" %(len(accel_data) // 4)
    float_data = list(struct.unpack(fmt, accel_data))

    mean_x = np.mean(accel_x)
    mean_y = np.mean(accel_y)
    mean_z = np.mean(accel_z)

    accel_x.append(float_data[0])
    accel_y.append(float_data[1])
    accel_z.append(float_data[2])

    if len(accel_x) > 4096:
        accel_x.pop(0)
        accel_y.pop(0)
        accel_z.pop(0)

    x_curve.setData(accel_x - mean_x)
    y_curve.setData(accel_y - mean_y)
    z_curve.setData(accel_z - mean_z)

    std_x = np.std(accel_x)
    std_y = np.std(accel_y)
    std_z = np.std(accel_z)

    noise_x = std_x / (np.sqrt(len(accel_x)))
    noise_y = std_y / (np.sqrt(len(accel_y)))
    noise_z = std_z / (np.sqrt(len(accel_z)))

    print(round(1000*noise_x, 2), "\t", round(1000*noise_y, 2), "\t", round(1000*noise_z, 2))

timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(5)

app.exec_()

