import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib import style
import time

import threading

# Read values from the serial port and add then to the function array.
#   so they can be plotted. Fill the time array to have x-axis values.
def update_plot(frame_num):    
    # Update the limits to get the scolling effect
    if len(time_axis) > 0 :
        ax.set_xlim(left = max(0, time_axis[len(time_axis) - 1] - 50), 
                right = time_axis[len(time_axis) - 1] + 1)
    
    if len(data_pnts) > 0 :
        ax.set_ylim(top = max(data_pnts) + 10, 
                bottom = min(data_pnts) - 10)    

    # update the data
    line.set_data(time_axis, data_pnts)
    return line,

# Value used to eventually kill the thread
keep_reading = True
# Thread function for displaying the plot independently from reading
#   the data from the serial port.
def serial_read(name) :
    print("Starting plot func")

    # clear out any possibly incomplete line
    ser.readline()
    while keep_reading == 1 :
        # Read in the data title to throw it away
        print(ser.read(8))

        # Read in the time stamp and the data point
        time_val = ser.read(8)
        print(time_val)
        time_axis.append(int(time_val))
        print("time: ", time_axis[len(time_axis) - 1])

        data_val = ser.read(8)
        print(data_val)
        data_pnts.append(int(data_val))
        print("data: ", data_pnts[len(data_pnts) - 1])
        
        # Read in the end-of-transmission mesage: b'end_msg\n'
        print(ser.read(8))

        # Check that the plot isn't "zig-zagging" - if the x axis
        #   value just received is less than the previously known
        #   value, just throw out all the previous values.
        if time_axis[len(time_axis) - 1] < time_axis[len(time_axis) - 2] :
            time_axis.clear()
            data_pnts.clear()   


# only works for windows since serial ports are called COM ports.
#   This would have to be changed for Linux in which a serial port
#   is read from /dev/ttyUSB<port number>
ser = serial.Serial("COM4", 115200, timeout = None)

# Set up the plot
fig = plt.figure()
ax = fig.add_subplot(1,1,1) # 1 subplot in 1st row of 1st column
line, = plt.plot([], [], 'b')   # plt.plot() returns a tuple and line is assigned
                                #  to the first element.

time_axis = []      # time axis for the plot
data_pnts = []      # function values for the plot


serial_read_thread = threading.Thread(target=serial_read, args=(1,))


# clear the buffer to get the new line for the plot
ser.readline()
# Set the title using the titles that is transmitted
plot_title = str(ser.read(8).decode())

ax.set_title(plot_title)
ax.plot(time_axis, data_pnts)

serial_read_thread.start()

ani = FuncAnimation(fig, update_plot)
plt.show()

keep_reading = False