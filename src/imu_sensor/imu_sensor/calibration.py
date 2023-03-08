import serial
import time
import numpy as np
import matplotlib.pyplot as plt
arduino = serial.Serial(port='/dev/ttyACM3',baudrate=9600,timeout=0.1)
from tqdm import tqdm
from scipy.optimize import curve_fit

t0 = time.time()


# while(True):
#     l1 = str(arduino.readline())
#     l1 = l1.split(',')
#     l1 = [float(l1[l]) for l in range(1,7)]
    # print(l1)

time.sleep(2) # wait for MPU to load and settle
# cal_size = 500
def get_data():
    data = str(arduino.readline())
    while(len(data)<8):
        data = str(arduino.readline())
    data = data.split(',')
    # print(data,len(data))
    data = [float(data[l]) for l in range(1,7)]
    return data
#####################################
# Gyro calibration (Steady)
#####################################
# #
def get_gyro():
    gx,gy,gz = get_data()[3:6]
    return gx,gy,gz

def gyro_cal():
    print("-"*50)
    print('Gyro Calibrating - Keep the IMU Steady')
    [get_gyro() for ii in range(0,cal_size)] # clear buffer before calibration
    mpu_array = []
    gyro_offsets = [0.0,0.0,0.0]
    while True:
        try:
            gx,gy,gz = get_gyro() # get gyro vals
        except:
            continue

        mpu_array.append([gx,gy,gz])

        if np.shape(mpu_array)[0]==cal_size:
            for qq in range(0,3):
                gyro_offsets[qq] = np.mean(np.array(mpu_array)[:,qq]) # average
            break
    print('Gyro Calibration Complete')
    return gyro_offsets

def accel_fit(x_input,m_x,b):
    return (m_x*x_input)+b # fit equation for accel calibration
#
def get_accel():
    ax,ay,az= get_data()[0:3] # read and convert accel data
    return ax,ay,az
    
def accel_cal():
    print("-"*50)
    print("Accelerometer Calibration")
    mpu_offsets = [[],[],[]] # offset array to be printed
    axis_vec = ['z','y','x'] # axis labels
    cal_directions = ["upward","downward","perpendicular to gravity"] # direction for IMU cal
    cal_indices = [2,1,0] # axis indices
    for qq,ax_qq in enumerate(axis_vec):
        ax_offsets = [[],[],[]]
        print("-"*50)
        for direc_ii,direc in enumerate(cal_directions):
            #input("-"*8+" Press Enter and Keep IMU Steady to Calibrate the Accelerometer with the -"+\
            #  ax_qq+"-axis pointed "+direc)
            print("Axis: "+direc)
            [get_accel() for ii in tqdm(range(0,cal_size))] # clear buffer between readings
            mpu_array = []
            while len(mpu_array)<cal_size:
                try:
                    ax,ay,az = get_accel() # get accel variables
                    mpu_array.append([ax,ay,az]) # append to array
                except:
                    continue
            ax_offsets[direc_ii] = np.array(mpu_array)[:,cal_indices[qq]] # offsets for direction

        # Use three calibrations (+1g, -1g, 0g) for linear fit
        popts,_ = curve_fit(accel_fit,np.append(np.append(ax_offsets[0],
                                 ax_offsets[1]),ax_offsets[2]),
                   np.append(np.append(1.0*np.ones(np.shape(ax_offsets[0])),
                    -1.0*np.ones(np.shape(ax_offsets[1]))),
                        0.0*np.ones(np.shape(ax_offsets[2]))),
                            maxfev=10000)
        mpu_offsets[cal_indices[qq]] = popts # place slope and intercept in offset array
    print('Accelerometer Calibrations Complete')
    return mpu_offsets

if __name__ == '__main__':
    #
    ###################################
    # Offset Calculation
    ###################################
    #
    gyro_labels = ['g_x','g_y','g_z'] # gyro labels for plots
    accel_labels = ['a_x','a_y','a_z'] # acc labels for plots
    cal_size = 1000 # points to use for calibration
    # gyro_offsets = gyro_cal() # calculate gyro offsets
    accel_coeffs = accel_cal() # grab accel coefficients
    #
    ###################################
    # Record new data 
    ###################################
    #
    # gyro_data = np.array([get_gyro() for ii in tqdm(range(0,cal_size))]) # new values
    acc_data = np.array([get_accel() for ii in tqdm(range(0,cal_size))]) # new values
    #
    ###################################
    # Plot with and without offsets
    ###################################
    #
    # offsets = [gyro_offsets[0],gyro_offsets[1],gyro_offsets[2]]
    offsets = [accel_coeffs[0],accel_coeffs[1],accel_coeffs[2]]
    print(offsets)
    # plt.style.use('ggplot')
    # fig,axs = plt.subplots(2,1,figsize=(12,9))
    # for ii in range(0,3):
    #     axs[0].plot(gyro_data[:,ii],
    #                 label='${}$, Uncalibrated'.format(gyro_labels[ii]))
    #     axs[1].plot(gyro_data[:,ii]-gyro_offsets[ii],
    #                 label='${}$, Calibrated'.format(gyro_labels[ii]))
    # axs[0].legend(fontsize=14);axs[1].legend(fontsize=14)
    # axs[0].set_ylabel('$w_{x,y,z}$ [$^{\circ}/s$]',fontsize=18)
    # axs[1].set_ylabel('$w_{x,y,z}$ [$^{\circ}/s$]',fontsize=18)
    # axs[1].set_xlabel('Sample',fontsize=18)
    # axs[0].set_ylim([-2,2]);axs[1].set_ylim([-2,2])
    # axs[0].set_title('Gyroscope Calibration Offset Correction',fontsize=22)
    # # fig.savefig('gyro_calibration_output.png',dpi=300,bbox_inches='tight',facecolor='#FCFCFC')
    # plt.show()
    
    plt.style.use('ggplot')
    fig,axs = plt.subplots(2,1,figsize=(12,9))
    for ii in range(0,3):
        axs[0].plot(acc_data[:,ii],
                    label='${}$, Uncalibrated'.format(accel_labels[ii]))
        axs[1].plot(accel_fit(acc_data[:,ii],*accel_coeffs[ii]),
                    label='${}$, Calibrated'.format(accel_labels[ii]))
    axs[0].legend(fontsize=14);axs[1].legend(fontsize=14)
    axs[0].set_ylabel('$a_{x,y,z}$ [g]',fontsize=18)
    axs[1].set_ylabel('$a_{x,y,z}$ [g]',fontsize=18)
    axs[1].set_xlabel('Sample',fontsize=18)
    axs[0].set_ylim([-2,2]);axs[1].set_ylim([-2,2])
    axs[0].set_title('Accelerometer Calibration Calibration Correction',fontsize=18)
    fig.savefig('accel_calibration_output.png',dpi=300,
                bbox_inches='tight',facecolor='#FCFCFC')
    plt.show()