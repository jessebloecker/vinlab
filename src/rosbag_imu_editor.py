#!/usr/bin/env python

import sys

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from rosbags.serde import serialize_cdr
from color_helper import ColorHelper
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from rosbags.serde import serialize_cdr
from color_helper import ColorHelper

import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

from scipy.fft import fft, fftfreq
from scipy import signal
from scipy.signal import lfilter, iirfilter
from scipy.signal import butter


def extract_imu_data(name):
    """
    name = 'openvins_outdoor_park_hover_2024-12-20-22-31-39'
    Assuming only one topic with type sensor_msgs/msg/Imu
    export the imu data as a numpy array format [timestamp, ax, ay, az, wx, wy, wz]
    save it as a npy file: <name>_<topic>.npy
    """
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    imu_data = []
    msg_count = 0
    imu_msg_count = 0
    imu_topic = None
    with Reader(data_path_in) as reader:
        for connection, timestamp, rawdata in reader.messages():
            msg_count += 1
            if connection.msgtype == 'sensor_msgs/msg/Imu':
                if not imu_topic:
                    imu_topic = connection.topic  
                imu_msg_count += 1
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                formatted_timestamp = datetime.fromtimestamp(timestamp/1e9).strftime('%Y-%m-%d %H:%M:%S')
                print('Extracting IMU data from at timestamp: {}. IMU message count: {} Total message count: {}'.format(formatted_timestamp, imu_msg_count, msg_count), end='\r', flush=True)
                sec = msg.header.stamp.sec
                nanosec = msg.header.stamp.nanosec
                floatsec = sec + nanosec*1e-9
                imu_data.append([floatsec, msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                                  msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

    imu_data = np.array(imu_data)
    # check if first column is monotonic, if not then sort
    if np.all(np.diff(imu_data[:,0]) > 0):
        print('IMU data is monotonic')
    else:
        print('IMU data is not monotonic (monotonicity: {}\% sorting...'.format(np.sum(np.diff(imu_data[:,0]) > 0)/imu_data.shape[0]))
        imu_data = imu_data[np.argsort(imu_data[:,0])]

    #save as npy file
    dst = '{}_{}.npy'.format(name,imu_topic[1:].replace('/','_'))
    print('\nSaving IMU data to file: {}'.format(dst))
    np.save(dst, imu_data)

    return imu_data, imu_topic


def new_imu_writer(data_path_in, data_dict, data_path_out):
    """
    data_original: the original data to iterate through
    data_dict: the new data to add to the original data
    data_path_out: the path to save the new data
    """

    # copy all of the initial original data
    #loop through again add the new data alongside the original data
    bag_index = 0
    imu_msg_index = 0
    write_connections = {} #dictionary where the key is the topic and the value is the write connection
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    with Writer(data_path_out) as writer:
        #add the new topics which will not exist in the original bag
        for k in data_dict.keys():
            new_topic = k
            write_connection = writer.add_connection(new_topic, 'sensor_msgs/msg/Imu', typestore=typestore)
            write_connections[new_topic] = write_connection
        with Reader(data_path_in) as reader:
            for read_connection, timestamp, rawdata in reader.messages():
                bag_index += 1
                if read_connection.topic not in write_connections.keys() and 'ov_msckf' not in read_connection.topic:
                    write_connection = writer.add_connection(read_connection.topic, read_connection.msgtype, typestore=typestore)
                    write_connections[read_connection.topic] = write_connection
                
                if read_connection.msgtype == 'sensor_msgs/msg/Imu': #write the original imu message and the new smoothed imu messages
                    write_connection = write_connections[read_connection.topic]
                    writer.write(write_connection, timestamp, rawdata)
                    msg = typestore.deserialize_cdr(rawdata, read_connection.msgtype)
                    t=0
                    for topic in data_dict.keys():
                        t+=1
                        # print('writing new imu msg {} of {}'.format(i,len(window_sizes)))
                        write_connection = write_connections[topic]
                        new_msg = construct_imu_message(msg, data_dict[topic][imu_msg_index])
                        writer.write(write_connection, timestamp+t, typestore.serialize_cdr(new_msg, 'sensor_msgs/msg/Imu'))
                    imu_msg_index += 1
                elif 'ov_msckf' not in read_connection.topic:
                    write_connection = write_connections[read_connection.topic]
                    writer.write(write_connection, timestamp, rawdata)
                print('imu messages {}, bag index {}'.format(imu_msg_index, bag_index), end='\r', flush=True)
    print('done.')
    print('wrote file: {}'.format(data_path_out))
    pass

def sliding_window_average(arr, window_size):
    """
    arr = (n,m) array
    apply averiging convolution over each column of arr using the provided window size

    returns:
        out = (n,m) array with the same shape as arr
    """
    print('applying sliding window size {} over {} array'.format(window_size,arr.shape))
    kernel = np.ones(window_size)/window_size
    first_row = arr[0]
    pad = np.tile(first_row,(window_size-1,1))
    arr_padded = np.vstack((pad, arr)) #shifts the output to make it causal
    out_padded = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), axis=0, arr=arr_padded)
    out = out_padded[window_size-1:,:]
    # print('window_size: {}, kernel {}, first_row: {}, pad: {}, arr_padded: {}, out_padded: {}, out: {}'.format(window_size, kernel.shape, first_row.shape, pad.shape, arr_padded.shape, out_padded.shape, out.shape))
    # print('')
    return out

def construct_imu_message(msg, data):
    """
    msg = sensor_msgs/msg/Imu
    data = [t,ax, ay, az, wx, wy, wz]
    keep all other fields the same
    save as a npy file: <name>_<topic>_smoothed_<window_size>.npy
    """
    new_msg = msg
    new_msg.linear_acceleration.x = data[1]
    new_msg.linear_acceleration.y = data[2]
    new_msg.linear_acceleration.z = data[3]
    new_msg.angular_velocity.x = data[4]
    new_msg.angular_velocity.y = data[5]
    new_msg.angular_velocity.z = data[6]
    return new_msg





def fft_plot(imu_data, imu_topic, time_window, start_time, force_binary=True, save=False):
    """
    show simple frequency analysis of 1d array of data
    show window for each file matching <name>
    """
    dur = imu_data[-1,0] - imu_data[0,0]
    n = imu_data.shape[0]
    sample_rate = n/dur
    # print('Sample rate {:.3f} Total Duration: {:.3f}'.format(sample_rate, dur))

    #make sub plot for each axis
    fig, axes = plt.subplots(6, sharex=True)
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Magnitude')
    # print('data: {}, duration: {}'.format(imu_data.shape, imu_data[-1,0] - imu_data[0,0]))
    # Create a time series with time values


    if force_binary:
        num_values = 2**int(np.log2(time_window * n/dur))
    else:
        num_values = int(time_window * n/dur)
    start_index = int(start_time * n/dur)

    time = imu_data[start_index:start_index+num_values,0]   
    
    for i in range(1,7): #get the signal on each axis
        signal = imu_data[start_index:start_index+num_values,i] 
        frequencies = fftfreq(num_values, 1.0/sample_rate)
        # freq_start = 5
        # freq_end = len(frequencies)//2

        #compuet fft, enforce frequnceis are below 0.5*sample_rate
        fft_values = fft(signal)

        # Compute the frequencies corresponding to the FFT values                                     

        # add sub stitle  
        labels = ['ax','ay','az','wx','wy','wz']
        colors = ['r','g','b','lightcoral','lightgreen','lightblue']
        axes[i-1].set_ylabel('{}'.format(labels[i-1]))
        # axes[i-1].set_xlim(0, 300)
        near_zero = int(len(frequencies)*0.03)
        axes[i-1].plot(frequencies[near_zero:len(frequencies)//2], np.abs(fft_values)[near_zero:len(frequencies)//2], color=colors[i-1])
        

    title1 = 'FFT\nDATASET: {} TOPIC: {}, SAMPLE RATE: {:0.2f} '.format(name,imu_topic, sample_rate)
    title2 = 'Based on {:.2f}s window of {:.2f}s dataset ({} of {} samples) starting at t={:0.2f}s '.format(
        time_window, dur, num_values, n,  start_time)
    fig.suptitle(title1+'\n'+title2, fontsize=16)
    if save:
        filename = 'fft_{}_{}_w{:06.2f}_s{:06.2f}.png'.format(name, imu_topic[1:].replace('/','_'), time_window, start_time)
        #make bigger
        fig.set_size_inches(18.5, 10.5)
        fig.savefig('/home/jesse/data/fft/{}'.format(filename))
        print('saved file: /home/jesse/data/fft/{}'.format(filename))
    else:
        plt.show()


# def psd_plot():
def psd_plot(imu_data, imu_topic, time_window, start_time, force_binary=False, save=False):
    dur = imu_data[-1,0] - imu_data[0,0]
    dur = imu_data[-1,0] - imu_data[0,0]
    n = imu_data.shape[0]
    sample_rate = n/dur
    if force_binary:
        num_values = 2**int(np.log2(time_window * n/dur))
    else:
        num_values = int(time_window * n/dur)

    start_index = int(start_time * n/dur)
    if num_values > n - start_index:
        num_values = n - start_index
        time_window = dur * num_values/n
        print('time_window at start time exceeds data length, setting time_window to {}'.format(time_window))

    #make sub plot for each axis
    fig, axes = plt.subplots(6, sharex=True)

    time = imu_data[start_index:start_index+num_values,0]  

    labels = ['ax','ay','az','wx','wy','wz']
    # colors = ['r','g','b','lightcoral','lightgreen','lightblue']
    colors = ['r','g','b','r','g','b']
    bg_colors = ['white','white','white','lightgray','lightgray','lightgray']
    for i in range(1,7): #get the signal on each axis
        x = imu_data[start_index:start_index+num_values,i]
        f, Pxx_den = signal.welch(x, sample_rate, nperseg=1024)
        axes[i-1].set_facecolor(bg_colors[i-1])
        axes[i-1].semilogy(f, Pxx_den, color=colors[i-1])
        axes[i-1].set_ylabel('{}'.format(labels[i-1]))
        #turn x axis grid on and label Hz
        axes[i-1].grid(axis='x', linestyle='--')
        # axes[i-1].set_label('Frequency (Hz)')

    title1 = 'PSD\nDATASET: {} TOPIC: {}, SAMPLE RATE: {:0.2f} '.format(name,imu_topic, sample_rate)
    title2 = 'Based on {:.2f}s window of {:.2f}s dataset ({} of {} samples) starting at t={:0.2f}s '.format(
        time_window, dur, num_values, n,  start_time)
    fig.suptitle(title1+'\n'+title2, fontsize=16)
    if save:
        filename = 'psd_{}_{}_w{:06.2f}_s{:06.2f}.png'.format(name, imu_topic[1:].replace('/','_'), time_window, start_time)
        #make bigger
        fig.set_size_inches(18.5, 10.5)
        fig.savefig('/home/jesse/data/psd/{}'.format(filename))
        print('saved file: /home/jesse/data/psd/{}'.format(filename))
    else:
        plt.show()

        

    
def psd_plot_example():
    
    rng = np.random.default_rng()
    N = 1e5
    #generate signal
    sample_rate = 1e4
    amp = 2
    freq = 1234.0
    time = np.arange(N)/sample_rate
    print('total duration: {:.2f}'.format(time[-1] - time[0]))
    x1 = amp*np.sin(2*np.pi*freq*time)
    x2 = amp*np.sin(2*np.pi*freq*1.5*time)
    x = x1 + x2
    #add noise 
    noise_power = 0.001 * sample_rate / 2
    x += rng.normal(scale=np.sqrt(noise_power), size=len(time))
    f, Pxx_den = signal.welch(x, sample_rate, nperseg=1024)


    plt.semilogy(f, Pxx_den)
    plt.ylim([0.5e-3, 1])
    plt.xlabel('frequency [Hz]')
    plt.ylabel('PSD [V**2/Hz]')
    plt.show()


def butter_example():

    #define time values
    sample_rate = 1000
    dur = 3
    t = np.linspace(0, dur, sample_rate*dur, endpoint=False)

    #define sinusoidal signals
    a = 2
    f1 = 20
    f2 = 134
    x1 = np.sin(a * np.pi * f1 * t)  
    x2 = np.sin(a * np.pi * f2 * t)  
    x = x1 + x2
    x += 0.1 * np.random.randn(sample_rate*dur)  # add noise

    #apply butter filter
    order = 2
    center = f1
    width = 3

    high = center + width
    low = center - width
    sos = signal.butter(order, [low,high], 'bandstop', fs=sample_rate, output='sos')
    x_filtered = signal.sosfiltfilt(sos, x) #filtfilt applies filter twice to remove phase shift

    #make 2x2 plot, original signal, psd of original signal, filtered signal, psd of filtered signal
    fig, axes = plt.subplots(2,2)
    axes[0,0].plot(t, x1, 'r', alpha=0.5)
    axes[0,0].plot(t, x2, 'b', alpha=0.5)
    axes[0,0].plot(t, x, 'purple')
    axes[0,0].set_title('Original Signal (components shown)')
    #add legend for x1, x2, and combined
    axes[0,0].legend(['{} Hz'.format(f1), '{} Hz'.format(f2), 'Sum+noise'])
    axes[0,0].set_xlabel('Time [s]')

    #set limits same as prev
    axes[0,1].set_ylim(axes[0,0].get_ylim())
    axes[0,1].plot(t, x2, 'b', alpha=0.5)
    axes[0,1].plot(t, x_filtered, 'purple')
    axes[0,1].set_title('Filtered Signal')
    axes[0,1].legend(['{} Hz'.format(f2), 'Filtered Sum'])
    axes[0,1].set_xlabel('Time [s]')

    freqs_1, Pxx_den1 = signal.welch(x, sample_rate, nperseg=1024)
    freqs_2, Pxx_den2 = signal.welch(x_filtered, sample_rate, nperseg=1024)

    axes[1,0].semilogy(freqs_1, Pxx_den1, color='purple')
    axes[1,0].set_xlim([0, 1.5*f2])
    axes[1,0].set_title('PSD Original Signal')

    #add vertical line at f1 and f2
    axes[1,0].axvline(f1, color='r', linestyle='--')
    axes[1,0].axvline(f2, color='b', linestyle='--')
    axes[1,0].set_xlabel('Frequency [Hz]')


    axes[1,1].semilogy(freqs_2, Pxx_den2, color='purple') 
    axes[1,1].set_xlim([0, 1.5*f2])
    #add vertical line at f1 and f2
    axes[1,1].axvline(f1, color='r', linestyle='--')
    axes[1,1].axvline(f2, color='b', linestyle='--')
    
    axes[1,1].set_title('PSD Filtered Signal')
    axes[1,1].set_xlabel('Frequency [Hz]')

    #set tilte of whole figure
    fig.suptitle('Butterworth Bandstop Filter Example: Original = Sum of 2 sinusoids at {} and {} Hz with high freq noise. Remove Low Freq component'.format(f1,f2), fontsize=16)
    plt.show()


def apply_bandstop_filter(imu_data, imu_topic, time_window, start_time, order, center, width, save=False):
    dur = imu_data[-1,0] - imu_data[0,0]
    n = imu_data.shape[0]
    sample_rate = n/dur
    if time_window < 0:
        time_window = dur
    num_values = int(time_window * n/dur)

    start_index = int(start_time * n/dur)
    if num_values > n - start_index:
        num_values = n - start_index
        time_window = dur * num_values/n
        print('time_window at start time exceeds data length, setting time_window to {}'.format(time_window))

    fig, axes = plt.subplots(12,2)

    times = imu_data[start_index:start_index+num_values,0]  
    initial_time = imu_data[0,0]
    t = times - initial_time
    # print(t[0:100])
    out = np.zeros_like(imu_data)
    out[:,0] = times

    labels = ['ax','PSD_ax','ay','PSD_ay','az','PSD_az','wx','PSD_wx','wy','PSD_wy','wz','PSD_wz']
    colors = ['r','g','b','r','g','b']
    bg_colors = ['white','white','white','lightgray','lightgray','lightgray']
    for side in range(2):
        for i in range(1,7): #get the signal on each axis
            x = imu_data[start_index:start_index+num_values,i]
            f, Pxx_den = signal.welch(x, sample_rate, nperseg=1024)
            low = center - width/2 if center - width/2 > 0 else 1
            high = center + width/2
            sos = signal.butter(order, [low,high], 'bandstop', fs=sample_rate, output='sos')
            x_filtered = signal.sosfiltfilt(sos, x) #filtfilt applies filter twice to remove phase shift
            out[:,i] = x_filtered
            f_filtered, Pxx_den_filtered = signal.welch(x_filtered, sample_rate, nperseg=1024)  
            j = 2*(i-1) #index of corresponding subplot axis
            if side==0: #original
                axes[j,side].set_facecolor(bg_colors[i-1]) 
                axes[j,side].set_xlim([t[0], t[-1]])
                axes[j,side].set_ylabel('{}'.format(labels[i-1]))
                axes[j,side].set_xlabel('{}'.format('Time [s]'))
                axes[j,side].grid(axis='x', linestyle='--')
                axes[j,side].plot(t, x, color=colors[i-1])


                axes[j+1,side].set_facecolor(bg_colors[i-1])
                axes[j+1,side].set_xlim([0, sample_rate/2])
                axes[j+1,side].set_ylabel('{}'.format(labels[j+1]))
                axes[j+1,side].set_xlabel('{}'.format('Frequency [Hz]'))
                axes[j+1,side].grid(axis='x', linestyle='--')
                axes[j+1,side].semilogy(f, Pxx_den, color='black')
    
            if side==1: #filtered
                axes[j,side].set_facecolor(bg_colors[i-1]) 
                axes[j,side].set_xlim([t[0], t[-1]])
                axes[j,side].set_ylabel('{}'.format(labels[i-1]))
                axes[j,side].set_xlabel('{}'.format('Time [s]'))
                axes[j,side].grid(axis='x', linestyle='--')
                axes[j,side].plot(t, x_filtered, color=colors[i-1])

                axes[j+1,side].set_facecolor(bg_colors[i-1])
                axes[j+1,side].set_xlim([0, sample_rate/2])
                axes[j+1,side].set_ylabel('{}'.format(labels[j+1]))
                axes[j+1,side].set_xlabel('{}'.format('Frequency [Hz]'))
                axes[j+1,side].grid(axis='x', linestyle='--')
                axes[j+1,side].semilogy(f_filtered, Pxx_den_filtered, color='black')

    #set the title of the top left and top right
    axes[0,0].set_title('Original Signal')
    axes[0,1].set_title('Filtered Signal')
    title1 = 'PSD\nDATASET: {} TOPIC: {}, SAMPLE RATE: {:0.2f} '.format(name,imu_topic, sample_rate)
    title2 = 'Based on {:.2f}s window of {:.2f}s dataset ({} of {} samples) starting at t={:0.2f}s '.format(
        time_window, dur, num_values, n,  start_time)
    title3 = 'Bandstop Filter: Order: {}, Center: {}Hz, Width: {}Hz'.format(order, center, width)
    fig.suptitle(title1+'\n'+title2+'\n'+title3, fontsize=16)
    plt.subplots_adjust(left=None, bottom=None, right=None, top=None, wspace=0.5, hspace=1)

    if save:
        filename = 'psd_{}_{}_filt_ord{:02d}_c{:02d}_w{:02d}.png'.format(name, imu_topic[1:].replace('/','_'), order, center, width)
        fig.set_size_inches(18.5, 10.5)
        fig.savefig('/home/jesse/data/psd/{}'.format(filename))
        print('saved file: /home/jesse/data/psd/{}'.format(filename))
    else:
        # fig.plt.show()
        plt.show()


    outfile = '/home/jesse/data/psd/{}_{}_filt_ord{:02d}_c{:02d}_w{:02d}.npy'.format(name, imu_topic[1:].replace('/','_'), order, center, width)
    np.save(outfile, out)
    return out







if __name__ == '__main__':

    # read the original rosbag
    # extract all imu messsages into local array
    # make several copies, each with a differnt filter applied (different center frequencies and widths e.g.), store each instance as new sequence
    # write a new rosbag, with the original sequence and all of the filtered sequences as additional topics


    # name = 'imu_arm_disarm_03'
    # name = 'imu_arm_disarm_03_mcap'
    # name = 'imu_smoothed_openvins_mono_tracking_2024-12-19-20-22-53'
    # name = 'imu_smoothed_openvins_mono_tracking_se_2024-12-19-22-00-57'
    # name = 'imu_smoothed_reduced_openvins_mono_tracking_se_2024-12-19-22-00-57'
    # name = 'imu_smoothed_reduced_openvins_outdoor_2024-12-20-00-05-14'
    # name = 'imu_smoothed_reduced_openvins_outdoor_park_dynamic_2024-12-20-22-49-32'
    # name = 'imu_smoothed_reduced_openvins_outdoor_park_high_2024-12-20-22-23-00'
    # name = 'imu_smoothed_reduced_openvins_outdoor_park_hover_2024-12-20-22-31-39'
    # name = 'imu_smoothed_reduced_openvins_outdoor_park_low_fast_2024-12-20-22-25-27'
    # name = 'indoor_forward_9_snapdragon_with_gt'
    # name = 'openvins_dark_2025-01-16-01-16-07'
    # name = 'openvins_dark_2025-01-16-01-16-26'
    # name = 'openvins_dark_hover_2025-01-16-01-18-05'
    # name = 'openvins_dark_hover_2025-01-16-01-19-10'
    # name = 'openvins_dark_hover_2025-01-16-01-20-03'
    # name = 'openvins_day1_2025-01-22-21-56-59'
    # name = 'openvins_day1_2025-01-22-21-58-22'
    # name = 'openvins_day1_2025-01-22-21-59-56'
    # name = 'openvins_mono_tracking_2024-12-19-20-22-53'
    # name = 'openvins_mono_tracking_se_2024-12-19-22-00-57'
    name = 'openvins_outdoor_2024-12-20-00-05-14'
    # name = 'openvins_outdoor_2024-12-20-00-08-11'
    # name = 'openvins_outdoor_park_2024-12-20-22-32-33'
    # name = 'openvins_outdoor_park_dynamic_2024-12-20-22-49-32'
    # name = 'openvins_outdoor_park_dynamic_2024-12-20-22-49-32_mcap'
    # name = 'openvins_outdoor_park_high_2024-12-20-22-23-00'
    # name = 'openvins_outdoor_park_hover_2024-12-20-22-30-17'
    # name = 'openvins_outdoor_park_hover_2024-12-20-22-30-17_mcap'
    # name = 'openvins_outdoor_park_hover_2024-12-20-22-31-39'
    # name = 'openvins_outdoor_park_low_fast_2024-12-20-22-25-27'
    # name = 'openvins_outdoor_park_low_fast_2024-12-20-22-26-42'
    # name = 'openvins_outdoor_park_low_fast_2024-12-20-22-27-05'
    # name = 'V1_01_easy'


    formatted_date = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    data_path_in = '/home/jesse/data/{}'.format(name) 
    data_path_out = '/home/jesse/data/{}_{}'.format(name,'edit_'+formatted_date)
    print('Using data: {}'.format(data_path_in))

    imu_data, imu_topic = extract_imu_data(name)


    # bandstop_example()
    # butter_example()
    order = 2
    centers = [5,10,15,20,25,30,35,40,45,50,55,60]
    widths = [5,10,15,20]
    data_dict = {}
    for c in centers:
        for w in widths:
    
            topic = '{}_filt_ord{:02d}_c{:02d}_w{:02d}'.format(imu_topic,order,c,w)
            filename = '/home/jesse/data/psd/{}_{}_filt_ord{:02d}_c{:02d}_w{:02d}.npy'.format(name, imu_topic[1:].replace('/','_'),order,c,w)
        
            filtered_data = apply_bandstop_filter(imu_data, imu_topic, time_window=-1, start_time=0, order=order, center=c, width=w, save=True)
            # try:
            #     filtered_data = np.load(filename) #use the saved data instead if it exists
            #     print('loaded file: {}'.format(filename))
            # except:
            #     print('file {} not found'.format(filename))
            #     sys.exit()

            # print('topic: {}',topic)
            data_dict[topic] = filtered_data

    new_imu_writer(data_path_in, data_dict, data_path_out)  



