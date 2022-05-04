import argparse
import h5py
import hdf5plugin
import os
import progressbar
from prophesee_event_msgs.msg import EventArray
import rosbag
import numpy as np

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Convert ROS bag to hdf5 for event stream(s)')
    parser.add_argument('dataset', type=str, help='ROS bag path')
    args = parser.parse_args()
    
    left_event_topic = '/prophesee/left/events'
    right_event_topic = '/prophesee/right/events'
    receive_left_event = False
    receive_right_event = False
    event_left_x = []
    event_left_y = []
    event_left_p = []
    event_left_t = []
    event_left_ms_to_idx = []
    event_right_x = []
    event_right_y = []
    event_right_p = []
    event_right_t = []
    event_right_ms_to_idx = []

    data_path = args.dataset
    data_path_without_extension, _ = os.path.splitext(data_path)
    event_left_out_path = data_path_without_extension + '.event_left.hdf5'
    event_right_out_path = data_path_without_extension + '.event_right.hdf5'
    event_left_file = h5py.File(event_left_out_path, 'w')
    event_right_file = h5py.File(event_right_out_path, 'w')

    bag = rosbag.Bag(data_path, 'r')
    event_left_msg_num = bag.get_message_count(left_event_topic)
    event_left_msg_idx = 0
    event_left_curr_idx = 0
    event_left_ms_idx = 0
    event_right_msg_num = bag.get_message_count(right_event_topic)
    event_right_msg_idx = 0
    event_right_idx = 0
    event_right_curr_idx = 0
    progress = progressbar.ProgressBar(maxval = (event_left_msg_num + event_right_msg_num))
    progress.start()
    for topic, msg, t in bag.read_messages([left_event_topic, right_event_topic]):
        if (topic == left_event_topic):
            if not receive_left_event:
                receive_left_event = True
                event_left_t_offset = msg.events[0].ts
            for event in msg.events:
                event_left_x.append(event.x)
                event_left_y.append(event.y)
                event_left_p.append(event.polarity)
                event_left_t.append(int((event.ts.to_nsec() - event_left_t_offset.to_nsec()) / 1e3))
                while (event_left_t[-1] >= 1000 * event_left_ms_idx):
                    event_left_ms_to_idx.append(event_left_curr_idx)
                    event_left_ms_idx += 1
                event_left_curr_idx += 1
            event_left_msg_idx += 1
            
        elif (topic == right_event_topic):
            if not receive_right_event:
                receive_right_event = True
                event_right_t_offset = msg.events[0].ts
            for event in msg.events:
                event_right_x.append(event.x)
                event_right_y.append(event.y)
                event_right_p.append(event.polarity)
                event_right_t.append(int((event.ts.to_nsec() - event_right_t_offset.to_nsec()) / 1e3))
                while (event_right_t[-1] >= 1000 * event_right_curr_idx):
                    event_right_ms_to_idx.append(event_right_idx)
                    event_right_curr_idx += 1
                event_right_idx += 1
            event_right_msg_idx += 1

        progress.update(event_left_msg_idx + event_right_msg_idx)
    progress.finish()

    event_left_x = np.array(event_left_x, dtype = 'u2')
    event_left_y = np.array(event_left_y, dtype = 'u2')
    event_left_p = np.array(event_left_p, dtype = 'u1')
    event_left_t = np.array(event_left_t, dtype = 'u4')
    event_left_ms_to_idx = np.array(event_left_ms_to_idx, dtype= 'u8')
    event_right_x = np.array(event_right_x, dtype = 'u2')
    event_right_y = np.array(event_right_y, dtype = 'u2')
    event_right_p = np.array(event_right_p, dtype = 'u1')
    event_right_t = np.array(event_right_t, dtype = 'u4')
    event_right_ms_to_idx = np.array(event_right_ms_to_idx, dtype= 'u8')

    event_left_file.create_dataset('events/x', dtype = 'u2', data = event_left_x, **hdf5plugin.Zstd())
    event_left_file.create_dataset('events/y', dtype = 'u2', data = event_left_y, **hdf5plugin.Zstd())
    event_left_file.create_dataset('events/p', dtype = 'u1', data = event_left_p, **hdf5plugin.Zstd())
    event_left_file.create_dataset('events/t', dtype = 'u4', data = event_left_t, **hdf5plugin.Zstd())
    event_left_file.create_dataset('ms_to_idx', dtype = 'u8', data = event_left_ms_to_idx, **hdf5plugin.Zstd())
    event_left_file.create_dataset('t_offset', shape = (1,), dtype = 'i8', data = int(event_left_t_offset.to_nsec() / 1e3), **hdf5plugin.Zstd())
    event_right_file.create_dataset('events/x', dtype = 'u2', data = event_right_x, **hdf5plugin.Zstd())
    event_right_file.create_dataset('events/y', dtype = 'u2', data = event_right_y, **hdf5plugin.Zstd())
    event_right_file.create_dataset('events/p', dtype = 'u1', data = event_right_p, **hdf5plugin.Zstd())
    event_right_file.create_dataset('events/t', dtype = 'u4', data = event_right_t, **hdf5plugin.Zstd())
    event_right_file.create_dataset('ms_to_idx', dtype = 'u8', data = event_right_ms_to_idx, **hdf5plugin.Zstd())
    event_right_file.create_dataset('t_offset', shape = (1,), dtype = 'i8', data = int(event_right_t_offset.to_nsec() / 1e3), **hdf5plugin.Zstd())

    event_left_file.close()
    event_right_file.close()
