--- START OF FILE dca.py ---

#!/usr/bin/env python3

"""Simple publisher of raw radar data.
"""
import os
import sys
import time
import socket
import serial
import argparse
import numpy as np
import json # Added: Import json module
from pathlib import Path # Added: Import Path from pathlib

import rospy
import rospkg
from rospy.numpy_msg import numpy_msg
from xwr_raw_ros.msg import RadarFrame
from xwr_raw_ros.msg import RadarFrameStamped
from xwr_raw_ros.msg import RadarFrameFull
from xwr_raw.radar_config import RadarConfig
from xwr_raw.radar_pub import DCAPub

if __name__ == '__main__':

    # Read path to config file.
    parser = argparse.ArgumentParser()
    parser.add_argument('cfg', help="Path to configuration file for radar.")
    parser.add_argument('--dca_ip',         default='192.168.33.180', help='IP address of DCA1000.')
    parser.add_argument('--dca_cmd_port',   default=4096,             help='CMD port of DCA1000.')
    parser.add_argument('--host_ip',        default='192.168.33.30',  help='IP address of host.')
    parser.add_argument('--host_cmd_port',  default=4096,             help='CMD port of host.')
    parser.add_argument('--host_data_port', default=4098,             help='Data port of host.')
    # Added: Arguments for saving frames
    parser.add_argument('--save_frames', action='store_true', help = 'If set, save each received frame to disk.')
    parser.add_argument('--save-dir', default='/home/robot/ws_multimodal/src/data', help = 'Directory to store captured frames and metadata.')
    args = parser.parse_args(rospy.myargv()[1:])

    # Initialize node and topic.
    rospy.init_node('xwr_radar')   #创建xwr_radar的ROS节点

    # Parse and publish config file.
    rospack = rospkg.RosPack()
    with open(os.path.join(rospack.get_path('xwr_raw_ros'), args.cfg), 'r') as f:
        cfg = f.readlines()
    radar_config = RadarConfig(cfg)
    rospy.set_param('radar_config', dict(**radar_config))

    # Extract params from config.
    radar_params = radar_config.get_params()
    rospy.set_param('radar_params', dict(**radar_params))

    #
    publisher = rospy.Publisher('radar_data',
                                numpy_msg(RadarFrameFull),
                                queue_size=1)

    dca = DCAPub(cfg,
                 dca_ip          = args.dca_ip,
                 dca_cmd_port    = int(args.dca_cmd_port),
                 host_ip         = args.host_ip,
                 host_cmd_port   = int(args.host_cmd_port),
                 host_data_port  = int(args.host_data_port))

    dca.configure()
    dca.start_capture()

    # --- Added code for saving frames and metadata ---
    save_dir = Path(args.save_dir)
    save_dir.mkdir(parents = True, exist_ok = True)
    metadata_path = save_dir /'radar_metadata.json'
    frame_count = 0
    start_time = time.time()
    end_time = start_time

    def _to_serializable(obj):
        """Helper function to convert numpy types to serializable types for JSON."""
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, np.generic):
            return obj.item()
        # Fallback for other types, though usually not hit if handling numpy specifics
        raise TypeError(f"Object of type {type(obj)} is not JSON serializable")

    def _write_metadata(final_end_time = None):
        """Writes or updates the radar metadata to a JSON file."""
        metadata = {
            'start_time': start_time,
            'end_time': final_end_time if final_end_time is not None else end_time,
            'radar_params': radar_params
        }
        with open(metadata_path,'w') as meta_file:
            json.dump(metadata, meta_file, indent = 2, default = _to_serializable)

    def _shutdown():
        """Custom shutdown hook to flush metadata and close DCA."""
        final_time = time.time()
        _write_metadata(final_time)
        dca.close()

    # Register the custom shutdown handler
    rospy.on_shutdown(_shutdown)
    # --- End of added code for saving frames and metadata ---

    # Publish radar data to topic.
    while True:
        frame_data, new_frame = dca.update_frame_buffer()
        if new_frame:
            # --- Added code for saving each frame ---
            end_time =time.time() # Update end_time with each new frame
            if args.save_frames:
                frame_count += 1
                frame_path = save_dir / f"{frame_count:06d}.npy"
                np.save(frame_path, frame_data)
                # Optional: print for debug
                # print(f"Saved frame {frame_count} to {frame_path}")
            # --- End of added code for saving each frame ---

            # Publish raw radar frame only.
            # msg = RadarFrame()
            # msg.data = frame_data

            # Publish with timestamp.
            # msg = RadarFrameStamped()
            # msg.header.stamp = rospy.get_rostime()
            # msg.data = frame_data

            # Publish with all metadata.
            msg = RadarFrameFull()   #确定数据类型
            msg.header.stamp   = rospy.Time.now() # Added: Header timestamp
            msg.platform       = radar_params['platform']
            msg.adc_output_fmt = radar_params['adc_output_fmt']
            msg.range_bias     = radar_params['range_bias']
            msg.rx_phase_bias  = radar_params['rx_phase_bias']

            msg.chirp_time   = radar_params['chirp_time']
            msg.chirp_slope  = radar_params['chirp_slope']
            msg.frame_time   = radar_params['frame_time']
            msg.velocity_max = radar_params['velocity_max']
            msg.velocity_res = radar_params['velocity_res']

            msg.sample_rate  = radar_params['sample_rate']
            msg.range_max    = radar_params['range_max']
            msg.range_res    = radar_params['range_res']

            msg.rx = radar_params['rx']
            msg.tx = radar_params['tx']
            msg.shape = (radar_params['n_chirps'],
                         radar_params['n_rx'],
                         radar_params['n_samples'])

            msg.data = frame_data

            publisher.publish(msg)
