#/usr/bin/python3
import pyrealsense2 as rs
import numpy as np

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)


# Get the intrinsics of the depth stream
profile = pipeline.get_active_profile()
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
intrinsics = depth_profile.get_intrinsics()

# Intrinsic parameters
f_x = intrinsics.fx
f_y = intrinsics.fy
c_x = intrinsics.ppx
c_y = intrinsics.ppy

print(f_x, f_y, c_x, c_y)
# exit()
try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            continue

        # Get the depth data as a numpy array
        depth_image = np.asanyarray(depth_frame.get_data())

        # Specify the pixel coordinates (u, v)
        u = 100  # example pixel x-coordinate
        v = 360  # example pixel y-coordinate

        # Get the depth value at (u, v)
        d = depth_image[v, u]

        # Convert from pixel coordinates to 3D coordinates
        x = (u - c_x) * d / f_x
        y = (v - c_y) * d / f_y
        z = d

        print(f"3D coordinates: x={x}, y={y}, z={z}")

finally:
    # Stop streaming
    pipeline.stop()