import open3d as o3d
import numpy as np

# Replace '/dev/media1' with the correct path to your Astra camera device
device_path = '/dev/media1'

# Create a depth sensor object
depth_sensor = o3d.io.AzureKinectSensor()
depth_sensor.open(device_path)

# Get camera intrinsic parameters
intrinsic = depth_sensor.get_intrinsic()

# Create a visualizer object
visualizer = o3d.visualization.Visualizer()
visualizer.create_window()

# Main loop for capturing and visualizing depth frames
while True:
    # Capture a depth frame
    rgbd = depth_sensor.capture_depth_as_ir()

    # Convert the depth frame to a point cloud
    pcd = o3d.geometry.create_point_cloud_from_depth_image(
        rgbd.depth, intrinsic)

    # Visualize the point cloud
    visualizer.clear_geometries()
    visualizer.add_geometry(pcd)
    visualizer.poll_events()
    visualizer.update_renderer()

# Close the depth sensor when done
depth_sensor.close()
