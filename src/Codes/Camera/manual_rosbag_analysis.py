import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

def extract_video_and_save_images(rosbag_filename, topic_name, output_folder, window_size=(640, 480)):
    bag = rosbag.Bag(rosbag_filename)
    bridge = CvBridge()

    # Create a folder to save images
    os.makedirs(output_folder, exist_ok=True)

    # Create a window to display the video with custom size
    cv2.namedWindow('Video', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Video', window_size[0], window_size[1])

    image_count = 0

    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        # Convert the Image message to a NumPy array
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Resize the image
        cv_image_resized = cv2.resize(cv_image, window_size)

        # Display the image
        cv2.imshow('Video', cv_image_resized)

        # Save the image as JPG
        image_filename = os.path.join(output_folder, f'image_{image_count:04d}.jpg')
        cv2.imwrite(image_filename, cv_image)

        # Increment the image count
        image_count += 1

        # Wait for 30 ms (corresponding to approximately 33 frames per second)
        if cv2.waitKey(30) & 0xFF == 27:
            break

    # Close the window after processing all images
    cv2.destroyAllWindows()

    # Close the ROS bag
    bag.close()

if __name__ == "__main__":
    # Replace 'your_rosbag.bag' with the path to your ROS bag file
    rosbag_filename = '/home/icam/Downloads/_2024-01-12-17-30-19.bag'

    # Replace '/camera/image_raw' with the name of the topic where images are located in your ROS bag
    topic_name = '/pylon_camera_node/image_raw'

    # Specify the custom window size (e.g., 800x600)
    window_size = (800, 600)

    # Specify the output folder for saving images
    output_folder = 'images_raw'

    extract_video_and_save_images(rosbag_filename, topic_name, output_folder, window_size)
