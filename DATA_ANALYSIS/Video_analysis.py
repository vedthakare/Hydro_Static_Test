import rosbag
import cv2
import numpy as np
from tqdm import tqdm

def rosbag_to_video(bag_file, topic, output_video_file):
    bag = rosbag.Bag(bag_file, 'r')

    images = []
    timestamps = []

    messages = list(bag.read_messages(topics=[topic]))

    for topic, msg, t in tqdm(messages, desc="Processing frames"):
        try:
            # For compressed image topics
            if 'compressed' in topic:
                # Directly decode the compressed image data
                image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
                # No color conversion needed for compressed images as OpenCV already handles it correctly
            else:
                # For raw image topics - assuming the encoding is RGB8
                image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                # Convert from RGB to BGR for OpenCV
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            if image is None:
                print("Failed to decode an image, skipping...")
                continue

            images.append(image)
            timestamps.append(t.to_sec())  # Convert ROS time to seconds
        except Exception as e:
            print(f"Skipping message due to error: {e}")
            continue

    bag.close()

    if len(images) < 2:
        print("Not enough frames to create a video.")
        return

    # Compute frame rates based on time differences
    time_diffs = np.diff(timestamps)  # Compute time gaps between frames
    avg_fps = 1.0 / np.mean(time_diffs)  # Compute the average frame rate
    print(f"Computed FPS: {avg_fps:.2f}")

    # Get resolution from the first image
    height, width, _ = images[0].shape
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(output_video_file, fourcc, avg_fps, (width, height))

    for img in tqdm(images, desc="Writing video"):
        out.write(img)

    out.release()
    print(f"Video saved to {output_video_file}")

# Usage example
rosbag_to_video('DATA_ANALYSIS/2025-03-01-10-11-31.bag', '/usb_cam/image_raw/compressed', 'output_video.avi')