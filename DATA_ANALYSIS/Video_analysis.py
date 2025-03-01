import rosbag
import cv2 
import numpy as np

def rosbag_to_video(bag_file, topic, output_video_file):
    bag = rosbag.Bag(bag_file, 'r')

    images = []
    timestamps = []

    for topic, msg, t in bag.read_messages(topics=[topic]):
        try:
            # Convert the ROS image message to a NumPy array
            image = np.frombuffer(msg.data, dtype=np.uint8)
            image = cv2.imdecode(image, cv2.IMREAD_COLOR)

            if image is None:
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

    for img in images:
        out.write(img)

    out.release()
    print(f"Video saved to {output_video_file}")

# Usage example
rosbag_to_video('your_bag_file.bag', '/your/image/topic', 'output_video.avi')
