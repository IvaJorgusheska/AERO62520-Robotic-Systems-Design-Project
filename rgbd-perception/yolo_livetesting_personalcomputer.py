import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO

# Loads trained model
model = YOLO("best.pt")  

# Configure RealSense pipeline (RGB only)
pipeline = rs.pipeline()
config = rs.config()

# Enable RGB stream
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    while True:
        # Wait for a coherent frame pair
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        # Convert RealSense RGB frame to numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Run YOLOv8 inference
        results = model(color_image, conf=0.5, verbose=False)

        # Annotate image with bounding boxes
        annotated = results[0].plot()

        # Display
        cv2.imshow("YOLOv8 + RealSense RGB", annotated)

        # Quit on ESC
        if cv2.waitKey(1) & 0xFF == 27:
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()