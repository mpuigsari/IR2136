import glob
import cv2
from ultralytics import YOLO
import matplotlib.pyplot as plt

# Load the YOLOv8 segmentation model
model = YOLO("best.pt")  # Change to your model path if necessary

# Path to test images folder (modify as needed)
image_paths = glob.glob("./test_images/*.png")  # Change extension if needed

# Iterate over images and make predictions
for img_path in image_paths:
    results = model.predict(img_path, conf=0.5)  # Adjust confidence threshold as needed
    
    # Display the image with detections
    img_with_detections = results[0].plot()
    
    # Show using OpenCV
    cv2.imshow("YOLOv8-Seg Results", img_with_detections)
    cv2.waitKey(0)  # Press any key to continue to the next image

cv2.destroyAllWindows()
