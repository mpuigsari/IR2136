import os
import json
import shutil

# Define paths
base_dir = "/home/max/Documents/GitHub/IR2136-Proyecto/yolov8-seg/coco_database"
output_dir = "/home/max/Documents/GitHub/IR2136-Proyecto/yolov8-seg/yolo_dataset"
splits = ["train", "valid", "test"]

# Ensure output directories exist
for split in splits:
    os.makedirs(os.path.join(output_dir, "images", split), exist_ok=True)
    os.makedirs(os.path.join(output_dir, "labels", split), exist_ok=True)

# Process each dataset split
for split in splits:
    json_path = os.path.join(base_dir, split, "_annotations.coco.json")

    # Load COCO JSON
    with open(json_path, "r") as f:
        data = json.load(f)

    # Create class map
    category_map = {c["id"]: c["name"] for c in data["categories"]}

    # Process images
    for image in data["images"]:
        image_id = image["id"]
        file_name = image["file_name"]
        img_width, img_height = image["width"], image["height"]

        # Copy images to YOLO dataset
        src_img_path = os.path.join(base_dir, split, file_name)
        dst_img_path = os.path.join(output_dir, "images", split, file_name)
        if os.path.exists(src_img_path):
            shutil.copy2(src_img_path, dst_img_path)

        # Get annotations for the image
        annotations = [ann for ann in data["annotations"] if ann["image_id"] == image_id]

        # Write YOLO annotation file
        yolo_label_path = os.path.join(output_dir, "labels", split, file_name.replace(".jpg", ".txt"))
        with open(yolo_label_path, "w") as label_file:
            for ann in annotations:
                category_id = ann["category_id"]
                segmentation = ann["segmentation"][0]  # Take first segmentation mask
                bbox = ann["bbox"]

                # Normalize bbox
                x, y, w, h = bbox
                x_center = (x + w / 2) / img_width
                y_center = (y + h / 2) / img_height
                w /= img_width
                h /= img_height

                # Normalize segmentation points
                normalized_seg = []
                for i in range(0, len(segmentation), 2):
                    norm_x = segmentation[i] / img_width
                    norm_y = segmentation[i + 1] / img_height
                    normalized_seg.extend([norm_x, norm_y])

                # Write YOLOv8 format
                label_file.write(f"{category_id} " + " ".join(map(str, normalized_seg)) + "\n")

print("Conversion completed successfully!")
