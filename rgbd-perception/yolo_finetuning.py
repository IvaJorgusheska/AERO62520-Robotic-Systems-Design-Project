from ultralytics import YOLO

# Load a pretrained YOLOv8 model
model = YOLO("yolov8s.pt")  


model.train(
    data="objects-yolov8/data.yaml", 
    epochs=150,
    imgsz=640,
    batch=16
)