from torch import classes
from ultralytics import YOLO

class YOLOv5:
    def __init__(self, model_path, classes, conf_thresh, iou_thresh):
        self.model = YOLO(model_path)
        name_to_id = {name: idx for idx, name in self.model.names.items()}
        self.classes = []

        for name in classes:
            if name in name_to_id:
                self.classes.append(name_to_id[name])

        if not self.classes:
            self.classes = None
        
        self.conf_thresh = conf_thresh
        self.iou_thresh = iou_thresh

    def predict(self, image):
        # Filter the results based on the specified classes and confidence threshold
        results = self.model.predict(
            source=image,
            imgsz=640,
            conf=self.conf_thresh,
            iou=self.iou_thresh,
            classes=self.classes,
            verbose=False
        )
        
        r = results[0]

        boxes = r.boxes.xyxy.cpu().numpy()
        scores = r.boxes.conf.cpu().numpy()
        classid = r.boxes.cls.cpu().numpy().astype(int)

        return boxes, scores, classid