import cv2
import torch
from PIL import Image
from transformers import AutoModelForImageClassification, MobileNetV2ImageProcessor

MODEL_NAME = "linkanjarad/mobilenet_v2_1.0_224-plant-disease-identification"
CAMERA_INDEX = 0
OUTPUT_IMAGE = "/home/amen/Desktop/AGROBOT/ai_vision/captured_leaf.jpg"


def capture_frame(camera_index: int = 0, save_path: str = OUTPUT_IMAGE) -> Image.Image:
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera at index {camera_index}")
    ret, frame_bgr = cap.read()
    cap.release()
    if not ret or frame_bgr is None:
        raise RuntimeError("Failed to capture frame from camera")
    cv2.imwrite(save_path, frame_bgr)
    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    return Image.fromarray(frame_rgb)


def classify_leaf(image: Image.Image, top_k: int = 5):
    model = AutoModelForImageClassification.from_pretrained(MODEL_NAME)
    model.eval()
    processor = MobileNetV2ImageProcessor()

    inputs = processor(images=image, return_tensors="pt")
    with torch.no_grad():
        logits = model(**inputs).logits
        probs = torch.softmax(logits, dim=-1)[0]
        vals, idxs = torch.topk(probs, k=min(top_k, probs.numel()))

    return [(model.config.id2label[int(i)], float(v.item())) for v, i in zip(vals, idxs)]


if __name__ == "__main__":
    img = capture_frame(CAMERA_INDEX, OUTPUT_IMAGE)
    for label, score in classify_leaf(img, top_k=5):
        print(f"  {label}: {score:.3f}")
