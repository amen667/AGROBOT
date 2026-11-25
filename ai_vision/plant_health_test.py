import cv2
import torch
from PIL import Image
from transformers import AutoModelForImageClassification, MobileNetV2ImageProcessor

MODEL_NAME = "linkanjarad/mobilenet_v2_1.0_224-plant-disease-identification"
CAMERA_INDEX = 0
CROP_FILTER = "tomato"
CROP_ZOOM = 0.75
OUTPUT_IMAGE = "/home/amen/Desktop/AGROBOT/ai_vision/captured_leaf.jpg"


def capture_frame(camera_index: int = 0, save_path: str = OUTPUT_IMAGE, zoom: float = 1.0) -> Image.Image:
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera at index {camera_index}")
    ret, frame_bgr = cap.read()
    cap.release()
    if not ret or frame_bgr is None:
        raise RuntimeError("Failed to capture frame from camera")
    if 0.0 < zoom < 1.0:
        h, w, _ = frame_bgr.shape
        nh, nw = int(h * zoom), int(w * zoom)
        y0, x0 = (h - nh) // 2, (w - nw) // 2
        frame_bgr = frame_bgr[y0:y0 + nh, x0:x0 + nw]
    cv2.imwrite(save_path, frame_bgr)
    return Image.fromarray(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))


def classify_tomato(image: Image.Image, crop: str = CROP_FILTER, top_k: int = 5):
    model = AutoModelForImageClassification.from_pretrained(MODEL_NAME).eval()
    processor = MobileNetV2ImageProcessor()
    idxs = [int(k) for k, v in model.config.id2label.items() if crop.lower() in v.lower()]
    inputs = processor(images=image, return_tensors="pt")
    with torch.no_grad():
        probs = torch.softmax(model(**inputs).logits, dim=-1)[0][idxs]
        probs = probs / probs.sum()
    ranked = sorted(((model.config.id2label[i], float(p.item())) for i, p in zip(idxs, probs)), key=lambda x: x[1], reverse=True)
    p_healthy = sum(s for lbl, s in ranked if "healthy" in lbl.lower())
    return p_healthy, ranked[:top_k]


if __name__ == "__main__":
    img = capture_frame(CAMERA_INDEX, OUTPUT_IMAGE, zoom=CROP_ZOOM)
    p_healthy, top = classify_tomato(img)
    print(f"Saved capture to: {OUTPUT_IMAGE} (zoom={CROP_ZOOM})")
    print(f"p_healthy: {p_healthy:.3f} | p_diseased: {1.0 - p_healthy:.3f}")
    for label, score in top:
        print(f"  {label}: {score:.3f}")
