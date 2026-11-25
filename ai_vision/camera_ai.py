import cv2
from PIL import Image
from plant_health_tflite import PlantClassifier

classifier = PlantClassifier()

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

try:
    if not cap.isOpened():
        raise RuntimeError("Camera unavailable")

    print("Camera connected")

    for _ in range(10):
        ret, frame = cap.read()

    if not ret or frame is None:
        raise RuntimeError("Capture failed")
finally:
    cap.release()

cv2.imwrite("captured_plant.jpg", frame)

image = Image.fromarray(
    cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
)

results, elapsed, tomato_mass = classifier.classify(image)

label, confidence = results[0]

print("\n--- INSPECTION RESULT ---")
print("Prediction:", label)
print(f"Tomato-relative score: {confidence * 100:.2f}%")
print(f"Inference time: {elapsed:.2f} ms")
print("Inspection complete")
print("Ready for movement request")
