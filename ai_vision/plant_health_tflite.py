import json
import time
import sys
import numpy as np
from pathlib import Path
from PIL import Image
from ai_edge_litert.interpreter import Interpreter

MODEL_DIR = Path(__file__).resolve().parent / "models"
MODEL_PATH = MODEL_DIR / "plant_health_model_float32.tflite"


def preprocess(image):
    image = image.convert("RGB")

    w, h = image.size
    scale = 256 / min(w, h)

    image = image.resize(
        (int(w * scale), int(h * scale)),
        Image.Resampling.BILINEAR
    )

    w, h = image.size
    x = (w - 224) // 2
    y = (h - 224) // 2

    image = image.crop((x, y, x + 224, y + 224))

    image = np.asarray(image, dtype=np.float32)
    image = image / 255.0
    image = (image - 0.5) / 0.5

    return np.expand_dims(image, axis=0)


class PlantClassifier:

    def __init__(self):
        self.interpreter = Interpreter(
            model_path=str(MODEL_PATH),
            num_threads=4
        )
        self.interpreter.allocate_tensors()

        self.input_index = self.interpreter.get_input_details()[0]["index"]
        self.output_index = self.interpreter.get_output_details()[0]["index"]

        with open(MODEL_DIR / "labels.json") as f:
            self.labels = json.load(f)

    def classify(self, image):
        data = preprocess(image)

        self.interpreter.set_tensor(self.input_index, data)

        start = time.perf_counter()
        self.interpreter.invoke()
        elapsed = (time.perf_counter() - start) * 1000

        logits = self.interpreter.get_tensor(self.output_index)[0]

        if not np.isfinite(logits).all():
            raise RuntimeError("Invalid model output")

        probs = np.exp(logits - np.max(logits))
        probs /= probs.sum()

        indices = [
            int(i) for i, label in self.labels.items()
            if "tomato" in label.lower()
        ]

        tomato_probs = probs[indices]
        tomato_mass = float(tomato_probs.sum())
        tomato_probs /= tomato_mass

        ranked = sorted(
            [(self.labels[str(i)], float(p))
             for i, p in zip(indices, tomato_probs)],
            key=lambda x: x[1],
            reverse=True
        )

        return ranked, elapsed, tomato_mass


if __name__ == "__main__":
    classifier = PlantClassifier()
    image = Image.open(sys.argv[1])

    results, elapsed, tomato_mass = classifier.classify(image)

    print(f"Inference time: {elapsed:.2f} ms")
    print(f"Total tomato class probability: {tomato_mass:.3f}")
    print("\nTop tomato predictions:")

    for label, score in results[:5]:
        print(f"{label}: {score * 100:.2f}%")
