import os
from ultralytics import YOLO
import onnx
from onnx_tf.backend import prepare
import tensorflow as tf
import visualkeras

# 1. Load YOLOv11 Model
model_path = "best.pt"  # Ganti dengan model YOLOv11-mu
yolo_model = YOLO(model_path)
print(f"Model YOLOv11 berhasil dimuat dari: {model_path}")

# 2. Ekspor Model ke ONNX
onnx_path = "bestv11.onnx"
if not os.path.exists(onnx_path):
    yolo_model.export(format="onnx")  # Mengekspor ke ONNX
    print(f"Model berhasil diekspor ke ONNX: {onnx_path}")
else:
    print(f"File ONNX sudah ada: {onnx_path}")

# 3. Load Model ONNX
onnx_model = onnx.load(onnx_path)
print("Model ONNX berhasil dimuat.")

# 4. Konversi ONNX ke TensorFlow
print("Mengonversi ONNX ke TensorFlow...")
tf_model_dir = "yolo11_tf"
if not os.path.exists(tf_model_dir):
    tf_rep = prepare(onnx_model)  # Konversi ke format TensorFlow
    tf_rep.export_graph(tf_model_dir)  # Simpan model TensorFlow
    print(f"Model berhasil disimpan di: {tf_model_dir}")
else:
    print(f"Model TensorFlow sudah ada: {tf_model_dir}")

# 5. Load TensorFlow Model
tf_model = tf.saved_model.load(tf_model_dir)
print("Model TensorFlow berhasil dimuat.")

# 6. Konversi TensorFlow Model ke Keras
print("Mengonversi TensorFlow Model ke Keras...")
keras_model = tf.keras.models.load_model(tf_model_dir)
print("Model berhasil dikonversi ke Keras.")

# 7. Visualisasi dengan Visualkeras
output_file = "yolo11_visualization.png"
visualkeras.layered_view(keras_model, to_file=output_file, draw_volume=False)
print(f"Visualisasi model disimpan sebagai: {output_file}")
