#!/usr/bin/env python3
"""
YOLOv11n Webcam Detection - Direct Run
"""

from ultralytics import YOLO
import cv2

print("Loading YOLOv11n model...")
model = YOLO('best.pt')
print("Model loaded successfully!\n")

print("Starting webcam detection...")
print("Press 'q' to quit\n")

# Buka webcam
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Cannot open webcam!")
    exit()

# Set resolusi webcam (opsional)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

frame_count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Cannot read frame from webcam!")
        break
    
    # Jalankan inferensi YOLOv11n
    results = model(frame, conf=0.25, verbose=False)
    
    # Gambar hasil deteksi pada frame
    annotated_frame = results[0].plot()
    
    # Tampilkan jumlah objek yang terdeteksi
    num_detections = len(results[0].boxes)
    cv2.putText(annotated_frame, f"Objects: {num_detections}", 
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    # Tampilkan frame
    cv2.imshow('YOLOv11n Webcam Detection - Press Q to Quit', annotated_frame)
    
    # Print deteksi setiap 30 frame
    if frame_count % 30 == 0 and num_detections > 0:
        print(f"Frame {frame_count}: Detected {num_detections} objects:")
        for box in results[0].boxes:
            class_id = int(box.cls[0])
            confidence = float(box.conf[0])
            class_name = model.names[class_id]
            print(f"  - {class_name}: {confidence:.2f}")
    
    frame_count += 1
    
    # Tekan 'q' untuk keluar
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
print("\nWebcam detection stopped.")
