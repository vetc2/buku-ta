import cv2
import math
import mediapipe as mp
import socket
import time
import csv
from enum import Enum
from ultralytics import YOLO

source = "http://192.168.4.1:81/stream"
# source = 0
host = "192.168.4.1" 
port = 80 

# Initialize MediaPipe Pose
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(static_image_mode=False, min_detection_confidence=0.7)
mirror = True

fp = 481  # Focal length in pixelsCV2
tb = 175  # Real-world height of the object in cm
k = 24.422  # Calibration factor for MediaPipe

# Load YOLO model
model = YOLO("yolov10m.pt")

def hitung_jarak(tinggi_bounding_box, focal_length_pixel, tinggi_objek_nyata):
    if tinggi_bounding_box == 0:
        return float('inf')
    jarak = (tinggi_objek_nyata * focal_length_pixel) / tinggi_bounding_box
    return jarak / 100  # Convert to meters

def hitung_jarak_euclidean(landmark1, landmark2, lebar_img):
    return math.sqrt((landmark1.x - landmark2.x) ** 2 + (landmark1.y - landmark2.y) ** 2) * lebar_img

def hitung_lebar_mediapipe(pose_results, lebar_img):
    if pose_results.pose_landmarks:
        bahu_kiri = pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER]
        bahu_kanan = pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER]
        jarak_pix = hitung_jarak_euclidean(bahu_kiri, bahu_kanan, lebar_img)
        return (k / jarak_pix) * 10
    return 0

class SocketCommunicator:
    def __init__(self, host, port):
        self.socket = None
        self.interval = 0
        self.data = 'C\n'.encode()
        self.sent = True
        self.host = host
        self.port = port
        self.create_socket()

    def create_socket(self):
        print("Loading...", end="", flush=True)
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(1)
        try:
            s.connect((self.host, self.port))
            print()  
            self.socket = s
        except socket.error:
            print(".", end="", flush=True)
            time.sleep(1)

    def send_data(self, data, ms=500):
        current_time = time.time()
        if self.socket:
            if data != self.data:
                self.interval = time.time()
                self.socket.send('C\n'.encode())
                self.sent = False
                self.data = data
            elif (current_time - self.interval) * 1000 < ms and not self.sent:
                self.socket.send(self.data.encode())
                self.sent = True

# Prepare CSV file
csv_file = open('log_belokan.csv', mode='w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['Waktu', 'Inference Time', 'Jarak YOLO', 'Jarak MediaPipe', 'Arah'])

# Initialize video capture
cap = cv2.VideoCapture(source)
# \

class Direction(Enum):
    DIAM = 0
    MAJU = 1
    BELIKANAN = 2
    BELIKIRI = 3

class PosisiManusiaTerakhir(Enum):
    DIAM = 0
    KIRI = 1
    KANAN = 2
    MAJU = 3

posisi_terakhir = PosisiManusiaTerakhir.DIAM

def draw_arrow(img, direction):
    height, width, _ = img.shape
    start_point = (width // 2, height - 50)  # Start point of arrow
    if direction == Direction.MAJU:
        end_point = (width // 2, height - 150)
    elif direction == Direction.BELIKANAN:
        end_point = (width // 2 + 100, height - 100)
    elif direction == Direction.BELIKIRI:
        end_point = (width // 2 - 100, height - 100)
    else:
        end_point = start_point

    color = (0, 255, 0)
    thickness = 5
    cv2.arrowedLine(img, start_point, end_point, color, thickness)


# Initialize the wheelchair controller
controller = SocketCommunicator(host, port)
controller.send_data('2\n') # Speed level .140

while True:
    success, frame = cap.read()
    if not success:
        break

    frame = cv2.rotate(frame,cv2.ROTATE_180)
    img = cv2.flip(frame,1) if mirror else frame

    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    height, width, _ = img.shape

    # Define screen partitions
    left_bound = width // 3
    right_bound = 2 * (width // 3)

    # Draw partition lines
    cv2.line(img, (left_bound, 0), (left_bound, height), (0, 255, 0), 2)
    cv2.line(img, (right_bound, 0), (right_bound, height), (0, 255, 0), 2)

    start_inference_time = time.time()
    results = model.track(img, classes=0, stream=True, persist=True, conf=0.7)
    inference_time = time.time() - start_inference_time

    arah = 'C\n'
    arah_log = "Stop"   # Default value
    jarak_mediapipe = 0 # Default value
    jarak_yolo = 0
    direction = Direction.DIAM  # Default direction

    for r in results:
        min_id = None
        min_box = None
        boxes = r.boxes
        for box in boxes:
            idex = box.id
            if min_id is None or idex < min_id:
                min_id = idex
                min_box = box

        if min_id:
            x1, y1, x2, y2 = map(int, min_box.xyxy[0])
            tinggi_bounding_box = y2 - y1

            # Draw bounding box
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.rectangle(img, (x1, y1), (x1+20, y1+30), (255, 0, 0), -1)
            cv2.putText(img, str(int(min_id)), (x1, y1 + 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            # Calculate distance using YOLO
            jarak_yolo = hitung_jarak(tinggi_bounding_box, fp, tb)

            # Process with MediaPipe
            person_img_rgb = cv2.cvtColor(img[y1:y2, x1:x2], cv2.COLOR_BGR2RGB)
            pose_results = pose.process(person_img_rgb)

            if pose_results.pose_landmarks:
                # Draw pose landmarks
                mp.solutions.drawing_utils.draw_landmarks(img[y1:y2, x1:x2], pose_results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

                lebar_img = person_img_rgb.shape[1]
                jarak_mediapipe = hitung_lebar_mediapipe(pose_results, lebar_img)

                print(f"YOLO Distance: {jarak_yolo:.2f} m, MediaPipe Distance: {jarak_mediapipe:.2f} m")
                cv2.putText(img, f"YOLO Distance: {jarak_yolo:.2f} m", (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(img, f"MP Distance: {jarak_mediapipe:.2f} m", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                cp_Shape = lebar_img / img.shape[1]
                print(cp_Shape)
                center_x = (x1 + x2) // 2

                if jarak_mediapipe > 1.0:
                    if cp_Shape > 0.5 :
                        print("Kursi roda berhenti")
                        # arah = 'C\n'
                        cv2.putText(img, "Kursi roda berhenti", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                        arah_log = "BatasDekat"
                        direction = Direction.DIAM
                        posisi_terakhir = PosisiManusiaTerakhir.DIAM
                    elif center_x > right_bound:
                        print("Kursi roda belok Kiri")
                        arah = 'A\n'
                        cv2.putText(img, "Kursi roda belok Kiri", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                        arah_log = "BelokKiri"
                        direction = Direction.BELIKIRI
                        posisi_terakhir = PosisiManusiaTerakhir.KIRI
                    elif center_x < left_bound:
                        print("Kursi roda belok Kanan")
                        arah = 'E\n'
                        cv2.putText(img, "Kursi roda belok Kanan", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                        arah_log = "BelokKanan"
                        direction = Direction.BELIKANAN
                        posisi_terakhir = PosisiManusiaTerakhir.KANAN
                    else:
                        arah = 'B\n'
                        print("Kursi roda Maju")
                        cv2.putText(img,  "Kursi roda Maju", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                        arah_log = "Maju"
                        direction = Direction.MAJU
                        posisi_terakhir = PosisiManusiaTerakhir.MAJU
                    # delay(200)
                else:
                    # delay(200)
                    arah_log = "Stop"
                    direction = Direction.MAJU
        else:
            if posisi_terakhir == PosisiManusiaTerakhir.KIRI:
                print("Mencari manusia ke Kiri")
                arah = 'A\n'
                cv2.putText(img, "Mencari manusia ke Kiri", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                direction = Direction.BELIKIRI
            elif posisi_terakhir == PosisiManusiaTerakhir.KANAN:
                print("Mencari manusia ke Kanan")
                arah = 'E\n'
                cv2.putText(img, "Mencari manusia ke Kanan", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                direction = Direction.BELIKANAN
            elif posisi_terakhir == PosisiManusiaTerakhir.MAJU:
                print("Mencari manusia ke Depan")
                arah = 'B\n'
                cv2.putText(img, "Mencari manusia ke Depan", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                direction = Direction.MAJU
            elif posisi_terakhir == PosisiManusiaTerakhir.DIAM:
                print("Menunggu jarak aman")
                # arah = 'C\n'
                cv2.putText(img, "Menunggu jarak aman", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                direction = Direction.MAJU
            

    # Write log to CSV
    waktu = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
    csv_writer.writerow([waktu, inference_time, jarak_yolo, jarak_mediapipe, arah_log])
    csv_file.flush()  # Ensure data is written immediately

    controller.send_data(arah)
    draw_arrow(img, direction)

    cv2.imshow('Webcam', img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
csv_file.close()
