import cv2
import math
import mediapipe as mp
import socket
import time
import csv
from enum import Enum
from ultralytics import YOLO

source = "http://camera.local:81/stream"
# source = 0
host = "192.168.4.1" 
port = 80 

# Initialize MediaPipe Pose
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(static_image_mode=False, min_detection_confidence=0.5)
mirror = True

fp = 481  # Focal length in pixelsCV2
tb = 175  # Real-world height of the object in cm
k = 24.422  # Calibration factor for MediaPipe

# Load YOLO model
model = YOLO("best.pt")

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
        self.sentr = True
        self.host = host
        self.port = port
        self.ex_time = 0
        self.ex_data = 'C\n'.encode()
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

    def send_data(self, data, regulator='C\n'):
        verbose = False
        over_delay = (time.time() - self.interval) > .25
        if over_delay and self.data != regulator:
            if not self.sent:
                verbose = True
                self.ex_time = self.interval
                self.interval = time.time()
                if self.socket:
                    self.socket.send(self.data.encode())
                self.ex_data = self.data
                print("Mengirim ", self.data.encode())
                self.sent = True
                self.sentr = True
            elif self.data != data:
                verbose = True
                self.ex_time = self.interval
                self.interval = time.time()
                if (data != regulator): 
                    self.data = data
                    self.sent = False
                if self.sentr == True:
                    if self.socket:
                        self.socket.send(regulator.encode())
                    self.ex_data = regulator
                    print("Mengirim ", regulator.encode())
                    self.sentr = False
        return verbose

# Prepare CSV file
csv_file = open('log_belokan.csv', mode='w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['Waktu', 'Inference Time', 
        'Jarak YOLO', 'Jarak MediaPipe', 'x-mid', 'near',
        'Regulasi Waktu', 'Menyimpan', 'Terkirim',
        'Mencari', 'Keterangan'])

# Initialize video capture
cap = cv2.VideoCapture(source)
cap.set(3, 1366)
cap.set(4, 768)

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
controller.send_data('3\n') # Speed level .140

while True:
    success, frame = cap.read()
    if not success:
        break

    # frame =  cv2.rotate(frame, cv2.ROTATE_180) if source else frame
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
    results = model.track(img, classes=0, stream=True, persist=True, conf=0.7, verbose=False)
    inference_time = time.time() - start_inference_time

    lf = False
    arah = 'C\n'
    arah_log = "Stop"   # Default value
    jarak_mediapipe = 0 # Default value
    jarak_yolo = 0
    center_x = 0
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

                # print(f"YOLO Distance: {jarak_yolo:.2f} m, MediaPipe Distance: {jarak_mediapipe:.2f} m")
                cv2.putText(img, f"YOLO Distance: {jarak_yolo:.2f} m", (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(img, f"MP Distance: {jarak_mediapipe:.2f} m", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                cp_Shape = lebar_img / img.shape[1]
                center_x = (x1 + x2) // 2

                if jarak_mediapipe > 1.0:
                    if center_x < left_bound:
                        arah = 'A\n'
                        arah_log = "Kursi roda belok Kiri"
                        direction = Direction.BELIKIRI
                        posisi_terakhir = PosisiManusiaTerakhir.KIRI
                    elif center_x > right_bound:
                        arah = 'E\n'
                        arah_log = "Kursi roda belok Kanan"
                        direction = Direction.BELIKANAN
                        posisi_terakhir = PosisiManusiaTerakhir.KANAN
                    else:
                        arah = 'B\n'
                        arah_log = "Kursi roda Maju"
                        direction = Direction.MAJU
                        posisi_terakhir = PosisiManusiaTerakhir.MAJU
                else:
                    arah_log = "Menunggu jarak aman"
                    direction = Direction.DIAM
        else:
            lf = True
            if posisi_terakhir == PosisiManusiaTerakhir.KIRI:
                arah = 'A\n'
                arah_log = "Mencari manusia ke Kiri"
                direction = Direction.BELIKIRI
            elif posisi_terakhir == PosisiManusiaTerakhir.KANAN:
                arah = 'E\n'
                arah_log = "Mencari manusia ke Kanan"
                direction = Direction.BELIKANAN
            elif posisi_terakhir == PosisiManusiaTerakhir.MAJU:
                arah = 'B\n'
                arah_log = "Mencari manusia ke Depan"
                direction = Direction.MAJU
            elif posisi_terakhir == PosisiManusiaTerakhir.DIAM:
                arah_log = "Menunggu jarak aman"
                direction = Direction.DIAM

        cv2.putText(img, arah_log, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # Send and Write log to CSV
    if controller.send_data(arah): 
        waktu = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        text_lf = "luar frame" if lf else "dalam frame"
        inter_send = time.time() - controller.ex_time
        near_bound = 0 if not center_x else\
            left_bound if abs(center_x-left_bound)\
            < abs(center_x-right_bound) else right_bound
        csv_writer.writerow([waktu, inference_time,
            jarak_yolo, jarak_mediapipe, center_x, near_bound,
            inter_send, arah.encode(), controller.ex_data.encode(),
            text_lf, arah_log])
        csv_file.flush()  # Ensure data is written immediately
    
    draw_arrow(img, direction)
    cv2.imshow('Webcam', img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
csv_file.close()
