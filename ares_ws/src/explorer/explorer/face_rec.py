import os
import cv2
import numpy as np
import joblib
from sklearn.svm import SVC
from insightface.app import FaceAnalysis
from datetime import datetime

class ARESLiveTracker:
    def __init__(self, known_dir="ROS\ARES\\ares_ws\src\known_faces", model_path="face_svm_insight.pkl"):
        self.known_dir = known_dir
        self.model_path = model_path
        self.model = None
        self.class_names = []
        self.app = FaceAnalysis(name='buffalo_l', providers=['CPUExecutionProvider'])
        self.app.prepare(ctx_id=0)

    def get_embedding(self, img):
        faces = self.app.get(img)
        return [face.embedding for face in faces], [face.bbox.astype(int) for face in faces]

    def train(self):
        embeddings, labels = [], []

        for person_name in os.listdir(self.known_dir):
            person_dir = os.path.join(self.known_dir, person_name)
            if not os.path.isdir(person_dir):
                continue

            for img_name in os.listdir(person_dir):
                img_path = os.path.join(person_dir, img_name)
                img = cv2.imread(img_path)
                if img is None:
                    continue
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                embs, _ = self.get_embedding(img)
                if not embs:
                    print(f"[WARN] No face in {img_path}")
                    continue
                embeddings.append(embs[0])
                labels.append(person_name)

        print(f"[INFO] Training on {len(embeddings)} embeddings")
        clf = SVC(probability=True)
        clf.fit(embeddings, labels)
        joblib.dump((clf, list(set(labels))), self.model_path)
        self.model = clf
        print("[INFO] Model trained and saved.")

    def load_model(self):
        self.model, self.class_names = joblib.load(self.model_path)

    def run_live_tracking(self):
        if self.model is None:
            self.load_model()

        cap = cv2.VideoCapture(0)
        horizontal_fov = 60.0  # degrees
        vertical_fov = 40.0    # degrees

        frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        center_x = frame_width // 2
        center_y = frame_height // 2

        log_file = open("face_angles_log.txt", "a")

        print("[INFO] ARES is live. Press 'q' to terminate.")

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            embeddings, bboxes = self.get_embedding(rgb)

            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            for emb, bbox in zip(embeddings, bboxes):
                name = "Unknown"
                prob = 0.0

                try:
                    pred = self.model.predict([emb])[0]
                    prob = np.max(self.model.predict_proba([emb]))
                    name = pred if prob > 0.7 else "Unknown"
                except:
                    pass

                # Bounding box center
                cx = (bbox[0] + bbox[2]) // 2
                cy = (bbox[1] + bbox[3]) // 2

                # Calculate yaw (horizontal) and pitch (vertical) angles
                dx = cx - center_x
                dy = cy - center_y
                yaw = (dx / frame_width) * horizontal_fov
                pitch = -(dy / frame_height) * vertical_fov  # negative = upward

                # Visuals
                cv2.rectangle(frame, tuple(bbox[:2]), tuple(bbox[2:]), (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                label = f"{name} ({prob:.2f}) Y:{yaw:+.1f}° P:{pitch:+.1f}°"
                cv2.putText(frame, label, (bbox[0], bbox[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                # Log entry
                log_file.write(f"{timestamp},{name},{yaw:+.2f},{pitch:+.2f}\n")

            cv2.imshow("ARES Live Face Tracker", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        log_file.close()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    tracker = ARESLiveTracker()
    
    # Train once (only rerun if faces change)
    tracker.train()
    
    # Start real-time recognition and tracking
    tracker.run_live_tracking()
