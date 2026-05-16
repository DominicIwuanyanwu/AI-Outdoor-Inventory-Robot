"""
camera_scan_node.py
Session deduplication + visual obstacle feed.
Inventory saved as JSON for easy parsing.
"""
import time
import csv
import json
import re
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort
from pyzbar.pyzbar import decode

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32

class CameraScanNode(Node):
    def __init__(self):
        super().__init__('camera_scan_node')

        # Paths
        self.base = Path.home() / "inventory_project"
        self.model_path = self.base / "models" / "best.onnx"
        self.inventory_file = self.base / "inventory" / "inventory.json"   # <-- JSON
        self.scan_log_file = self.base / "inventory" / "scans_log.csv"
        self.items_file = self.base / "config" / "items.csv"

        # Model
        self.img_size = 320
        self.conf_thres = 0.35
        self.nms_thres = 0.45
        self.max_dets = 10

        # Session dedup
        self.scanned_this_session = set()
        self.scan_block_until = 0.0
        self.scan_pause_seconds = 2.0
        self.item_code_re = re.compile(r"^ITEM\d{3}$")

        # ROS pubs
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_active_pub = self.create_publisher(Bool, '/scan_active', 10)
        self.vis_pub = self.create_publisher(Float32, '/camera/visual_proximity', 10)

        # Ensure dirs/files exist
        self.ensure_files_exist()
        self.items_map = self.load_items_map(self.items_file)

        # ONNX
        if not self.model_path.exists():
            raise FileNotFoundError(f"ONNX model not found at: {self.model_path}")

        self.sess = ort.InferenceSession(
            str(self.model_path),
            providers=["CPUExecutionProvider"]
        )
        self.input_name = self.sess.get_inputs()[0].name

        # Webcam
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError("Could not open USB webcam on /dev/video0")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.get_logger().info(
            f"Webcam opened at {self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)}x"
            f"{self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}"
        )
        time.sleep(1.0)

        self.timer = self.create_timer(0.15, self.scan_loop)
        self.get_logger().info("Camera scan node: JSON inventory + visual feed")

    # ---------------------------------------------------------
    def ensure_files_exist(self):
        self.inventory_file.parent.mkdir(parents=True, exist_ok=True)
        self.scan_log_file.parent.mkdir(parents=True, exist_ok=True)

        # Create empty JSON inventory if missing
        if not self.inventory_file.exists():
            with open(self.inventory_file, "w") as f:
                json.dump({}, f, indent=2)

        # Create CSV log if missing
        if not self.scan_log_file.exists():
            with open(self.scan_log_file, "w", newline="") as f:
                csv.writer(f).writerow(["timestamp", "barcode", "item_name", "event"])

    def load_items_map(self, path: Path) -> dict:
        items = {}
        if path.exists():
            with open(path, newline="") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    items[row["barcode"].strip()] = row["item_name"].strip()
        return items

    def load_inventory(self) -> dict:
        if self.inventory_file.exists():
            try:
                with open(self.inventory_file, "r") as f:
                    return json.load(f)
            except json.JSONDecodeError:
                self.get_logger().warn("Corrupt inventory.json, starting fresh")
                return {}
        return {}

    def save_inventory(self, inv: dict):
        with open(self.inventory_file, "w") as f:
            json.dump(inv, f, indent=2)

    def log_scan(self, ts: str, barcode: str, item_name: str, event: str):
        with open(self.scan_log_file, "a", newline="") as f:
            csv.writer(f).writerow([ts, barcode, item_name, event])

    def publish_stop(self):
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        self.cmd_pub.publish(t)

    def set_scan_active(self, active: bool):
        msg = Bool()
        msg.data = active
        self.scan_active_pub.publish(msg)

    # ---------------------------------------------------------
    def scan_loop(self):
        try:
            if time.time() < self.scan_block_until:
                self.set_scan_active(True)
                self.publish_stop()
                return
            else:
                self.set_scan_active(False)

            for _ in range(2):
                self.cap.grab()

            ret, frame_bgr = self.cap.read()
            if not ret or frame_bgr is None:
                return

            h, w = frame_bgr.shape[:2]

            # Visual obstacle feed (bottom-centre of frame)
            vis_roi = frame_bgr[int(h*0.7):h, int(w*0.25):int(w*0.75)]
            if vis_roi.size > 0:
                gray = cv2.cvtColor(vis_roi, cv2.COLOR_BGR2GRAY)
                blur = cv2.GaussianBlur(gray, (5, 5), 0)
                edges = cv2.Canny(blur, 50, 150)
                edge_density = np.count_nonzero(edges) / edges.size
                vis_score = float(min(edge_density * 8.0, 1.0))
                self.vis_pub.publish(Float32(data=vis_score))

            # Barcode detection
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            lb_rgb, ratio, (padx, pady) = self.letterbox(frame_rgb, (self.img_size, self.img_size))
            inp = lb_rgb.astype(np.float32) / 255.0
            inp = np.transpose(inp, (2, 0, 1))
            inp = np.expand_dims(inp, axis=0)

            raw = self.sess.run(None, {self.input_name: inp})[0]
            dets = self.parse_yolo_output(raw, self.conf_thres)

            boxes, confs, idxs = [], [], []
            if dets:
                for (cx, cy, bw, bh, conf, cls_id) in dets:
                    x1 = cx - bw / 2
                    y1 = cy - bh / 2
                    boxes.append([x1, y1, bw, bh])
                    confs.append(conf)
                idxs = cv2.dnn.NMSBoxes(boxes, confs, self.conf_thres, self.nms_thres)
                idxs = idxs.flatten().tolist() if len(idxs) else []

            for i in idxs[:self.max_dets]:
                x, y, bw, bh = boxes[i]
                x1 = int(self.clamp((x - padx) / ratio, 0, w - 1))
                y1 = int(self.clamp((y - pady) / ratio, 0, h - 1))
                x2 = int(self.clamp((x + bw - padx) / ratio, 0, w - 1))
                y2 = int(self.clamp((y + bh - pady) / ratio, 0, h - 1))

                pad = 10
                x1p = self.clamp(x1 - pad, 0, w - 1)
                y1p = self.clamp(y1 - pad, 0, h - 1)
                x2p = self.clamp(x2 + pad, 0, w - 1)
                y2p = self.clamp(y2 + pad, 0, h - 1)

                roi = frame_bgr[y1p:y2p, x1p:x2p]
                if roi.size == 0 or roi.shape[0] < 10 or roi.shape[1] < 10:
                    continue

                codes = decode(roi)
                for code in codes:
                    code_str = code.data.decode("utf-8", errors="ignore").strip()
                    if not self.item_code_re.match(code_str):
                        continue
                    if code_str in self.scanned_this_session:
                        continue

                    self.scanned_this_session.add(code_str)
                    self.handle_detected_code(code_str)
                    return

        except Exception as e:
            self.get_logger().warn(f"Camera scan loop error: {e}")

    def handle_detected_code(self, code_str: str):
        self.set_scan_active(True)
        self.publish_stop()

        inventory = self.load_inventory()
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        item_name = self.items_map.get(code_str, "UNKNOWN")

        if code_str not in inventory:
            inventory[code_str] = {
                "item_name": item_name,
                "count": 0,
                "last_seen": ""
            }

        inventory[code_str]["item_name"] = item_name
        inventory[code_str]["count"] += 1
        inventory[code_str]["last_seen"] = ts

        self.save_inventory(inventory)
        self.log_scan(ts, code_str, item_name, "count_increment_session")

        self.get_logger().info(
            f"SCANNED (SESSION LOCK): {code_str} | {item_name} | count={inventory[code_str]['count']}"
        )

        self.scan_block_until = time.time() + self.scan_pause_seconds

    # ---------------------------------------------------------
    def clamp(self, value, low, high):
        return max(low, min(high, int(value)))

    def letterbox(self, image, new_shape=(320, 320), color=(114, 114, 114)):
        h, w = image.shape[:2]
        nh, nw = new_shape
        r = min(nw / w, nh / h)
        new_unpad = (int(round(w * r)), int(round(h * r)))
        dw = (nw - new_unpad[0]) / 2
        dh = (nh - new_unpad[1]) / 2
        resized = cv2.resize(image, new_unpad, interpolation=cv2.INTER_LINEAR)
        top = int(round(dh - 0.1))
        bottom = int(round(dh + 0.1))
        left = int(round(dw - 0.1))
        right = int(round(dw + 0.1))
        out = cv2.copyMakeBorder(resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
        return out, r, (left, top)

    def parse_yolo_output(self, raw_output: np.ndarray, conf_thres: float):
        out = raw_output
        if isinstance(out, (list, tuple)):
            out = out[0]
        out = np.array(out)
        if out.ndim == 3 and out.shape[0] == 1:
            out = out[0]
        if out.ndim != 2:
            return []
        if out.shape[0] in (5, 6, 7, 84, 85, 86) and out.shape[1] > out.shape[0]:
            out = out.T
        if out.shape[1] < 5:
            return []
        dets = []
        for row in out:
            cx, cy, w, h, conf = float(row[0]), float(row[1]), float(row[2]), float(row[3]), float(row[4])
            cls_id = 0
            if row.shape[0] > 5:
                class_probs = row[5:]
                cls_id = int(np.argmax(class_probs))
                conf = conf * float(class_probs[cls_id])
            if conf >= conf_thres:
                dets.append((cx, cy, w, h, conf, cls_id))
        return dets

    def destroy_node(self):
        if hasattr(self, "cap") and self.cap is not None:
            self.cap.release()
        super().destroy_node()

def main():
    rclpy.init()
    node = CameraScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
