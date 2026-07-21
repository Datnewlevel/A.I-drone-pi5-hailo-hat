import os
os.environ["GST_PLUGIN_FEATURE_RANK"] = "vaapidecodebin:NONE"

import time
import math
import threading
import gi
gi.require_version("Gst", "1.0")
import cv2
import hailo
from gi.repository import Gst

from hailo_apps.python.pipeline_apps.detection.detection_pipeline import GStreamerDetectionApp
from hailo_apps.python.core.common.buffer_utils import get_caps_from_pad, get_numpy_from_buffer
from hailo_apps.python.core.common.hailo_logger import get_logger
from hailo_apps.python.core.gstreamer.gstreamer_app import app_callback_class

from pymavlink import mavutil
from simple_pid import PID

hailo_logger = get_logger(__name__)


# MAVLINK CONNECTION CONFIG
MAVLINK_PORT = 'udp:127.0.0.1:14551'
MAVLINK_BAUD = 921600     # only used for direct UART connection

# PID CONTROL
PID_KP = 0.04
PID_KI = 0.0
PID_KD = 0.005

MAX_YAW_RATE_DEG = 20.0
DEADZONE_PX = 30
DETECTION_TIMEOUT = 1.0
CONTROL_RATE_HZ = 10
HEARTBEAT_RATE_HZ = 1        # required, or ArduPilot ignores commands
TARGET_LABEL = "tag"

# VIDEO STREAM (MJPEG over TCP) 
STREAM_ENABLED = True
STREAM_PORT = 5600
STREAM_PRESET = 'low'        # 'low' | 'medium' | 'high'
DRAW_FPS = 10

# LOCAL RECORDING FOR DATASET 
RECORD_ENABLED = True
RECORD_DIR = '/home/pi/dataset/recordings'
RECORD_QUALITY = 90


# VIDEO STREAMER (MJPEG over TCP) + independent local recording
class VideoStreamer:
    PRESETS = {
        "low":    {"width": 640,  "height": 360, "fps": 15, "quality": 50},
        "medium": {"width": 854,  "height": 480, "fps": 20, "quality": 70},
        "high":   {"width": 1280, "height": 720, "fps": 25, "quality": 80},
    }

    def __init__(self, port=5600, preset="medium", record=False,
                 record_dir="/home/pi/dataset/recordings", record_quality=90):
        Gst.init(None)
        cfg = self.PRESETS[preset]
        self.stream_w, self.stream_h = cfg["width"], cfg["height"]
        self.fps = cfg["fps"]
        self.stream_quality = cfg["quality"]
        self.record = record
        self.record_dir = record_dir
        self.record_quality = record_quality
        self.record_path = None
        self.port = port

        # Pipeline is built lazily on first frame to know the native resolution
        self.pipeline = None
        self.appsrc = None
        self.width = None
        self.height = None
        self._ready = False

        self.frame_count = 0
        self._t0 = None
        self._last_pts = 0
        self._latest = None
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._push_loop, daemon=True)
        self._thread.start()

        print(f"[Stream] Waiting for first frame to build pipeline... "
              f"(stream {self.stream_w}x{self.stream_h} q{self.stream_quality}, "
              f"record={'ON q'+str(self.record_quality) if record else 'OFF'})")

    def _build_pipeline(self, native_w, native_h):
        self.width, self.height = native_w, native_h

        src_branch = (
            f"appsrc name=src is-live=true block=false format=time "
            f"caps=video/x-raw,format=BGR,width={native_w},height={native_h},framerate={self.fps}/1 ! "
            f"queue max-size-buffers=2 leaky=downstream ! "
        )

        if self.record:
            os.makedirs(self.record_dir, exist_ok=True)
            ts = time.strftime("%Y%m%d_%H%M%S")
            self.record_path = os.path.join(self.record_dir, f"detect_{ts}.mkv")
            pipeline_str = (
                src_branch +
                "tee name=t "
                # Record branch: native resolution, high quality
                "t. ! queue max-size-buffers=60 leaky=downstream ! "
                "videoconvert ! video/x-raw,format=I420 ! "
                f"jpegenc quality={self.record_quality} ! "
                "matroskamux streamable=true ! "
                f"filesink location={self.record_path} sync=false "
                # Stream branch: downscaled, low quality for 4G
                "t. ! queue max-size-buffers=4 leaky=downstream ! "
                f"videoscale ! video/x-raw,width={self.stream_w},height={self.stream_h} ! "
                "videoconvert ! video/x-raw,format=I420 ! "
                f"jpegenc quality={self.stream_quality} ! "
                "matroskamux streamable=true ! "
                f"tcpserversink host=0.0.0.0 port={self.port} sync=false async=false"
            )
        else:
            pipeline_str = (
                src_branch +
                f"videoscale ! video/x-raw,width={self.stream_w},height={self.stream_h} ! "
                "videoconvert ! video/x-raw,format=I420 ! "
                f"jpegenc quality={self.stream_quality} ! "
                "matroskamux streamable=true ! "
                f"tcpserversink host=0.0.0.0 port={self.port} sync=false async=false"
            )

        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsrc = self.pipeline.get_by_name("src")
        self.pipeline.set_state(Gst.State.PLAYING)
        self._ready = True

        print(f"[Stream] TCP {self.port}  native {native_w}x{native_h} "
              f"-> stream {self.stream_w}x{self.stream_h} q{self.stream_quality} @ {self.fps}fps")
        if self.record:
            print(f"[Record] {native_w}x{native_h} q{self.record_quality} -> {self.record_path}")

    def update_frame(self, frame_bgr):
        with self._lock:
            self._latest = frame_bgr

    def _push_loop(self):
        period = 1.0 / self.fps
        while self._running:
            t0 = time.time()
            with self._lock:
                frame = self._latest
                self._latest = None
            if frame is not None:
                try:
                    self._push(frame)
                except Exception as e:
                    print(f"[Stream] push error: {e}")
            elapsed = time.time() - t0
            if elapsed < period:
                time.sleep(period - elapsed)

    def _push(self, frame_bgr):
        if not self._ready:
            h, w = frame_bgr.shape[:2]
            self._build_pipeline(w, h)

        if frame_bgr.shape[1] != self.width or frame_bgr.shape[0] != self.height:
            frame_bgr = cv2.resize(frame_bgr, (self.width, self.height))

        data = frame_bgr.tobytes()
        buf = Gst.Buffer.new_allocate(None, len(data), None)
        buf.fill(0, data)

        # PTS based on real elapsed time, not assumed fps
        now = time.time()
        if self._t0 is None:
            self._t0 = now
        pts = int((now - self._t0) * Gst.SECOND)
        buf.pts = pts
        gap = pts - self._last_pts
        buf.duration = gap if gap > 0 else (Gst.SECOND // max(1, DRAW_FPS))
        self._last_pts = pts

        self.frame_count += 1
        self.appsrc.emit("push-buffer", buf)

    def stop(self):
        self._running = False
        try:
            if self._thread.is_alive():
                self._thread.join(timeout=1.0)
        except Exception:
            pass
        if self.record and self.appsrc is not None:
            try:
                self.appsrc.emit("end-of-stream")
                bus = self.pipeline.get_bus()
                bus.timed_pop_filtered(
                    3 * Gst.SECOND,
                    Gst.MessageType.EOS | Gst.MessageType.ERROR
                )
            except Exception as e:
                print(f"[Record] EOS error: {e}")
        try:
            if self.pipeline is not None:
                self.pipeline.set_state(Gst.State.NULL)
        except Exception:
            pass
        if self.record and self.record_path:
            print(f"[Record] Saved clip: {self.record_path}")


# DRONE CONTROLLER (MAVLink)

class DroneController:
    def __init__(self):
        if MAVLINK_PORT.startswith(('udp', 'tcp')):
            print(f"[Drone] Connecting {MAVLINK_PORT} (network)...")
            self.master = mavutil.mavlink_connection(MAVLINK_PORT)
        else:
            print(f"[Drone] Connecting {MAVLINK_PORT} @ {MAVLINK_BAUD} (serial)...")
            self.master = mavutil.mavlink_connection(MAVLINK_PORT, baud=MAVLINK_BAUD)

        print("[Drone] Waiting for FC heartbeat...")
        self.master.wait_heartbeat()
        print(f"[Drone] Heartbeat OK. SysID={self.master.target_system} CompID={self.master.target_component}")

        self.pid = PID(PID_KP, PID_KI, PID_KD, setpoint=0)
        self.pid.output_limits = (-MAX_YAW_RATE_DEG, MAX_YAW_RATE_DEG)

        self.last_detection_time = 0
        self.current_x_error = None
        self.current_yaw_rate = 0.0
        self.current_mode = "UNKNOWN"
        self.lock = threading.Lock()
        self.running = True

        threading.Thread(target=self._heartbeat_loop, daemon=True).start()
        threading.Thread(target=self._receiver_loop, daemon=True).start()
        threading.Thread(target=self._control_loop, daemon=True).start()

    def update_target(self, x_error_px):
        with self.lock:
            self.current_x_error = x_error_px
            self.last_detection_time = time.time()

    def _heartbeat_loop(self):
        period = 1.0 / HEARTBEAT_RATE_HZ
        while self.running:
            try:
                self.master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
            except Exception as e:
                print(f"[HB] Error: {e}")
            time.sleep(period)

    def _receiver_loop(self):
        while self.running:
            try:
                msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                if msg is None:
                    continue
                # Ignore heartbeats from other GCS (e.g. Mission Planner)
                if msg.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
                    continue
                mode = mavutil.mode_string_v10(msg)
                if mode != self.current_mode:
                    print(f"[Drone] Mode changed: {self.current_mode} -> {mode}")
                    self.current_mode = mode
            except Exception as e:
                print(f"[RX] Error: {e}")
                time.sleep(0.5)

    def _send_yaw_rate(self, yaw_rate_deg_per_sec):
        yaw_rate_rad = math.radians(yaw_rate_deg_per_sec)
        # bit 11 (yaw_rate) = 0 -> use yaw_rate; bit 10 (yaw) = 1 -> ignore absolute yaw
        type_mask = 0b0000010111000111   # 0x05C7

        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            type_mask,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0,
            yaw_rate_rad
        )
        self.current_yaw_rate = yaw_rate_deg_per_sec

    def _control_loop(self):
        period = 1.0 / CONTROL_RATE_HZ
        while self.running:
            t0 = time.time()
            try:
                if self.current_mode != "GUIDED":
                    self.pid.reset()
                    self.current_yaw_rate = 0.0
                    time.sleep(period)
                    continue

                with self.lock:
                    x_err = self.current_x_error
                    last_t = self.last_detection_time

                if x_err is None or (time.time() - last_t) > DETECTION_TIMEOUT:
                    self._send_yaw_rate(0)
                    time.sleep(period)
                    continue

                if abs(x_err) < DEADZONE_PX:
                    self._send_yaw_rate(0)
                    time.sleep(period)
                    continue

                yaw_rate = -self.pid(x_err)
                self._send_yaw_rate(yaw_rate)
                print(f"[CTRL] mode=GUIDED  x_err={x_err:+.0f}px  yaw_rate={yaw_rate:+.2f}°/s")

            except Exception as e:
                print(f"[CTRL] Error: {e}")

            elapsed = time.time() - t0
            if elapsed < period:
                time.sleep(period - elapsed)

    def shutdown(self):
        print("[Drone] Shutting down...")
        self.running = False
        try:
            self._send_yaw_rate(0)
            time.sleep(0.2)
            self.master.close()
        except Exception as e:
            print(f"[Drone] Shutdown error: {e}")


drone_ctrl = DroneController()
video_streamer = (
    VideoStreamer(STREAM_PORT, STREAM_PRESET,
                  record=RECORD_ENABLED, record_dir=RECORD_DIR,
                  record_quality=RECORD_QUALITY)
    if STREAM_ENABLED else None
)

# HAILO CALLBACK + VISUALIZATION
class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()
        self.fps_t0 = time.time()
        self.fps_count = 0
        self.fps = 0.0
        self.last_frame_time = 0.0

    def tick_fps(self):
        self.fps_count += 1
        now = time.time()
        if now - self.fps_t0 >= 1.0:
            self.fps = self.fps_count / (now - self.fps_t0)
            self.fps_count = 0
            self.fps_t0 = now


def app_callback(element, buffer, user_data):
    if buffer is None:
        return

    user_data.tick_fps()

    pad = element.get_static_pad("src")
    format, width, height = get_caps_from_pad(pad)
    if width is None:
        return

    image_center_x = width / 2
    image_center_y = height / 2

    # Runs every frame: detection + x_error + MAVLink command
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

    best = None
    best_conf = 0
    for d in detections:
        if d.get_label() == TARGET_LABEL and d.get_confidence() > best_conf:
            best = d
            best_conf = d.get_confidence()

    x_error = 0.0
    locked = False
    bbox_px = None

    if best is not None:
        bbox = best.get_bbox()
        x1 = int(bbox.xmin() * width)
        y1 = int(bbox.ymin() * height)
        x2 = int((bbox.xmin() + bbox.width()) * width)
        y2 = int((bbox.ymin() + bbox.height()) * height)
        center_x = int((x1 + x2) / 2)
        center_y = int((y1 + y2) / 2)
        x_error = center_x - image_center_x
        locked = True
        bbox_px = (x1, y1, x2, y2, center_x, center_y)

        drone_ctrl.update_target(x_error)

    # Throttled: extract frame + draw + stream at DRAW_FPS
    now = time.time()
    if now - user_data.last_frame_time < 1.0 / DRAW_FPS:
        return
    user_data.last_frame_time = now

    if not (user_data.use_frame and format is not None):
        return

    frame = get_numpy_from_buffer(buffer, format, width, height)
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    red = (0, 0, 255)
    green = (0, 255, 0)

    if bbox_px is not None:
        x1, y1, x2, y2, center_x, center_y = bbox_px
        cv2.rectangle(frame, (x1, y1), (x2, y2), red, 2)
        cv2.putText(frame, f"{TARGET_LABEL} {best_conf*100:.1f}%",
                    (x1, max(y1 - 8, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, red, 2)
        cv2.circle(frame, (center_x, center_y), 8, red, -1)
        cv2.circle(frame, (int(image_center_x), int(image_center_y)), 8, green, -1)
        cv2.line(frame, (int(image_center_x), int(image_center_y)),
                 (center_x, center_y), (255, 0, 0), 2)

    mode = drone_ctrl.current_mode
    yaw_out = drone_ctrl.current_yaw_rate

    cv2.putText(frame, f"fps: {user_data.fps:.1f}   yaw_rate: {yaw_out:+.2f}",
                (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, red, 2)
    cv2.putText(frame, f"mode: {mode}",
                (10, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.6, red, 2)
    cv2.putText(frame, f"x_delta: {x_error:+.1f}   target: {'LOCKED' if locked else 'SEARCHING'}",
                (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, red, 2)

    bar_x = width - 30
    bar_top = 60
    bar_bottom = height - 60
    bar_mid = (bar_top + bar_bottom) // 2
    cv2.line(frame, (bar_x, bar_top), (bar_x, bar_bottom), green, 2)
    cv2.line(frame, (bar_x - 10, bar_mid), (bar_x + 10, bar_mid), green, 2)
    normalized = max(-1.0, min(1.0, x_error / (width / 2)))
    bar_pos = int(bar_mid + normalized * ((bar_bottom - bar_top) / 2))
    cv2.circle(frame, (bar_x, bar_pos), 10, green, -1)
    cv2.putText(frame, f"x: {x_error:+.0f}px",
                (bar_x - 110, bar_top - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, green, 2)

    user_data.set_frame(frame)
    if video_streamer is not None:
        video_streamer.update_frame(frame)


def main():
    hailo_logger.info("Starting Hailo + pymavlink yaw-follow + TCP stream.")
    user_data = user_app_callback_class()
    app = GStreamerDetectionApp(app_callback, user_data)
    try:
        app.run()
    finally:
        drone_ctrl.shutdown()
        if video_streamer is not None:
            video_streamer.stop()


if __name__ == "__main__":
    main()