"""
Main script for 1D Gantry Control System (REVISED)
Integrates GUI target selection with ArUco marker detection and tracking.

Key fixes:
 - Repeatedly send target setpoint while moving (prevents "coast past" if controller needs periodic setpoints)
 - Robust UDP position parsing
 - Use <= threshold to avoid off-by-one acceptance problems
 - Non-blocking laser firing (threaded) so tracking continues while laser is on
 - Clear/advance targets after a successful fire
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import time
import sys
import socket
import threading
from pathlib import Path

# Import the GUI module
from GUI_1 import run_gui, TARGET_IDS


class GantryController:
    """Main controller for the 1D Gantry system."""
    
    def __init__(self, calibration_path='ImageProcessing/workdir/CalibrationGantry.npz', 
                 udp_ip_send='138.38.227.126', udp_ip_receive='0.0.0.0',
                 udp_port_position=50001, udp_port_receive=50002, 
                 udp_port_laser=50003):
        # Calibration and aruco setup
        self.calibration_path = Path(calibration_path)
        self.load_calibration()
        self.setup_aruco()

        # GUI targets
        self.target_config = None
        self.processing_period = 0.25  # seconds between processing iterations

        # UDP: send (setpoint) and laser control sockets
        self.udp_ip_send = udp_ip_send
        self.udp_port_position = udp_port_position
        self.udp_socket_position = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"✓ UDP Position Send configured: Sending TO {udp_ip_send}:{udp_port_position}")

        self.udp_ip_receive = udp_ip_receive
        self.udp_port_receive = udp_port_receive
        self.udp_socket_receive = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # bind to local receive address (0.0.0.0 listens on all interfaces)
        self.udp_socket_receive.bind((udp_ip_receive, udp_port_receive))
        self.udp_socket_receive.settimeout(0.01)
        self.current_position = 0
        print(f"✓ UDP Position Receive configured: Listening ON {udp_ip_receive}:{udp_port_receive}")

        self.udp_port_laser = udp_port_laser
        self.udp_socket_laser = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"✓ UDP Laser Control configured: Sending TO {udp_ip_send}:{udp_port_laser}")

        # Position thresholds and bookkeeping
        self.position_threshold = 5  # steps (use <= to accept)
        self.current_target_position = None  # in steps
        self.target_locked = False           # true after we've selected/sent a target
        self.current_target_index = 0
        self.laser_firing = False
        self.laser_cooldown = False

        # Conversion 520 mm -> 400 steps
        self.mm_to_steps = 400.0 / 520.0

        # optional: how long to keep trying a lost target (seconds) before giving up
        self.target_lost_timeout = 3.0
        self._last_detected_time = None

    # -------------------------
    # Calibration & aruco setup
    # -------------------------
    def load_calibration(self):
        try:
            camera_calibration = np.load(str(self.calibration_path))
            self.camera_matrix = camera_calibration['CM']
            self.dist_coef = camera_calibration['dist_coef']
            print(f"✓ Camera calibration loaded from {self.calibration_path}")
        except FileNotFoundError:
            print(f"✗ Error: Calibration file not found at {self.calibration_path}")
            sys.exit(1)
        except Exception as e:
            print(f"✗ Error loading calibration: {e}")
            sys.exit(1)

    def setup_aruco(self):
        self.marker_size = 50  # mm
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        print("✓ ArUco detection configured (DICT_4X4_50)")

    # -------------------------
    # GUI
    # -------------------------
    def get_targets_from_gui(self):
        print("\n" + "="*50)
        print("LAUNCHING TARGET SELECTION GUI")
        print("="*50 + "\n")
        self.target_config = run_gui()
        if self.target_config is None:
            print("✗ No target configuration received. Exiting.")
            return False
        print("\n" + "="*50)
        print("TARGET CONFIGURATION RECEIVED")
        print("="*50)
        print(f"Mode: {self.target_config['mode']}")
        print(f"Targets: {self.target_config['targets']}")
        print(f"Target IDs: {self.target_config['target_ids']}")
        if self.target_config['manual_value'] is not None:
            print(f"Manual Value: {self.target_config['manual_value']:.3f}")
        print("="*50 + "\n")
        return True

    # -------------------------
    # ArUco detection helpers
    # -------------------------
    def detect_markers(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        return corners, ids, rejected, gray

    def estimate_poses(self, corners):
        if len(corners) > 0:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coef
            )
            return rvecs, tvecs
        return None, None

    def check_target_detected(self, detected_ids):
        if detected_ids is None or self.target_config is None:
            return []
        target_ids = self.target_config.get('target_ids', [])
        if not target_ids:
            return []
        detected_targets = []
        flat_ids = detected_ids.flatten()
        for tid in target_ids:
            matches = np.where(flat_ids == tid)[0]
            if matches.size > 0:
                idx = int(matches[0])
                detected_targets.append((int(tid), idx))
        return detected_targets

    # -------------------------
    # Position conversion & UDP
    # -------------------------
    def position_mm_to_steps(self, position_mm):
        steps = int(round(position_mm * self.mm_to_steps))
        return steps

    def parse_position_packet(self, data: bytes):
        """
        Robust position parsing:
         - If length == 1: treat as raw byte 0-255
         - Else: try ascii int decode
         - Else: fallback to int.from_bytes (big-endian)
        """
        if not data:
            return None
        try:
            if len(data) == 1:
                return int(data[0])
            # try ascii
            s = data.decode('ascii', errors='ignore').strip()
            if s:
                return int(float(s))
            # fallback
            return int.from_bytes(data[:4], byteorder='big', signed=False)
        except Exception:
            # final fallback: first byte
            try:
                return int(data[0])
            except Exception:
                return None

    def receive_current_position(self):
        try:
            data, addr = self.udp_socket_receive.recvfrom(1024)
            if data:
                parsed = self.parse_position_packet(data)
                if parsed is not None:
                    self.current_position = parsed
                    # print short debug
                    # print(f"  📥 RECEIVED from {addr}: Raw={list(data)}, Parsed={self.current_position}")
                    return True
        except socket.timeout:
            pass
        except Exception as e:
            print(f"✗ UDP receive error: {e}")
        return False

    def send_target_position_udp(self, target_position_steps):
        # clamp to byte range
        if target_position_steps < 0:
            target_position_steps = 0
        elif target_position_steps > 255:
            target_position_steps = 255
        message = bytes([int(target_position_steps)])
        try:
            self.udp_socket_position.sendto(message, (self.udp_ip_send, self.udp_port_position))
            print(f"  📤 SENT TARGET → {self.udp_ip_send}:{self.udp_port_position}  Value={target_position_steps}")
            return True
        except Exception as e:
            print(f"✗ UDP send error: {e}")
            return False

    def send_laser_control(self, state):
        message = bytes([int(state)])
        try:
            self.udp_socket_laser.sendto(message, (self.udp_ip_send, self.udp_port_laser))
            print(f"  📤 SENT LASER {state} → {self.udp_ip_send}:{self.udp_port_laser}")
            return True
        except Exception as e:
            print(f"✗ Laser control UDP send error: {e}")
            return False

    # -------------------------
    # Threshold & laser firing
    # -------------------------
    def check_position_threshold(self):
        """Use <= to avoid off-by-one acceptance issues."""
        if self.current_target_position is None:
            return False
        position_error = abs(self.current_target_position - self.current_position)
        return position_error <= self.position_threshold

    def _laser_thread_fn(self, target_steps):
        """Runs in separate thread to not block tracking loop."""
        self.laser_firing = True
        print("\n" + "="*50)
        print("🎯 TARGET ALIGNED - FIRING LASER (threaded)")
        print(f"Target Position: {target_steps} steps")
        print(f"Current Position: {self.current_position} steps")
        print(f"Error: {abs(target_steps - self.current_position)} steps")
        print("="*50)
        # turn ON
        self.send_laser_control(1)
        print("🔴 LASER ON")
        # keep on for 0.3s (or desired duration)
        time.sleep(0.3)
        # turn OFF
        self.send_laser_control(0)
        print("⚫ LASER OFF")
        # Finish sequence
        self.laser_firing = False
        self.laser_cooldown = True
        
        # Flag target as hit and remove from arrays
        if self.current_target_index < len(self.target_config.get('target_ids', [])):
            hit_target_id = self.target_config['target_ids'][self.current_target_index]
            hit_target_name = self.target_config['targets'][self.current_target_index]
            print(f"✅ TARGET HIT: {hit_target_name} (ID: {hit_target_id})")
            
            # Remove the hit target from both arrays
            self.target_config['target_ids'].pop(self.current_target_index)
            self.target_config['targets'].pop(self.current_target_index)
            
            print(f"📋 Remaining targets: {self.target_config['targets']}")
            print(f"📋 Remaining target IDs: {self.target_config['target_ids']}")
        
        # Clear current target (index stays at same position since we removed the element)
        self.current_target_position = None
        self.target_locked = False
        print(f"✓ Ready for next target at index {self.current_target_index}")
        
        # small cooldown delay to prevent immediate re-fire on jitter
        time.sleep(0.2)
        self.laser_cooldown = False

    def fire_laser_sequence(self):
        if self.laser_firing or self.laser_cooldown:
            return
        # Launch thread
        target = self.current_target_position
        t = threading.Thread(target=self._laser_thread_fn, args=(target,), daemon=True)
        t.start()

    # -------------------------
    # Target selection / position calc
    # -------------------------
    def get_target_position(self, tvecs, detected_targets):
        """Return (steps, target_id, x_mm) or None if not available."""
        if not detected_targets or tvecs is None or self.target_config is None:
            return None
        target_ids = self.target_config.get('target_ids', [])
        if self.current_target_index >= len(target_ids):
            # all done
            return None
        current_target_id = target_ids[self.current_target_index]
        # find current target in detections
        selected = None
        for detected_id, idx in detected_targets:
            if detected_id == current_target_id:
                selected = (detected_id, idx)
                break
        if not selected:
            # not detected
            return None
        # extract x mm
        target_id, idx = selected
        x_position_mm = float(tvecs[idx][0][0])
        target_position_steps = self.position_mm_to_steps(x_position_mm)
        return target_position_steps, target_id, x_position_mm

    # -------------------------
    # Frame annotation (unchanged logic mostly)
    # -------------------------
    def annotate_frame(self, frame, corners, ids, rvecs, tvecs, fps, detected_targets):
        if ids is not None:
            frame = aruco.drawDetectedMarkers(frame, corners, ids)
            if rvecs is not None and tvecs is not None:
                for rvec, tvec in zip(rvecs, tvecs):
                    frame = cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coef, rvec, tvec, 100)
        cv2.putText(frame, f"CAMERA FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"PROCESSING FPS: {1/self.processing_period:.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        if self.target_config:
            mode = self.target_config['mode']
            cv2.putText(frame, f"Mode: {mode}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            if self.current_target_position is not None:
                cv2.putText(frame, f"Target: {self.current_target_position} steps", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
            cv2.putText(frame, f"Current: {self.current_position} steps", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            if self.current_target_position is not None:
                error = abs(self.current_target_position - self.current_position)
                within_threshold = (error <= self.position_threshold)
                color = (0, 255, 0) if within_threshold else (0, 165, 255)
                cv2.putText(frame, f"Error: {error} steps", (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            if self.laser_firing:
                cv2.putText(frame, "LASER: FIRING", (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            # show each configured target detection state
            y_offset = 240
            for target_name in self.target_config['targets']:
                if target_name.startswith('Target_'):
                    target_id = TARGET_IDS.get(target_name)
                    is_detected = any(tid == target_id for tid, _ in detected_targets)
                    color = (0, 255, 0) if is_detected else (0, 0, 255)
                    status = "✓ DETECTED" if is_detected else "✗ Not detected"
                    text = f"{target_name} (ID:{target_id}): {status}"
                    cv2.putText(frame, text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                    y_offset += 25
        return frame

    # -------------------------
    # Main tracking loop
    # -------------------------
    def run_tracking(self):
        if not self.target_config:
            print("✗ Error: No target configuration. Run get_targets_from_gui() first.")
            return

        print("\n" + "="*50)
        print("STARTING ARUCO TRACKING")
        print("="*50)
        print("Press 'q' to quit")
        print("="*50 + "\n")

        cv2.namedWindow("Gantry Tracking", cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow("Gray", cv2.WINDOW_AUTOSIZE)
        cv2.moveWindow("Gray", 640, 100)
        cv2.moveWindow("Gantry Tracking", 0, 100)

        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("✗ Error: Cannot open camera")
            return

        start_time = time.time()
        fps = 0.0

        try:
            while True:
                # break if completed all targets
                if self.current_target_index >= len(self.target_config.get('target_ids', [])):
                    print("\n" + "="*50)
                    print("🎉 ALL TARGETS COMPLETED!")
                    print("="*50)
                    print("All targets have been hit successfully.")
                    print("Stopping program...\n")
                    break

                ret, frame = cap.read()
                if not ret:
                    print("✗ Can't receive frame. Exiting...")
                    break

                corners, ids, rejected, gray = self.detect_markers(frame)
                cv2.imshow('Gray', gray)

                rvecs, tvecs = self.estimate_poses(corners)

                # receive current position (update if new data)
                self.receive_current_position()

                detected_targets = self.check_target_detected(ids)

                # If marker for current target is detected, compute its setpoint
                target_result = self.get_target_position(tvecs, detected_targets)

                if target_result is not None:
                    target_steps, active_target_id, x_pos_mm = target_result
                    # If target not locked yet, send and lock
                    if not self.target_locked:
                        self.send_target_position_udp(target_steps)
                        self.current_target_position = target_steps
                        self.target_locked = True
                        self._last_detected_time = time.time()
                        print(f"  🎯 TARGET LOCKED: {target_steps} steps - waiting for alignment...")
                    else:
                        # If target is locked and marker still visible, refresh last seen time
                        self._last_detected_time = time.time()
                        # If detected position changed notably, update setpoint (keeps setpoint consistent with marker)
                        if self.current_target_position is None or abs(self.current_target_position - target_steps) > 1:
                            # send updated setpoint
                            self.send_target_position_udp(target_steps)
                            self.current_target_position = target_steps
                            print(f"  ↺ Updated locked target setpoint to {target_steps} steps (marker moved)")
                else:
                    # no detection for current target id this frame
                    # if we have a locked target, keep sending the last setpoint (controller often expects periodic setpoints)
                    if self.target_locked and self.current_target_position is not None:
                        # re-send setpoint to ensure controller maintains target
                        self.send_target_position_udp(self.current_target_position)
                        # If target has been lost for too long, unlock to allow waiting for re-detection
                        if self._last_detected_time is not None and (time.time() - self._last_detected_time) > self.target_lost_timeout:
                            print("⚠ Target lost for too long - unlocking and waiting for detection again.")
                            self.target_locked = False
                            self.current_target_position = None
                            self._last_detected_time = None

                # If we have a target set and it's not within threshold, keep sending setpoint
                if self.current_target_position is not None and not self.check_position_threshold():
                    # send setpoint each loop (ensures controller continuously sees target)
                    self.send_target_position_udp(self.current_target_position)

                # If within threshold and not currently firing/cooling, start laser
                if self.current_target_position is not None and self.check_position_threshold() and not self.laser_firing and not self.laser_cooldown:
                    # Fire laser in background thread
                    self.fire_laser_sequence()

                # Print debug info when markers exist
                if detected_targets and tvecs is not None:
                    print(f"\n--- Frame Update ---")
                    for target_id, idx in detected_targets:
                        target_name = f"Target_{target_id}"
                        pos = tvecs[idx][0]
                        print(f"{target_name}: Position (x={pos[0]:.1f}, y={pos[1]:.1f}, z={pos[2]:.1f}) mm")
                        steps = self.position_mm_to_steps(pos[0])
                        print(f"  → 1D Position: {pos[0]:.2f} mm ({steps} steps)")
                    if self.current_target_position is not None:
                        error = abs(self.current_target_position - self.current_position)
                        print(f"  ★ ACTIVE TARGET: Target_{self.current_target_index} → {self.current_target_position} steps (Error: {error} steps)")

                # Annotate & display
                frame = self.annotate_frame(frame, corners, ids, rvecs, tvecs, fps, detected_targets)
                cv2.imshow('Gantry Tracking', frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("\n✓ Tracking stopped by user")
                    break

                # Maintain steady rate
                elapsed_time = time.time() - start_time
                fps = 1 / elapsed_time if elapsed_time > 0 else 0
                if elapsed_time < self.processing_period:
                    time.sleep(self.processing_period - elapsed_time)
                start_time = time.time()

        finally:
            cap.release()
            cv2.destroyAllWindows()
            try:
                self.udp_socket_position.close()
                self.udp_socket_receive.close()
                self.udp_socket_laser.close()
            except Exception:
                pass
            print("✓ Camera released and windows closed")
            print("✓ All UDP sockets closed")

    # -------------------------
    # Run
    # -------------------------
    def run(self):
        print("\n" + "="*60)
        print("1D GANTRY CONTROL SYSTEM")
        print("="*60 + "\n")
        if not self.get_targets_from_gui():
            return
        self.run_tracking()
        print("\n" + "="*60)
        print("SYSTEM SHUTDOWN COMPLETE")
        print("="*60 + "\n")


def main():
    try:
        controller = GantryController()
        controller.run()
    except KeyboardInterrupt:
        print("\n\n✗ Interrupted by user (Ctrl+C)")
        sys.exit(0)
    except Exception as e:
        print(f"\n✗ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()