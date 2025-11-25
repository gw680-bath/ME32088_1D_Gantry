"""
Main script for 1D Gantry Control System
Integrates GUI target selection with ArUco marker detection and tracking.
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
                 udp_ip_send='138.38.227.126', udp_ip_receive='172.26.4.254',
                 udp_port_position=50001, udp_port_receive=50002, 
                 udp_port_laser=50003):
        # udp_ip_send: IP address of RPi/Simulink (where to SEND target position)
        # udp_ip_receive: Local binding IP (use '0.0.0.0' to listen on all interfaces)
        # Current config: Send to Giles Mac IP, receive on all local interfaces

        """Initialize the gantry controller with camera calibration."""
        self.calibration_path = Path(calibration_path)
        self.load_calibration()
        self.setup_aruco()
        self.target_config = None
        self.processing_period = 0.25  # Processing rate in seconds
        
        # UDP communication setup for sending position to RPi/Simulink
        self.udp_ip_send = udp_ip_send
        self.udp_port_position = udp_port_position
        self.udp_socket_position = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"✓ UDP Position Send configured: Sending TO {udp_ip_send}:{udp_port_position}")
        
        # UDP communication setup for receiving current position from RPi
        self.udp_ip_receive = udp_ip_receive
        self.udp_port_receive = udp_port_receive
        self.udp_socket_receive = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket_receive.bind((udp_ip_receive, udp_port_receive))
        self.udp_socket_receive.settimeout(0.01)  # Non-blocking with short timeout
        self.current_position = 0  # Current position in steps from RPi
        print(f"✓ UDP Position Receive configured: Listening ON {udp_ip_receive}:{udp_port_receive}")
        
        # UDP communication setup for laser control
        self.udp_port_laser = udp_port_laser
        self.udp_socket_laser = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"✓ UDP Laser Control configured: Sending TO {udp_ip_send}:{udp_port_laser}")
        
        # Position tracking and threshold
        self.position_threshold = 5  # steps
        self.current_target_position = None
        self.target_locked = False  # Lock target until eliminated
        self.current_target_index = 0  # Track which target we're currently pursuing
        self.laser_firing = False
        self.laser_cooldown = False
        
        # Conversion: 520 mm = 400 steps, so 1 mm = 400/520 steps
        self.mm_to_steps = 400.0 / 520.0  # ~0.769 steps/mm
        
    def load_calibration(self):
        """Load camera calibration data."""
        try:
            camera_calibration = np.load(str(self.calibration_path))
            self.camera_matrix = camera_calibration['CM']
            self.dist_coef = camera_calibration['dist_coef']
            print(f"✓ Camera calibration loaded from {self.calibration_path}")
        except FileNotFoundError:
            print(f"✗ Error: Calibration file not found at {self.calibration_path}")
            print("  Please ensure the calibration file exists.")
            sys.exit(1)
        except Exception as e:
            print(f"✗ Error loading calibration: {e}")
            sys.exit(1)
    
    def setup_aruco(self):
        """Setup ArUco marker detection parameters."""
        self.marker_size = 50  # mm
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        print("✓ ArUco detection configured (DICT_4X4_50)")
    
    def get_targets_from_gui(self):
        """Launch GUI and get target configuration from user."""
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
    
    def detect_markers(self, frame):
        """Detect ArUco markers in the given frame."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.parameters
        )
        return corners, ids, rejected, gray
    
    def estimate_poses(self, corners):
        """Estimate pose of detected markers."""
        if len(corners) > 0:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coef
            )
            return rvecs, tvecs
        return None, None
    
    def check_target_detected(self, detected_ids):
        """Check if any of the target markers are detected."""
        if detected_ids is None or self.target_config is None:
            return []
        
        target_ids = self.target_config['target_ids']
        if not target_ids:
            return []
        
        detected_targets = []
        for target_id in target_ids:
            if target_id in detected_ids.flatten():
                idx = np.where(detected_ids.flatten() == target_id)[0][0]
                detected_targets.append((target_id, idx))
        
        return detected_targets
    
    def position_mm_to_steps(self, position_mm):
        """Convert position from mm to steps.
        
        Args:
            position_mm: Position in millimeters
            
        Returns:
            Position in steps (integer)
        """
        steps = int(round(position_mm * self.mm_to_steps))
        return steps
    
    def receive_current_position(self):
        """Receive current position from RPi via UDP (non-blocking).
        
        Returns:
            True if new position received, False otherwise
        """
        try:
            data, addr = self.udp_socket_receive.recvfrom(1024)
            if len(data) > 0:
                # Assume single byte representing position
                self.current_position = data[0]
                print(f"  📥 RECEIVED from {addr}: Raw bytes={list(data)}, Value={self.current_position} (type: {type(self.current_position).__name__})")
                return True
        except socket.timeout:
            # No data available, continue
            pass
        except Exception as e:
            print(f"✗ UDP receive error: {e}")
        return False
    
    def send_target_position_udp(self, target_position_steps):
        """Send target position to Simulink via UDP.
        
        Args:
            target_position_steps: Target position in steps (will be clamped to 0-255)
        """
        # Clamp to byte range (0-255)
        if target_position_steps < 0:
            target_position_steps = 0
        elif target_position_steps > 255:
            target_position_steps = 255
        
        # Convert to byte and send
        message = bytes([target_position_steps])
        try:
            self.udp_socket_position.sendto(message, (self.udp_ip_send, self.udp_port_position))
            print(f"  📤 SENT TARGET to {self.udp_ip_send}:{self.udp_port_position} - Raw bytes={list(message)}, Value={target_position_steps} (type: {type(target_position_steps).__name__})")
            return True
        except Exception as e:
            print(f"✗ UDP send error: {e}")
            return False
    
    def send_laser_control(self, state):
        """Send laser ON/OFF command to RPi via UDP.
        
        Args:
            state: 1 for ON, 0 for OFF
        """
        message = bytes([state])
        try:
            self.udp_socket_laser.sendto(message, (self.udp_ip_send, self.udp_port_laser))
            print(f"  📤 SENT LASER to {self.udp_ip_send}:{self.udp_port_laser} - Raw bytes={list(message)}, Value={state} (type: {type(state).__name__})")
            return True
        except Exception as e:
            print(f"✗ Laser control UDP send error: {e}")
            return False
    
    def check_position_threshold(self):
        """Check if current position is within threshold of target position.
        
        Returns:
            True if within threshold, False otherwise
        """
        if self.current_target_position is None:
            return False
        
        position_error = abs(self.current_target_position - self.current_position)
        return position_error < self.position_threshold
    

    
    def get_target_position(self, tvecs, detected_targets):
        """Calculate target position based on detected markers.
        
        Priority: Current target in sequence from the target list.
        
        Args:
            tvecs: Translation vectors from ArUco detection
            detected_targets: List of (target_id, idx) tuples
            
        Returns:
            Position in steps (int) or None if no targets detected
        """
        if not detected_targets or tvecs is None:
            return None
        
        # Get the ordered target IDs from config
        target_ids = self.target_config.get('target_ids', [])
        
        # Check if we've completed all targets
        if self.current_target_index >= len(target_ids):
            print("\n" + "="*50)
            print("✓ ALL TARGETS COMPLETED!")
            print("="*50 + "\n")
            return None
        
        # Get the current target we should be pursuing
        current_target_id = target_ids[self.current_target_index]
        
        # Find if current target is detected
        selected_target = None
        for detected_id, idx in detected_targets:
            if detected_id == current_target_id:
                selected_target = (detected_id, idx)
                break
        
        # If current target not detected, return None (wait for it)
        if not selected_target:
            return None
        
        # Reset cooldown when we start tracking a new target
        if self.laser_cooldown:
            self.laser_cooldown = False
        
        # Get x-position in mm and convert to steps
        target_id, idx = selected_target
        x_position_mm = tvecs[idx][0][0]
        target_position_steps = self.position_mm_to_steps(x_position_mm)
        
        return target_position_steps, target_id, x_position_mm
    
    def annotate_frame(self, frame, corners, ids, rvecs, tvecs, fps, detected_targets):
        """Annotate the frame with detection information."""
        # Draw detected markers
        if ids is not None:
            frame = aruco.drawDetectedMarkers(frame, corners, ids)
            
            # Draw axis for each marker
            if rvecs is not None and tvecs is not None:
                for rvec, tvec in zip(rvecs, tvecs):
                    frame = cv2.drawFrameAxes(
                        frame, self.camera_matrix, self.dist_coef, rvec, tvec, 100
                    )
        
        # Add FPS information
        cv2.putText(frame, f"CAMERA FPS: {fps:.2f}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"PROCESSING FPS: {1/self.processing_period:.2f}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Add target information
        if self.target_config:
            mode = self.target_config['mode']
            cv2.putText(frame, f"Mode: {mode}", (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # Show current target position being sent
            if self.current_target_position is not None:
                cv2.putText(frame, f"Target: {self.current_target_position} steps", (10, 120),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
            
            # Show current position from RPi
            cv2.putText(frame, f"Current: {self.current_position} steps", (10, 150),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # Show position error and threshold status
            if self.current_target_position is not None:
                error = abs(self.current_target_position - self.current_position)
                within_threshold = error < self.position_threshold
                color = (0, 255, 0) if within_threshold else (0, 165, 255)
                cv2.putText(frame, f"Error: {error} steps", (10, 180),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # Show laser status
            if self.laser_firing:
                cv2.putText(frame, "LASER: FIRING", (10, 210),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            # Show target detection status
            y_offset = 240
            for target_name in self.target_config['targets']:
                if target_name.startswith('Target_'):
                    target_id = TARGET_IDS.get(target_name)
                    is_detected = any(tid == target_id for tid, _ in detected_targets)
                    color = (0, 255, 0) if is_detected else (0, 0, 255)
                    status = "✓ DETECTED" if is_detected else "✗ Not detected"
                    text = f"{target_name} (ID:{target_id}): {status}"
                    cv2.putText(frame, text, (10, y_offset),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                    y_offset += 25
        
        return frame
    
    def run_tracking(self):
        """Main tracking loop with ArUco detection."""
        if not self.target_config:
            print("✗ Error: No target configuration. Run get_targets_from_gui() first.")
            return
        
        print("\n" + "="*50)
        print("STARTING ARUCO TRACKING")
        print("="*50)
        print("Press 'q' to quit")
        print("="*50 + "\n")
        
        # Create OpenCV windows
        cv2.namedWindow("Gantry Tracking", cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow("Gray", cv2.WINDOW_AUTOSIZE)
        cv2.moveWindow("Gray", 640, 100)
        cv2.moveWindow("Gantry Tracking", 0, 100)
        
        # Start video capture
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("✗ Error: Cannot open camera")
            return
        
        start_time = time.time()
        fps = 0
        
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("✗ Can't receive frame. Exiting...")
                    break
                
                # Detect markers
                corners, ids, rejected, gray = self.detect_markers(frame)
                
                # Show gray frame
                cv2.imshow('Gray', gray)
                
                # Estimate poses
                rvecs, tvecs = self.estimate_poses(corners)
                
                # Receive current position from RPi
                self.receive_current_position()
                
                # Check if target markers are detected
                detected_targets = self.check_target_detected(ids)
                
                # Reset laser_firing and laser_cooldown when starting to track a new target
                if detected_targets and not self.target_locked:
                    self.laser_firing = False
                    self.laser_cooldown = False
                
                # Calculate and send target position
                target_result = self.get_target_position(tvecs, detected_targets)
                if target_result is not None:
                    target_steps, active_target_id, x_pos_mm = target_result
                    
                    # Only update target if not currently locked on a target
                    if not self.target_locked:
                        # Send to RPi via UDP
                        self.send_target_position_udp(target_steps)
                        self.current_target_position = target_steps
                        self.target_locked = True  # Lock this target
                        print(f"  🎯 TARGET LOCKED: {target_steps} steps - waiting for alignment...")
                    
                    # Check if we're within threshold and should fire laser
                    within_threshold = self.check_position_threshold()
                    if within_threshold:
                        print(f"  ✅ WITHIN THRESHOLD! Error={abs(self.current_target_position - self.current_position)} < {self.position_threshold}")
                        if not self.laser_firing and not self.laser_cooldown:
                            print(f"  🚀 Sending laser trigger pulse...")
                            # Send trigger pulse: 1 then 0 to create rising edge
                            self.send_laser_control(1)
                            self.send_laser_control(0)
                            print(f"  ✓ Pulse sent (1→0)")
                            self.laser_firing = True
                            self.laser_cooldown = True
                            self.target_locked = False  # Unlock for next target
                            # Move to next target
                            self.current_target_index += 1
                            print(f"✓ Moving to next target (index {self.current_target_index})")
                        else:
                            if self.laser_firing:
                                print(f"  ⏳ Laser already firing...")
                            if self.laser_cooldown:
                                print(f"  ❄️  Laser in cooldown...")
                
                # Print position information when targets are detected
                if detected_targets and tvecs is not None:
                    print(f"\n--- Frame Update ---")
                    for target_id, idx in detected_targets:
                        target_name = f"Target_{target_id}"
                        pos = tvecs[idx][0]
                        print(f"{target_name}: Position (x={pos[0]:.1f}, y={pos[1]:.1f}, z={pos[2]:.1f}) mm")
                        
                        # For 1D gantry, the x-position is most relevant
                        steps = self.position_mm_to_steps(pos[0])
                        print(f"  → 1D Position: {pos[0]:.2f} mm ({steps} steps)")
                    
                    # Show active target being tracked
                    if target_result is not None:
                        error = abs(target_steps - self.current_position)
                        print(f"  ★ ACTIVE TARGET: Target_{active_target_id} → {target_steps} steps (UDP sent)")
                        print(f"  📍 Current Position: {self.current_position} steps (Error: {error} steps)")
                
                # Annotate frame
                frame = self.annotate_frame(frame, corners, ids, rvecs, tvecs, 
                                           fps, detected_targets)
                
                # Display frame
                cv2.imshow('Gantry Tracking', frame)
                
                # Handle key press
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("\n✓ Tracking stopped by user")
                    break
                
                # Maintain steady processing rate
                elapsed_time = time.time() - start_time
                fps = 1 / elapsed_time if elapsed_time > 0 else 0
                if elapsed_time < self.processing_period:
                    time.sleep(self.processing_period - elapsed_time)
                start_time = time.time()
        
        finally:
            cap.release()
            cv2.destroyAllWindows()
            self.udp_socket_position.close()
            self.udp_socket_receive.close()
            self.udp_socket_laser.close()
            print("✓ Camera released and windows closed")
            print("✓ All UDP sockets closed")
    
    def run(self):
        """Main execution flow: GUI selection -> Tracking."""
        print("\n" + "="*60)
        print("1D GANTRY CONTROL SYSTEM")
        print("="*60 + "\n")
        
        # Step 1: Get targets from GUI
        if not self.get_targets_from_gui():
            return
        
        # Step 2: Run tracking with selected targets
        self.run_tracking()
        
        print("\n" + "="*60)
        print("SYSTEM SHUTDOWN COMPLETE")
        print("="*60 + "\n")


def main():
    """Entry point for the gantry control system."""
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
