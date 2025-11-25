"""
Main script for 1D Gantry Control System
Integrates GUI target selection with ArUco marker detection and tracking.
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import time
import sys
from pathlib import Path

# Import the GUI module
from GUI_1 import run_gui, TARGET_IDS


class GantryController:
    """Main controller for the 1D Gantry system."""
    
    def __init__(self, calibration_path='ImageProcessing/workdir/CalibrationGantry.npz'):
        """Initialize the gantry controller with camera calibration."""
        self.calibration_path = Path(calibration_path)
        self.load_calibration()
        self.setup_aruco()
        self.target_config = None
        self.processing_period = 0.25  # Processing rate in seconds
        
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
        self.marker_size = 40  # mm
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
            
            # Show target detection status
            y_offset = 120
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
                
                # Check if target markers are detected
                detected_targets = self.check_target_detected(ids)
                
                # Print position information when targets are detected
                if detected_targets and tvecs is not None:
                    print(f"\n--- Frame Update ---")
                    for target_id, idx in detected_targets:
                        target_name = f"Target_{target_id}"
                        pos = tvecs[idx][0]
                        print(f"{target_name}: Position (x={pos[0]:.1f}, y={pos[1]:.1f}, z={pos[2]:.1f}) mm")
                        
                        # For 1D gantry, the x-position is most relevant
                        print(f"  → 1D Position: {pos[0]:.2f} mm")
                
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
            print("✓ Camera released and windows closed")
    
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
