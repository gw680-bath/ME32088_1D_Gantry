#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Main script for 1D Gantry Control System
Integrates GUI, ArUco marker detection, and UDP communication for laser targeting.

This script:
1. Runs GUI to get target selection
2. Captures ArUco marker positions and converts to steps
3. Manages UDP communication with gantry and laser
4. Executes target sequence with laser firing

@author: Gantry Control System
"""

import socket
import time
import numpy as np
import cv2
import cv2.aruco as aruco
from GUI_1 import run_gui

# ============================================================================
# CONFIGURATION CONSTANTS
# ============================================================================

# UDP Communication endpoints
UDP_LISTEN_IP = "176.26.4.254"      # IP to listen for requests
UDP_LISTEN_PORT = 50005              # Port to listen for first-target requests

UDP_GANTRY_IP = "138.38.229.138"    # Gantry IP
UDP_GANTRY_PORT = 50001              # Port for sending target locations

UDP_LASER_IP = "138.38.229.138"     # Laser control IP
UDP_LASER_PORT = 50003               # Port for laser on/off commands

UDP_END_IP = "138.38.229.138"       # End signal IP
UDP_END_PORT = 50004                 # Port for end-of-sequence message

# Conversion constants
MM_TO_STEPS_DISTANCE = 520.0         # Reference distance in mm
MM_TO_STEPS_VALUE = 400.0            # Reference step count
INITIAL_TARGET_STEPS = 5             # Target_0 initial position

# Camera and ArUco configuration
MARKER_SIZE_MM = 50                  # ArUco marker size in mm
CAMERA_INDEX = 0                     # Camera index (0 for default)
CALIBRATION_FILE = 'ImageProcessing/workdir/CalibrationGantry.npz'

# Timing constants
WAIT_AFTER_REQUEST = 1.0             # Seconds to wait after receiving request
LASER_ON_DURATION = 0.3              # Seconds to keep laser on
WAIT_BEFORE_EXIT = 2.0               # Seconds to wait before exit


# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def mm_to_steps(location_mm):
    """
    Convert millimeter location to step count.
    
    Formula: steps = (location_mm / 520) * 400
    
    Args:
        location_mm (float): Position in millimeters
        
    Returns:
        int: Position in steps (rounded)
    """
    steps = (location_mm / MM_TO_STEPS_DISTANCE) * MM_TO_STEPS_VALUE
    return int(round(steps))


def send_udp(ip, port, message):
    """
    Send a UDP datagram to specified IP and port.
    
    Args:
        ip (str): Destination IP address
        port (int): Destination port
        message (bytes): Message to send as bytes
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(message, (ip, port))
    sock.close()
    print(f"[UDP SEND] Sent {message} to {ip}:{port}")


def receive_udp_blocking(ip, port):
    """
    Listen for a UDP packet on specified IP and port (blocking).
    
    Args:
        ip (str): IP address to bind to
        port (int): Port to listen on
        
    Returns:
        tuple: (data, sender_address)
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))
    print(f"[UDP LISTEN] Waiting for packet on {ip}:{port}...")
    data, addr = sock.recvfrom(1024)
    sock.close()
    print(f"[UDP RECEIVE] Received {data} from {addr}")
    return data, addr


def fire_laser():
    """
    Fire the laser for the configured duration.
    
    Sends laser ON command, waits, then sends laser OFF command.
    """
    print("[LASER] Firing laser...")
    
    # Turn laser ON
    send_udp(UDP_LASER_IP, UDP_LASER_PORT, b'\x01')
    
    # Keep laser on for specified duration
    time.sleep(LASER_ON_DURATION)
    
    # Turn laser OFF
    send_udp(UDP_LASER_IP, UDP_LASER_PORT, b'\x00')
    
    print("[LASER] Laser firing complete")


def send_target_location(steps):
    """
    Send target location in steps to the gantry.
    
    Args:
        steps (int): Target position in steps
    """
    # Convert steps to bytes (assuming integer format)
    message = str(steps).encode('utf-8')
    send_udp(UDP_GANTRY_IP, UDP_GANTRY_PORT, message)
    print(f"[GANTRY] Sent target location: {steps} steps")


def send_end_signal():
    """
    Send end-of-sequence signal to indicate completion.
    """
    send_udp(UDP_END_IP, UDP_END_PORT, b'END')
    print("[END] Sent end-of-sequence signal")


def capture_aruco_positions():
    """
    Capture current positions of all ArUco markers in view.
    Shows live camera feed with marker detection overlays.
    Press 'q' to finish capturing and proceed.
    
    Returns:
        dict: Mapping of marker ID to position in mm (using tvec[0][0] as x-position)
              Example: {1: 245.3, 2: 189.7, ...}
    """
    print("\n[ARUCO] Starting marker detection...")
    print("[ARUCO] Press 'q' when all markers are visible to continue...")
    
    # Load camera calibration
    try:
        camera_calibration = np.load(CALIBRATION_FILE)
        CM = camera_calibration['CM']
        dist_coef = camera_calibration['dist_coef']
        print("[ARUCO] Camera calibration loaded")
    except Exception as e:
        print(f"[ERROR] Failed to load calibration: {e}")
        return {}
    
    # Setup ArUco detection
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    
    # Open camera
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera {CAMERA_INDEX}")
        return {}
    
    print("[ARUCO] Camera opened, displaying live feed...")
    
    # Create windows for display
    cv2.namedWindow("ArUco Detection", cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow("Gray", cv2.WINDOW_AUTOSIZE)
    cv2.moveWindow("Gray", 640, 100)
    cv2.moveWindow("ArUco Detection", 0, 100)
    
    # Dictionary to store detected positions
    marker_positions = {}
    
    # Processing rate
    processing_period = 0.25
    start_time = time.time()
    fps = 0.0
    
    # Capture frames until user presses 'q'
    while True:
        ret, frame = cap.read()
        if not ret:
            print(f"[WARNING] Frame capture failed")
            break
        
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.imshow('Gray', gray)
        
        # Detect markers
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        
        # If markers detected, estimate pose and draw
        rvecs, tvecs = None, None
        if ids is not None:
            # Draw detected markers
            frame = aruco.drawDetectedMarkers(frame, corners, ids)
            
            # Estimate pose of each marker
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE_MM, CM, dist_coef)
            
            # Draw axis for each marker
            for rvec, tvec in zip(rvecs, tvecs):
                frame = cv2.drawFrameAxes(frame, CM, dist_coef, rvec, tvec, 100)
            
            for marker_id, tvec in zip(ids.flatten(), tvecs):
                # Use x-position (tvec[0][0]) as the position along gantry axis
                position_mm = float(tvec[0][0])
                
                # Store or update position (average if seen multiple times)
                if marker_id in marker_positions:
                    marker_positions[marker_id].append(position_mm)
                else:
                    marker_positions[marker_id] = [position_mm]
        
        # Add FPS info to frame
        cv2.putText(frame, f"CAMERA FPS: {fps:.2f}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"PROCESSING FPS: {1/processing_period:.2f}", (10, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Add detection status
        y_offset = 100
        cv2.putText(frame, "Press 'q' to finish capturing", (10, y_offset), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        y_offset += 40
        
        # Show detected markers info
        if ids is not None:
            cv2.putText(frame, f"Detected {len(ids)} markers", (10, y_offset), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            y_offset += 35
            
            # List each detected marker with position
            for marker_id, tvec in zip(ids.flatten(), tvecs):
                position_mm = float(tvec[0][0])
                steps = mm_to_steps(position_mm)
                text = f"ID {marker_id}: {position_mm:.1f}mm ({steps} steps)"
                cv2.putText(frame, text, (10, y_offset), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                y_offset += 25
        else:
            cv2.putText(frame, "No markers detected", (10, y_offset), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Display the frame
        cv2.imshow('ArUco Detection', frame)
        
        # Check for 'q' key press to finish
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("\n[ARUCO] User pressed 'q' - finishing capture...")
            break
        
        # Maintain steady processing rate
        elapsed_time = time.time() - start_time
        fps = 1 / elapsed_time if elapsed_time > 0 else 0
        if elapsed_time < processing_period:
            time.sleep(processing_period - elapsed_time)
        start_time = time.time()
    
    # Release camera
    cap.release()
    cv2.destroyAllWindows()
    
    # Average positions for markers seen multiple times
    averaged_positions = {}
    for marker_id, positions in marker_positions.items():
        averaged_positions[marker_id] = np.mean(positions)
        print(f"[ARUCO] ID {marker_id} average position: {averaged_positions[marker_id]:.2f} mm")
    
    print(f"[ARUCO] Detection complete. Found {len(averaged_positions)} markers\n")
    
    return averaged_positions


def generate_target_locations(target_ids, marker_positions):
    """
    Generate list of target locations in steps based on target IDs.
    
    Args:
        target_ids (list): List of target IDs from GUI (e.g., [1, 3, 5])
        marker_positions (dict): Mapping of marker ID to position in mm
        
    Returns:
        list: List of target positions in steps, with Target_0 prepended
    """
    print("[SETUP] Generating target location list...")
    
    # Start with Target_0
    target_locations = [INITIAL_TARGET_STEPS]
    print(f"[SETUP] Target_0: {INITIAL_TARGET_STEPS} steps (initial position)")
    
    # Add each target from the GUI selection
    for target_id in target_ids:
        if target_id is None:
            continue
            
        if target_id in marker_positions:
            position_mm = marker_positions[target_id]
            steps = mm_to_steps(position_mm)
            target_locations.append(steps)
            print(f"[SETUP] Target_{target_id}: {position_mm:.2f} mm → {steps} steps")
        else:
            print(f"[WARNING] Target ID {target_id} not found in detected markers, skipping")
    
    print(f"[SETUP] Total targets in sequence: {len(target_locations)}\n")
    
    return target_locations


def execute_target_sequence(target_locations):
    """
    Execute the main target sequence with UDP communication and laser firing.
    
    Args:
        target_locations (list): List of target positions in steps
    """
    print("[SEQUENCE] Starting target sequence execution...\n")
    
    # Make a copy so we can modify it
    locations = target_locations.copy()
    target_count = len(locations)
    
    while locations:
        current_target_index = target_count - len(locations) + 1
        print(f"[SEQUENCE] === Target {current_target_index}/{target_count} ===")
        print(f"[SEQUENCE] Remaining targets: {len(locations)}")
        
        # Step 3: Wait for first-target-location request
        print(f"[SEQUENCE] Waiting for request on {UDP_LISTEN_IP}:{UDP_LISTEN_PORT}...")
        data, addr = receive_udp_blocking(UDP_LISTEN_IP, UDP_LISTEN_PORT)
        
        # Parse received data as boolean (1 = send target, 0 = don't send target)
        try:
            received_value = int(data.decode('utf-8').strip())
            send_target = bool(received_value)
            print(f"[SEQUENCE] Received command: {received_value} (send_target={send_target})")
            
            if not send_target:
                print("[SEQUENCE] Received '0' - skipping target, not sending location")
                continue
        except (ValueError, UnicodeDecodeError) as e:
            print(f"[WARNING] Failed to parse received data: {e}. Assuming send_target=True")
            send_target = True
        
        # Step 4: After receiving request
        print(f"[SEQUENCE] Request received. Waiting {WAIT_AFTER_REQUEST}s...")
        time.sleep(WAIT_AFTER_REQUEST)
        
        # Fire the laser
        fire_laser()
        
        # Step 5: Remove first element from array
        locations.pop(0)
        print(f"[SEQUENCE] Target completed. Targets remaining: {len(locations)}")
        
        # Step 6: Send next target (if any remain)
        if locations:
            next_target = locations[0]
            send_target_location(next_target)
        else:
            print("[SEQUENCE] No more targets remaining")
        
        print()  # Blank line for readability
    
    # Step 7: Send end signal when array is empty
    print("[SEQUENCE] All targets completed!")
    send_end_signal()
    
    print(f"[SEQUENCE] Waiting {WAIT_BEFORE_EXIT}s before exit...")
    time.sleep(WAIT_BEFORE_EXIT)
    
    print("[SEQUENCE] Sequence complete. Exiting.\n")


# ============================================================================
# MAIN EXECUTION
# ============================================================================

def main():
    """
    Main execution function.
    
    Flow:
    1. Run GUI to get target selection
    2. Capture ArUco marker positions
    3. Generate target location list
    4. Execute target sequence
    """
    print("=" * 70)
    print("1D GANTRY CONTROL SYSTEM")
    print("=" * 70)
    print()
    
    # Step 1: Run GUI and get target selection
    print("[MAIN] Launching GUI...")
    gui_result = run_gui()
    
    if gui_result is None:
        print("[MAIN] GUI closed without selection. Exiting.")
        return
    
    print(f"[MAIN] GUI Result: {gui_result}")
    
    # Extract target IDs from GUI result
    target_ids = gui_result.get('target_ids', [])
    mode = gui_result.get('mode', 'Unknown')
    
    print(f"[MAIN] Mode: {mode}")
    print(f"[MAIN] Target IDs: {target_ids}")
    
    # Handle special case: Manual Mode (no ArUco detection needed)
    if mode == "Manual Mode":
        manual_value = gui_result.get('manual_value', 0.0)
        # Convert manual value (0-1) to steps (assuming 0-1 maps to 0-400 steps)
        manual_steps = int(manual_value * MM_TO_STEPS_VALUE)
        target_locations = [INITIAL_TARGET_STEPS, manual_steps]
        print(f"[MAIN] Manual mode: {manual_value:.3f} → {manual_steps} steps")
    else:
        # Step 2: Capture ArUco marker positions
        if not target_ids:
            print("[MAIN] No targets selected. Exiting.")
            return
        
        marker_positions = capture_aruco_positions()
        
        if not marker_positions:
            print("[MAIN] No markers detected. Cannot proceed. Exiting.")
            return
        
        # Step 3: Generate target location list
        target_locations = generate_target_locations(target_ids, marker_positions)
    
    if len(target_locations) <= 1:
        print("[MAIN] Only initial target (Target_0) in list. Nothing to do. Exiting.")
        return
    
    print(f"[MAIN] Final target sequence: {target_locations}")
    
    # Step 4: Execute the target sequence
    try:
        execute_target_sequence(target_locations)
    except KeyboardInterrupt:
        print("\n[MAIN] Interrupted by user. Exiting.")
    except Exception as e:
        print(f"\n[ERROR] Unexpected error: {e}")
        import traceback
        traceback.print_exc()
    
    print("[MAIN] Program complete.")


if __name__ == "__main__":
    main()
