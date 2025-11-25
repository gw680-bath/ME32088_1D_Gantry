# This script is an entry point to the Aruco marker detection and pose estimation.
# It uses the camera calibration values to estimate the pose of the markers.
# The script will display the original image and the image with the detected markers and their pose.
# It is using the OpenCV library 4.10+ which has the latest Aruco functions

import argparse
import sys
import socket
import cv2
import cv2.aruco as aruco
import numpy as np
import time # We will use this to ensure a steady processing rate


# --- Command-line arguments -------------------------------------------------
# This script can send a 1/0 byte over UDP when a detected ArUco marker is
# "close" (on a user-chosen axis) to the camera. This is useful for driving
# Simulink blocks that listen for a single-byte UDP message.
parser = argparse.ArgumentParser(description='Aruco detector + UDP laser on/off output')
parser.add_argument('--ip', default='127.0.0.1', help='Destination IP for UDP messages')
parser.add_argument('--port', type=int, default=25000, help='Destination UDP port')
parser.add_argument('--axis', choices=['x', 'y', 'z'], default='x', help='Axis to check (x,y,z)')
parser.add_argument('--threshold', type=float, default=0.05, help='Threshold on chosen axis (units same as tvecs)')
parser.add_argument('--range-multiplier', type=float, default=1.0, help='Multiplier to expand (or shrink) the threshold range')
parser.add_argument('--camera', type=int, default=1, help='Camera device index for VideoCapture')
parser.add_argument('--marker', type=int, default=40, help='Marker size used for pose estimation')
parser.add_argument('--debug', action='store_true', help='Enable debug prints')
args = parser.parse_args()

# Load the camera calibration values
# old version: camera_calibration = np.load(r'workdir/CalibrationGantry.npz')
try:
    camera_calibration = np.load('ME32088_1D_Gantry/ImageProcessing/workdir/CalibrationGantry.npz')
    CM=camera_calibration['CM'] #camera matrix
    dist_coef=camera_calibration['dist_coef']# distortion coefficients from the camera
except FileNotFoundError:
    print('ERROR: Calibration file not found: ME32088_1D_Gantry/ImageProcessing/workdir/CalibrationGantry.npz')
    print('Please run the calibration script or place the calibration file in the workdir folder.')
    sys.exit(1)
except KeyError as e:
    print('ERROR: Calibration file does not contain expected keys:', e)
    sys.exit(1)

# Define the ArUco dictionary and parameters
marker_size = args.marker
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

# Define a processing rate
processing_period = 0.25

# Create two OpenCV named windows
cv2.namedWindow("Frame", cv2.WINDOW_AUTOSIZE)
cv2.namedWindow("Gray", cv2.WINDOW_AUTOSIZE)

# Position the windows next to each other
cv2.moveWindow("Gray", 640, 100)
cv2.moveWindow("Frame", 0, 100)
# Start capturing video
#(0) for default camera, (1) for external camera (prefered)
cap = cv2.VideoCapture(args.camera)

# Set the starting time
start_time = time.time()
fps = 0


try:
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        # Convert frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.imshow('gray-image', gray)

        # Detect markers
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        

        # If markers are detected
        if ids is not None:
            # Draw detected markers
            frame = aruco.drawDetectedMarkers(frame, corners, ids)

            # Estimate pose of each marker
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, CM, dist_coef)

            for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
                # Draw axis for each marker
                frame = cv2.drawFrameAxes(frame, CM, dist_coef, rvec, tvec, 100)
                # Show X coordinate at the marker center (tvec[0]) and a small visual
                # helpful for tuning. corner coords are in pixel coordinates.
                try:
                    corner = corners[i][0]
                    center_px = tuple(np.round(np.mean(corner, axis=0)).astype(int))
                except Exception:
                    center_px = (10, 150)
                x_val = float(tvec.reshape(-1)[0])
                cv2.putText(frame, f"X:{x_val:.3f}", (center_px[0]+10, center_px[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

            # Print tvecs to console (only when markers are detected)
            if args.debug:
                print('tvecs:', tvecs)
                print('rvecs:', rvecs)

        # --- UDP output logic: send 1 if any detected marker is "close" on
        #     the chosen axis, otherwise send 0. If no marker detected, send 0.
        try:
            send_value = 0
            effective_threshold = args.threshold * args.range_multiplier
            if ids is not None:
                close_found = False
                for tvec in tvecs:
                    # tvec can be shape (1,3) so flatten to 1D vector
                    t = np.array(tvec).reshape(-1)
                    if args.axis == 'x':
                        val = float(t[0])
                        if abs(val) < effective_threshold:
                            close_found = True
                            break
                    elif args.axis == 'y':
                        val = float(t[1])
                        if abs(val) < effective_threshold:
                            close_found = True
                            break
                    else:  # z axis
                        val = float(t[2])
                        if val < effective_threshold:
                            close_found = True
                            break
                send_value = 1 if close_found else 0
            else:
                send_value = 0

            # Send the UDP byte (single-byte, value 0 or 1) to Simulink
            # We create the socket once, outside the loop for efficiency. If a
            # socket is not already created, create it now.
            if 'udp_socket' not in globals():
                udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                udp_addr = (args.ip, args.port)
            # Use sendto with a single byte representing 0/1
            udp_socket.sendto(bytes([send_value]), udp_addr)
            # Draw LASER status on-frame
            if send_value == 1:
                cv2.putText(frame, "LASER: ON", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                cv2.putText(frame, "LASER: OFF", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            # Put the effective threshold and ON/OFF overlay top-left
            cv2.putText(frame, f"THR: {effective_threshold:.3f}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)
            # show a big ON/OFF indicator
            if send_value == 1:
                cv2.rectangle(frame, (10, 10), (200, 50), (0,255,0), -1)
                cv2.putText(frame, "LASER ON", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,0,0), 2)
            else:
                cv2.rectangle(frame, (10, 10), (200, 50), (0,0,255), -1)
                cv2.putText(frame, "LASER OFF", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255,255,255), 2)
            if args.debug:
                # print last seen val if available, otherwise just threshold
                if 'x_val' in locals():
                    print(f'UDP sent: {send_value} to {udp_addr} [axis {args.axis} x {x_val:.3f} thr {effective_threshold:.3f}]')
                else:
                    print(f'UDP sent: {send_value} to {udp_addr} [axis {args.axis} thr {effective_threshold}]')
            # Overlay showing effective threshold on the frame
            cv2.putText(frame, f"RANGE: {effective_threshold:.3f}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        except Exception as e:
            # Don't break the camera loop on network errors; just print the issue
            if args.debug:
                print('UDP send error:', e)

        # Add the frame rate to the image
        cv2.putText(frame, f"CAMERA FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"PROCESSING FPS: {1/processing_period:.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Display the resulting frame
        cv2.imshow('Frame', frame)

        # Break the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Ensure a steady processing rate
        elapsed_time = time.time() - start_time
        fps = 1 / elapsed_time
        if elapsed_time < processing_period:
            time.sleep(processing_period - elapsed_time)
        start_time = time.time()




except KeyboardInterrupt:
    print('\nInterrupted by user')
finally:
    # Clean-up: close socket (if created), release camera, close windows
    if 'udp_socket' in globals():
        try:
            udp_socket.close()
        except Exception:
            pass
    # When everything is done, release the capture and close windows
    if 'cap' in globals() and cap is not None:
        try:
            cap.release()
        except Exception:
            pass
    cv2.destroyAllWindows()

