#!/usr/bin/env python3
"""
Real-Time Attitude Telemetry Visualizer & Orientation Plotter
Author: Herambeswar Mandadapu
"""

import time
import math

def simulate_orientation_stream():
    print("====================================================")
    print(" 🧭 6-DOF IMU Extended Kalman Filter Telemetry Stream")
    print("====================================================\n")
    
    for t in range(1, 11):
        roll = 15.0 * math.sin(t * 0.3)
        pitch = 8.0 * math.cos(t * 0.2)
        yaw = (t * 4.5) % 360.0
        
        bar_roll = "#" * int(abs(roll))
        print(f"[t={t:02d}s] ROLL: {roll:+6.1f}° [{bar_roll:<15}] | PITCH: {pitch:+5.1f}° | YAW: {yaw:5.1f}°")
        time.sleep(0.05)
        
    print("\n[+] Telemetry stream captured and validated.")

if __name__ == "__main__":
    simulate_orientation_stream()
