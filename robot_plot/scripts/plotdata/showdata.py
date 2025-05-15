#!/usr/bin/env python3
import json
import os
import matplotlib.pyplot as plt
import numpy as np
import math

def load_plot_data(filename):
    with open(filename, 'r') as f:
        data = json.load(f)
    return data

def quaternion_to_yaw(qx, qy, qz, qw):
    """Convert quaternion to yaw angle (rotation around Z axis)"""
    # Calculate yaw (z-axis rotation) from quaternion
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw

def extract_trajectory(data, key='odom'):
    # Extract trajectory data from either odom or ground_truth
    trajectory_data = data.get(key, [])
    if key == 'odom':
        trajectory_data = data.get('odom', [])
    elif key == 'ground_truth':
        trajectory_data = data.get('base_pose_ground_truth', [])
    
    x_vals = [entry.get('x', 0) for entry in trajectory_data]
    y_vals = [entry.get('y', 0) for entry in trajectory_data]
    return x_vals, y_vals

def extract_yaw(data, key='odom'):
    """Extract yaw angles from quaternion data"""
    trajectory_data = data.get(key, [])
    if key == 'odom':
        trajectory_data = data.get('odom', [])
    elif key == 'ground_truth':
        trajectory_data = data.get('base_pose_ground_truth', [])
    
    yaw_vals = []
    for entry in trajectory_data:
        qx = entry.get('qx', 0)
        qy = entry.get('qy', 0)
        qz = entry.get('qz', 0)
        qw = entry.get('qw', 0)
        yaw = quaternion_to_yaw(qx, qy, qz, qw)
        yaw_vals.append(yaw)
        
    return yaw_vals

def calculate_errors(data):
    # Calculate position errors between odom and ground_truth
    odom_data = data.get('odom', [])
    truth_data = data.get('base_pose_ground_truth', [])
    timestamps = data.get('timestamps', [])
    
    errors = []
    for odom, truth in zip(odom_data, truth_data):
        dx = odom.get('x', 0) - truth.get('x', 0)
        dy = odom.get('y', 0) - truth.get('y', 0)
        error = math.sqrt(dx*dx + dy*dy)  # Euclidean distance
        errors.append(error)
    
    return timestamps, errors

def calculate_yaw_errors(data):
    """Calculate yaw angle errors between odom and ground_truth"""
    odom_yaws = extract_yaw(data, 'odom')
    truth_yaws = extract_yaw(data, 'ground_truth')
    timestamps = data.get('timestamps', [])
    
    yaw_errors = []
    for odom_yaw, truth_yaw in zip(odom_yaws, truth_yaws):
        # Calculate the smallest angle difference (handles wrap-around)
        error = math.atan2(math.sin(odom_yaw - truth_yaw), math.cos(odom_yaw - truth_yaw))
        yaw_errors.append(abs(error))  # Use absolute error
    
    return timestamps, yaw_errors

def plot_trajectories_comparison(data):
    plt.figure(figsize=(12, 15))
    
    # Plot 1: Trajectories comparison
    plt.subplot(3, 1, 1)
    
    # Plot odometry path
    odom_x, odom_y = extract_trajectory(data, 'odom')
    plt.plot(odom_x, odom_y, 'b-', linewidth=2, label="Odometry")
    
    # Plot ground truth path
    truth_x, truth_y = extract_trajectory(data, 'ground_truth')
    plt.plot(truth_x, truth_y, 'r--', linewidth=2, label="Ground Truth")
    
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Trajectory Comparison')
    plt.grid(True)
    plt.legend()
    
    # Plot 2: Position error over time
    plt.subplot(3, 1, 2)
    timestamps, errors = calculate_errors(data)
    
    # Convert timestamps to relative time (starting at 0)
    if timestamps:
        rel_timestamps = [t - timestamps[0] for t in timestamps]
        plt.plot(rel_timestamps, errors, 'g-', linewidth=2)
        
        # Calculate and display average error
        avg_error = sum(errors) / len(errors) if errors else 0
        max_error = max(errors) if errors else 0
        plt.axhline(y=avg_error, color='r', linestyle='--')
        plt.text(rel_timestamps[-1]/2, avg_error*1.1, f'Avg: {avg_error:.4f} m', 
                 color='r', fontsize=10)
        plt.text(rel_timestamps[-1]/2, max_error*0.9, f'Max: {max_error:.4f} m', 
                 color='b', fontsize=10)
    
    plt.xlabel('Time (s)')
    plt.ylabel('Position Error (m)')
    plt.title('Position Error Over Time')
    plt.grid(True)
    
    # Plot 3: Yaw error over time
    plt.subplot(3, 1, 3)
    timestamps, yaw_errors = calculate_yaw_errors(data)
    
    # Convert timestamps to relative time (starting at 0)
    if timestamps:
        rel_timestamps = [t - timestamps[0] for t in timestamps]
        # Convert to degrees for better readability
        yaw_errors_deg = [e * 180/math.pi for e in yaw_errors]
        plt.plot(rel_timestamps, yaw_errors_deg, 'm-', linewidth=2)
        
        # Calculate and display average yaw error
        avg_yaw_error = sum(yaw_errors_deg) / len(yaw_errors_deg) if yaw_errors_deg else 0
        max_yaw_error = max(yaw_errors_deg) if yaw_errors_deg else 0
        plt.axhline(y=avg_yaw_error, color='r', linestyle='--')
        plt.text(rel_timestamps[-1]/2, avg_yaw_error*1.1, f'Avg: {avg_yaw_error:.4f}°', 
                 color='r', fontsize=10)
        plt.text(rel_timestamps[-1]/2, max_yaw_error*0.9, f'Max: {max_yaw_error:.4f}°', 
                 color='b', fontsize=10)
    
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw Error (degrees)')
    plt.title('Yaw Error Over Time')
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()

def main():
    # The JSON file is assumed to be in the same directory as this script,
    # or adjust the path accordingly.
    json_filename = os.path.join(os.path.dirname(__file__), "data/plot_data.json")
    data = load_plot_data(json_filename)

    # Plot comprehensive comparison
    plot_trajectories_comparison(data)

if __name__ == '__main__':
    main()