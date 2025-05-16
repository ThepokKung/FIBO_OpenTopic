#!/usr/bin/env python3
import json
import math
import os

def quaternion_to_yaw(orientation):
    # Assuming orientation is a dict with keys: "w", "x", "y", "z".
    # Yaw is calculated by: yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    w = orientation["w"]
    x = orientation["x"]
    y = orientation["y"]
    z = orientation["z"]
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return yaw

def convert_orientations(input_file, output_file):
    with open(input_file, "r") as f:
        data = json.load(f)
    
    # For list structure
    for item in data:
        if "orientation" in item:
            item["orientation"] = quaternion_to_yaw(item["orientation"])
            
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    with open(output_file, "w") as f:
        json.dump(data, f, indent=4)

def main():
    # Assuming the JSON files are in the same directory as this script.
    current_dir = os.path.dirname(__file__)
    input_file = os.path.join(current_dir, "data/log_of_arUco_position.json")
    output_file = os.path.join(current_dir, "convert", "arucoyaw.json")
    convert_orientations(input_file, output_file)

if __name__ == "__main__":
    main()
