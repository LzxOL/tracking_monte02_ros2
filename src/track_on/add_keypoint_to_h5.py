import os
import json
import h5py
import argparse
import shutil
import numpy as np

def extract_keypoints_from_json(json_data, num_frames):
    """
    Extracts keypoints (x, y, visible_status) for point ID 0 from each frame.
    Returns a NumPy array of shape (num_frames, 1, 3).
    - x, y are coordinates.
    - visible_status is 1.0 if true, 0.0 if false.
    If point ID 0 is not found or x,y are missing, [-1.0, -1.0, 0.0] is used.
    """
    # Initialize with x=-1, y=-1, visible=0.0 (false)
    keypoints = np.full((num_frames, 1, 3), [-1.0, -1.0, 0.0], dtype=np.float32)
    
    if 'frames' not in json_data or not isinstance(json_data['frames'], list):
        print("Warning: JSON data does not contain a 'frames' list.")
        return keypoints

    for frame_data in json_data['frames']:
        frame_idx = frame_data.get('frame_index')
        if frame_idx is None or frame_idx >= num_frames:
            continue

        # point_found = False # Not strictly needed anymore with np.full initialization
        if 'points' in frame_data and isinstance(frame_data['points'], list):
            for point_info in frame_data['points']:
                if point_info.get('id') == 0:
                    x = point_info.get('x')
                    y = point_info.get('y')
                    visible = point_info.get('visible', False) # Default to False if 'visible' key is missing
                    
                    if x is not None and y is not None:
                        keypoints[frame_idx, 0, 0] = float(x)
                        keypoints[frame_idx, 0, 1] = float(y)
                        keypoints[frame_idx, 0, 2] = 1.0 if visible else 0.0
                    # If x or y is None, it keeps the default [-1, -1, 0.0]
                    # point_found = True # Not strictly needed
                    break 
        # If point ID 0 was not found in frame_data['points'], it remains [-1.0, -1.0, 0.0]
    return keypoints

def process_trajectory(traj_dir_path, output_root_dir):
    """
    Processes a single trajectory: reads keypoints from JSON, adds to a new H5 file.
    """
    h5_files = [f for f in os.listdir(traj_dir_path) if f.endswith('.h5')]
    json_files = [f for f in os.listdir(traj_dir_path) if f.endswith('_tracking_results.json')]

    if not h5_files:
        print(f"Warning: No H5 file found in {traj_dir_path}")
        return
    if not json_files:
        print(f"Warning: No '*_tracking_results.json' file found in {traj_dir_path}")
        return

    if len(h5_files) > 1:
        print(f"Warning: Multiple H5 files found in {traj_dir_path}. Using {h5_files[0]}.")
    if len(json_files) > 1:
        print(f"Warning: Multiple JSON tracking files found in {traj_dir_path}. Using {json_files[0]}.")

    h5_file_name = h5_files[0]
    json_file_name = json_files[0]

    original_h5_path = os.path.join(traj_dir_path, h5_file_name)
    json_path = os.path.join(traj_dir_path, json_file_name)

    print(f"Processing trajectory: {traj_dir_path}")

    # 1. Read JSON
    try:
        with open(json_path, 'r') as f:
            tracking_data = json.load(f)
    except Exception as e:
        print(f"Error reading JSON file {json_path}: {e}")
        return

    if 'frames' not in tracking_data or not tracking_data['frames']:
        print(f"Warning: No frames found in JSON {json_path}. Skipping.")
        return
        
    num_json_frames = 0
    if isinstance(tracking_data.get('frames'), list) and tracking_data['frames']:
         # Ensure frame_index exists and is an integer for proper max finding
        valid_frames_indices = [f.get('frame_index') for f in tracking_data['frames'] if isinstance(f.get('frame_index'), int)]
        if not valid_frames_indices:
            print(f"Warning: No valid 'frame_index' found in JSON frames from {json_path}. Skipping.")
            return
        num_json_frames = max(valid_frames_indices) + 1
    else:
        print(f"Warning: 'frames' key missing or empty in {json_path}. Skipping.")
        return


    # 2. Prepare output path and copy H5 file
    traj_base_name = os.path.basename(traj_dir_path)
    output_traj_dir = os.path.join(output_root_dir, traj_base_name)
    os.makedirs(output_traj_dir, exist_ok=True)
    
    output_h5_path = os.path.join(output_traj_dir, h5_file_name)
    try:
        shutil.copy2(original_h5_path, output_h5_path)
    except Exception as e:
        print(f"Error copying H5 file from {original_h5_path} to {output_h5_path}: {e}")
        return

    # 3. Process H5 file
    try:
        with h5py.File(output_h5_path, 'r+') as hf:
            # Determine number of timesteps from H5
            num_h5_timesteps = -1
            if 'images' in hf and isinstance(hf['images'], h5py.Group) and 'obs_left_img' in hf['images']:
                if isinstance(hf['images']['obs_left_img'], h5py.Dataset):
                    num_h5_timesteps = hf['images']['obs_left_img'].shape[0]
                else:
                    print(f"Warning: '/images/obs_left_img' is not a dataset in {output_h5_path}. Cannot determine H5 timesteps.")
                    return
            else:
                # Fallback: try inferring from another common dataset structure if needed
                # For now, strictly expect /images/obs_left_img
                print(f"Warning: '/images/obs_left_img' not found in {output_h5_path}. Cannot determine H5 timesteps.")
                return

            if num_h5_timesteps == -1:
                print(f"Could not determine number of timesteps for H5 file {output_h5_path}. Skipping.")
                return

            # 4. Timestep Check
            if num_json_frames != num_h5_timesteps:
                print(f"Warning: Mismatch in timesteps for {traj_base_name}. JSON: {num_json_frames}, H5: {num_h5_timesteps}. Skipping.")
                # Optionally, clean up the copied H5 if skipping
                # os.remove(output_h5_path) 
                return

            # 5. Extract keypoints using the determined number of frames
            keypoints_to_add = extract_keypoints_from_json(tracking_data, num_h5_timesteps)

            # 6. Add Keypoint Dataset
            images_group = hf.get('images')
            if images_group is None:
                images_group = hf.create_group('images') # Should exist based on prior inspection
            
            dataset_name = 'left_img_keypoint'
            if dataset_name in images_group:
                print(f"Warning: Dataset '{dataset_name}' already exists in {output_h5_path}. Overwriting.")
                del images_group[dataset_name]
            
            images_group.create_dataset(dataset_name, data=keypoints_to_add, dtype='float32')
            print(f"Successfully added '{dataset_name}' to {output_h5_path}")

    except Exception as e:
        print(f"Error processing H5 file {output_h5_path}: {e}")
        # Optionally, clean up the copied H5 on error
        # if os.path.exists(output_h5_path):
        #     os.remove(output_h5_path)

def main():
    parser = argparse.ArgumentParser(description="Add keypoint data from JSON to H5 files.")
    parser.add_argument('--input_dir', type=str, required=True, 
                        help="Root directory containing trajectory subdirectories.")
    parser.add_argument('--output_dir', type=str, required=True,
                        help="Directory to save new H5 files with added keypoints.")
    
    args = parser.parse_args()

    if not os.path.isdir(args.input_dir):
        print(f"Error: Input directory '{args.input_dir}' not found.")
        return

    os.makedirs(args.output_dir, exist_ok=True)
    print(f"Output will be saved to: {args.output_dir}")

    for item_name in os.listdir(args.input_dir):
        item_path = os.path.join(args.input_dir, item_name)
        if os.path.isdir(item_path):
            process_trajectory(item_path, args.output_dir)
        else:
            print(f"Skipping non-directory item: {item_name}")

if __name__ == '__main__':
    main() 