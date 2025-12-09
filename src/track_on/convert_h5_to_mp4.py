#!/usr/bin/env python3
import os
import h5py
import cv2
import numpy as np
import argparse
from tqdm import tqdm

def convert_h5_to_mp4(h5_file, output_dir=None, fps=30, left_only=True):
    """
    Convert an HDF5 file containing camera images to MP4 videos.
    
    Args:
        h5_file (str): Path to the HDF5 file
        output_dir (str, optional): Directory to save the output videos. Defaults to same directory as h5_file.
        fps (int, optional): Frames per second for the output videos. Defaults to 30.
        left_only (bool, optional): If True, only convert left camera. Defaults to True.
    """
    if output_dir is None:
        output_dir = os.path.dirname(h5_file)
    
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Get the base name without extension
    base_name = os.path.splitext(os.path.basename(h5_file))[0]
    
    # Output file paths
    left_video_path = os.path.join(output_dir, f"{base_name}.mp4") if left_only else os.path.join(output_dir, f"{base_name}_left.mp4")
    right_video_path = os.path.join(output_dir, f"{base_name}_right.mp4")
    # PNG thumbnail path
    left_thumbnail_path = os.path.join(output_dir, f"{base_name}.png") if left_only else os.path.join(output_dir, f"{base_name}_left.png")
    right_thumbnail_path = os.path.join(output_dir, f"{base_name}_right.png")
    
    print(f"Converting {h5_file}...")
    
    try:
        with h5py.File(h5_file, 'r') as f:
            # Get image dimensions
            left_images = f['images']['obs_left_img']
            if not left_only:
                right_images = f['images']['obs_right_img']
            
            num_frames, height, width, channels = left_images.shape
            
            # Create VideoWriter objects
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            left_writer = cv2.VideoWriter(left_video_path, fourcc, fps, (width, height))
            if not left_only:
                right_writer = cv2.VideoWriter(right_video_path, fourcc, fps, (width, height))
            
            # Save first frame as PNG
            first_frame_saved = False
            
            # Write frames
            for i in tqdm(range(num_frames), desc=f"Processing {base_name}"):
                # Read frames
                left_frame = left_images[i]
                if not left_only:
                    right_frame = right_images[i]
                
                # OpenCV uses BGR format
                if channels == 3:  # If RGB format, convert to BGR
                    left_frame = cv2.cvtColor(left_frame, cv2.COLOR_RGB2BGR)
                    if not left_only:
                        right_frame = cv2.cvtColor(right_frame, cv2.COLOR_RGB2BGR)
                
                # Save the first frame as PNG
                if i == 0:
                    cv2.imwrite(left_thumbnail_path, left_frame)
                    print(f"Thumbnail saved to {left_thumbnail_path}")
                    if not left_only:
                        cv2.imwrite(right_thumbnail_path, right_frame)
                        print(f"Thumbnail saved to {right_thumbnail_path}")
                
                # Write frames
                left_writer.write(left_frame)
                if not left_only:
                    right_writer.write(right_frame)
            
            # Release VideoWriter objects
            left_writer.release()
            if not left_only:
                right_writer.release()
        
        if left_only:
            print(f"Video saved to {left_video_path}")
            return left_video_path
        else:
            print(f"Videos saved to {left_video_path} and {right_video_path}")
            return left_video_path, right_video_path
    
    except Exception as e:
        print(f"Error processing {h5_file}: {e}")
        return None

def process_directory(directory, output_dir=None, fps=10, left_only=True):
    """Process all h5 files in a directory"""
    if not os.path.isdir(directory):
        print(f"Directory not found: {directory}")
        return
    
    h5_files = [os.path.join(directory, f) for f in os.listdir(directory) if f.endswith('.h5')]
    
    if not h5_files:
        print(f"No h5 files found in {directory}")
        return
    
    print(f"Found {len(h5_files)} h5 files to process")
    
    for h5_file in h5_files:
        convert_h5_to_mp4(h5_file, output_dir, fps, left_only)

def main():
    parser = argparse.ArgumentParser(description='Convert HDF5 files to MP4 videos')
    parser.add_argument('input', help='HDF5 file or directory containing HDF5 files')
    parser.add_argument('--output-dir', help='Directory to save output videos')
    parser.add_argument('--fps', type=int, default=10, help='Frames per second for output videos')
    parser.add_argument('--both-cameras', action='store_true', help='Convert both left and right cameras')
    
    args = parser.parse_args()
    
    if os.path.isdir(args.input):
        process_directory(args.input, args.output_dir, args.fps, not args.both_cameras)
    elif os.path.isfile(args.input):
        if not args.input.endswith('.h5'):
            print(f"File is not an HDF5 file: {args.input}")
            return
        convert_h5_to_mp4(args.input, args.output_dir, args.fps, not args.both_cameras)
    else:
        print(f"Input not found: {args.input}")

if __name__ == '__main__':
    main() 