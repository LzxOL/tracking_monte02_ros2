import os
import json
import argparse
import time
from pathlib import Path

import torch
import numpy as np
import imageio.v2 as imageio
import cv2
import matplotlib.pyplot as plt # For distinct colors

from model.track_on_ff import TrackOnFF
from utils.train_utils import restart_from_checkpoint_not_dist

class Args:
    def __init__(self, checkpoint_path_cli):
        self.input_size = [384, 512]

        self.N = 384 # This might be overridden by actual queries
        self.T = 18 # Max frames to process in one go by model, not total video frames
        self.stride = 4
        self.transformer_embedding_dim = 256
        self.cnn_corr = False
        self.linear_visibility = False
        
        self.num_layers = 3
        self.num_layers_offset_head = 3
        
        self.num_layers_rerank = 3
        self.num_layers_rerank_fusion = 1
        self.top_k_regions = 16

        self.num_layers_spatial_writer = 3
        self.num_layers_spatial_self = 1
        self.num_layers_spatial_cross = 1
        
        self.memory_size = 12
        self.val_memory_size = 96 # As used in demo.py for eval
        self.val_vis_delta = 0.9 # As used in demo.py for eval
        self.random_memory_mask_drop = 0

        # These lambdas are for training, may not be needed for inference
        self.lambda_point = 5.0
        self.lambda_vis = 1.0
        self.lambda_offset = 1.0
        self.lambda_uncertainty = 1.0
        self.lambda_top_k = 1.0
        
        # Training specific, but good to have defaults
        self.epoch_num = 4 
        self.lr = 1e-3
        self.wd = 1e-4
        self.bs = 1 # Batch size for inference should be 1
        self.gradient_acc_steps = 1

        self.validation = False # Set to False for inference script
        self.checkpoint_path = checkpoint_path_cli
        self.seed = 1234
        self.loss_after_query = True # From demo.py

        self.gpus = torch.cuda.device_count()

def read_video_frames(video_path):
    """Reads video frames using imageio and returns a PyTorch tensor (T, 3, H, W)."""
    try:
        reader = imageio.get_reader(video_path)
        frames = []
        for i, im in enumerate(reader):
            frames.append(np.array(im))
        video_np = np.stack(frames) # (T, H, W, C)
        video = torch.from_numpy(video_np).permute(0, 3, 1, 2).float()  # (T, C, H, W)
        reader.close()
        print(f"Read {video.shape[0]} frames from {video_path}, shape: {video.shape}")
        return video
    except Exception as e:
        print(f"Error reading video {video_path}: {e}")
        return None

def load_queries_from_json(json_path):
    """Loads keypoints from a JSON annotation file (LabelMe format)."""
    queries = []
    try:
        with open(json_path, 'r') as f:
            data = json.load(f)
        for shape in data.get('shapes', []):
            if shape.get('shape_type') == 'point' and shape.get('points'):
                # Assuming points are [[x, y]]
                point = shape['points'][0]
                queries.append([float(point[0]), float(point[1])])
        if not queries:
            print(f"Warning: No points found or extracted from {json_path}")
            return None
        return torch.tensor(queries, dtype=torch.float32) # (N, 2)
    except Exception as e:
        print(f"Error loading queries from {json_path}: {e}")
        return None

def draw_tracks_on_frame(frame_np, points_np, visibility_np, colors_hex, radius=5, thickness=-1):
    """Draws tracked points on a single frame.
    frame_np: (H, W, C) NumPy array (BGR for cv2)
    points_np: (N, 2) NumPy array of (x, y) coordinates
    visibility_np: (N,) NumPy array of booleans
    colors_hex: List of hex color strings for each point
    """
    vis_frame = frame_np.copy()
    for i in range(points_np.shape[0]):
        if visibility_np[i]:
            pt_x, pt_y = int(round(points_np[i, 0])), int(round(points_np[i, 1]))
            hex_color = colors_hex[i].lstrip('#')
            bgr_color = tuple(int(hex_color[j:j+2], 16) for j in (4, 2, 0)) # RGB to BGR
            cv2.circle(vis_frame, (pt_x, pt_y), radius, bgr_color, thickness)
        else:
            pt_x, pt_y = int(round(points_np[i, 0])), int(round(points_np[i, 1]))
            # Use a different color for invisible points (gray)
            bgr_color = (0, 0, 250)  # Gray color in BGR
            cv2.circle(vis_frame, (pt_x, pt_y), radius, bgr_color, thickness)
    return vis_frame

def save_tracked_video(frames_list_bgr, output_video_path, fps):
    """Saves a list of BGR frames as a video using imageio.
    frames_list_bgr: List of (H, W, C) BGR NumPy arrays.
    """
    if not frames_list_bgr:
        print(f"No frames to save for {output_video_path}")
        return
    frames_list_rgb = [cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) for frame in frames_list_bgr]
    imageio.mimsave(output_video_path, frames_list_rgb, fps=fps, macro_block_size=1) # macro_block_size=1 to avoid green screen for some codecs
    print(f"Saved tracking video to {output_video_path}")

def save_tracking_results(all_points_data, all_visibility_data, output_json_path):
    """Saves all tracking data (coordinates and visibility) to a JSON file."""
    results = {
        'frames': []
    }
    num_frames = len(all_points_data)
    if num_frames == 0:
        print(f"No tracking data to save for {output_json_path}")
        return

    num_queries = all_points_data[0].shape[0] # Assuming all frames have same num_queries

    for t in range(num_frames):
        frame_data = {'frame_index': t, 'points': []}
        points_t = all_points_data[t] # (N, 2)
        visibility_t = all_visibility_data[t] # (N)
        for q_idx in range(num_queries):
            frame_data['points'].append({
                'id': q_idx,
                'x': float(points_t[q_idx, 0]),
                'y': float(points_t[q_idx, 1]),
                'visible': bool(visibility_t[q_idx])
            })
        results['frames'].append(frame_data)
    
    try:
        with open(output_json_path, 'w') as f:
            json.dump(results, f, indent=2)
        print(f"Saved tracking results to {output_json_path}")
    except Exception as e:
        print(f"Error saving tracking results to {output_json_path}: {e}")

# Placeholder for Args class, will be defined later
# class Args:
#     pass

def main(args_cli):
    """
    Main function to process videos and annotations.
    """
    print(f"Input directory: {args_cli.input_dir}")
    print(f"Output directory: {args_cli.output_dir}")
    print(f"Checkpoint path: {args_cli.checkpoint_path}")

    # Ensure output directory exists
    Path(args_cli.output_dir).mkdir(parents=True, exist_ok=True)

    # --- Model Initialization ---
    model_args = Args(args_cli.checkpoint_path)
    model = TrackOnFF(model_args)
    if not Path(model_args.checkpoint_path).is_file():
        print(f"Error: Checkpoint file not found at {model_args.checkpoint_path}")
        return

    restart_from_checkpoint_not_dist(model_args, run_variables={}, model=model)
    
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model.to(device)
    model.eval()
    model.set_memory_size(model_args.val_memory_size, model_args.val_memory_size)
    model.visibility_treshold = model_args.val_vis_delta
    print(f"Model loaded on {device}")
    # --- --- --- 

    # --- File Discovery and Processing Loop ---
    input_path = Path(args_cli.input_dir)
    video_files = sorted(list(input_path.glob("*.mp4")))

    if not video_files:
        print(f"No .mp4 files found in {args_cli.input_dir}")
        return

    for video_file_path in video_files:
        base_name = video_file_path.stem
        json_file_path = input_path / f"{base_name}.json"

        if not json_file_path.is_file():
            print(f"Annotation file {json_file_path} not found for video {video_file_path}. Skipping.")
            continue

        print(f"\nProcessing video: {video_file_path} with annotations: {json_file_path}")

        # Create output subdirectory for this video
        video_output_dir = Path(args_cli.output_dir) / base_name
        video_output_dir.mkdir(parents=True, exist_ok=True)

        # Load video and queries
        video_frames_tensor = read_video_frames(str(video_file_path))
        if video_frames_tensor is None:
            continue
        
        initial_queries = load_queries_from_json(str(json_file_path))
        if initial_queries is None or initial_queries.nelement() == 0:
            print(f"No valid queries loaded for {base_name}. Skipping.")
            continue
        
        num_actual_queries = initial_queries.shape[0]
        video_height, video_width = video_frames_tensor.shape[2], video_frames_tensor.shape[3]

        # Generate distinct colors for points
        # Ensure we have enough colors, repeat if necessary
        cmap = plt.cm.get_cmap('tab20', num_actual_queries if num_actual_queries <= 20 else 20)
        colors_rgba = [cmap(i % cmap.N) for i in range(num_actual_queries)]
        hex_colors = ['#%02x%02x%02x' % (int(r*255), int(g*255), int(b*255)) for r, g, b, _ in colors_rgba]

        # --- Model Inference per video ---
        # Reset model state for each new video by re-initializing queries and memory.
        # The model's internal 't' counter is also reset in init_queries_and_memory.
        model.init_queries_and_memory(initial_queries.clone().to(device), 
                                      video_frames_tensor[0].unsqueeze(0).to(device))
        # N in model_args is not updated, but model.N internal attribute is set by init_queries_and_memory
        # if extend_queries is True, model.N becomes N_actual + N_extra.
        # The output processing in ff_forward handles this by slicing to original N (self.N in ff_forward which is set during init_queries_and_memory)

        all_pred_points_cpu = []
        all_pred_visibility_cpu = []
        annotated_frames_for_video = []
        
        # Get video FPS for output (use a default if not available)
        try:
            meta = imageio.get_reader(str(video_file_path)).get_meta_data()
            fps = meta.get('fps', 30)
        except:
            fps = 30 # Default FPS

        with torch.no_grad():
            for t in range(video_frames_tensor.shape[0]):
                current_frame_tensor = video_frames_tensor[t].unsqueeze(0).to(device) # (1, C, H, W)
                
                # Model forward for the current frame
                # ff_forward is expected to handle coordinate scaling internally and return points in original image scale
                pred_points_frame, pred_visibility_frame = model.ff_forward(current_frame_tensor)
                
                # Store results (move to CPU, convert to numpy)
                pred_points_cpu = pred_points_frame.cpu().numpy()
                pred_visibility_cpu = pred_visibility_frame.cpu().numpy()

                all_pred_points_cpu.append(pred_points_cpu)
                all_pred_visibility_cpu.append(pred_visibility_cpu)

                # Prepare frame for visualization
                # Convert current frame from (C, H, W) tensor to (H, W, C) BGR numpy array
                frame_to_draw_np = video_frames_tensor[t].permute(1, 2, 0).cpu().numpy().astype(np.uint8)
                frame_to_draw_np_bgr = cv2.cvtColor(frame_to_draw_np, cv2.COLOR_RGB2BGR)

                # Draw tracks
                # The points from ff_forward should be already in original video coordinates
                annotated_frame = draw_tracks_on_frame(frame_to_draw_np_bgr, 
                                                       pred_points_cpu, 
                                                       pred_visibility_cpu, 
                                                       hex_colors)
                annotated_frames_for_video.append(annotated_frame)
                
                print(f"Processed frame {t+1}/{video_frames_tensor.shape[0]} for {base_name}")

        # Save results for the current video
        output_video_file = video_output_dir / f"{base_name}_tracked.mp4"
        save_tracked_video(annotated_frames_for_video, str(output_video_file), fps)

        output_json_file = video_output_dir / f"{base_name}_tracking_results.json"
        save_tracking_results(all_pred_points_cpu, all_pred_visibility_cpu, str(output_json_file))
        # --- --- ---

    print("\nAll videos processed.")
    # --- --- --- 

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Track keypoints in videos using TrackOnFF.")
    parser.add_argument("--input_dir", type=str, required=True,
                        help="Directory containing .mp4 video files and .json annotation files.")
    parser.add_argument("--output_dir", type=str, required=True,
                        help="Directory to save tracking results (videos and json data).")
    parser.add_argument("--checkpoint_path", type=str, required=True,
                        help="Path to the TrackOnFF model checkpoint (.pt file).")
    # Potentially add other arguments for model params if needed

    cli_args = parser.parse_args()
    main(cli_args) 