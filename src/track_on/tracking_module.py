import torch
import numpy as np
import cv2
from pathlib import Path

from model.track_on_ff import TrackOnFF
from utils.train_utils import restart_from_checkpoint_not_dist
### usage example ####
'''
# 初始化追踪模块
from tracking_module import TrackingModule
tracker = TrackingModule('/path/to/checkpoint.pt')

# 准备初始查询点和第一帧
initial_queries = np.array([[100, 200], [300, 400]])  # 两个初始关键点
first_frame = cv2.imread('first_frame.jpg')  # 第一帧图像

# 初始化追踪
points, visibility = tracker.initialize_tracking(initial_queries, first_frame)
print(f"First frame tracking results: {points}, visible: {visibility}")

# 处理后续帧
for i in range(num_frames):
    # 获取新的一帧
    frame = get_next_frame()  # 你自己的函数来获取下一帧
    
    # 追踪这一帧中的关键点
    points, visibility = tracker.track_next_frame(frame)
    
    # 使用结果
    print(f"Frame {i+1} tracking results: {points}, visible: {visibility}")

'''

class TrackingModule:
    """
    Tracking module for keypoint tracking in videos.
    
    Initializes with a model checkpoint and provides methods to:
    1. Initialize tracking with initial keypoints and the first frame
    2. Track keypoints in subsequent frames
    """
    
    def __init__(self, checkpoint_path, device=None):
        """
        Initialize the tracking module with a model checkpoint.
        
        Args:
            checkpoint_path (str): Path to the TrackOnFF model checkpoint
            device (str, optional): Device to run the model on ('cuda' or 'cpu').
                                   If None, will use CUDA if available.
        """
        self.device = device if device else ('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = None
        self.is_initialized = False
        self.checkpoint_path = checkpoint_path
        self.num_queries = 0
        
        self._init_model()
        
    def _init_model(self):
        """Initialize the TrackOnFF model with the checkpoint."""
        # Create args for model initialization
        class Args:
            def __init__(self, checkpoint_path):
                self.input_size = [384, 512]
                self.N = 384  # Will be overridden by actual queries
                self.T = 18
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
                self.val_memory_size = 96
                self.val_vis_delta = 0.9
                self.random_memory_mask_drop = 0

                self.lambda_point = 5.0
                self.lambda_vis = 1.0
                self.lambda_offset = 1.0
                self.lambda_uncertainty = 1.0
                self.lambda_top_k = 1.0
                
                self.epoch_num = 4
                self.lr = 1e-3
                self.wd = 1e-4
                self.bs = 1
                self.gradient_acc_steps = 1

                self.validation = False
                self.checkpoint_path = checkpoint_path
                self.seed = 1234
                self.loss_after_query = True

                self.gpus = torch.cuda.device_count()
        
        # Initialize model
        self.args = Args(self.checkpoint_path)
        self.model = TrackOnFF(self.args)
        
        # Load checkpoint
        if not Path(self.checkpoint_path).is_file():
            raise FileNotFoundError(f"Checkpoint file not found at {self.checkpoint_path}")
            
        restart_from_checkpoint_not_dist(self.args, run_variables={}, model=self.model)
        
        # Move model to device and set to evaluation mode
        self.model.to(self.device)
        self.model.eval()
        self.model.set_memory_size(self.args.val_memory_size, self.args.val_memory_size)
        self.model.visibility_treshold = self.args.val_vis_delta
        
        print(f"Model loaded on {self.device}")
    
    def initialize_tracking(self, queries, first_frame):
        """
        Initialize tracking with keypoints and the first frame.
        
        Args:
            queries: List of [x, y] coordinates or numpy array of shape (N, 2)
            first_frame: RGB image as numpy array of shape (H, W, 3)
                         or torch tensor of shape (3, H, W)
                
        Returns:
            Tuple of (points, visibility) for the first frame
            - points: numpy array of shape (N, 2) with [x, y] coordinates
            - visibility: numpy array of shape (N,) with boolean visibility flags
        """
        # Convert queries to tensor if needed
        if not isinstance(queries, torch.Tensor):
            queries = torch.tensor(queries, dtype=torch.float32)
        
        # Convert first_frame to tensor if needed
        if isinstance(first_frame, np.ndarray):
            # Convert from (H, W, 3) to (3, H, W)
            if first_frame.shape[2] == 3:  # HWC format
                first_frame = first_frame.transpose(2, 0, 1)
            first_frame = torch.from_numpy(first_frame).float()
        
        # Ensure frame is 4D: (1, 3, H, W)
        if first_frame.dim() == 3:
            first_frame = first_frame.unsqueeze(0)
            
        # Move to device
        queries = queries.to(self.device)
        first_frame = first_frame.to(self.device)
        
        # Store number of queries
        self.num_queries = queries.shape[0]
        
        # Initialize model's queries and memory
        self.model.init_queries_and_memory(queries.clone(), first_frame)
        self.is_initialized = True
        
        # Get tracking results for the first frame
        with torch.no_grad():
            pred_points, pred_visibility = self.model.ff_forward(first_frame)
        
        return pred_points.cpu().numpy(), pred_visibility.cpu().numpy()
    
    def track_next_frame(self, frame):
        """
        Track points in the next frame.
        
        Args:
            frame: RGB image as numpy array of shape (H, W, 3)
                  or torch tensor of shape (3, H, W)
        
        Returns:
            Tuple of (points, visibility) for the current frame
            - points: numpy array of shape (N, 2) with [x, y] coordinates
            - visibility: numpy array of shape (N,) with boolean visibility flags
        
        Raises:
            RuntimeError: If tracking has not been initialized
        """
        if not self.is_initialized:
            raise RuntimeError("Tracking has not been initialized. Call initialize_tracking first.")
        
        # Convert frame to tensor if needed
        if isinstance(frame, np.ndarray):
            # Convert from (H, W, 3) to (3, H, W)
            if frame.shape[2] == 3:  # HWC format
                frame = frame.transpose(2, 0, 1)
            frame = torch.from_numpy(frame).float()
        
        # Ensure frame is 4D: (1, 3, H, W)
        if frame.dim() == 3:
            frame = frame.unsqueeze(0)
            
        # Move to device
        frame = frame.to(self.device)
        
        # Get tracking results
        with torch.no_grad():
            pred_points, pred_visibility = self.model.ff_forward(frame)
        
        return pred_points.cpu().numpy(), pred_visibility.cpu().numpy()
    
    def reset(self):
        """Reset the tracking state without reinitializing the model."""
        self.is_initialized = False
        # No need to reset the model itself, just the tracking state
        # The model will be reinitialized with new queries and first frame
        # when initialize_tracking is called again
        
    def get_tracking_state(self):
        """
        Get the current tracking state.
        
        Returns:
            Dictionary with tracking state information
        """
        return {
            "initialized": self.is_initialized,
            "num_queries": self.num_queries,
            "device": self.device
        } 