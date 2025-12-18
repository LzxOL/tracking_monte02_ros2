
"""实时推理脚本：使用训练好的 ACT 模型进行机器人控制

输入：8维状态（7关节+1gripper）+ 2维关键点 + 图像
输出：8维动作（7关节+1gripper）

支持两种模式：
1. 在线模式（默认）：从共享内存和ROS2话题获取实时数据
2. 离线模式：从h5文件或视频文件加载数据进行推理
"""

from pathlib import Path
import time
import numpy as np
import torch
import sys
import cv2
import einops
import os
import threading
import argparse
import rclpy
from rclpy.node import Node

try:
    from track_on_ros2_msgs.msg import Keypoints
    KEYPOINTS_AVAILABLE = True
except (ImportError, AttributeError) as e:
    print(f"Warning: track_on_ros2_msgs not available: {e}")
    print("Keypoint tracking will be disabled. Make sure to source the ROS2 workspace.")
    KEYPOINTS_AVAILABLE = False
    class Keypoints:
        pass

# 添加项目路径
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = current_dir
if project_root not in sys.path:
    sys.path.insert(0, project_root)

teleop_src = os.path.join(os.path.dirname(project_root), "teleoperation/src")
if teleop_src not in sys.path:
    sys.path.insert(0, teleop_src)

from lerobot.common.policies.act.configuration_act import ACTConfig
from lerobot.common.policies.act.modeling_act import ACTPolicy
import hydra

from corenetic_teleoperation.robot.teleop_robot import TeleopRobot
from corenetic_teleoperation.utils.robot_obs import ImageObs


class KeypointSubscriber(Node):
    """ROS2 节点：接收关键点跟踪信息"""
    def __init__(self, topic_name='/tracking/keypoints'):
        super().__init__('keypoint_listener_infer')
        if not KEYPOINTS_AVAILABLE:
            raise RuntimeError("Keypoints message type not available")
        try:
            self.subscription = self.create_subscription(
                Keypoints, topic_name, self.listener_callback, 10)
            self.latest_msg = None
            self.lock = threading.Lock()
            self.received_first = False
            self.get_logger().info(f'Listening for keypoints on {topic_name}...')
        except Exception as e:
            self.get_logger().error(f'Failed to create subscription: {e}')
            raise

    def listener_callback(self, msg):
        with self.lock:
            self.latest_msg = msg
            self.received_first = True

    def get_latest_keypoints(self):
        """获取最新的关键点数据 (x, y)"""
        with self.lock:
            if self.latest_msg is None:
                return None
            kps = sorted(self.latest_msg.keypoints, key=lambda k: k.id)
            return np.array([[k.x, k.y] for k in kps], dtype=np.float32)


def prepare_image_for_inference(image: np.ndarray, target_size: tuple = (480, 480)) -> torch.Tensor:
    """准备推理用的图像,格式与训练时一致"""
    if image.shape[:2] != target_size:
        image = cv2.resize(image, target_size, interpolation=cv2.INTER_LINEAR)
    
    if image.dtype != np.uint8:
        if image.max() <= 1.0:
            image = (image * 255.0).astype(np.uint8)
        else:
            image = image.astype(np.uint8)
    
    if len(image.shape) == 3:
        image = image[np.newaxis, ...]
    
    img = torch.from_numpy(image)
    assert img.dtype == torch.uint8, f"expect torch.uint8, but instead {img.dtype=}"
    img = einops.rearrange(img, "b h w c -> b c h w").contiguous()
    img = img.type(torch.float32) / 255.0
    return img.squeeze(0)


def load_model(checkpoint_path: str | Path, device: str = "cuda:0"):
    """加载 ACT 模型"""
    checkpoint_path = Path(checkpoint_path)
    device = torch.device(device)
    
    cfg = ACTConfig.from_pretrained(checkpoint_path)
    policy = ACTPolicy.from_pretrained(checkpoint_path, config=cfg, map_location=str(device))
    policy.eval()
    policy.to(device)
    policy.reset()
    
    print(f"✓ Loaded ACT model from {checkpoint_path}")
    
    # 打印模型配置信息
    print("\n" + "=" * 60)
    print("模型配置信息")
    print("=" * 60)
    print(f"Image features 顺序: {list(policy.config.image_features.keys())}")
    print(f"Normalization mapping: {policy.config.normalization_mapping}")
    print(f"Input features: {list(policy.config.input_features.keys())}")
    print(f"Output features: {list(policy.config.output_features.keys())}")
    print(f"Chunk size: {policy.config.chunk_size}")
    print(f"N action steps: {policy.config.n_action_steps}")
    
    # 检查归一化统计信息
    if hasattr(policy, 'unnormalize_outputs'):
        try:
            # 尝试获取 action 的统计信息
            buffer_name = "buffer_action"
            if hasattr(policy.unnormalize_outputs, buffer_name):
                action_buffer = getattr(policy.unnormalize_outputs, buffer_name)
                if action_buffer is not None:
                    print(f"\nAction 归一化统计信息:")
                    if "mean" in action_buffer:
                        print(f"  Mean: {action_buffer['mean'].cpu().numpy()}")
                    if "std" in action_buffer:
                        print(f"  Std: {action_buffer['std'].cpu().numpy()}")
                    if "min" in action_buffer:
                        print(f"  Min: {action_buffer['min'].cpu().numpy()}")
                    if "max" in action_buffer:
                        print(f"  Max: {action_buffer['max'].cpu().numpy()}")
        except Exception as e:
            print(f"无法读取归一化统计信息: {e}")
    
    print("=" * 60 + "\n")
    
    return policy


def init_robot(robot_interface):
    """初始化机器人"""
    robot_interface.enable_servo_control("left")
    robot_interface.enable_servo_control("right")
    robot_interface.reset()


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='ACT模型推理脚本')
    parser.add_argument('--checkpoint', type=str, default=None,
                        help='模型检查点路径（默认使用配置路径）')
    parser.add_argument('--device', type=str, default='cuda:0',
                        help='设备（默认: cuda:0）')
    parser.add_argument('--data_len', type=int, default=500,
                        help='处理的数据长度（默认: 500）')
    args = parser.parse_args()
    
    # 配置
    if args.checkpoint:
        checkpoint_path = args.checkpoint
    else:
        checkpoint_path = os.path.join(
            project_root, 
            "outputs/train/monte02_insert_needle_act_10d_input/checkpoints/005000/pretrained_model"
        )
    device = args.device
    data_len = args.data_len
    
    # 加载模型
    policy = load_model(checkpoint_path, device)
    
    # 加载配置
    config_abs_path = os.path.join(project_root, "corenetic_teleoperation", "infer_need")
    if not os.path.exists(config_abs_path):
        raise FileNotFoundError(f"配置文件目录不存在: {config_abs_path}")
    
    original_cwd = os.getcwd()
    os.chdir(project_root)
    
    try:
        # 创建 robot 配置的软链接（如果不存在）
        infer_need_path = os.path.join(project_root, "corenetic_teleoperation", "infer_need")
        robot_link = os.path.join(infer_need_path, "robot")
        if not os.path.exists(robot_link):
            try:
                target = os.path.join("..", "configs", "robot")
                os.symlink(target, robot_link)
            except Exception as e:
                print(f"Failed to create symlink: {e}")
        
        hydra.initialize_config_dir(config_dir=infer_need_path, version_base="1.3")
        config = hydra.compose(config_name="config.yaml")
        teleop_config = hydra.compose(config_name="teleoperation_config.yaml")
    finally:
        os.chdir(original_cwd)
    
    # 在线模式：初始化机器人接口和图像观测
    print("=" * 60)
    print("在线模式：从共享内存和ROS2获取实时数据")
    print("=" * 60)
    robot_interface = TeleopRobot(config)
    image_obs = ImageObs(**teleop_config.image_obs)
    
    # 诊断：检查共享内存连接状态
    print("\n检查共享内存连接状态...")
    if image_obs is not None and image_obs.robot_obs is not None:
        available_obs = image_obs.robot_obs.list_available_observations()
        print(f"已连接的共享内存观测: {available_obs}")
        
        # 检查必需的图像是否可用
        required_images = ['head_rgb_img', 'left_rgb_img']
        missing_images = [img for img in required_images if img not in available_obs]
        if missing_images:
            print(f"\n⚠️  警告: 以下图像观测不可用: {missing_images}")
            print("可能的原因:")
            print("  1. ObsManager 节点未运行")
            print("  2. ROS2 图像话题未发布")
            print("  3. 共享内存配置不匹配")
            print("\n请确保:")
            print("  - 已启动 ObsManager 节点")
            print("  - 图像话题正在发布 (使用 'ros2 topic list' 检查)")
            print("  - 话题名称与 data_source.yaml 配置一致")
        else:
            print("✓ 所有必需的图像观测已连接")
            
        # 测试读取一次图像数据
        print("\n测试读取图像数据...")
        test_image_states = image_obs.get_all_data_with_metadata()
        for img_name in required_images:
            if img_name in test_image_states:
                img_data = test_image_states[img_name]
                if img_data is not None:
                    print(f"✓ {img_name}: shape={img_data.shape}, 可用")
                else:
                    print(f"✗ {img_name}: 数据为 None (ObsManager 可能未收到话题数据)")
            else:
                print(f"✗ {img_name}: 不在返回的数据中")
    else:
        print("⚠️  警告: ImageObs 未初始化")
    
    input("Press Enter to initialize robot...")
    init_robot(robot_interface)
    
    # 初始化 ROS2 关键点订阅（如果可用）
    kp_sub = None
    spinner_thread = None
    if KEYPOINTS_AVAILABLE:
        try:
            rclpy.init()
            kp_sub = KeypointSubscriber()
            spinner_thread = threading.Thread(target=rclpy.spin, args=(kp_sub,), daemon=True)
            spinner_thread.start()
            
            print("Waiting for keypoints...")
            timeout = 10.0  # 10秒超时
            start_time = time.time()
            while not kp_sub.received_first:
                if time.time() - start_time > timeout:
                    print("Warning: Timeout waiting for keypoints. Continuing without keypoints...")
                    break
                time.sleep(0.1)
            if kp_sub.received_first:
                print("Keypoints received!")
        except Exception as e:
            print(f"Warning: Failed to initialize keypoint subscriber: {e}")
            print("Continuing without keypoint tracking...")
            kp_sub = None
    else:
        print("Keypoint tracking disabled (ROS2 messages not available)")
    
    input("Press Enter to start inference...")
    
    # 初始化变量
    first_frame_captured = False
    head_init_img = None
    left_init_img = None
    
    print("=" * 60)
    print("开始推理循环")
    print("=" * 60)
    
    with torch.no_grad():
        for i in range(data_len):
            time1 = time.time()
            
            # 获取机器人状态：使用单独接口读取（避免 get_all_data 超时）
            robot_states = {}
            try:
                # get_joint_positions("left") 返回 7 维数组（关节 6-12）
                js = robot_interface.get_joint_positions("left")
                if js is not None:
                    robot_states["left_jpos"] = np.array(js) if not isinstance(js, np.ndarray) else js
                else:
                    robot_states["left_jpos"] = np.zeros(7)
            except Exception as e_js:
                print(f"[WARN] get_joint_positions fail: {e_js}")
                robot_states["left_jpos"] = np.zeros(7)
            
            try:
                # robot.get_gripper_position(1) 可能返回：
                # - Monte02Robot: position (float) 或 None
                # - 底层 driver: (success, position) 元组
                gp_result = robot_interface.robot.get_gripper_position(1)
                if gp_result is None:
                    robot_states["gripper_position"] = 0.0
                elif isinstance(gp_result, tuple):
                    # 如果是元组，取第二个值（position）
                    if len(gp_result) >= 2:
                        robot_states["gripper_position"] = float(gp_result[1])
                    else:
                        robot_states["gripper_position"] = 0.0
                else:
                    # 单个值
                    robot_states["gripper_position"] = float(gp_result)
            except Exception as e_gp:
                print(f"[WARN] get_gripper_position fail: {e_gp}")
                robot_states["gripper_position"] = 0.0
            
            # 获取图像数据
            image_states = image_obs.get_all_data_with_metadata()
            
            # 处理图像（与原版保持一致）
            head_rgb_img_raw = image_states['head_rgb_img']
            left_rgb_img_raw = image_states['left_rgb_img']
            
            # 获取关键点
            if kp_sub is not None:
                keypoints = kp_sub.get_latest_keypoints()
            else:
                keypoints = None
            
            # 转换为 torch tensor 并 resize 到 480x480（与原版保持一致）
            head_rgb_img = prepare_image_for_inference(head_rgb_img_raw).unsqueeze(0).to(device, non_blocking=True)
            left_rgb_img = prepare_image_for_inference(left_rgb_img_raw).unsqueeze(0).to(device, non_blocking=True)
            
            # 捕获第一帧作为初始图像（与原版保持一致）
            if not first_frame_captured:
                head_init_img = head_rgb_img.clone()
                left_init_img = left_rgb_img.clone()
                first_frame_captured = True
            
            # 准备状态：8维（7关节 + 1gripper）
            left_jpos_raw = robot_states.get('left_jpos', None)
            if left_jpos_raw is None:
                left_jpos_arr = np.zeros(7)
            elif isinstance(left_jpos_raw, np.ndarray):
                if left_jpos_raw.ndim == 1 and left_jpos_raw.size == 7:
                    left_jpos_arr = left_jpos_raw
                else:
                    left_jpos_arr = np.zeros(7)
            elif isinstance(left_jpos_raw, (list, tuple)):
                left_jpos_arr = np.array(left_jpos_raw)
                if left_jpos_arr.ndim != 1 or left_jpos_arr.size != 7:
                    left_jpos_arr = np.zeros(7)
            else:
                left_jpos_arr = np.zeros(7)
            
            gripper_pos_val = robot_states.get('gripper_position', 0.0)
            if not isinstance(gripper_pos_val, (int, float, np.number)):
                gripper_pos_val = 0.0
            
            left_arm_pose = torch.as_tensor(left_jpos_arr, dtype=torch.float32).reshape(-1)  # (7,)
            gripper_pos = torch.as_tensor([float(gripper_pos_val)], dtype=torch.float32).reshape(-1)  # (1,)
            state = torch.cat([left_arm_pose, gripper_pos], dim=0)  # (8,)
            
            # 验证状态维度
            if state.shape[0] != 8:
                print(f"[ERROR] State dimension mismatch: expected 8, got {state.shape[0]}")
                state = torch.zeros(8, dtype=torch.float32)
            
            # 获取关键点：2维（x, y）
            img_width, img_height = 480, 480
            
            # 构建观测字典
            obs = {
                "observation.state": state.unsqueeze(0).to(device, non_blocking=True),  # (1, 8)
            }
            
            # 添加关键点（2维）
            kp_xy_debug = None  # 用于调试
            if keypoints is not None and len(keypoints) > 0:
                # 处理不同形状的关键点数据
                if isinstance(keypoints, np.ndarray):
                    if len(keypoints.shape) == 1:  # (2,)
                        kp_xy = keypoints.copy()
                    elif len(keypoints.shape) == 2:  # (N, 2) 或 (1, 2)
                        kp_xy = keypoints[0].copy() if keypoints.shape[0] > 0 else np.zeros(2)
                    else:
                        kp_xy = np.zeros(2)
                else:
                    kp_xy = np.array([0.0, 0.0])
                
                kp_xy_debug = kp_xy.copy()  # 保存原始值用于调试
                
                # 归一化到 [0, 1]（如果关键点是像素坐标）
                if kp_xy[0] > 1.0 or kp_xy[1] > 1.0:
                    kp_xy[0] = kp_xy[0] / img_width
                    kp_xy[1] = kp_xy[1] / img_height
                kp_tensor = torch.tensor(kp_xy, dtype=torch.float32)  # (2,)
                obs["observation.keypoints"] = kp_tensor.unsqueeze(0).to(device, non_blocking=True)  # (1, 2)
            else:
                obs["observation.keypoints"] = torch.zeros(1, 2, device=device)
            
            # 添加图像
            if hasattr(policy, 'config') and hasattr(policy.config, 'image_features'):
                for cam_key in policy.config.image_features:
                    if cam_key == "observation.images.head":
                        obs[cam_key] = head_rgb_img
                    elif cam_key == "observation.images.left":
                        obs[cam_key] = left_rgb_img
                    elif cam_key == "observation.images.head_init":
                        obs[cam_key] = head_init_img
                    elif cam_key == "observation.images.left_init":
                        obs[cam_key] = left_init_img
            else:
                # 默认配置
                obs["observation.images.head"] = head_rgb_img
                obs["observation.images.left"] = left_rgb_img
                obs["observation.images.head_init"] = head_init_img
                obs["observation.images.left_init"] = left_init_img
            
            # 推理动作：8维（7关节 + 1gripper）
            action = policy.select_action(obs)  # (1, 8)
            action = action.squeeze(0).cpu().numpy()  # (8,)
            
            # 调试：前3帧打印详细信息
            if i < 3:
                print(f"\n{'='*60}")
                print(f"Frame {i} 调试信息")
                print(f"{'='*60}")
                print(f"当前状态 (observation.state): {state.cpu().numpy()}")
                print(f"关键点 (原始): {kp_xy_debug if kp_xy_debug is not None else 'None'}")
                print(f"关键点 (归一化后): {obs['observation.keypoints'].cpu().numpy() if 'observation.keypoints' in obs else 'None'}")
                print(f"图像形状: head={head_rgb_img.shape}, left={left_rgb_img.shape}")
                print(f"图像形状: head_init={head_init_img.shape}, left_init={left_init_img.shape}")
                print(f"图像数据范围: head=[{head_rgb_img.min().item():.3f}, {head_rgb_img.max().item():.3f}], "
                      f"left=[{left_rgb_img.min().item():.3f}, {left_rgb_img.max().item():.3f}]")
                
                # 检查归一化后的观测
                with torch.no_grad():
                    normalized_obs = policy.normalize_inputs(obs)
                    print(f"\n归一化后的状态:")
                    print(f"  observation.state: {normalized_obs['observation.state'].cpu().numpy()}")
                    print(f"  observation.state mean: {normalized_obs['observation.state'].mean().item():.6f}")
                    print(f"  observation.state std: {normalized_obs['observation.state'].std().item():.6f}")
                    if 'observation.keypoints' in normalized_obs:
                        print(f"  observation.keypoints: {normalized_obs['observation.keypoints'].cpu().numpy()}")
                    
                    # 检查图像堆叠
                    if policy.config.image_features:
                        image_keys = list(policy.config.image_features.keys())
                        print(f"\n图像键顺序: {image_keys}")
                        stacked_images = torch.stack([normalized_obs[key] for key in image_keys], dim=-4)
                        print(f"堆叠后图像形状: {stacked_images.shape}")
                    
                    # 获取模型原始输出（归一化后）
                    # 注意：这里需要手动调用模型，因为 select_action 内部会处理
                    # 为了调试，我们直接调用模型的前向传播
                    try:
                        # 准备 batch（与 select_action 内部一致）
                        batch = normalized_obs.copy()
                        if policy.config.image_features:
                            batch["observation.images"] = torch.stack(
                                [batch[key] for key in policy.config.image_features], dim=-4
                            )
                        
                        # 调用模型
                        actions_norm, _ = policy.model(batch)  # (1, chunk_size, 8)
                        actions_norm_first = actions_norm[0, 0].cpu().numpy()  # 取第一个 action
                        
                        print(f"\n模型输出 (归一化后): {actions_norm_first}")
                        
                        # 反归一化
                        actions_unnorm = policy.unnormalize_outputs({"action": actions_norm})["action"]
                        actions_unnorm_first = actions_unnorm[0, 0].cpu().numpy()
                        
                        print(f"模型输出 (反归一化后): {actions_unnorm_first}")
                        
                        # 对比当前状态和 action
                        state_np = state.cpu().numpy()
                        action_diff = actions_unnorm_first - state_np
                        print(f"\n当前状态: {state_np}")
                        print(f"预测 action (从模型直接获取): {actions_unnorm_first}")
                        print(f"实际使用的 action (从 select_action): {action}")
                        print(f"Action - 当前状态 (差值): {action_diff}")
                        print(f"Action 变化幅度: {np.abs(action_diff)}")
                        print(f"Action 是否合理 (变化 < 0.5 rad): {np.all(np.abs(action_diff[:7]) < 0.5)}")
                        
                        # 检查模型输出的统计信息
                        print(f"\n模型输出统计:")
                        print(f"  模型输出范围: [{actions_norm_first.min():.6f}, {actions_norm_first.max():.6f}]")
                        print(f"  模型输出均值: {actions_norm_first.mean():.6f}")
                        print(f"  模型输出标准差: {actions_norm_first.std():.6f}")
                        print(f"  当前状态范围: [{state_np.min():.6f}, {state_np.max():.6f}]")
                        print(f"  当前状态均值: {state_np.mean():.6f}")
                        print(f"  当前状态标准差: {state_np.std():.6f}")
                        
                        # 对比训练数据统计（从 stats.json）
                        print(f"\n训练数据统计 (参考):")
                        print(f"  训练数据 observation.state 均值: [-0.322, 0.111, 0.009, 1.700, 0.080, 0.285, -0.151, 0.007]")
                        print(f"  训练数据 action 均值: [-0.322, 0.111, 0.010, 1.700, 0.080, 0.286, -0.150, 0.007]")
                        print(f"  训练数据 observation.state std: [0.177, 0.053, 0.106, 0.101, 0.088, 0.092, 0.064, 0.012]")
                        print(f"  训练数据 action std: [0.176, 0.053, 0.106, 0.101, 0.088, 0.093, 0.064, 0.012]")
                        print(f"\n注意: 训练数据中 action 和 observation.state 的统计分布几乎相同，")
                        print(f"      说明训练数据中状态变化很小。模型输出小的变化是正常的。")
                        
                    except Exception as e:
                        print(f"调试模型输出时出错: {e}")
                        import traceback
                        traceback.print_exc()
                
                print(f"{'='*60}\n")
            
            # 执行动作
            robot_interface.robot.set_arm_servo_angle(1, action[:7], 0.4, 0, True)
            robot_interface.robot.set_gripper_position(1, action[7])
            
            # 打印输出（每10帧）
            if i % 10 == 0 or i < 3:
                kp_info = f", kps=({keypoints[0][0]:.3f}, {keypoints[0][1]:.3f})" if keypoints is not None and len(keypoints) > 0 else ""
                execution_time = (time.time() - time1) * 1000
                # 显示完整的 7 个关节 + 1 个 gripper（8维）
                joints_str = f"[{', '.join([f'{x:.3f}' for x in action[:7]])}]"
                print(f"Frame {i:4d}/{data_len} | "
                      f"action: joints={joints_str}, gripper={action[7]:.3f} | "
                      f"time={execution_time:.3f}ms{kp_info}")
    
    # 清理
    if spinner_thread is not None:
        rclpy.shutdown()
        spinner_thread.join(timeout=1.0)
    print("\n推理完成！")


if __name__ == "__main__":
    main()