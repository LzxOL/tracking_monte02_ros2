#!/usr/bin/env python3
"""
实时摄像头关键点跟踪脚本

使用方法:
    python track_camera.py --checkpoint_path /home/root1/Corenetic/code/project/tracking_with_cameara_ws/src/track_on/checkpoints/track_on_checkpoint.pt

功能:
    1. 打开摄像头
    2. 在第一帧中点击鼠标选择要跟踪的关键点
    3. 按空格键开始跟踪
    4. 实时显示跟踪结果
    5. 按 'q' 键退出
"""

import cv2
import numpy as np
import argparse
from pathlib import Path
import matplotlib.pyplot as plt

from tracking_module import TrackingModule


class CameraTracker:
    """摄像头跟踪器类"""
    
    def __init__(self, checkpoint_path, camera_id=0):
        """
        初始化摄像头跟踪器
        
        Args:
            checkpoint_path: 模型检查点路径
            camera_id: 摄像头ID（默认0）
        """
        self.tracker = TrackingModule(checkpoint_path)
        self.camera_id = camera_id
        self.cap = None
        self.selected_points = []
        self.tracking_started = False
        self.window_name = "Camera Tracking - Click points, press SPACE to start, Q to quit"
        
    def mouse_callback(self, event, x, y, flags, param):
        """鼠标回调函数，用于选择关键点"""
        if event == cv2.EVENT_LBUTTONDOWN and not self.tracking_started:
            self.selected_points.append([x, y])
            print(f"已选择点 {len(self.selected_points)}: ({x}, {y})")
    
    def run(self):
        """运行摄像头跟踪"""
        # 打开摄像头
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            print(f"错误: 无法打开摄像头 {self.camera_id}")
            return
        
        # 设置窗口和鼠标回调
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        
        # 生成不同颜色用于显示不同的跟踪点
        colors = self._generate_colors(100)  # 预生成100种颜色
        
        print("\n使用说明:")
        print("1. 在窗口中点击鼠标左键选择要跟踪的关键点")
        print("2. 按空格键开始跟踪")
        print("3. 按 'r' 键重置并重新选择点")
        print("4. 按 'q' 键退出")
        print("-" * 50)
        
        first_frame_captured = False
        frame_count = 0
        
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("错误: 无法读取摄像头画面")
                    break
                
                display_frame = frame.copy()
                
                # 如果还没有开始跟踪，显示已选择的点
                if not self.tracking_started:
                    # 在帧上绘制已选择的点
                    for i, point in enumerate(self.selected_points):
                        x, y = int(point[0]), int(point[1])
                        color = colors[i % len(colors)]
                        cv2.circle(display_frame, (x, y), 8, color, -1)
                        cv2.putText(display_frame, f"{i+1}", (x+10, y-10), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                    
                    # 显示提示信息
                    if len(self.selected_points) > 0:
                        cv2.putText(display_frame, 
                                   f"Selected {len(self.selected_points)} points. Press SPACE to start tracking",
                                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    else:
                        cv2.putText(display_frame, 
                                   "Click to select points, then press SPACE",
                                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
                # 如果已经开始跟踪，显示跟踪结果
                else:
                    if not first_frame_captured:
                        # 初始化跟踪
                        if len(self.selected_points) > 0:
                            queries = np.array(self.selected_points, dtype=np.float32)
                            # 转换BGR到RGB
                            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                            try:
                                points, visibility = self.tracker.initialize_tracking(queries, frame_rgb)
                                first_frame_captured = True
                                print(f"跟踪已初始化，共 {len(queries)} 个关键点")
                            except Exception as e:
                                print(f"初始化跟踪时出错: {e}")
                                self.tracking_started = False
                                continue
                    
                    # 跟踪当前帧
                    if first_frame_captured:
                        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        try:
                            points, visibility = self.tracker.track_next_frame(frame_rgb)
                            
                            # 在帧上绘制跟踪结果
                            for i in range(len(points)):
                                x, y = int(points[i, 0]), int(points[i, 1])
                                color = colors[i % len(colors)]
                                
                                if visibility[i]:
                                    # 可见的点：实心圆
                                    cv2.circle(display_frame, (x, y), 8, color, -1)
                                    cv2.circle(display_frame, (x, y), 12, color, 2)
                                else:
                                    # 不可见的点：空心圆
                                    cv2.circle(display_frame, (x, y), 8, color, 2)
                                
                                # 显示点编号
                                cv2.putText(display_frame, f"{i+1}", (x+10, y-10), 
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                            
                            # 显示帧计数
                            cv2.putText(display_frame, f"Frame: {frame_count}", 
                                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                            
                            frame_count += 1
                            
                        except Exception as e:
                            print(f"跟踪时出错: {e}")
                
                # 显示窗口
                cv2.imshow(self.window_name, display_frame)
                
                # 处理按键
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("退出程序")
                    break
                elif key == ord(' '):  # 空格键
                    if len(self.selected_points) > 0 and not self.tracking_started:
                        self.tracking_started = True
                        first_frame_captured = False
                        frame_count = 0
                        print("开始跟踪...")
                    elif self.tracking_started:
                        print("跟踪已在进行中...")
                    else:
                        print("请先选择至少一个关键点")
                elif key == ord('r'):  # 重置
                    if self.tracking_started:
                        self.tracker.reset()
                        self.tracking_started = False
                        first_frame_captured = False
                        self.selected_points = []
                        frame_count = 0
                        print("已重置，请重新选择关键点")
        
        except KeyboardInterrupt:
            print("\n程序被用户中断")
        finally:
            # 清理资源
            if self.cap is not None:
                self.cap.release()
            cv2.destroyAllWindows()
            print("资源已释放")
    
    def _generate_colors(self, num_colors):
        """生成不同颜色的列表（BGR格式，用于OpenCV）"""
        cmap = plt.cm.get_cmap('tab20', min(num_colors, 20))
        colors = []
        for i in range(num_colors):
            rgba = cmap(i % cmap.N)
            # 转换为BGR格式（OpenCV使用BGR）
            bgr = (int(rgba[2] * 255), int(rgba[1] * 255), int(rgba[0] * 255))
            colors.append(bgr)
        return colors


def main():
    parser = argparse.ArgumentParser(
        description="使用摄像头进行实时关键点跟踪",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  python track_camera.py --checkpoint_path checkpoints/track_on_checkpoint.pt
  python track_camera.py --checkpoint_path checkpoints/track_on_checkpoint.pt --camera_id 1

操作说明:
  - 点击鼠标左键选择要跟踪的关键点
  - 按空格键开始跟踪
  - 按 'r' 键重置并重新选择点
  - 按 'q' 键退出
        """
    )
    parser.add_argument(
        "--checkpoint_path",
        type=str,
        required=True,
        help="模型检查点文件路径 (.pt文件)"
    )
    parser.add_argument(
        "--camera_id",
        type=int,
        default=0,
        help="摄像头ID（默认: 0）"
    )
    
    args = parser.parse_args()
    
    # 检查检查点文件是否存在
    if not Path(args.checkpoint_path).is_file():
        print(f"错误: 检查点文件不存在: {args.checkpoint_path}")
        return
    
    # 创建并运行跟踪器
    tracker = CameraTracker(args.checkpoint_path, args.camera_id)
    tracker.run()


if __name__ == "__main__":
    main()

