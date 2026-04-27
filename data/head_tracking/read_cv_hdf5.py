import h5py
import cv2
import numpy as np
import time

# ================= 1. 数据集路径配置 =================
# 替换为你刚刚录制的文件的绝对路径
FILE_PATH = '/media/hjx/PSSD/hjx_ws/Dual_servo_gimbal/data/head_tracking/episode_0.hdf5'


def replay_hdf5_video(file_path):
    print(f"📂 正在加载数据集: {file_path}")

    try:
        # 以只读模式 ('r') 打开 HDF5 文件
        with h5py.File(file_path, 'r') as root:
            # 1. 检查是否存在图像数据
            if '/observations/images/cam_high' not in root:
                print("❌ 数据集中未找到相机画面！")
                return

            # 2. 将数据从磁盘提取到内存 (使用 [:] 切片操作)
            print("⏳ 正在解压图像数据到内存...")
            images = root['/observations/images/cam_high'][:]
            actions = root['/action'][:]

            num_frames = len(images)
            print(f"✅ 加载成功！共包含 {num_frames} 帧同步数据 (约 {num_frames / 100.0:.1f} 秒)。")
            print("\n▶️ 开始回放...")
            print("👉 提示：按 'q' 键退出，按 '空格键' 暂停")

            # 3. 逐帧回放循环
            for i in range(num_frames):
                start_time = time.time()

                # --- 提取与处理图像 ---
                # 提取当前帧 (此时是 RGB 格式的 numpy array)
                img_rgb = images[i]

                # 【关键】因为录制时转成了深度学习标准的 RGB，现在用 OpenCV 显示必须转回 BGR
                img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)

                # 将原本 320x240 的图像放大到 640x480，方便人眼观看 (采用最近邻插值保持原像素感)
                img_display = cv2.resize(img_bgr, (640, 480), interpolation=cv2.INTER_NEAREST)

                # --- 提取与处理动作指令 ---
                # 动作在数据集中是弧度制 (Radians)
                pan_cmd_rad, tilt_cmd_rad = actions[i]

                # 换算回度数 (Degrees) 以便在屏幕上直观显示
                pan_deg = pan_cmd_rad * (180.0 / np.pi)
                tilt_deg = tilt_cmd_rad * (180.0 / np.pi)

                # --- UI 渲染层 (HUD) ---
                # 在画面左上角绘制黑色半透明背景板
                cv2.rectangle(img_display, (5, 5), (280, 100), (0, 0, 0), cv2.FILLED)

                # 写入文本信息
                cv2.putText(img_display, f"Frame: {i}/{num_frames}", (15, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(img_display, f"Action P: {pan_deg:5.1f} Deg", (15, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                cv2.putText(img_display, f"Action T: {tilt_deg:5.1f} Deg", (15, 85),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

                # --- 显示画面 ---
                cv2.imshow("VLA Dataset Replay", img_display)

                # --- 播放控制与锁频 ---
                # 数据是 100Hz 录制的，所以我们尝试以每帧 10ms 的速度播放
                elapsed = (time.time() - start_time) * 1000
                wait_ms = max(1, int(10 - elapsed))

                key = cv2.waitKey(wait_ms) & 0xFF

                if key == ord('q'):
                    print("\n🛑 用户主动终止回放。")
                    break
                elif key == ord(' '):  # 如果按下空格键
                    print("\n⏸️ 视频已暂停。按键盘上任意键继续...")
                    cv2.waitKey(0)

    except FileNotFoundError:
        print(f"\n❌ 找不到文件: {file_path}")
        print("请检查路径是否正确，或者是否成功运行了上一步的采集脚本。")
    except Exception as e:
        print(f"\n❌ 读取数据时发生未预期错误: {e}")
    finally:
        cv2.destroyAllWindows()
        print("✨ 数据查看器已关闭。")


if __name__ == '__main__':
    replay_hdf5_video(FILE_PATH)
