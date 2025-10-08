#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import mediapipe as mp
import numpy as np
import matplotlib
# --- GUIを表示しないバックエンド 'Agg' に変更 ---
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from types import SimpleNamespace
from collections import defaultdict

# --- ROS関連のライブラリ ---
import rospy
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError

# MediaPipeの初期設定 (変更なし)
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

# 定数の定義 (変更なし)
JOINT_DATA = [
    {"name": "THUMB2", "indices": [4, 3, 2]},
    {"name": "THUMB1", "indices": [3, 2, 1]},
    
    {"name": "INDEX3", "indices": [8, 7, 6]},
    {"name": "INDEX2", "indices": [7, 6, 5]},
    {"name": "INDEX1", "indices": [6, 5, 0]},

    {"name": "MIDDLE3", "indices": [12, 11, 10]},
    {"name": "MIDDLE2", "indices": [11, 10, 9]},
    {"name": "MIDDLE1", "indices": [10, 9, 0]},

    {"name": "RING3", "indices": [16, 15, 14]},
    {"name": "RING2", "indices": [15, 14, 13]},
    {"name": "RING1", "indices": [14, 13, 0]},

    {"name": "LITTLE3", "indices": [20, 19, 18]},
    {"name": "LITTLE2", "indices": [19, 18, 17]},
    {"name": "LITTLE1", "indices": [18, 17, 0]},
]
KINEMATIC_CHAINS = [
    [0, 1, 2, 3, 4], [0, 5, 6, 7, 8], [0, 9, 10, 11, 12],
    [0, 13, 14, 15, 16], [0, 17, 18, 19, 20]
]

# キャリブレーション関連の変数 (変更なし)
CALIBRATION_FRAMES = 100
is_calibrated = False
is_calibrating = False
calibration_frame_counter = 0
calibration_data = defaultdict(list)
calibrated_lengths = {}

# 主要な関数 (内容は前回と同じ)
def start_calibration():
    global is_calibrating, is_calibrated, calibration_frame_counter, calibration_data, calibrated_lengths
    is_calibrating = True; is_calibrated = False; calibration_frame_counter = 0
    calibration_data.clear(); calibrated_lengths.clear()
    rospy.loginfo("キャリブレーションを開始します。手を静止させてください...")

def collect_calibration_data(world_landmarks):
    global calibration_frame_counter, calibration_data
    if not is_calibrating: return
    for connection in mp_hands.HAND_CONNECTIONS:
        p1_idx, p2_idx = connection
        p1 = world_landmarks.landmark[p1_idx]; p2 = world_landmarks.landmark[p2_idx]
        length = np.linalg.norm([p1.x - p2.x, p1.y - p2.y, p1.z - p2.z])
        key = tuple(sorted((p1_idx, p2_idx)))
        calibration_data[key].append(length)
    calibration_frame_counter += 1

def finalize_calibration():
    global is_calibrating, is_calibrated, calibrated_lengths, calibration_data
    if not is_calibrating: return
    for key, values in calibration_data.items():
        if values: calibrated_lengths[key] = np.mean(values)
    is_calibrating = False; is_calibrated = True
    rospy.loginfo(f"キャリブレーション完了。{CALIBRATION_FRAMES}フレームの平均値を使用します。")

def apply_kinematic_constraints(world_landmarks, lengths):
    corrected_coords = np.array([[lm.x, lm.y, lm.z] for lm in world_landmarks.landmark])
    for chain in KINEMATIC_CHAINS:
        for i in range(len(chain) - 1):
            p_idx, c_idx = chain[i], chain[i+1]
            key = tuple(sorted((p_idx, c_idx)))
            if key not in lengths: continue
            p_pos = corrected_coords[p_idx]
            c_pos_detected = corrected_coords[c_idx]
            calibrated_length = lengths[key]
            direction = c_pos_detected - p_pos
            current_length = np.linalg.norm(direction)
            if current_length == 0: continue
            corrected_coords[c_idx] = p_pos + (direction / current_length) * calibrated_length
    return SimpleNamespace(landmark=[SimpleNamespace(x=p[0], y=p[1], z=p[2]) for p in corrected_coords])

def get_joint_axis_and_angle(a_lm, b_lm, c_lm):
    a = np.array([a_lm.x, a_lm.y, a_lm.z]); b = np.array([b_lm.x, b_lm.y, b_lm.z]); c = np.array([c_lm.x, c_lm.y, c_lm.z])
    vec_ba = a - b; vec_bc = c - b
    rotation_axis = np.cross(vec_ba, vec_bc)
    norm = np.linalg.norm(rotation_axis)
    if norm != 0: rotation_axis /= norm
    norm_ba = np.linalg.norm(vec_ba); norm_bc = np.linalg.norm(vec_bc)
    if norm_ba == 0 or norm_bc == 0: return np.array([0,0,0]), 0.0
    cosine_angle = np.clip(np.dot(vec_ba, vec_bc) / (norm_ba * norm_bc), -1.0, 1.0)
    angle_deg = np.degrees(np.arccos(cosine_angle))
    return rotation_axis, max(0.0, 180.0 - angle_deg)

def update_plot_buffer(ax, landmarks_to_draw, axes_info, title, fig):
    """プロットを更新し、GUI表示なしでキャンバスに描画する"""
    ax.clear(); ax.set_title(title); ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
    lm_coords = np.array([[lm.x, lm.y, lm.z] for lm in landmarks_to_draw.landmark])
    max_range = (lm_coords.max(axis=0) - lm_coords.min(axis=0)).max() / 2.0
    mid = lm_coords.mean(axis=0)
    ax.set_xlim(mid[0] - max_range, mid[0] + max_range); ax.set_ylim(mid[1] - max_range, mid[1] + max_range); ax.set_zlim(mid[2] - max_range, mid[2] + max_range)
    ax.scatter(lm_coords[:,0], lm_coords[:,1], lm_coords[:,2], c='blue', marker='o')
    for conn in mp_hands.HAND_CONNECTIONS:
        p1, p2 = conn
        ax.plot(lm_coords[[p1,p2], 0], lm_coords[[p1,p2], 1], lm_coords[[p1,p2], 2], c='gray', alpha=0.5)
    for info in axes_info:
        b_lm = landmarks_to_draw.landmark[info["indices"][1]]; center = np.array([b_lm.x, b_lm.y, b_lm.z])
        scale = max_range / 5
        ax.quiver(center[0], center[1], center[2], info["axis"][0], info["axis"][1], info["axis"][2], length=scale, color='red')
        ax.text(center[0], center[1], center[2] + 0.01, f'{info["angle"]:.0f}°', color='red')
    fig.canvas.draw()

def fig_to_ros_image(fig, bridge):
    """MatplotlibのfigureをROS Imageメッセージに変換する"""
    buf = fig.canvas.buffer_rgba()
    img_np = np.frombuffer(buf, dtype=np.uint8).reshape(fig.canvas.get_width_height()[::-1] + (4,))
    img_bgr = cv2.cvtColor(img_np, cv2.COLOR_RGBA2BGR)
    try:
        return bridge.cv2_to_imgmsg(img_bgr, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)
        return None

# --- メイン処理 ---
def run_realtime_detection():
    rospy.init_node('hand_kinematics_publisher_headless', anonymous=True)
    bridge = CvBridge()
    joint_pub = rospy.Publisher('/hand_joint_states', JointState, queue_size=1)
    image_pub = rospy.Publisher('/camera_image', Image, queue_size=1)
    plot_pub = rospy.Publisher('/plot_image', Image, queue_size=1)
    rospy.loginfo("Publisher nodes started.")

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        rospy.logerr("Webカメラを開けませんでした。")
        return

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    rospy.loginfo("5秒後にキャリブレーションを開始します。手をカメラに見せて準備してください。")
    rospy.sleep(5.0)
    start_calibration()

    with mp_hands.Hands(model_complexity=0, min_detection_confidence=0.5, min_tracking_confidence=0.2) as hands:
        while cap.isOpened() and not rospy.is_shutdown():
            success, image = cap.read()
            if not success: continue

            image = cv2.flip(image, 1)
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = hands.process(image_rgb)

            landmarks_to_process = None
            
            if results.multi_hand_world_landmarks:
                world_landmarks = results.multi_hand_world_landmarks[0]
                
                if is_calibrating:
                    if calibration_frame_counter % 20 == 0:
                        rospy.loginfo(f"キャリブレーション中... ({calibration_frame_counter}/{CALIBRATION_FRAMES})")
                    collect_calibration_data(world_landmarks)
                    landmarks_to_process = world_landmarks
                    if calibration_frame_counter >= CALIBRATION_FRAMES:
                        finalize_calibration()
                elif is_calibrated:
                    landmarks_to_process = apply_kinematic_constraints(world_landmarks, calibrated_lengths)
                
                if landmarks_to_process:
                    axes_info = []
                    joint_state_msg = JointState(header=Header(stamp=rospy.Time.now()))
                    for joint_info in JOINT_DATA:
                        a, b, c = joint_info["indices"]
                        axis, angle_deg = get_joint_axis_and_angle(
                            landmarks_to_process.landmark[a],
                            landmarks_to_process.landmark[b],
                            landmarks_to_process.landmark[c])
                        axes_info.append({"indices": [a,b,c], "axis": axis, "angle": angle_deg})
                        joint_state_msg.name.append(joint_info["name"])
                        joint_state_msg.position.append(np.deg2rad(angle_deg))
                    joint_pub.publish(joint_state_msg)

                    title = "Kinematically Corrected" if is_calibrated else "Raw"
                    update_plot_buffer(ax, landmarks_to_process, axes_info, title, fig)
                    plot_image_msg = fig_to_ros_image(fig, bridge)
                    if plot_image_msg:
                        plot_pub.publish(plot_image_msg)

                mp_drawing.draw_landmarks(image, results.multi_hand_landmarks[0], mp_hands.HAND_CONNECTIONS)
                try:
                    image_msg = bridge.cv2_to_imgmsg(image, "bgr8")
                    image_pub.publish(image_msg)
                except CvBridgeError as e:
                    rospy.logerr(e)
            

    cap.release()
    plt.close(fig) # メモリを解放
    rospy.loginfo("Node shutting down.")

if __name__ == "__main__":
    try:
        run_realtime_detection()
    except rospy.ROSInterruptException:
        print("ROS node interrupted.")
    except Exception as e:
        print(f"An error occurred: {e}")
