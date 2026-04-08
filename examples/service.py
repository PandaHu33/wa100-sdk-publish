'''
#一版可以输出
import socket
import threading

# 设置UDP接收端
UDP_IP = "127.0.0.1"
UDP_PORT = 25001

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for glove data on {UDP_IP}:{UDP_PORT}...")

def parse_data(data_str):
    """解析从Unity发来的数据"""
    # 数据格式示例：L,0,10.5,20.3,30.1;L,1,11.2,21.4,31.5;...
    joints = data_str.split(';')
    for joint in joints:
        if joint:  # 跳过空字符串
            parts = joint.split(',')
            if len(parts) == 5:  # 格式：hand,index,x,y,z
                hand = parts[0]  # 'L' 或 'R'
                index = int(parts[1])
                x, y, z = map(float, parts[2:5])
                print(f"{hand} hand bone {index}: ({x:.2f}, {y:.2f}, {z:.2f})")

while True:
    data, addr = sock.recvfrom(65536)  # 接收数据
    data_str = data.decode('utf-8')
    parse_data(data_str)
'''
'''
二版规范输出
import socket
import sys

# UDP 配置
UDP_IP = "127.0.0.1"
UDP_PORT = 25001

def parse_custom_format(data_str):
    """
    解析自定义格式数据，格式示例：
    L,0,10.5,20.3,30.1;L,1,11.2,21.4,31.5;...
    每段由分号分隔，每段内用逗号分隔：hand,index,x,y,z
    """
    left_joints = []
    # 按分号分割数据包（可能包含多个关节数据）
    joints = data_str.split(';')
    for joint in joints:
        if not joint:  # 跳过空字符串
            continue
        parts = joint.split(',')
        if len(parts) == 5:
            hand = parts[0]           # 'L' 或 'R'
            idx = int(parts[1])       # 关节索引
            try:
                x, y, z = map(float, parts[2:5])
            except ValueError:
                # 如果数据格式异常，跳过该关节
                continue
            if hand == 'L':
                left_joints.append((idx, x, y, z))
        else:
            # 可选：打印格式错误的信息，便于调试
            print(f"忽略格式错误的数据段: {joint}")

    # 按索引排序，确保输出顺序正确
    left_joints.sort(key=lambda j: j[0])

    # 打印前四个关节（索引 0 ~ 3）
    for i in range(min(4, len(left_joints))):
        idx, x, y, z = left_joints[i]
        print(f"左手关节 {idx}: x={x:.2f}, y={y:.2f}, z={z:.2f}")

    if left_joints:
        print("-" * 50)  # 分隔线，便于观察多个数据包

def main():
    # 创建 UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((UDP_IP, UDP_PORT))
    except socket.error as e:
        print(f"绑定端口失败: {e}")
        sys.exit(1)

    print(f"正在监听 {UDP_IP}:{UDP_PORT} ...")
    print("期望数据格式: hand,index,x,y,z;hand,index,x,y,z;...")
    print("例如: L,0,10.5,20.3,30.1;L,1,11.2,21.4,31.5;...")

    try:
        while True:
            # 接收数据，缓冲区大小 65536 足够容纳较大的数据包
            data, addr = sock.recvfrom(65536)
            data_str = data.decode('utf-8')
            # 可选：打印原始数据，用于调试（正式运行时可注释掉）
            # print(f"收到原始数据: {repr(data_str)}")
            parse_custom_format(data_str)
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    finally:
        sock.close()
        print("Socket 已关闭")

if __name__ == "__main__":
    main()
'''
'''
版本三，还行
import socket
import json

UDP_IP = "127.0.0.1"
UDP_PORT = 25001

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
print(f"正在监听 {UDP_IP}:{UDP_PORT} ...")

# 后续可改为保存 CSV，目前仅打印前两个关节
def parse_and_print(data_dict):
    left_pos = data_dict['leftPositions']   # 63 个 float
    left_rot = data_dict['leftRotations']
    # 当前仅打印左手前两个关节（索引 0 和 1）的位置和旋转
    print("=== 左手关节0 ===")
    print(f"位置: x={left_pos[0]:.2f}, y={left_pos[1]:.2f}, z={left_pos[2]:.2f}")
    print(f"旋转: x={left_rot[0]:.2f}, y={left_rot[1]:.2f}, z={left_rot[2]:.2f}")

    print("=== 左手关节5 ===")
    print(f"位置: x={left_pos[21]:.2f}, y={left_pos[22]:.2f}, z={left_pos[23]:.2f}")
    print(f"旋转: x={left_rot[21]:.2f}, y={left_rot[22]:.2f}, z={left_rot[23]:.2f}")
    print("-" * 60)

try:
    while True:
        data, addr = sock.recvfrom(65536)
        json_str = data.decode('utf-8')
        try:
            hi5_data = json.loads(json_str)
            parse_and_print(hi5_data)
        except json.JSONDecodeError as e:
            print(f"JSON 解析错误: {e}")
        except Exception as e:
            print(f"处理数据时出错: {e}")
except KeyboardInterrupt:
    print("\n程序被中断，关闭 socket")
    sock.close()
'''
import socket
import json
import csv
import time
import sys

UDP_IP = "127.0.0.1"
UDP_PORT = 25001

CSV_FILE = "hi5_data.csv"

# 创建 UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
print(f"正在监听 {UDP_IP}:{UDP_PORT} ...")

# 打开 CSV 文件并写入表头
csv_file = open(CSV_FILE, 'w', newline='', encoding='utf-8')
csv_writer = csv.writer(csv_file)

# 定义表头
header = ["frameId"]
for hand in ["L", "R"]:
    for joint in range(21):
        for axis in ["x", "y", "z"]:
            header.append(f"{hand}{joint}_pos_{axis}")
        for axis in ["x", "y", "z"]:
            header.append(f"{hand}{joint}_rot_{axis}")
csv_writer.writerow(header)
print(f"CSV 文件已创建: {CSV_FILE}")
csv_file.flush()

def write_to_csv(data_dict):
    """将接收到的数据写入 CSV 一行"""
    row = [data_dict['frameId']]
    row.extend(data_dict['leftPositions'])
    row.extend(data_dict['leftRotations'])
    row.extend(data_dict['rightPositions'])
    row.extend(data_dict['rightRotations'])
    csv_writer.writerow(row)
    csv_file.flush()

def parse_and_print(data_dict):
    """打印左手前两个关节用于验证"""
    left_pos = data_dict['leftPositions']
    left_rot = data_dict['leftRotations']
    print("=== 左手关节0 ===")
    print(f"位置: x={left_pos[0]:.2f}, y={left_pos[1]:.2f}, z={left_pos[2]:.2f}")
    print(f"旋转: x={left_rot[0]:.2f}, y={left_rot[1]:.2f}, z={left_rot[2]:.2f}")
    print("=== 左手关节1 ===")
    print(f"位置: x={left_pos[3]:.2f}, y={left_pos[4]:.2f}, z={left_pos[5]:.2f}")
    print(f"旋转: x={left_rot[3]:.2f}, y={left_rot[4]:.2f}, z={left_rot[5]:.2f}")
    print("-" * 60)

start_time = time.time()
duration = 10  # 接收 10 秒数据

try:
    while time.time() - start_time < duration:
        # 计算剩余超时时间
        remaining = duration - (time.time() - start_time)
        if remaining <= 0:
            break
        sock.settimeout(remaining)  # 设置超时，确保不会阻塞超过剩余时间
        try:
            data, addr = sock.recvfrom(65536)
            json_str = data.decode('utf-8')
            try:
                hi5_data = json.loads(json_str)
                write_to_csv(hi5_data)
                parse_and_print(hi5_data)
            except json.JSONDecodeError as e:
                print(f"JSON 解析错误: {e}")
            except Exception as e:
                print(f"处理数据时出错: {e}")
        except socket.timeout:
            # 超时退出循环
            break
except KeyboardInterrupt:
    print("\n程序被用户中断，正在关闭文件...")
finally:
    csv_file.close()
    sock.close()
    print(f"数据已保存至 {CSV_FILE}")
