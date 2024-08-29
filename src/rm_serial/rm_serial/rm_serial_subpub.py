import struct
import serial
import time
import rclpy
import json
from rclpy.node import Node
#from auto_aim_interfaces.msg import Target,Gimbal
import rclpy.publisher
from std_msgs.msg import String,Bool,Float32
from threading import Thread
# import tf_transformations
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from math import pi
from geometry_msgs.msg import Twist
from referee_msg.msg import Referee
import glob

# CRC-8 校验表
CRC08_Table = [
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
]

# gimbal_msg = Gimbal()
parsed_data=None
referee_data = Referee()
pub_referee = None
# result=[]
# 定义结构体
class All_Data_Rx:
    def __init__(self,vx,vy,rotate,yaw_speed,running_state):
        self.vx = vx
        self.vy = vy
        self.rotate =rotate
        self.yaw_speed = yaw_speed
        self.running_state = running_state


# class Chassis_Data_Rx:
#     def __init__(self, vx, vy, rotate, yaw_speed, pitch_speed):
#         self.vx = vx
#         self.vy = vy
#         self.yaw_speed = yaw_speed
#         self.pitch_speed = pitch_speed
#         self.rotate =rotate

# class Rotate_Data_Rx:
#     def __init__(self, rotate):
#         self.rotate = rotate


# 定义帧头和命令字
HEADER = 0xaa
# CMD_ID_AUTOAIM_DATA_RX = 0x81
# CMD_ID_CHASSIS_DATA_RX = 0x82
# CMD_ID_ROTATE_DATA_RX = 0x85
# 打包结构体为字节流
def pack_all(data:All_Data_Rx):
    return struct.pack('<ffff?',data.vx, data.vy, data.rotate, data.yaw_speed,data.running_state)
# def pack_all(data:All_Data_Rx):
#     return struct.pack('<ffffB',1.0, 2.0,3.0, 2.8,0)

# def pack_chassis_data(data):
#     return struct.pack('<fffff', data.vx, data.vy, data.rotate, data.yaw_speed, data.pitch_speed
#                        data.vx, data.vy, data.rotate, data.yaw_speed, data.pitch_speed)

# def pack_rotate_data(data):
#     return struct.pack('<h', data.rotate)

# 计算CRC校验位
# def crc8(data):
#     crc = 0xff
#     for byte in data:
#         crc = CRC08_Table[crc ^ byte]
#     return crc

# 构建消息
def build_all_message(data):
    data_bytes = pack_all(data)
    length = len(data_bytes) + 2  # 包含帧头、帧长度（shiki写的）
    print('the length is',length)  #shiki打印的一帧数据的长度·
    return struct.pack('<BB', HEADER,length) + data_bytes 

# def build_chassis_message(data):
#     data_bytes = pack_chassis_data(data)
#     #print(len(data_bytes))
#     length = len(data_bytes) + 4  # 包含帧头、帧长度、命令字和校验位
#     crc = crc8(struct.pack('<BBB', HEADER, length, CMD_ID_CHASSIS_DATA_RX) + data_bytes)
#     return struct.pack('<BBB', HEADER, length, CMD_ID_CHASSIS_DATA_RX) + data_bytes + struct.pack('<B', crc)

# def build_rotate_message(data):
#     data_bytes = pack_rotate_data(data)
#     length = len(data_bytes) + 4  # 包含帧头、帧长度、命令字和校验位
#     crc = crc8(struct.pack('<BBB', HEADER, length, CMD_ID_ROTATE_DATA_RX) + data_bytes)
#     return struct.pack('<BBB', HEADER, length, CMD_ID_ROTATE_DATA_RX) + data_bytes + struct.pack('<B', crc)


# 连接串口
devices=glob.glob('/dev/ttyUSB*')
if len(devices) == 0: 
    raise ValueError("No serial port found")
device=devices[0]
ser = serial.Serial(device, 115200)  # 假设串口为COM1，波特率为9600

# def send_auto_aim(Pos_x, Pos_y, Pos_z, Vel_x, Vel_y, Vel_z,Yaw,V_yaw,Radius_1,Radius_2 ,tracking, shoot_freq):
#     data = AutoAim_Data_Rx(Pos_x, Pos_y, Pos_z, Vel_x, Vel_y, Vel_z, Yaw,V_yaw,Radius_1,Radius_2,tracking, shoot_freq)
#     message = build_autoaim_message(data)
#     ser.write(message)

# def send_nav(x_speed, y_speed, yaw_speed):
#     data = Chassis_Data_Rx(x_speed, y_speed,0,yaw_speed,0)
#     message = build_chassis_message(data)
#     print(1)
#     ser.write(message)

def send_all(x_speed, y_speed,rotate,yaw_speed,running_state):
    data= All_Data_Rx(x_speed, y_speed, rotate, yaw_speed,running_state)
    message= build_all_message(data)
    ser.write(message)

# 打开串口
# ser = serial.Serial('COM1', 9600)  # 请根据实际情况更改串口号和波特率

# # 读取数据并解析JSON
# def parse_message():
#     frame = ""
#     # while True:
#     data = ser.read().decode('utf-8')  # 读取一个字节并解码为字符串
#     if data == '\n':  # 以\n为一帧的结束符
#         try:
#             json_data = json.loads(frame)  # 解析JSON数据
#             print("收到的JSON数据：", json_data)
#         except json.JSONDecodeError as e:
#                 print("JSON解析错误：", e)
#         frame = ""  # 重置frame
#     else:
#         frame += data
#     try:
#         parse_message()
#     except KeyboardInterrupt:
#         ser.close()
#         print("程序已结束。")
#     def parse_message(json_data):
#         # 解析JSON数据
#         referee_result = []
change_data =[]
rx_buffer =[]
vx=0
vy =0
v_yaw=0
rotate=0
running =True
game_pro = 0
def parse_message(json_data):
    # Create a new instance of Referee
    global game_pro
    referee_data = Referee()

    # Populate the fields of the new instance from json_data
    if 'remain_hp' in json_data:
        referee_data.remain_hp = json_data['remain_hp']
    if 'max_hp' in json_data:
        referee_data.max_hp = json_data['max_hp']
    if 'game_type' in json_data:
        referee_data.game_type = json_data['game_type']
    if 'game_progress' in json_data:
        referee_data.game_progress = json_data['game_progress']

        game_pro=json_data['game_progress']     ###########################

    if 'stage_remain_time' in json_data:
        referee_data.stage_remain_time = json_data['stage_remain_time']
    if 'bullet_remaining_num_17mm' in json_data:
        referee_data.bullet_remaining_num_17mm = json_data['bullet_remaining_num_17mm']
    if 'red_outpost_hp' in json_data:
        referee_data.red_outpost_hp = json_data['red_outpost_hp']
    if 'red_base_hp' in json_data:
        referee_data.red_base_hp = json_data['red_base_hp']
    if 'blue_outpost_hp' in json_data:
        referee_data.blue_outpost_hp = json_data['blue_outpost_hp']
    if 'blue_base_hp' in json_data:
        referee_data.blue_base_hp = json_data['blue_base_hp']
    if 'rfid_status' in json_data:
        referee_data.rfid_status = json_data['rfid_status']

    # Return the newly created and populated Referee object
    return referee_data

class SPNode(Node):
    def __init__(self):
        global pub_referee
        super().__init__("subscriber_publisher_node")
        # self.br = TransformBroadcaster(self)
        # self.subscription = self.create_subscription(Target, '/tracker/target', self.all_callback, qos_profile=rclpy.qos.qos_profile_sensor_data)  # CHANGE
        #self.subscription  # 防止未使用变量def listener_callback(self, msg):警告
        # self.publisherss = self.create_publisher(String,"refree",10)
        # self.publish_gimbal = self.create_publisher(Gimbal,"gimbal_status",10)
        # self.publisher_timer = self.create_timer(0.0067,self.publish_message)
        pub_referee = self.create_publisher(Referee, '/Referee', 10)
        self.priority = False #0 aim 1 nav
        self.sub_priority = self.create_subscription(Bool,"/running_state",self.running_callback,10)
        self.sub_rot = self.create_subscription(Bool,"/nav_rotate",self.rot_callback,10)

        #导航
        # self.sub_nav = self.create_subscription(Twist, '/nav_cmd', self.nav_callback,rclpy.qos.qos_profile_sensor_data)
        self.sub_nav = self.create_subscription(Twist, '/cmd_vel', self.nav_callback,10)
        
    def rot_callback(self,msg:Bool):
        global rotate
        if msg.data:
            rotate = 15.0 
        else:
            rotate = 0.0
    
    def running_callback(self,msg:Bool):
        global running
        running = msg.data

    # def update_pri(self,msg:Bool):
    #     self.priority = msg.data

    def nav_callback(self, msg:Twist):
         global vx
         global vy
         global v_yaw
         global rotate
         global running

         if game_pro!=4:
             vx=0.0
             vy=0.0
             v_yaw=0.0
             
         if running:   
            vx=msg.linear.x
            vy=msg.linear.y
            v_yaw=msg.angular.z

        #  print("1"*10 +' send') #调试用
         send_all(vx,vy,rotate,v_yaw,running)
        # send_nav(msg.linear.x,msg.linear.y,msg.angular.z)

    # def listener_callback(self, msg):
    #     # print(msg.position.x)
    #     #tem=ser.out_waiting
    #     #print(tem)
    # #     send_auto_aim(msg.position.x, msg.position.y, msg.position.z, msg.velocity.x, msg.velocity.y, msg.velocity.z,msg.yaw,msg.v_yaw,msg.radius_1,msg.radius_2, msg.tracking, 10)
    # def all_callback(self,msg):
    #     global vx
    #     global vy
    #     global v_yaw
    #     global rotate
    #     # global position_x
    #     # global position_y
        # global position_z
        # global velocity_x
        # global velocity_y
        # global velocity_z
        # global msg_yaw
        # global msg_v_yaw
        # global msg_radius_1
        # global msg_radius_2
        # global msg_tracking
        # global msg_shoot
        # # if isinstance(msg,Target):
        # position_x=msg.position.x
        # position_y=msg.position.y
        # position_z=msg.position.z
        # velocity_x=msg.velocity.x
        # velocity_y=msg.velocity.y
        # velocity_z=msg.velocity.z
        # msg_yaw=msg.yaw
        # msg_v_yaw=msg.v_yaw
        # msg_radius_1=msg.radius_1
        # msg_radius_2=msg.radius_2
        # if self.priority:
        #     msg_tracking = 0
        # else:
        #     msg_tracking=msg.tracking
        # msg_shoot=10
        # armors_num=msg.armors_num
        # if isinstance(msg,Twist):
        # vx=msg.linear.x
        # vy=msg.linear.y
        # v_yaw=msg.angular.z
        # print('1'*20)
        # send_all(position_x,position_y,position_z,velocity_x,velocity_y,velocity_z,msg_yaw,msg_v_yaw,msg_radius_1,msg_radius_2,msg_tracking,msg_shoot,vx,vy,rotate,v_yaw,armors_num)
        # time.sleep(0.001)

    # def publish_message(self):
    #     # msg = Gimbal
    #     # msg.data = data
    #     #print(gimbal_msg.yaw)
    #     t = TransformStamped()

    #     t.header.stamp = self.get_clock().now().to_msg()
    #     t.header.frame_id = 'odom'
    #     t.child_frame_id = 'gimbal_link'

    #     t.transform.translation.x = 0.0
    #     t.transform.translation.y = 0.0
    #     t.transform.translation.z = 0.0

    #     q = tf_transformations.quaternion_from_euler((pi/180)*gimbal_msg.roll, (pi/180)*gimbal_msg.pitch, (pi/180)*gimbal_msg.yaw)
    #     t.transform.rotation.x = q[0]
    #     t.transform.rotation.y = q[1]
    #     t.transform.rotation.z = q[2]
    #     t.transform.rotation.w = q[3]
    #     self.br.sendTransform(t)
    #     self.publish_gimbal.publish(gimbal_msg)
        # # 读取数据并解析JSON
# def parse_message():
#     frame = ""
#     # while True:
#     data = ser.read().decode('utf-8')  # 读取一个字节并解码为字符串
#     if data == '\n':  # 以\n为一帧的结束符
#         try:
#             json_data = json.loads(frame)  # 解析JSON数据
#             print("收到的JSON数据：", json_data)
#         except json.JSONDecodeError as e:
#                 print("JSON解析错误：", e)
#         frame = ""  # 重置frame
#     else:
#         frame += data

# def receive_message():  #完成裁判系统数据接收和发送
#     global rx_buffer
#     global parsed_data
#     global change_data
#     # publish_gimbal = subscriber_publisher_node.create_publisher(Gimbal,"gimbal_status",10)
#     while 1:
#         data = ser.read()  # 读取一个字节
#         print('ready to read header************')
#         # print(head)
#         # if head.hex() !=0:
#             print('header have been received')
#             # rx_buffer = change_data[parse_message(parsed_data)]
#             print('message have been stroed')
#             rx_buffer = []
#             rx_buffer.append(head.hex())
#             print("header have been appended")
#             # length = ser.read()
#             # rx_buffer.append(length.hex())
#             # while len(rx_buffer) < int.from_bytes(length, byteorder='big'):  # 至少包含帧头、帧长度和CRC校验位的长度
#             # rx_buffer.append(ser.read().hex())
#             # print(rx_buffer)
#             # rx_buffer = []
#             try:
#                 # parsed_data = parse_message(rx_buffer)
#                 # referee_data.game_progress = 4
#                 # referee_data.red_outpost_hp = 0
#                 #referee_data.blue_outpost_hp = 0
#                 pub_referee.publish(referee_data)

#                 print('game progress:',referee_data.game_progress)  #shiki打印的

#                 #print(gimbal_msg.pitch) 
#                 # rx_buffer = []
#                 # gimbal_msg.yaw = result[0]
#                 # gimbal_msg.roll  = result[1]
#                 # gimbal_msg.pitch= result[2]
#                 #     # 发布消息
#                 # print(gimbal_msg.pitch)
                
#             except:
#                 pass
#                 #print("Error:")
#         else:
#             rx_buffer.append(ser.read().hex())

# # 读取数据并解析JSON
# def parse_message():
#     frame = ""
#     # while True:
#     data = ser.read().decode('utf-8')  # 读取一个字节并解码为字符串
#     if data == '\n':  # 以\n为一帧的结束符
#         try:
#             json_data = json.loads(frame)  # 解析JSON数据
#             print("收到的JSON数据：", json_data)
#         except json.JSONDecodeError as e:
#                 print("JSON解析错误：", e)
#         frame = ""  # 重置frame
#     else:
#         frame += data
# 读取数据并解析JSON
def receive_message():
    global rx_buffer
    global parsed_data
    global change_data
    frame = b''  # 初始化一个空的字节串来存储帧

    while True:
        data = ser.read()  # 读取一个字节并解码为字符串
        # print(data)
        if data == b'\n':  # 使用字节比较
            # print('commming')
            try:
                # print('ready to ')
                # 将接收到的字节串转换为字符串，然后解析JSON数据
                json_data = json.loads(frame.decode('utf-8'))
                # print("收到的JSON数据：", json_data)

                # 使用解析的JSON数据更新referee_data
                referee_data = parse_message(json_data)
              
                print(referee_data)
                # 发布referee_data到ROS话题
                pub_referee.publish(referee_data)
                # print('send all &&&&&&&&&&&&&&&&&&&')   

            except json.JSONDecodeError as e:
                 print("JSON解析错误：", e)
            finally:
                frame = b''  # 清空帧，准备接收下一帧数据

        else:
            frame += data  # 如果不是换行符，将数据添加到帧中
def main(args=None):
    rclpy.init(args=args)
    subscriber_publisher_node = SPNode()
    p_message = Thread(target=receive_message)
    p_message.start()
    rclpy.spin(subscriber_publisher_node)
    subscriber_publisher_node.destroy_node()
    rclpy.shutdown()

# if __name__ == '__main__':
#     main() 