import argparse
import math
from model import static
from visualization import visualization as vs 

def angle_to_torque_task(**parameters):
    print("输入角度单位: 度")
    angle1 = float(input("请输入关节一的角度: ")) / 180 * math.pi
    angle2 = float(input("请输入关节二的角度: ")) / 180 * math.pi
    angle3 = float(input("请输入关节三的角度: ")) / 180 * math.pi
    q = [angle1, angle2, angle3]
    M = static.static_torque(q, **parameters)
    print(f"关节一力矩: {M[0]:.4f}, 关节二力矩: {M[1]:.4f}, 关节三力矩: {M[2]:.4f}  单位: (N * m)")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="平面三连杆机械臂静力学分析")
    parser.add_argument("--lengths", type=float, nargs=3, metavar=("L1", "L2", "L3"), help="连杆长度（m）")
    parser.add_argument("--masses", type=float, nargs=3, metavar=("M1", "M2", "M3"), help="连杆质量（kg）")
    parser.add_argument("--load-mass", type=float, help="末端负载质量（kg）")
    parser.add_argument("--gravity", type=float, help="重力加速度（m/s²）")
    parameters = vars(parser.parse_args())
    static.static_torque([0, 0, 0], **parameters)
    success = False
    while (success == False):
        choice = input("请输入需要执行的任务(输入序号): \n可选: \n1. 输入关节角度输出关节力矩\n2. 画出固定23电机时力矩图像\n3. 画出固定13电机时力矩图像\n4. 画出固定12电机时力矩图像\n5. 画出自锁三维图\n")
        if (choice == "1"):
            angle_to_torque_task(**parameters)
            success = True
        elif (choice == "2"):
            print("输入角度单位: 度")
            angle2 = float(input("请输入关节二的角度: ")) / 180 * math.pi
            angle3 = float(input("请输入关节三的角度: ")) / 180 * math.pi
            vs.draw_effort_with_joint23_fixed(angle2, angle3, **parameters)
            success = True
        elif (choice == "3"):
            print("输入角度单位: 度")
            angle1 = float(input("请输入关节一的角度: ")) / 180 * math.pi
            angle3 = float(input("请输入关节三的角度: ")) / 180 * math.pi
            vs.draw_effort_with_joint13_fixed(angle1, angle3, **parameters)
            success = True
        elif (choice == "4"):
            print("输入角度单位: 度")
            angle1 = float(input("请输入关节一的角度: ")) / 180 * math.pi
            angle2 = float(input("请输入关节二的角度: ")) / 180 * math.pi
            vs.draw_effort_with_joint12_fixed(angle1, angle2, **parameters)
            success = True
        elif (choice == "5"):
            Fc = float(input("输入最大静摩擦力矩: (单位: N * m) \n"))
            vs.draw_self_lock_position_with_margin(Fc, **parameters)
            success = True
        else:
            print("please type the right task name")

