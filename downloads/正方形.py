from controller import Robot
import numpy as np

class FiveBarRobot:
    def __init__(self):
        # --- 初始化 Webots 機器人 ---
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # 獲取馬達 (請確保名稱與你的 Webots 模型一致)
        self.motor_left = self.robot.getDevice('motor-a1')
        self.motor_right = self.robot.getDevice('motor-b1')

        # 設置馬達為位置控制模式
        self.motor_left.setPosition(float('inf'))
        self.motor_right.setPosition(float('inf'))
        self.motor_left.setVelocity(1.0) # 初始速度，之後會在迴圈中動態調整
        self.motor_right.setVelocity(1.0)

        # --- 機構參數 ---
        self.L1 = 150.0
        self.L2 = 265.0
        self.B = 200.0

    def ik_elbow_out(self, x, y):
        """ 逆運動學 (中心座標系): 左桿向左，右桿向右 """
        # 左馬達 (-100, 0), 右馬達 (100, 0)
        m1 = np.array([-self.B / 2, 0])
        m2 = np.array([self.B / 2, 0])
        target = np.array([x, y])

        # 左臂
        v1 = target - m1
        d1 = np.linalg.norm(v1)
        cos_a1 = (self.L1**2 + d1**2 - self.L2**2) / (2 * self.L1 * d1)
        if abs(cos_a1) > 1.0: return None
        theta1 = np.arctan2(v1[1], v1[0]) + np.arccos(cos_a1)

        # 右臂
        v2 = target - m2
        d2 = np.linalg.norm(v2)
        cos_a2 = (self.L1**2 + d2**2 - self.L2**2) / (2 * self.L1 * d2)
        if abs(cos_a2) > 1.0: return None
        theta2 = np.arctan2(v2[1], v2[0]) - np.arccos(cos_a2)

        return theta1, theta2

    def get_square_trajectory(self, time_now, speed=80.0):
        """
        生成正方形軌跡
        參數:
          time_now: 當前模擬時間
          speed: 移動速度 (單位/秒)
        返回:
          (x_ik, y_ik): 用於 IK 的目標點 (中心座標系)
        """
        # 正方形參數 (局部座標系，左下角為 0,0)
        width = 200.0  # 根據你的座標 (200-0)
        height = 200.0 # 根據你的座標 (350-150)
        perimeter = 2 * (width + height) # 總周長 800

        # 計算當前在周長上的位置 (循環)
        distance = (time_now * speed) % perimeter

        # 局部座標系中的位置 (x_local, y_local)
        if distance < width:
            # 1. 底部邊: (0,0) -> (200,0)
            pos_local = [distance, 0]
        elif distance < (width + height):
            # 2. 右側邊: (200,0) -> (200,200)
            pos_local = [width, distance - width]
        elif distance < (2 * width + height):
            # 3. 頂部邊: (200,200) -> (0,200)
            pos_local = [width - (distance - (width + height)), height]
        else:
            # 4. 左側邊: (0,200) -> (0,0)
            pos_local = [0, height - (distance - (2 * width + height))]

        # --- 座標轉換 ---
        # 將局部座標 (左下角為0,0) 轉換為 IK 中心座標
        # 局部 (0,0) 對應 IK 中的 (-100, 150)
        offset_x = -100.0
        offset_y = 150.0
        
        return pos_local[0] + offset_x, pos_local[1] + offset_y

    def run(self):
        print("Start drawing square...")
        while self.robot.step(self.timestep) != -1:
            t = self.robot.getTime()

            # 1. 獲取當前目標點 (IK座標系)
            target_x, target_y = self.get_square_trajectory(t, speed=75.0) # 調整 speed 可改變畫圖速度

            # 2. 計算逆運動學
            angles = self.ik_elbow_out(target_x, target_y)

            if angles:
                theta1, theta2 = angles
                # 3. 設置馬達角度
                # 注意：如果 Webots 模型中馬達 0 度不是指向水平向右，這裡可能需要加減偏移量
                self.motor_left.setPosition(theta1)
                self.motor_right.setPosition(theta2)
                
                # 可選：根據目標位置變化動態調整速度以獲得更平滑的運動
                self.motor_left.setVelocity(5.0)
                self.motor_right.setVelocity(5.0)
            else:
                pass # 目標點超出範圍，保持上一位置

# 執行機器人
bot = FiveBarRobot()
bot.run()