import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import numpy as np

class DogController(Node):
    def __init__(self):
        super().__init__('dog_controller')
        
        # Паблишер в топик контроллера
        self.publisher_ = self.create_publisher(
            Float64MultiArray, 
            '/joint_group_position_controller/commands', 
            10
        )
        
        # Таймер (50 Гц)
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        # Параметры робота (из твоего URDF)
        self.L1 = 0.05   # Hip length (отступ вбок)
        self.L2 = 0.25   # Thigh length (бедро)
        self.L3 = 0.25   # Shin length (голень)
        
        self.get_logger().info("Dog Controller Started. Standing up...")

    def inverse_kinematics(self, x, y, z, side_sign):
        """
        Простая IK для 3-DOF лапы.
        x, y, z - координаты стопы относительно плеча.
        side_sign - 1 для левых лап, -1 для правых (для коррекции y)
        """
        # 1. Hip joint (Roll) - движение вбок
        # Проекция на плоскость YZ
        # dy = y
        # dz = z
        # L1 (hip) - гипотенуза в этой плоскости быть не может, L1 это отступ.
        # Тут упрощенная модель: считаем, что Hip поворачивает всю плоскость ноги.
        
        # Расстояние от корня суста до стопы в плоскости YZ
        yz_dist = math.sqrt(y**2 + z**2)
        
        # Если цель слишком близко - ограничиваем (защита от math domain error)
        if yz_dist < self.L1:
            yz_dist = self.L1 + 0.001

        # Угол Hip
        # alpha = atan2(z, y) но нам надо учитывать смещение L1
        # Это сложная часть, давай пока упростим:
        # Представим, что мы пока не двигаем лапы вбок (y=0, x=0, z меняется)
        # Если y = 0, то hip_angle = 0.
        
        # ДЛЯ ТЕСТА "ВСТАТЬ" МЫ БУДЕМ ИГНОРИРОВАТЬ Y (Roll) пока что.
        theta1 = 0.0 

        # 2. Thigh и Shin (Pitch) - работают как 2D манипулятор в плоскости XZ
        # Нам нужно достичь глубины z (с учетом того, что L1 уже забрал часть длины, но при theta1=0 это не важно)
        
        # Расстояние от "плеча" до "пятки"
        dist = math.sqrt(x**2 + z**2)
        
        # Защита от вытягивания лапы дальше длины ноги
        max_reach = self.L2 + self.L3 - 0.01
        if dist > max_reach:
            dist = max_reach
            
        # Теорема косинусов
        # c^2 = a^2 + b^2 - 2ab cos(gamma)
        # Нам нужно найти углы треугольника со сторонами L2, L3 и dist
        
        # Угол в колене (между бедром и голенью)
        # dist^2 = L2^2 + L3^2 - 2*L2*L3*cos(PI - knee_angle)
        cos_knee = (self.L2**2 + self.L3**2 - dist**2) / (2 * self.L2 * self.L3)
        cos_knee = max(-1.0, min(1.0, cos_knee)) # Clamp
        phi = math.acos(cos_knee) # Внутренний угол треугольника
        
        # Угол голени (Shin) относительно бедра
        theta3 = -(math.pi - phi) # Сгибаем колено назад

        # Угол бедра (Thigh)
        # alpha - угол наклона вектора dist к горизонту
        alpha = math.atan2(x, -z) # -z потому что z вниз отрицательный
        
        # beta - внутренний угол треугольника у бедра
        # L3^2 = L2^2 + dist^2 - 2*L2*dist*cos(beta)
        cos_beta = (self.L2**2 + dist**2 - self.L3**2) / (2 * self.L2 * dist)
        cos_beta = max(-1.0, min(1.0, cos_beta))
        beta = math.acos(cos_beta)
        
        theta2 = alpha + beta # Или alpha - beta, зависит от конфигурации
        
        return theta1, theta2, theta3

    def timer_callback(self):
        msg = Float64MultiArray()
        
        # Целевая высота (постепенно встаем)
        # z = -0.35 (лапа должна быть на 35 см ниже плеча)
        # x = 0.0 (лапа ровно под плечом)
        target_z = -0.35
        target_x = 0.0
        target_y = 0.0 # Лапы не разъезжаются
        
        # Получаем углы для одной лапы (симметрично для всех)
        # ВНИМАНИЕ: Тут может понадобиться инверсия знаков для правых лап
        q1, q2, q3 = self.inverse_kinematics(target_x, target_y, target_z, 1)
        
        # Порядок суставов в controllers.yaml:
        # FL_hip, FL_thigh, FL_shin, FR_..., RL_..., RR_...
        
        # Для начала попробуем одинаковые углы для всех (простой тест)
        # Возможно, для правой стороны q1 нужно будет инвертировать
        
        # FL
        fl = [q1, q2, q3]
        # FR (обычно hip зеркальный)
        fr = [-q1, q2, q3] 
        # RL
        rl = [q1, q2, q3]
        # RR
        rr = [-q1, q2, q3]
        
        msg.data = fl + fr + rl + rr
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DogController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()