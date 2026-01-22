import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time

class TrotGait(Node):
    def __init__(self):
        super().__init__('trot_gait_node')
        
        # Паблишер для отправки команд контроллеру
        self.publisher_ = self.create_publisher(
            Float64MultiArray, 
            '/joint_group_position_controller/commands', 
            10
        )
        
        # Таймер: 50 Гц (каждые 0.02 сек)
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        # === РЕАЛЬНЫЕ РАЗМЕРЫ РОБОТА (Метры) ===
        self.L1 = 0.06    # Hip (вбок)
        self.L2 = 0.144   # Thigh (бедро)
        self.L3 = 0.1525  # Shin (голень)
        
        # === ПАРАМЕТРЫ ПОХОДКИ ===
        self.walking_height = -0.16  # Присядем еще ниже (стабильнее)
        self.step_length = 0.04      # Короткий шаг (4 см)
        self.step_height = 0.03      # Низкий подъем
        self.period = 2.0            # ОЧЕНЬ медленный цикл (2 сек)
        
        self.start_time = time.time()
        self.get_logger().info("Trot Gait Started! Watch out!")

    def inverse_kinematics(self, x, y, z):
        """
        Считает углы для одной лапы.
        x, y, z - координаты стопы относительно крепления ноги (плеча).
        """
        # 1. Hip (Roll) - пока упростим и оставим 0 (без движения вбок)
        theta1 = 0.0
        
        # 2. Thigh & Shin (Pitch)
        # Учитываем, что L1 сдвигает ногу вбок, но IK решаем в плоскости ноги
        # Проекция на плоскость вращения бедра:
        dist_2d = math.sqrt(x**2 + z**2)
        
        # Защита от выхода за пределы длины ноги
        max_reach = self.L2 + self.L3 - 0.005
        if dist_2d > max_reach:
            dist_2d = max_reach

        # Теорема косинусов для колена (Shin)
        # c^2 = a^2 + b^2 - 2ab cos(gamma)
        cos_knee = (self.L2**2 + self.L3**2 - dist_2d**2) / (2 * self.L2 * self.L3)
        cos_knee = max(-1.0, min(1.0, cos_knee)) # Clamp
        phi = math.acos(cos_knee)
        
        # Угол голени (сгиб назад)
        theta3 = -(math.pi - phi)

        # Теорема косинусов для бедра (Thigh)
        # Угол наклона вектора к горизонту
        alpha = math.atan2(x, -z) 
        
        # Внутренний угол треугольника
        cos_beta = (self.L2**2 + dist_2d**2 - self.L3**2) / (2 * self.L2 * dist_2d)
        cos_beta = max(-1.0, min(1.0, cos_beta))
        beta = math.acos(cos_beta)
        
        theta2 = alpha + beta
        
        return theta1, theta2, theta3

    def get_leg_trajectory(self, t, phase_offset):
        """
        Вычисляет координаты (x, z) для лапы в текущий момент времени.
        t - текущее время
        phase_offset - смещение фазы (0 для одной диагонали, 0.5 для другой)
        """
        # Приводим время к циклу [0, 1)
        cycle_t = (t / self.period + phase_offset) % 1.0
        
        # Фаза переноса (Swing) - 0.0 до 0.5
        if cycle_t < 0.5:
            # Прогресс внутри фазы переноса (от 0 до 1)
            swing_progress = cycle_t / 0.5
            
            # X: Двигаемся вперед от -step/2 до +step/2
            # Используем косинус для плавного разгона и торможения
            x = self.step_length/2 * math.cos(math.pi * swing_progress)
            
            # Z: Поднимаем лапу по синусоиде (полукруг)
            z = self.walking_height + self.step_height * math.sin(math.pi * swing_progress)
            
        # Фаза опоры (Stance) - 0.5 до 1.0
        else:
            # Прогресс внутри фазы опоры
            stance_progress = (cycle_t - 0.5) / 0.5
            
            # X: Толкаем землю НАЗАД (от плюса к минусу)
            # Оригинальная формула была (cos), мы её инвертируем
            # Было: raw_x = self.step_length/2 * math.cos(...)
            # Стало: добавляем минус перед math.cos
            x = self.step_length/2 * -math.cos(math.pi * stance_progress)
            
            z = self.walking_height

        return x, 0.0, z # Y пока 0

    def timer_callback(self):
        # Текущее время
        t = time.time() - self.start_time
        
        # Генерируем траектории для двух диагоналей
        # Диагональ 1: FL и RR (offset 0.0)
        x1, y1, z1 = self.get_leg_trajectory(t, 0.0)
        
        # Диагональ 2: FR и RL (offset 0.5 - в противофазе)
        x2, y2, z2 = self.get_leg_trajectory(t, 0.5)

        # Считаем IK для каждой лапы
        # Порядок: FL, FR, RL, RR
        
        # Front Left (Diag 1)
        q_fl = self.inverse_kinematics(x1, y1, z1)
        
        # Front Right (Diag 2) -> Для правой стороны hip (q1) инвертируем, но он пока 0
        q_fr = self.inverse_kinematics(x2, y2, z2)
        
        # Rear Left (Diag 2)
        q_rl = self.inverse_kinematics(x2, y2, z2)
        
        # Rear Right (Diag 1)
        q_rr = self.inverse_kinematics(x1, y1, z1)

        # Собираем сообщение (12 углов)
        # Структура: [Hip, Thigh, Shin] для каждой лапы
        # ВАЖНО: Для правых лап (FR, RR) часто нужно инвертировать Hip, 
        # но так как Hip=0, это не важно.
        
        msg = Float64MultiArray()
        msg.data = [
            q_fl[0], q_fl[1], q_fl[2],   # FL
            q_fr[0], q_fr[1], q_fr[2],   # FR
            q_rl[0], q_rl[1], q_rl[2],   # RL
            q_rr[0], q_rr[1], q_rr[2]    # RR
        ]
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrotGait()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()