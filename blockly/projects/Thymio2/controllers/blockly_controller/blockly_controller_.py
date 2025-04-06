from controller import Supervisor
import os
import importlib.util
import math

# 🔹 Инициализация Webots Supervisor
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# 🔹 Получаем узел робота
robot_node = robot.getFromDef("THYMIO")  # Убедитесь, что у вашего робота есть `DEF THYMIO`
if robot_node is None:
    print("Ошибка: узел THYMIO не найден! Убедитесь, что у робота есть DEF THYMIO в Webots.")
    exit()

position_field = robot_node.getField("translation")  # Для контроля позиции
rotation_field = robot_node.getField("rotation")  # Для контроля угла поворота

# 🔹 Подключаем моторы
left_motor = robot.getDevice("motor.left")
right_motor = robot.getDevice("motor.right")

# 🔹 Включаем режим управления скоростью
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# 🔹 Параметры робота Thymio II
WHEEL_RADIUS = 0.022  # Радиус колеса (м)
AXLE_LENGTH = 0.095  # Расстояние между колёсами (м)
VELOCITY = 5.0  # Скорость движения
TURN_VELOCITY = 1.0  # Скорость движения

# 🔹 Функции управления движением
def get_position():
    """Получает текущие координаты x, y робота"""
    pos = position_field.getSFVec3f()
    print(f"📍 Текущая позиция: x={pos[0]:.2f}, y={pos[1]:.2f}")
    return pos[0], pos[1]  # Берём только x и y

def get_yaw():
    """Получает текущий угол поворота (yaw) в градусах"""
    rotation = rotation_field.getSFVec3f()
    angle = rotation_field.getSFRotation()[3]  # Берём угол вращения в радианах
    yaw = math.degrees(angle)  # Переводим в градусы

    # Приводим угол к диапазону [-180, 180]
    if yaw > 180:
        yaw -= 360
    elif yaw < -180:
        yaw += 360

    print(f"↻ Текущий угол (yaw): {yaw:.2f}°")
    return yaw


def move_forward(distance=0.5):
    """Движение вперёд на заданную дистанцию (м) с контролем по координатам Supervisor"""
    start_x, start_y = get_position()
    target_x = start_x + distance * math.cos(math.radians(get_yaw()))
    target_y = start_y + distance * math.sin(math.radians(get_yaw()))

    print(f"🚀 Двигаемся вперед: цель x={target_x:.2f}, y={target_y:.2f}")

    left_motor.setVelocity(VELOCITY)
    right_motor.setVelocity(VELOCITY)

    while robot.step(timestep) != -1:
        x, y = get_position()
        if math.sqrt((x - start_x) ** 2 + (y - start_y) ** 2) >= distance:
            break

    stop()
    print("Остановились после движения вперед")

def move_backward(distance=0.5):
    """Движение назад на заданную дистанцию (м) с контролем Supervisor"""
    start_x, start_y = get_position()
    target_x = start_x - distance * math.cos(math.radians(get_yaw()))
    target_y = start_y - distance * math.sin(math.radians(get_yaw()))

    print(f"🔙 Двигаемся назад: цель x={target_x:.2f}, y={target_y:.2f}")

    left_motor.setVelocity(-VELOCITY)
    right_motor.setVelocity(-VELOCITY)

    while robot.step(timestep) != -1:
        x, y = get_position()
        if math.sqrt((x - start_x) ** 2 + (y - start_y) ** 2) >= distance:
            break

    stop()
    print("Остановились после движения назад")

def turn_left(angle=90):
    """Поворот влево на заданный угол (градусы) с контролем Supervisor"""
    initial_yaw = get_yaw()
    target_yaw = (initial_yaw + angle) % 360
    if target_yaw > 180:
        target_yaw = 0 - (360 - target_yaw)
    if target_yaw < -180:
        target_yaw = (360 + target_yaw)

    print(f"Поворот влево: {initial_yaw:.2f} цель угол={target_yaw:.2f}°")

    left_motor.setVelocity(-TURN_VELOCITY)
    right_motor.setVelocity(TURN_VELOCITY)

    while robot.step(timestep) != -1:
        current_yaw = get_yaw()
        if abs((current_yaw - target_yaw) % 360) < 0.3:  # Допустимая погрешность 2°
            break

    stop()
    print("Остановились после поворота влево")

def turn_right(angle=90):
    """Поворот вправо на заданный угол (градусы) с контролем Supervisor"""
    initial_yaw = get_yaw()
    target_yaw = (initial_yaw - angle) 
    if target_yaw > 180:
        target_yaw = 0 - (360 - target_yaw)
    if target_yaw < -180:
        target_yaw = (360 + target_yaw)

    print(f"↪️ Поворот вправо: цель угол={target_yaw:.2f}°")

    left_motor.setVelocity(TURN_VELOCITY)
    right_motor.setVelocity(-TURN_VELOCITY)

    while robot.step(timestep) != -1:
        current_yaw = get_yaw()
        if abs((current_yaw - target_yaw) % 360) < 0.3:
            break

    stop()
    print("Остановились после поворота вправо")

def stop():
    """Остановка робота"""
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

# Проверка и загрузка blockly_controller.py
def load_blockly_script():
    script_path = "blockly_code.py"
    if os.path.exists(script_path):
        print(f"Загружаем {script_path}...")
        try:
            with open(script_path, "r") as file:
                code = file.read()
            exec(code, globals())  # Выполняем код в глобальном контексте
            print(f"✅ {script_path} выполнен успешно.")
            os.remove(script_path)
        except Exception as e:
            print(f"Ошибка выполнения {script_path}: {e}")


while robot.step(timestep) != -1:
    load_blockly_script()  # Проверяем файл каждую секунду

