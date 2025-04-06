from vehicle import Driver
from controller import Supervisor
import os
import importlib.util
import math

robot = Driver()
timestep = int(robot.getBasicTimeStep())

VELOCITY = 5.0  # m/s
STEERING_ANGLE = 0.3  # radians

# Функции управления

def MoveForward(duration=2000):
    robot.setCruisingSpeed(VELOCITY)
    robot.setSteeringAngle(0)
    for _ in range(int(duration / timestep)):
        robot.step()
    robot.setCruisingSpeed(0)

def MoveBackward(duration=2000):
    robot.setCruisingSpeed(-VELOCITY)
    robot.setSteeringAngle(0)
    for _ in range(int(duration / timestep)):
        robot.step()
    robot.setCruisingSpeed(0)

def TurnRight(duration=1000):
    robot.setSteeringAngle(STEERING_ANGLE)
    robot.setCruisingSpeed(VELOCITY)
    for _ in range(int(duration / timestep)):
        robot.step()
    robot.setCruisingSpeed(0)
    robot.setSteeringAngle(0)

def TurnLeft(duration=1000):
    robot.setSteeringAngle(-STEERING_ANGLE)
    robot.setCruisingSpeed(VELOCITY)
    for _ in range(int(duration / timestep)):
        robot.step()
    robot.setCruisingSpeed(0)
    robot.setSteeringAngle(0)

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

while True:
    for _ in range(int(1000 / timestep)):
        robot.step()
    load_blockly_script()  # Проверяем файл каждую секунду
