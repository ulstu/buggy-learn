from webots_ros2_suv.states.AbstractState import AbstractState
from webots_ros2_suv.lib.map_utils import calc_dist_point
from webots_ros2_suv.lib.coords_transformer import CoordsTransformer
from webots_ros2_suv.lib.config_loader import GlobalConfigLoader
import math
import time

import pygame as pg
import numpy as np

screen_scale = 15

class ParkingFollowState(AbstractState):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__( *args, **kwargs)
        self.runs = 0
        self.__cur_path_point = 0
        self.directions = []
        self.current_direction = True
        self.prev_target_angle = 0
        pg.font.init()
        self.sysfont = pg.font.SysFont("Arial", 20)
        self.params = {}
        self.timer = 0
        self.allow_timer = False
        self.next_seg = False
        self.local_path = []
        self.coords_transformer = CoordsTransformer()
        self.current_direction = 1
        self.data = {"parking": [0, 0], "points": [], "parallel": True}
        self.before_points = []

    def find_goal_point_x(self, arr, val=100):
        current_length, max_length = 0
        current_start, max_start = 0

        for i, element in enumerate(arr):
            if element == val:
                current_length += 1

                if current_length > max_length:
                    max_length = current_length
                    max_start = current_start
            else:
                current_length = 0
                current_start = i + 1

        if max_length > 0:
            max_end = max_start + max_length - 1
            return max_start + int((max_end - max_start) / 3 * 2)
        else:
            return 0

    def is_obstacle_near(self, world_model, x, y, obstacle_val, robot_radius):
        for i in range(x - robot_radius, x + robot_radius):
            for j in range(y - robot_radius, y + robot_radius):
                if world_model.ipm_image[j, i] != obstacle_val:
                    return False
        return True

    def find_next_goal_point(self, world_model):
        if not world_model.global_map:
            return (0, 0)

        points = [e['coordinates'] for e in world_model.global_map if e['name'] == 'moving' and 'seg_num' in e and int(e['seg_num']) == world_model.cur_path_segment][0]
        points = points[self.__cur_path_point:]
        world_model.gps_path = points

        self.__cur_path_point = world_model.cur_path_point

        dist = math.sqrt(calc_dist_point(points[0], world_model.get_current_position()))
        if dist < self.config['change_point_dist']:
            self.__cur_path_point += 1

        world_model.cur_path_point = self.__cur_path_point

        x, y = world_model.coords_transformer.get_relative_coordinates(
            points[self.__cur_path_point][0], 
            points[self.__cur_path_point][1], 
            pos=world_model.get_current_position(),
            pov_point=world_model.pov_point
        )

        return (x, y)

    def gps_to_rect(self, dLon, dLat):
        # Номер зоны Гаусса-Крюгера
        zone = int(dLon / 6.0 + 1)

        # Параметры эллипсоида Красовского
        a = 6378245.0  # Большая (экваториальная) полуось
        b = 6356863.019  # Малая (полярная) полуось
        e2 = (a ** 2 - b ** 2) / a ** 2  # Эксцентриситет
        n = (a - b) / (a + b)  # Приплюснутость

        # Параметры зоны Гаусса-Крюгера
        F = 1.0  # Масштабный коэффициент
        Lat0 = 0.0  # Начальная параллель (в радианах)
        Lon0 = (zone * 6 - 3) * math.pi / 180  # Центральный меридиан (в радианах)
        N0 = 0.0  # Условное северное смещение для начальной параллели
        E0 = zone * 1e6 + 500000.0  # Условное восточное смещение для центрального меридиана

        # Перевод широты и долготы в радианы
        Lat = dLat * math.pi / 180.0
        Lon = dLon * math.pi / 180.0

        # Вычисление переменных для преобразования
        v = a * F * (1 - e2 * (math.sin(Lat) ** 2)) ** -0.5
        p = a * F * (1 - e2) * (1 - e2 * (math.sin(Lat) ** 2)) ** -1.5
        n2 = v / p - 1
        M1 = (1 + n + 5.0 / 4.0 * n ** 2 + 5.0 / 4.0 * n ** 3) * (Lat - Lat0)
        M2 = (3 * n + 3 * n ** 2 + 21.0 / 8.0 * n ** 3) * math.sin(Lat - Lat0) * math.cos(Lat + Lat0)
        M3 = (15.0 / 8.0 * n ** 2 + 15.0 / 8.0 * n ** 3) * math.sin(2 * (Lat - Lat0)) * math.cos(2 * (Lat + Lat0))
        M4 = 35.0 / 24.0 * n ** 3 * math.sin(3 * (Lat - Lat0)) * math.cos(3 * (Lat + Lat0))
        M = b * F * (M1 - M2 + M3 - M4)
        I = M + N0
        II = v / 2 * math.sin(Lat) * math.cos(Lat)
        III = v / 24 * math.sin(Lat) * (math.cos(Lat)) ** 3 * (5 - (math.tan(Lat) ** 2) + 9 * n2)
        IIIA = v / 720 * math.sin(Lat) * (math.cos(Lat) ** 5) * (61 - 58 * (math.tan(Lat) ** 2) + (math.tan(Lat) ** 4))
        IV = v * math.cos(Lat)
        V = v / 6 * (math.cos(Lat) ** 3) * (v / p - (math.tan(Lat) ** 2))
        VI = v / 120 * (math.cos(Lat) ** 5) * (5 - 18 * (math.tan(Lat) ** 2) + (math.tan(Lat) ** 4) + 14 * n2 - 58 * (math.tan(Lat) ** 2) * n2)

        # Вычисление северного и восточного смещения (в метрах)
        N = I + II * (Lon - Lon0) ** 2 + III * (Lon - Lon0) ** 4 + IIIA * (Lon - Lon0) ** 6
        E = E0 + IV * (Lon - Lon0) + V * (Lon - Lon0) ** 3 + VI * (Lon - Lon0) ** 5

        return [E, -N]
    
    def rotate_point(self, center, target, angle):
        return [math.cos(angle) * (target[0] - center[0]) - math.sin(angle) * (target[1] - center[1]) + center[0],
                math.sin(angle) * (target[0] - center[0]) + math.cos(angle) * (target[1] - center[1]) + center[1]]

    def move_screen(self, x, y):
        return [int(400 + x * screen_scale), int(400 + y * screen_scale)]

    def AngleOfReference(self, v):
        return self.NormalizeAngle(math.atan2(v[1], v[0]) / math.pi * 180)

    def AngleOfVectors(self, first, second):
        return self.NormalizeAngle(self.AngleOfReference(first) - self.AngleOfReference(second))

    def NormalizeAngle(self, angle):
        if angle > -180:
            turn = -360
        else:
            turn = 360
        while not (angle > -180 and angle <= 180):
            angle += turn
        return angle
    
    def MedianVector(self, first, second, k):
        return [first[0] * k + second[0] * (1 - k), first[1] * k + second[1] * (1 - k)]


    def on_event(self, event, world_model=None):
        world_model.software_state = 'Auto'
        self.runs = self.runs + 1
        self.log("gps follow state")
        
        # if world_model.traffic_light_state == 'red':
        #     return 'stop'

        # world_model.goal_point = (self.find_goal_point_x(world_model.ipm_image[10,:]), 10)
        # world_model.goal_point = self.find_next_goal_point(world_model)
        
        lat, lon, orientation = world_model.get_current_position() # Текущее месторасположение автомобиля
        orientation -= 1.5
        car_position = self.gps_to_rect(lat, lon)
        car_vector = [math.cos(orientation) * 2, math.sin(orientation) * 2]
        difference = None
        for event in pg.event.get():
            if event.type == pg.MOUSEBUTTONDOWN:
                x, y = pg.mouse.get_pos()
                if event.button == 1:
                    self.local_path = []
                    start_point = (0, 0)
                    if len(self.data["points"]) > 0:
                        start_point =  self.rotate_point((0, 0), self.data["points"][-1], orientation + math.pi / 2)
                    for i in range(len(self.data["points"])):
                        x_i, y_i, dir = self.data["points"][i]
                        x_i, y_i = self.rotate_point((0, 0), (x_i, y_i), orientation + math.pi / 2)
                        self.loge(f"{x_i - start_point[0]}, {y_i - start_point[1]}")
                        self.local_path.append(self.coords_transformer.get_global_coordinates((x - 400) + (x_i - start_point[0]) * screen_scale, (y - 400) + (y_i - start_point[1]) * screen_scale, (lat, lon, 0), (0, 0)))
                        self.directions.append(dir)
                    self.__cur_path_point = 0
                # if event.button == 3:  #  левая кнопка мыши
                    # self.before_points.append(self.coords_transformer.get_global_coordinates(x, y, (lat, lon, 0), (400, 400)))
            if event.type == pg.KEYDOWN:
            #     if event.key == pg.K_z:
            #         self.before_points.pop()
                if event.key == pg.K_x:
                    self.data = GlobalConfigLoader(self.config["parking_parallel"]).data[0]
            #     if event.key == pg.K_c:
            #         self.data = GlobalConfigLoader(self.config["parking_garage"]).data[0]
        
        points = self.before_points + self.local_path

        
        # while True:
        points_offset = self.__cur_path_point
        points = points[points_offset:] # Удаляем из него те точки, которые были достигнуты автомобилем
        directions = self.directions[points_offset:]
        world_model.gps_path = points

        if len(directions) > 0:
            if self.__cur_path_point == 0:
                self.current_direction = directions[0]
            elif self.current_direction != directions[0]:
                self.timer = 40
                self.allow_timer = True
                self.current_direction = directions[0]
        speed = self.config['default_GPS_speed']


        path_square_points = []
        for point in points:
            path_square_points.append(self.gps_to_rect(point[0], point[1]))

        if not self.allow_timer and len(path_square_points) > 0:
            difference = [path_square_points[0][0] - car_position[0], path_square_points[0][1] - car_position[1]]
            if len(path_square_points) > 1 and directions[1] == directions[0]:
                nearest_point = self.MedianVector(path_square_points[0], path_square_points[1], 0.75)
                difference = [nearest_point[0] - car_position[0], nearest_point[1] - car_position[1]]

            dist = math.sqrt(difference[0] ** 2 + difference[1] ** 2)
            conf = self.config['change_point_dist']
            if dist < self.config['change_point_dist']:
                self.__cur_path_point += 1
                world_model.cur_path_point = self.__cur_path_point
        if not self.allow_timer and len(path_square_points) > 0:        
            res_car_vector = car_vector
            if self.current_direction != 1:
                res_car_vector = [-car_vector[0], -car_vector[1]]
            difference_angle = -self.AngleOfVectors(res_car_vector, difference)
            Pk = 45
            Ik = 0.2
            if speed > 10:
                Pk = 90
                Ik = 0.2
            if self.current_direction != 1:
                Pk = 27
                Ik = 0.2
            world_model.gps_car_turn_angle = float(min(1, max(-1, difference_angle / Pk)))
            diff_angle = (self.prev_target_angle - world_model.gps_car_turn_angle) * Ik
            world_model.gps_car_turn_angle = (world_model.gps_car_turn_angle + diff_angle ) * self.current_direction
            # self.logi(f"diff = {self.prev_target_angle - world_model.gps_car_turn_angle} new = {world_model.gps_car_turn_angle}, old ={self.prev_target_angle}")
            
            self.params["new"] = world_model.gps_car_turn_angle
            self.params["old"] = self.prev_target_angle
            self.prev_target_angle = world_model.gps_car_turn_angle
        # else:
            # world_model.gps_car_turn_angle = 0.0
        self.params["diff"] = self.prev_target_angle - world_model.gps_car_turn_angle
        self.params["prevzone"] = world_model.previous_zone


        zones = world_model.get_current_zones()
        for zone in zones:
            if zone['name'].startswith("speed"):
                speed = int(zone['name'].split('speed')[1])  * self.current_direction
        pg.event.get()
        world_model.sc.fill((0, 0, 0))

        pg.image.frombuffer(world_model.ipm_colorized.tostring(), world_model.ipm_colorized.shape[1::-1], "BGR")

        is_obstacle_in_area = True
        rect1 = pg.Rect(-1, 0, 2, self.config["obstacle_stop_distance"])

        self.has_obstacle = False
        for obstacle in world_model.get_obstacles():
            obstacle_points = [[-obstacle[8], obstacle[10]],
                               [-obstacle[9], obstacle[10]],
                               [-obstacle[9], obstacle[11]],
                               [-obstacle[8], obstacle[11]]]
            inflated_obstacles = [[-obstacle[8] - 10, obstacle[10]],
                                  [-obstacle[9] + 10, obstacle[10]],
                                  [-obstacle[9] + 10, obstacle[11]],
                                  [-obstacle[8] - 10, obstacle[11]]]
            
            x_dif = -obstacle[8] + obstacle[9]
            y_dif = obstacle[11] - obstacle[10]
            if x_dif < 1:
                x_dif = 1
            if y_dif < 1:
                y_dif = 1
            rect2 = pg.Rect(-obstacle[9], obstacle[10], x_dif, y_dif)
            # self.draw_rect(world_model.sc, (rect2), orientation, (255, 255, 255))
            color = (255, 255, 0)
            if rect1.colliderect(rect2):
                color = (255, 0, 0)
                self.has_obstacle = True

            rotated_obstacle = []
            rotated_inflated_obstacle = []
            for i in range(4):
                rotated_obstacle.append(self.rotate_point(center=[0, 0], target=[obstacle_points[i][0], obstacle_points[i][1]], angle=(orientation - (math.pi / 2))))
                rotated_inflated_obstacle.append(self.rotate_point(center=[0, 0], target=[inflated_obstacles[i][0], inflated_obstacles[i][1]], angle=(orientation - (math.pi / 2))))
            self.draw_box(world_model.sc, rotated_obstacle, color)
            # self.draw_box(world_model.sc, rotated_inflated_obstacle, (255, 0, 0))

        pg.draw.line(world_model.sc, (255,0,0), self.move_screen(0, 0), self.move_screen(car_vector[0], car_vector[1]))
        if difference != None:
            pg.draw.line(world_model.sc, (0,255,0), self.move_screen(0, 0), self.move_screen(difference[0], difference[1]))
        for i in range(len(path_square_points)):
            point = path_square_points[i]
            color =  (255, 0, 0)
            if directions[i] != 1:
                color =  (0, 255, 0)
            pg.draw.circle(world_model.sc, color, self.move_screen(point[0] - car_position[0], point[1] - car_position[1]), 4)
        self.draw_rect(world_model.sc, pg.Rect(-1, 0, 2, self.config["obstacle_stop_distance"]), orientation, (255, 255, 255))
        # rect1 = pg.Rect(-1.5, 0, 3, self.config["obstacle_stop_distance"])
        
        # self.draw_rect(world_model.sc, (rect1), orientation, (255, 255, 255))
        self.params["has_obstacle"] = self.has_obstacle

        event = None

        # self.logi(f'ipm {world_model.ipm_colorized.shape}')
        speed = speed  * self.current_direction
        zones_names = []
        for zone in zones:
            if zone['name'] == 'turn':
                world_model.cur_turn_polygon = zone['coordinates'][0]
                self.__cur_path_point = 0
                event = 'turn'
            elif zone['name'] == 'terminal' and self.__cur_path_point > 10:
                self.logi(f"{world_model.previous_zone}")
                self.logi(f"Inside terminal")
                world_model.previous_zone = zone['name']
                speed = 0
                world_model.cur_path_point = 0
                self.__cur_path_point = 0
                event = 'pause'
            elif zone['name'] == 'traffic_light':
                if world_model.traffic_light_state == 'red':
                    speed = 0
                    # self.__cur_path_point = 0
                    event = 'stop'
            elif zone['name'] == 'crosswalk':
                if world_model.pedestrian_on_crosswalk:
                    speed = 0
                    # self.__cur_path_point = 0
                    event = 'stop'
            elif zone['name'] == 'stop':
                self.__cur_path_point = 0
                event = 'stop'
            elif zone["name"] == "obstacle_stop":
                speed = 7
                if self.has_obstacle and self.config["stop_obstacles"]:
                    print("stop")
                    speed = 0
            elif zone["name"] == "traffic_light":
                if self.has_obstacle and self.config["stop_obstacles"]:
                    print("stop")
                    speed = 0
            elif zone["name"] == "next_segment" and world_model.previous_zone != zone["name"]:
                self.next_seg = True
                self.timer = 40
                self.allow_timer = True
                speed = 0
            elif zone["name"] == "to_obstacles" and world_model.previous_zone != zone["name"]:
                world_model.cur_path_segment += 1
                world_model.cur_path_point = 0
                self.__cur_path_point = 0
                event = 'start_move'
            elif zone['name'].startswith('pause_tick') and world_model.previous_zone != zone["name"]:
                self.next_seg = True
                self.timer = int(zone['name'].split('pause_tick_')[1])
                self.allow_timer = True
                speed = 0
            if not zone['name'].startswith("speed"):
                world_model.previous_zone = zone["name"]
                zones_names.append(zone["name"])

        # if world_model.is_obstacle_before_path_point(filter_num=2, log=self.log):
        #     event = 'start_move'
        # if world_model.is_lane_road():
        #     event = 'start_lane_follow'
        self.params["zones"] = zones_names
        self.params["speed"] = speed
        self.params["allow_timer"] = self.allow_timer
        self.params["timer"] = self.timer
        self.params["traficlight"] = world_model.traffic_light_state
        self.params["segment"] = world_model.cur_path_segment
        self.params["event"] = event
        self.params["cur_path_point"] = self.__cur_path_point
        self.params["state"] = "TapToMove"
        self.params["current_direction"] = self.current_direction
        y = 10
        for k, v in self.params.items():
            text_reward = self.sysfont.render(f"{k}: {v}", False, (255, 0, 0))
            world_model.sc.blit(text_reward, (0, y))
            y += 15
        pg.display.update()
        if self.allow_timer and self.timer > 0:
            self.timer -= 1
            speed = 0
        elif self.allow_timer and self.timer <= 0:
            self.allow_timer = False
            if self.next_seg:
                world_model.cur_path_segment += 1
                self.__cur_path_point = 0
        if event:
            world_model.path = None
        world_model.set_speed(speed)
        self.drive(world_model, speed=speed)
        return event
    
    def draw_box(self, screen, bounds, color):
        pg.draw.line(screen,
                    color,
                    self.move_screen(bounds[0][0], bounds[0][1]),
                    self.move_screen(bounds[1][0], bounds[1][1]))
        pg.draw.line(screen,
                    color,
                    self.move_screen(bounds[1][0], bounds[1][1]),
                    self.move_screen(bounds[2][0], bounds[2][1]))
        pg.draw.line(screen,
                    color,
                    self.move_screen(bounds[2][0], bounds[2][1]),
                    self.move_screen(bounds[3][0], bounds[3][1]))
        pg.draw.line(screen,
                    color,
                    self.move_screen(bounds[3][0], bounds[3][1]),
                    self.move_screen(bounds[0][0], bounds[0][1]))

    def draw_rect(self, screen, rect : pg.Rect, orientation, color):
        orientation -= math.pi / 2
        bounds = [
            self.rotate_point((0, 0), (rect.left, rect.bottom), orientation),
            self.rotate_point((0, 0), (rect.right, rect.bottom), orientation),
            self.rotate_point((0, 0), (rect.right, rect.top), orientation),
            self.rotate_point((0, 0), (rect.left, rect.top), orientation),
        ]
        pg.draw.line(screen,
                    color,
                    self.move_screen(bounds[0][0], bounds[0][1]),
                    self.move_screen(bounds[1][0], bounds[1][1]))
        pg.draw.line(screen,
                    color,
                    self.move_screen(bounds[1][0], bounds[1][1]),
                    self.move_screen(bounds[2][0], bounds[2][1]))
        pg.draw.line(screen,
                    color,
                    self.move_screen(bounds[2][0], bounds[2][1]),
                    self.move_screen(bounds[3][0], bounds[3][1]))
        pg.draw.line(screen,
                    color,
                    self.move_screen(bounds[3][0], bounds[3][1]),
                    self.move_screen(bounds[0][0], bounds[0][1]))
        
    def get_cur_directrion(self) -> int:
        return self.directions[self.__cur_path_point] if len(self.directions) != 0 else 1
    
    def get_directrion(self, idx) -> int:
        return self.directions[idx] if len(self.directions) != 0 or 0 <= idx < len(self.directions) else 1
    
    def move_relative(self, x, y):
        return (x - 400) / screen_scale, (y - 400) / screen_scale
    # def has_obstacle(self, world_model):
    #     rect1 = pg.Rect(-1, 0, 2, self.config["obstacle_stop_distance"])
    #     for obstacle in world_model.obstacles:
    #         rect2 = pg.Rect(int(obstacle[8]), int(obstacle[10]), int(obstacle[9] - obstacle[8]), int(obstacle[11] - obstacle[10]))
    #         if rect1.colliderect(rect2):
    #             return True
    #     return False
