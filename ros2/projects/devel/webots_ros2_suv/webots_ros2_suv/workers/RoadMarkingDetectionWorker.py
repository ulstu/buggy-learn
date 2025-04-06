from AbstractWorker import AbstractWorker
import torch
import pathlib
import os
import numpy as np
import traceback
from PIL import Image
from ament_index_python.packages import get_package_share_directory
from fastseg import MobileV3Large
from fastseg.image import colorize, blend
from webots_ros2_suv.lib.timeit import timeit
from webots_ros2_suv.lib.lane_line_model import LaneLineModel
from webots_ros2_suv.lib.lane_line_model_utils import get_label_names, draw_lines, draw_segmentation, LaneLine, LaneMask, default_palette
from webots_ros2_suv.lib.map_builder import MapBuilder
from webots_ros2_suv.lib.LinesAnalizator import LinesAnalizator
from webots_ros2_suv.workers.IPMWorker import intersection_area
import cv2
import yaml
from ultralytics import YOLO

def intersect_person_with_crosswalk(world_model):
    crosswalk_boxes = []
    for points, label in world_model.detected_road_markings:
        if label == "crosswalk":
            p1 = np.array([np.min(points[:, 0]), np.min(points[:, 1])])
            p2 = np.array([np.max(points[:, 0]), np.max(points[:, 1])])
            crosswalk_boxes.append(np.array([p1, p2]))
    
    for yolo_box, label in world_model.yolo_detected_objects:
        if label == "person":
            for crosswalk_box in crosswalk_boxes:
                if intersection_area(yolo_box, crosswalk_box) > 0.5:
                    return True
    return False

PACKAGE_NAME = "webots_ros2_suv"
local_model_path = "resource/RMm/model.pt"
# local_model_config_path = "resource/lane_line_model/config.yaml"

package_dir = get_package_share_directory(PACKAGE_NAME)
project_settings_config_path = os.path.join(package_dir, "config/project_settings.yaml")
with open(project_settings_config_path, "r") as file:
    project_settings_config = yaml.safe_load(file)

class RoadMarkingDetectionWorker(AbstractWorker):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        package_dir = get_package_share_directory(PACKAGE_NAME)
        model_path = os.path.join(package_dir, local_model_path)
        # config_path = os.path.join(package_dir, local_model_config_path)

        self.model = YOLO(model_path)
        self.labels = self.model.names
        self.lines_analizator = LinesAnalizator()
        
        

    def on_event(self, event, scene=None):
        return None


    def on_data(self, world_model):
        try:
            if world_model:
                if project_settings_config["use_road_marking_detection"] == True:
                    img = Image.fromarray(world_model.rgb_image)

                    image_to_draw = np.copy(world_model.img_front_objects_prj_lines_signs)

                    # results = self.model.predict(np.array(img), verbose=True)
                    # # for mask in results[0].masks:
                    # #     for xy in mask.xy:
                    # #         cv2.drawContours(world_model.img_front_objects_lines_signs, [np.expand_dims(xy, 1).astype(int)], contourIdx=-1, color=0, thickness=-1)

                    # masks_points = None
                    # if results[0].masks is not None:
                    #     masks_points = np.array([np.array(xy) for xy in results[0].masks.xy])
                    
                    # labels = [self.model.names[int(label)] for label in results[0].boxes.cls]
                    
                    
                    # world_model.detected_road_markings = list(zip(masks_points, labels)) if masks_points is not None else []

                    # background_alpha = 0.7
                    # if results[0].masks is not None:
                    #     for xy in results[0].masks.xy:

                    #         # print("_*_" * 100)
                    #         # print(world_model.img_front_objects_prj_lines_signs_markings)
                    #         # print("_*_" * 100)

                    #         # image_mask = np.zeros_like(world_model.img_front_objects_prj_lines_signs_markings).astype(np.uint8)

                    #         # cv2.drawContours(image_mask, [np.expand_dims(xy, 1).astype(int)], 
                    #         #                  contourIdx=-1, 
                    #         #                  color=(255, 255, 255), thickness=-1)
                            
                    #         # indices = np.any(image_mask != np.array([0, 0, 0], dtype=np.uint8), axis=-1)
                    #         # world_model.img_front_objects_prj_lines_signs_markings[indices] = cv2.addWeighted(world_model.img_front_objects_prj_lines_signs_markings, 
                    #         #                                                                               1 - background_alpha, image_mask, background_alpha, 0, image_mask)[indices]
                    #         if xy.shape[0] == 0:
                    #             break
                    #         try:
                    #             cv2.drawContours(image_to_draw, [np.expand_dims(xy, 1).astype(int)], 
                    #                          contourIdx=-1, 
                    #                          color=(255, 210, 74), thickness=-1)
                    #         except:
                    #             pass
                    world_model.img_front_objects_prj_lines_signs_markings = image_to_draw
                
                # if intersect_person_with_crosswalk(world_model):
                #     zones = world_model.get_current_zones()
                #     pedestrian_on_crosswalk = False
                #     for zone in zones:
                #         if zone['name'] == 'crosswalk':
                #             pedestrian_on_crosswalk = True
                #             break
                #     world_model.pedestrian_on_crosswalk = pedestrian_on_crosswalk

        except  Exception as err:
            super().error(''.join(traceback.TracebackException.from_exception(err).format()))
        
        return world_model