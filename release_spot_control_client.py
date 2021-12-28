# Copyright (c) 2021 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""WASD driving of robot."""

from __future__ import print_function
from collections import OrderedDict
import curses
import logging
import signal
import sys
from threading import Thread
import time
import math
import io
import os
from bosdyn.client import image
import urllib3
import base64
import socketio
import json
import graph_nav_util


#add spot_cam_function
import argparse
import bosdyn.client
from bosdyn.client.util import (add_common_arguments, setup_logging)
from audio import AudioCommands
from compositor import CompositorCommands
from health import HealthCommands
from lighting import LightingCommands
from media_log import MediaLogCommands
from network import NetworkCommands
from power import PowerCommands
from ptz import PtzCommands
from streamquality import StreamQualityCommands
from utils import UtilityCommands
from version import VersionCommands
from webrtc import WebRTCCommands
from bosdyn.client import spot_cam

from bosdyn.api import geometry_pb2
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.api.graph_nav import graph_nav_pb2, map_pb2, nav_pb2

import bosdyn.mission.client
import bosdyn.api.mission
from bosdyn.api.mission import mission_pb2, nodes_pb2

import bosdyn.api.power_pb2 as PowerServiceProto
import bosdyn.api.robot_state_pb2 as robot_state_proto
import bosdyn.api.basic_command_pb2 as basic_command_pb2
import bosdyn.api.spot.robot_command_pb2 as spot_command_pb2
from bosdyn.client import create_standard_sdk, ResponseError, RpcError
from bosdyn.client.async_tasks import AsyncGRPCTask, AsyncPeriodicQuery, AsyncTasks
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.lease import Error as LeaseBaseError
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.client.image import ImageClient
from bosdyn.client.robot_state import RobotStateClient
import bosdyn.client.util
from bosdyn.client.frame_helpers import get_odom_tform_body
import numpy as np

LOGGER = logging.getLogger()

# 로봇 관절에 대해, URDF에서 사용되는 관절 이름으로 변경하여 저장하는 자료형
joint_names = {}
joint_names["fl.hx"] = "front_left_hip_x"
joint_names["fl.hy"] = "front_left_hip_y"
joint_names["fl.kn"] = "front_left_knee"
joint_names["fr.hx"] = "front_right_hip_x"
joint_names["fr.hy"] = "front_right_hip_y"
joint_names["fr.kn"] = "front_right_knee"
joint_names["hl.hx"] = "rear_left_hip_x"
joint_names["hl.hy"] = "rear_left_hip_y"
joint_names["hl.kn"] = "rear_left_knee"
joint_names["hr.hx"] = "rear_right_hip_x"
joint_names["hr.hy"] = "rear_right_hip_y"
joint_names["hr.kn"] = "rear_right_knee"

# socket.io 클라이언트 생성 (ssl_velrify : https 접속시 요구되는 ssl 인증에 대한 boolean)
socket = socketio.Client(ssl_verify=False)

##
# @brief 소켓 서버 접속시 수신되는 socket event
@socket.event
def connect():
    LOGGER.info('spot socket server connected')
    # It transmits the current pan, tilt, and zoom values ​​of 
    # the Spotcam to the robot server so that the Spotcam can operate from the position it last moved.
    socket.emit('spot_cam_init_position', get_spot_position())

##
# @brief 소켓 서버와의 접속이 해제될 때 수신되는 socket event
@socket.event
def disconnect():
    LOGGER.info('disconnect from spot socket server')

## 
# @brief 소켓 서버에 웹 클라이언트가 접속할 경우 수신되는 socket event
# @param data 웹 클라이언트의 String ID 값
@socket.event
def remote_client_connect(data):
    wasd_interface.set_remote_client_id(data)
    socket.emit('spot_cam_init_position', get_spot_position())

## 
# @brief 소켓 서버로 부터 웹 클라이언트가 연결 해제될 경우 수신되는 socket event
@socket.event
def remote_client_disconnect():
    remote_client_id = ""
    wasd_interface.set_remote_client_id(remote_client_id)
    wasd_interface._call_safe_power_off()

##
# @brief 파이썬 클라이언트가 서버에 접속한 이후에도, 웹 클라이언트가 접속되지 않았을때 수신되는 socket event
@socket.event
def remote_client_not_yet():
    remote_client_id = ""
    wasd_interface.set_remote_client_id(remote_client_id)

## 
# @brief 서버로부터 웹 클라이언트의 Estop 제어 요청을 수신하는 socket event
@socket.event
def spot_control_estop():
    wasd_interface._call_estop()

##
# @brief 서버로부터 웹클라이언트의 모터 Power 제어(ON) 요청을 수신하는 socket event
@socket.event
def spot_control_power_on():
    wasd_interface._call_power_on()

##
# @brief 서버로부터 웹클라이언트의 모터 Power 제어(OFF) 요청을 수신하는 socket event
@socket.event
def spot_control_power_off():
    wasd_interface._call_safe_power_off()

##
# @brief 서버로부터 웹클라이언트의 로봇 sit 요청을 수신하는 socket event
@socket.event
def spot_control_sit():
    wasd_interface._call_sit()

##
# @brief 서버로부터 웹클라이언트의 로봇 stand 요청을 수신하는 socket event
@socket.event
def spot_control_stand():
    wasd_interface._call_stand()

##
# @brief 서버로부터 웹클라이언트의 로봇 이동 제어 명령 요청을 수신하는 socket event
# @param data 로봇 이동 제어에 대한 파라미터 (data[0]: 전진 후진에 대한 속도, data[1]: 좌우 이동에 대한 속도, data[2]: 로봇 회전에 대한 속도)
@socket.event
def spot_control_cmd(data):
    wasd_interface._call_custom_drive(data[0], data[1], data[2])

##
# @brief 서버로부터 웹클라이언트의 로봇 자세 제어 명령 요청을 수신하는 socket event
# @param data yaw, roll, pitch 값 List (data[0]:yaw, data[1]:roll, data[2]:pitch]
@socket.event
def spot_pose_cmd(data):
    wasd_interface._call_custom_pose(data[0], data[1], data[2])

##
# @brief 서버로부터 Autowalk replay mission 기능 요청을 수신하는 socket event 
# @param data autowalk 파일 이름
@socket.event
def replay_misson(data): 
    wasd_interface._call_spot_replay_mission(data)

##
# @brief 서버로부터 Waypoint 이동 명령 요청을 수신하는 socket event
# @param data Autowalk 파일 이름
# @param waypoint_id Waypoint ID 값
@socket.event
def go_to_waypoint(data, waypoint_id):
    wasd_interface._navigate_to(data, waypoint_id)

# @author : Chulhee Lee
# @brief : All commands for spot cam operation received from robot web client
# @param : object(spot cam command parameter)
@socket.event
def spot_cam_control(data):
    """Processes all commands related to spot cam operation received from robot server"""

    # Operate the spot cam with the data received from the robot server
    cam_command_interface._set_command_options(data)

    # Initialize the spot cam command setting after performing the robot command
    cam_command_interface._initialize_command(data)
VELOCITY_CMD_DURATION = 0.8  # seconds

def _grpc_or_log(desc, thunk):
    try:
        return thunk()
    except (ResponseError, RpcError) as err:
        LOGGER.error("Failed %s: %s" % (desc, err))

# @author : Chulhee Lee
# @brief : Operate from the current position of spot ptz cam
# @param : None
# @return : dict(key : 'pan', 'tilt', 'zoom' / value : float)
def get_spot_position():
    """Operate from the current position of spot ptz cam"""
    
    #spot cam ptz position command
    get_spot_cam_posotion = cam_command_interface._set_command_options(
                            {'command' : 'ptz', 'ptz_command' : 'get_position', 'ptz_name' : 'mech'})
    
    #Stored as keys and values ​​in dict to manipulate spot cam on the web
    init_spot_cam_position = {}

    init_spot_cam_position['pan'] = get_spot_cam_posotion.pan.value
    init_spot_cam_position['tilt'] = get_spot_cam_posotion.tilt.value
    init_spot_cam_position['zoom'] = get_spot_cam_posotion.zoom.value

    return init_spot_cam_position

## 
# @brief 쓰레드 상태를 업데이트 하는 함수
# @param async_task 업데이트 할 thread task 리스트
def _update_thread(async_task):
    while True:
        async_task.update()
        time.sleep(0.01)

## 
# @brief 5개의 카메라 이미지 중, depth 정보를 제외한 fisheye 이미지 이름만 담은 List를 반환하는 함수
# @return image_list fisheye 카메라 이미지 이름을 담은 List
def get_source_list():    
    image_list = []
    image_list.append("frontleft_fisheye_image")
    image_list.append("frontright_fisheye_image")
    image_list.append("left_fisheye_image")
    image_list.append("right_fisheye_image")
    image_list.append("back_fisheye_image")    
  
    return image_list

##
# @brief 5개의 fisheye 이미지를 base64 포맷으로 인코딩하여 서버에 이미지 데이터를 전송하는 함수
# @param image_task 이미지 데이터
# @param sleep_between_capture Thread 진행 sleep 시간
def capture_images(image_task, sleep_between_capture):
    while True:
        images_responses = image_task.proto
        if not images_responses:
            continue

        for image in images_responses:
            try:
                img_str = base64.b64encode(image.shot.image.data)
                img_str_decode = img_str.decode("utf-8")
                socket.emit(str(image.source.name), img_str_decode)
            except Exception as exc:
                print(f'Exception occurred during image capture {exc}')
        time.sleep(sleep_between_capture)

## 
# @brief 5개의 fisheye 이미지에 대해 데이터를 요청하는 비동기 클래스
# @param AsyncPeriodicQuery 요청할 쿼리명과 API 클라이언트 정보
# @return 이미지 정보가 담긴 데이터 (cols, rows, bytes, ...)
class AsyncImage(AsyncPeriodicQuery):
    def __init__(self, image_client, image_sources):
        # Period is set to be about 15 FPS
        super(AsyncImage, self).__init__('images', image_client, LOGGER, period_sec=0.067)
        self.image_sources = image_sources
    def _start_query(self):
        return self._client.get_image_from_sources_async(self.image_sources)

##
# @brief 프로그램에 종료 여부를 확인하는 클래스
# @param object 
class ExitCheck(object):
    """A class to help exiting a loop, also capturing SIGTERM to exit the loop."""
    def __init__(self):
        self._kill_now = False
        signal.signal(signal.SIGTERM, self._sigterm_handler)
        signal.signal(signal.SIGINT, self._sigterm_handler)

    def __enter__(self):
        return self

    def __exit__(self, _type, _value, _traceback):
        return False

    def _sigterm_handler(self, _signum, _frame):
        self._kill_now = True

    def request_exit(self):
        """Manually trigger an exit (rather than sigterm/sigint)."""
        self._kill_now = True

    @property
    def kill_now(self):
        """Return the status of the exit checker indicating if it should exit."""
        return self._kill_now

## 
# @brief 로봇의 기본 상태 정보를 요청하는 비동기 클래스
# @param AsyncPeriodicQuery 요청할 쿼리명과 API 클라이언트 정보
# @return 로봇의 기본 상태 정보가 담긴 데이터
class AsyncRobotState(AsyncPeriodicQuery):
    """Grab robot state."""
    def __init__(self, robot_state_client):
        super(AsyncRobotState, self).__init__("robot_state", robot_state_client, LOGGER,
                                              period_sec=0.02)
    
    def _start_query(self):
        return self._client.get_robot_state_async()

## 
# @brief GraphNav API 기능 사용시, 로봇의 localization 정보를 요청하는 비동기 클래스
# @param AsyncPeriodicQuery 요청할 쿼리명과 API 클라이언트 정보
# @return GraphNav API 기능 사용시 로봇에 localization 상태 데이터 (position, rotation, joint state)
class AsyncGraphNavState(AsyncPeriodicQuery):
    """Grab graph nav state"""

    def __init__(self, graph_nav_client):
        super(AsyncGraphNavState, self).__init__("graphnav_state", graph_nav_client, LOGGER, period_sec=0.02)

    def _start_query(self):
        return self._client.get_localization_state_async()

##
# @brief SPOT 로봇 제어에 대한 기능을 포함하는 클래스
# @param object API 인증 완료후 생성된 robot object
class WasdInterface(object):
    """A curses interface for driving the robot."""
    
    # The constructor
    def __init__(self, robot):
        self._robot = robot
        self._lease_client = robot.ensure_client(LeaseClient.default_service_name) # 로봇 제어 권한 관련 API 클라이언트
        try:
            self._estop_client = self._robot.ensure_client(EstopClient.default_service_name) # Estop 상태 관련 API 클라이언트
            self._estop_endpoint = EstopEndpoint(self._estop_client, 'GNClient', 9.0)
        except:
            self._estop_client = None
            self._estop_endpoint = None

        self._power_client = robot.ensure_client(PowerClient.default_service_name) # 로봇 power 관련 API 클라이언트 
        self._robot_state_client = robot.ensure_client(RobotStateClient.default_service_name) # 로봇 상태 관련 API 클라이언트
        self._robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name) # 로봇 명령 관련 API 클라이언트
        self._image_client = robot.ensure_client(ImageClient.default_service_name) # 로봇 이미지 관련 API 클라이언트
        self._graph_nav_client = self._robot.ensure_client(GraphNavClient.default_service_name) # GraphNav API 클라이언트
        self._image_list = get_source_list()

        self._robot_state_task = AsyncRobotState(self._robot_state_client) # 로봇 상태에 대해 비동기로 grab 하는 클래스
        self._image_task = AsyncImage(self._image_client , self._image_list) # 로봇 이미지에 대해 비동기로 grab 하는 클래스 
        self._graph_task = AsyncGraphNavState(self._graph_nav_client) # graph_nav 상태에 대해 비동기로 grab 하는 클래스 

        self._task_list = [self._robot_state_task, self._image_task, self._graph_task] # 비동기 task에 대한 리스트     
        self._async_tasks = AsyncTasks(self._task_list) # 비동기 task_list 관련 API 클라이언트
    
        self._estop_keepalive = None
        self._exit_check = None 

        # Stuff that is set in start()
        self._robot_id = None
        self._lease = None
        self._lease_keepalive = None

        self._remote_client_id = ""

        self._current_graph = None # 로봇에 현재 업로드된 graph 유무를 체크하는 변수
        self._current_edges = dict()  # maps to_waypoint to list(from_waypoint)
        self._current_waypoint_snapshots = dict()  # maps id to waypoint snapshot
        self._current_edge_snapshots = dict()  # maps id to edge snapshot
        self._current_annotation_name_to_wp_id = dict()

        self._mission_sdk = bosdyn.client.create_standard_sdk('MissionReplay', [bosdyn.mission.client.MissionClient]) # 로봇 Mission 관련 API 클라이언트
        self._mission_robot = self._mission_sdk.create_robot("192.168.80.3")
        self._mission_robot.authenticate("admin", "uhkqr0sv0ko1")
        self._mission_robot.time_sync.wait_for_sync()
        
        self._current_walk_name = None

        update_thread = Thread(target=_update_thread, args=[self._async_tasks])
        update_thread.daemon = True
        update_thread.start()
        
        while any(task.proto is None for task in self._task_list):
            robot.logger.info("wait for task.proto...")
            time.sleep(0.1)

        image_capture_thread = Thread(target=capture_images, args=(self._image_task, 0.05))
        image_capture_thread.start()

    ##
    # @brief Autowalk replay 또는 Waypoint 이동 시 로봇의 position, rotation, joint 값을 서버에 전송하는 함수 
    def _joint_state(self):
        position_data = OrderedDict()
        rotation_data = OrderedDict()
        joint_data = OrderedDict()

        joint_states = self.graph_state.robot_kinematics.joint_states 
        autowalk_state = self.graph_state.localization.seed_tform_body 
        
        position_data['x'] = autowalk_state.position.x
        position_data['y'] = autowalk_state.position.y
        position_data['z'] = autowalk_state.position.z
        rotation_data['x'] = autowalk_state.rotation.x
        rotation_data['y'] = autowalk_state.rotation.y
        rotation_data['z'] = autowalk_state.rotation.z
        rotation_data['w'] = autowalk_state.rotation.w

        for i in joint_states:
            joint_data[joint_names.get(i.name)] = i.position.value
        
        json_data = json.dumps(joint_data, ensure_ascii=False, indent=4)
        joint_data = json.loads(json_data)
        socket.emit("joint_state", {'joint_state': joint_data, 'rotation_state':rotation_data, 'position_state':position_data})

    ##
    # @brief waypoint 이동 명령에 관한 navigate_to API command 호출 함수
    # @param path waypoint가 포함된 Autowalk 이름 (경로)
    # @param waypoint_id 이동할 waypoint의 id값
    # @return _check_success 함수 결과
    def _navigate_to(self, path, waypoint_id):
        """Navigate to a specific waypoint."""

        self._robot.logger.info('Autowalk name : {} navigate_to : {}'.format(path, waypoint_id))
        self._current_graph = self._graph_nav_client.download_graph()
        localization_id = self._graph_nav_client.get_localization_state().localization.waypoint_id
        self._robot.logger.info('Localization_id :{}'.format(localization_id))
        
        if localization_id is "":
            current_odom_tform_body = get_odom_tform_body(
            self.robot_state.kinematic_state.transforms_snapshot).to_proto()
            localization = nav_pb2.Localization()
            self._graph_nav_client.set_localization(initial_guess_localization=localization,
                                                    ko_tform_body=current_odom_tform_body)
            time.sleep(1)

        try:
            self._current_annotation_name_to_wp_id, self._current_edges = graph_nav_util.update_waypoints_and_edges(self._current_graph, localization_id)
            time.sleep(1)
        except:
            self._robot.logger.info('Exception : update_waypoints_and_edges')

        # 이동할 destination_waypoint 설정
        destination_waypoint = graph_nav_util.find_unique_waypoint_id(waypoint_id, self._current_graph, self._current_annotation_name_to_wp_id)        

        if not destination_waypoint:
            return
        
        nav_to_cmd_id = None
        is_finished = False
        
        vel_limit = self._set_vel_limit(0.4, 0.4, -0.4, -0.4) # 로봇 주행 속도 설정
        travel_params = self._graph_nav_client.generate_travel_params(max_distance=None, max_yaw=None, velocity_limit=vel_limit) # GraphNav 주행 파라미터 생성 API 호출
        
        while not is_finished:
         
            try:
                # navigate_to API 함수 호출
                nav_to_cmd_id = self._graph_nav_client.navigate_to(destination_waypoint, 2.0, travel_params=travel_params, leases=[self._lease.lease_proto], command_id=nav_to_cmd_id)
            except ResponseError as e:
                self._robot.logger.info("Error while navigating {}".format(e))
                self._call_sit()
                break
            
            time.sleep(0.01)  # Sleep for half a second to allow for command execution.
            self._joint_state() # 로봇의 위치, 회전, 관절 정보를 grab하여 서버에 전송하는 함수
            is_finished = self._check_success(waypoint_id, nav_to_cmd_id) # Waypoint 이동에 대한 결과 반환 함수 호출

    ##
    # @brief waypoint 이동 동작시, 로봇의 주행 속도를 설정하는 함수
    # @param max_linear_vel waypoint 이동 시, 로봇 주행 최대 속도 
    # @param max_rotation_vel waypoint 이동 시, 로봇 회전 최대 속도
    # @param min_linear_vel waypoint 이동 시, 로봇 주행 최소 속도
    # @param min_rotation_vel waypoint 이동 시, 로봇 회전 최소 속도
    # return 로봇 주행 속도에 대한 API 파라미터
    def _set_vel_limit(self, max_linear_vel, max_rotation_vel, min_linear_vel, 
                       min_rotation_vel):
        max_vel_linear = geometry_pb2.Vec2(x=max_linear_vel, y=max_linear_vel)
        min_vel_linear = geometry_pb2.Vec2(x=min_linear_vel, y=min_linear_vel)
 
        max_vel_se2 = geometry_pb2.SE2Velocity(linear=max_vel_linear,
                                           angular=max_rotation_vel)
        min_vel_se2 = geometry_pb2.SE2Velocity(linear=min_vel_linear, 
                                           angular=min_rotation_vel)
 
        vel_limit = geometry_pb2.SE2VelocityLimit(max_vel=max_vel_se2, 
                                            min_vel=min_vel_se2)
        
        return vel_limit

    ## 
    # @brief Waypoint 이동(navigate_to)에 대한 결과 반환 함수
    # @param waypoint_id 이동하는 waypoint id
    # @param command_id None 체크, None이 아닌 경우 지정된 waypoint id로 navigate_to api 함수를 호출함
    # @return 결과 로그 출력 및 소켓 메시지 전송
    def _check_success(self, waypoint_id, command_id=-1):
        """Use a navigation command id to get feedback from the robot and sit when command succeeds."""
        if command_id == -1:
            # No command, so we have not status to check.
            return False
        status = self._graph_nav_client.navigation_feedback(command_id)
        if status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL:
            self._robot.logger.info("Successfully completed the navigation commands, id : " + waypoint_id)
            socket.emit('go_to_response', "Successfully completed the navigation commands, id : " + waypoint_id)
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_LOST:
            self._robot.logger.info("Robot got lost when navigating the route, id : " + waypoint_id)
            socket.emit('go_to_response', "Robot got lost when navigating the route, the robot will now sit down.")
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_STUCK:
            self._robot.logger.info("Robot got stuck when navigating the route, id : " + waypoint_id)
            socket.emit('go_to_response', "Robot got stuck when navigating the route, id : " + waypoint_id)
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_ROBOT_IMPAIRED:
            socket.emit('go_to_response', "Robot is impaired.")
            self._robot.logger.info("Robot is impaired.")
            return True
        else:
            # Navigation command is not complete yet.
            return False
    
    ## 
    # @brief 로봇에게 Autowalk 의 graph, snapshots 정보를 업로드하는 함수
    # @param robot 로봇 객체
    # @param client GraphNav API 클라이언트
    # @param path Autowalk 파일 경로
    # @param disable_alternate_route_finding Autowalk 대체 경로 찾기에 대한 비활성화 boolean
    def upload_graph_and_snapshots(self, robot, client, lease, path, disable_alternate_route_finding):
        # Load the graph from disk.
        graph_filename = os.path.join(path, 'graph')
        robot.logger.info('Loading graph from ' + graph_filename)
        with open(graph_filename, 'rb') as graph_file:
            data = graph_file.read()
            self._current_graph = map_pb2.Graph()
            self._current_graph.ParseFromString(data)
            robot.logger.info('Loaded graph has {} waypoints and {} edges'.format(len(self._current_graph.waypoints), len(self._current_graph.edges)))
            
        if disable_alternate_route_finding:
            for edge in self._current_graph.edges:
                edge.annotations.disable_alternate_route_finding = True

        # Load the waypoint snapshots from disk.
        for waypoint in self._current_graph.waypoints:
            if len(waypoint.snapshot_id) == 0:
                continue
            snapshot_filename = os.path.join(path, 'waypoint_snapshots', waypoint.snapshot_id)
            robot.logger.info('Loading waypoint snapshot from ' + snapshot_filename)

            with open(snapshot_filename, 'rb') as snapshot_file:
                waypoint_snapshot = map_pb2.WaypointSnapshot()
                waypoint_snapshot.ParseFromString(snapshot_file.read())
                self._current_waypoint_snapshots[waypoint_snapshot.id] = waypoint_snapshot

        # Load the edge snapshots from disk.
        for edge in self._current_graph.edges:
            if len(edge.snapshot_id) == 0:
                continue
            snapshot_filename = os.path.join(path, 'edge_snapshots', edge.snapshot_id)
            robot.logger.info('Loading edge snapshot from ' + snapshot_filename)

            with open(snapshot_filename, 'rb') as snapshot_file:
                edge_snapshot = map_pb2.EdgeSnapshot()
                edge_snapshot.ParseFromString(snapshot_file.read())
                self._current_edge_snapshots[edge_snapshot.id] = edge_snapshot

        # Upload the graph to the robot.
        robot.logger.info('Uploading the graph and snapshots to the robot...')
        true_if_empty = not len(self._current_graph.anchoring.anchors)
        response = self._graph_nav_client.upload_graph(lease=self._lease.lease_proto,
                                                       graph=self._current_graph,
                                                       generate_new_anchoring=true_if_empty)
        robot.logger.info('Uploaded graph.')

        # Upload the snapshots to the robot.
        for snapshot_id in response.unknown_waypoint_snapshot_ids:
            waypoint_snapshot = self._current_waypoint_snapshots[snapshot_id]
            client.upload_waypoint_snapshot(waypoint_snapshot=waypoint_snapshot, lease=lease)
            robot.logger.info('Uploaded {}'.format(waypoint_snapshot.id))

        for snapshot_id in response.unknown_edge_snapshot_ids:
            edge_snapshot = self._current_edge_snapshots[snapshot_id]
            client.upload_edge_snapshot(edge_snapshot=edge_snapshot, lease=lease)
            robot.logger.info('Uploaded {}'.format(edge_snapshot.id))
    
        localization_state = self._graph_nav_client.get_localization_state()
        robot.logger.info('localization_state {}'.format(localization_state))
    
    ## 
    # @brief 로봇에게 Autowalk 의 mission 정보를 업로드하는 함수
    # @param robot 로봇 객체
    # @param client Mission API 클라이언트
    # @param filename mission 파일 이름
    # @param lease lease API 클라이언트    
    def upload_mission(self ,robot, client, filename, lease):
        '''Upload the mission to the robot'''
        # Load the mission from disk
        robot.logger.info('Loading mission from ' + filename)

        with open(filename, 'rb') as mission_file:
            data = mission_file.read()
            mission_proto = nodes_pb2.Node()
            mission_proto.ParseFromString(data)
           
        # Upload the mission to the robot
        robot.logger.info('Uploading the mission to the robot...')
        try:
            client.load_mission(mission_proto, leases=[lease])
            robot.logger.info('Uploaded mission to robot.')
        except (ResponseError, RpcError, LeaseBaseError) as err:
            robot.logger.info(err)

    ## 
    # @brief 로봇 lease, estop 서비스에 대한 설정 함수
    # 파이썬 클라이언트 시작시, 로봇 nickname을 전송하여 서버에 파이썬 클라이언트의 접속 여부를 알림
    def start(self):
        """Begin communication with the robot."""
        self._lease = self._lease_client.acquire()
        self._lease_keepalive = LeaseKeepAlive(self._lease_client)
        self._robot_id = self._robot.get_id()

        # server에 robot nickname 전송
        socket.emit('spot_control_client_id', self._robot_id.nickname)

        if self._estop_endpoint is not None:
            # Set this endpoint as the robot's sole estop.
            self._estop_endpoint.force_simple_setup()  

    ##
    # @brief 파이썬 클라이언트 종료 시, 로봇의 lease, estop 서비스에 대한 설정 함수
    def shutdown(self):
        """Release control of robot as gracefully as posssible."""
        LOGGER.info("Shutting down WasdInterface.")
        if self._estop_keepalive:
            # This stops the check-in thread but does not stop the robot.
            self._estop_keepalive.shutdown()
        if self._lease:
            _grpc_or_log("returning lease", lambda: self._lease_client.return_lease(self._lease))
            self._lease = None
    
    ##
    # @brief 서버에 접속한 웹 클라이언트의 아이디를 저장하는 setter 함수
    # @param remote_client_id 서버에 접속한 웹 클라이언트 아이디 (연결 해제 시 "")
    def set_remote_client_id(self, remote_client_id):
        self._remote_client_id = remote_client_id
        self._robot.logger.info("current remote client : " + self._remote_client_id)
    
    @property
    def remote_client_id(self):
        return self._remote_client_id

    @property
    def robot_state(self):
        """Get latest robot state proto."""
        return self._robot_state_task.proto

    @property
    def graph_state(self):
        """Get latest graph state proto."""
        return self._graph_task.proto

    ##
    # @brief 파이썬 클라이언트가 실행되는 동안 반복되는 함수
    # 로봇 상태값을 서버에 반복하여 전송하고, 파이썬 클라이언트 종료시 해당 함수가 종료되면서 로봇의 전원을 끄도록 구현
    def drive(self):
        
        with ExitCheck() as self._exit_check:
            try:
                while not self._exit_check.kill_now:
                    
                    if self._remote_client_id != "":
                        dic = {"battery" : self._battery_str(), "estop": self._estop_str(), "power": self._power_state_str()}
                        # battery 충전 정보, Estop 상태 정보, 모터 Power 정보를 서버에 전송
                        socket.emit("spot_running_state", dic) 

                        battery_data = self._battery_log_format()
                        # 배터리의 ID, 충전 정보, 전압, 전체 셀의 온도를 서버에 전송
                        socket.emit("battery_state", battery_data)
                    try:
                        time.sleep(0.01)
                    except Exception:
                        # On robot command fault, sit down safely before killing the program.
                        self._safe_power_off()
                        time.sleep(2.0)
                        raise

            finally:
                self._robot.logger.info('Exit')

    def _try_grpc(self, desc, thunk):
        try:
            return thunk()
        except (ResponseError, RpcError, LeaseBaseError) as err:
            return None

    def _try_grpc_async(self, desc, thunk):
        def on_future_done(fut):
            try:
                fut.result()
            except (ResponseError, RpcError, LeaseBaseError) as err:
                return None
        future = thunk()
        future.add_done_callback(on_future_done)

    ##
    # @brief 서버로부터 넘겨받은 Autowalk mission 에 대한 API 클라이언트 초기화, API 요청을 담당하는 함수
    # @param data autowalk 파일 이름
    def _call_spot_replay_mission(self, data):
        self._robot.logger.info("replay mission : " + data)
        path = "./autowalk_path/" + data
        mission_file = os.path.join(path, "missions", "autogenerated")
        self._current_walk_name = path
        do_localization = True

        try:
            with bosdyn.client.lease.LeaseKeepAlive(self._lease_client):
                # mission_client, graph_nav_client 초기화
                mission_client, self._graph_nav_client = self.init_graph_client(self._robot, self._lease, mission_file, path, True, True)

                assert not self._robot.is_estopped(), "Robot is estopped. " \
                                                      "Please use an external E-Stop client, " \
                                                       "such as the estop SDK example, to configure E-Stop."        
        
                localization_error = False

                if do_localization:
                    graph = self._graph_nav_client.download_graph()
                    self._robot.logger.info('Localizing robot...')
                    localization = nav_pb2.Localization()

                    # set_localization API 함수 호출
                    # Attempt to localize using any visible fiducial (ArUco)
                    self._graph_nav_client.set_localization(
                        initial_guess_localization=localization, ko_tform_body=None, max_distance=None,
                        max_yaw=None, fiducial_init=graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_NEAREST)

                # Run mission
                if not localization_error:
                        mission_result = self.run_mission(self._robot, mission_client, self._lease_client, True, 10.0, False)
                        if mission_result == "STATUS_SUCCESS":
                            self._robot.logger.info("mission_result : " + mission_result)
                            socket.emit("mission_result", "Success")
                        elif mission_result is "STATUS_PAUSED":
                            self._robot.logger.info("mission_result : " + mission_result)
                            socket.emit("mission_result", "Paused")
        finally:
            self._robot.logger.info("exit_call_spot_replay_mission")

    ## 
    # @brief Autowalk 미션 재생 요청 함수
    # @param robot 로봇 객체
    # @param mission_client Mission API 클라이언트 객체
    # @param lease_client Lease API 클라이언트 객체
    # @param fail_on_question 미션 도중 발생되는 에러에 대한 처리 boolean  
    # @param mission_timeout 미션 replay시 sleep 시간
    # @param disable_alternate_route_finding Autowalk 대체 경로 찾기에 대한 비활성화 boolean
    # @return 미션 replay에 대한 결과값
    def run_mission(self, robot, mission_client, lease_client, fail_on_question, mission_timeout, disable_directed_exploration):
        """Run mission once"""
             
        robot.logger.info('Running mission')

        mission_state = mission_client.get_state()

        while mission_state.status in (mission_pb2.State.STATUS_NONE, mission_pb2.State.STATUS_RUNNING):
            # We optionally fail if any questions are triggered. This often indicates a problem in
            # Autowalk missions.
            if mission_state.questions and fail_on_question:
                robot.logger.info('Mission failed by triggering operator question: {}'.format(
                    mission_state.questions))
                socket.emit('mission_result', 'Failed') # 미션 결과 (실패)에 대한 소켓 메시지
                return False

            body_lease = lease_client.lease_wallet.advance()
            local_pause_time = time.time() + mission_timeout

            # play하는 미션에 대한 설정 (로봇 속도, 대체 경로 찾기)
            play_settings = mission_pb2.PlaySettings(disable_directed_exploration=disable_directed_exploration)
            play_settings.velocity_limit.max_vel.linear.x = 0.5
            play_settings.velocity_limit.max_vel.linear.y = 0.5
            play_settings.velocity_limit.min_vel.linear.x = -0.2
            play_settings.velocity_limit.min_vel.linear.y = -0.2
        
            mission_client.play_mission(local_pause_time, [body_lease], play_settings) # 업로드된 mission 에 대한 Play API 함수 호출 
            time.sleep(0.01)
        
            mission_state = mission_client.get_state()

            self._joint_state() # 미션 실행 동안, 로봇의 위치, 회전, 관절 정보를 grab하여 서버에 전송하는 함수
        
        robot.logger.info('Mission status = ' + mission_state.Status.Name(mission_state.status))
        return mission_state.Status.Name(mission_state.status)

    ## 
    # @brief Autowalk 미션 또는 Waypoint 이동 명령에 사용되는 GraphNav API 클라이언트, Misson API 클라이언트 초기화
    # @param robot 로봇 객체
    # @param lease lease API 객체
    # @param mission_file Autowalk mission 파일 이름
    # @param map_directory Autowalk 경로 명
    # @param do_map_load Autowalk 맵 업로드에 대한 boolean
    # @param disable_alternate_route_finding Autowalk 대체 경로 찾기에 대한 비활성화 boolean
    # @return 초기화된 mission_client, graph_nav_client
    def init_graph_client(self, robot, lease, mission_file, map_directory, do_map_load, disable_alternate_route_finding):
        
        if not os.path.isfile(mission_file):
            robot.logger.fatal('Unable to find mission file: {}.'.format(mission_file))
            sys.exit(1)
        
        if do_map_load:
            if not os.path.isdir(map_directory):
                robot.logger.fatal('Unable to find map directory: {}.'.format(map_directory))
                sys.exit(1)        

            robot.logger.info('Creating graph-nav client...')
            self._graph_nav_client = robot.ensure_client(bosdyn.client.graph_nav.GraphNavClient.default_service_name) # GraphNav API 클라이언트 초기화
            robot.logger.info('Clearing graph-nav state...')
            self._graph_nav_client.clear_graph() # 로봇에게 업로드되어 있는 Autowalk graph 삭제 
            self.upload_graph_and_snapshots(robot, self._graph_nav_client, lease.lease_proto, map_directory,
                                   disable_alternate_route_finding) # Parameter로 입력된 Autowalk 의 graph, snapshot 업로드

        robot.logger.info('Creating mission client...')
        mission_client = self._mission_robot.ensure_client(bosdyn.mission.client.MissionClient.default_service_name) # Mission API 클라이언트 초기화
        self.upload_mission(robot, mission_client, mission_file, lease) # Parameter로 입력된 Autowalk 의 Mission 파일 업로드

        return mission_client, self._graph_nav_client 

    ##
    # @brief 로봇 Estop 제어에 대한 API command를 호출 요청하는 함수
    def _call_estop(self):
        self._toggle_estop()

    ##
    # @brief 로봇 모터 전원 제어(On)에 대한 API command를 호출 요청하는 함수
    def _call_power_on(self):
        self._toggle_power()

    ##
    # @brief 로봇 모터 전원 제어(Off)에 대한 API command를 호출 요청하는 함수
    def _call_safe_power_off(self):
        self._try_grpc("powering-off", self._safe_power_off)

    ##
    # @brief 로봇 stand 명령에 대한 API command를 호출 요청하는 함수
    def _call_stand(self):
        self._stand()

    ##
    # @brief 로봇 sit 명령에 대한 API command를 호출 요청하는 함수
    def _call_sit(self):
        self._sit()
    
    ##
    # @brief 로봇 이동 제어에 대한 API command를 호출 요청하는 함수
    def _call_custom_drive(self, v_x, v_y, v_rot):
        self._velocity_cmd_helper('custom_drive', v_x = v_x, v_y = v_y, v_rot = v_rot)
    
    ##
    # @brief 로봇 자세 제어에 대한 API command를 호출 요청하는 함수
    def _call_custom_pose(self, yaw, roll, pitch):
        self._robot.logger.info('Pose control yaw={} roll={} pitch={}'.format(yaw, roll, pitch))
        self._pose_cmd_helper('custom_pose', yaw = yaw, roll= roll, pitch = pitch)

    ##
    # @brief 현재 로봇의 Estop 상태를 파악하여, Estop에 대한 제어 API command를 호출하는 함수 
    def _toggle_estop(self):
        """toggle estop on/off. Initial state is ON"""

        self._robot.logger.info('command robot to estop')

        if self._estop_client is not None and self._estop_endpoint is not None:
            if not self._estop_keepalive:
                self._estop_keepalive = EstopKeepAlive(self._estop_endpoint)
            else:
                self._try_grpc("stopping estop", self._estop_keepalive.stop)
                self._estop_keepalive.shutdown()
                self._estop_keepalive = None

    ##
    # @brief 로봇 제어에 관한 API command를 호출하는 함수
    def _start_robot_command(self, desc, command_proto, end_time_secs=None):

        def _start_command():
            self._robot_command_client.robot_command(lease=None, command=command_proto,
                                                     end_time_secs=end_time_secs)

        self._try_grpc(desc, _start_command)

    ##
    # @brief 로봇의 sit 명령 API command를 호출하는 함수
    def _sit(self):
        self._robot.logger.info('command robot to sit')
        self._start_robot_command('sit', RobotCommandBuilder.synchro_sit_command())

    ##
    # @brief 로봇의 stand 명령 API command를 호출하는 함수
    def _stand(self):
        self._robot.logger.info('command robot to stand')
        self._start_robot_command('stand', RobotCommandBuilder.synchro_stand_command())

    ##
    # @brief 로봇의 자세 제어에 대한 API command를 호출하는 함수
    def _pose_cmd_helper(self, desc='', yaw=0.0, roll=0.0, pitch=0.0):
        self._start_robot_command(desc, RobotCommandBuilder.synchro_stand_command(footprint_R_body=bosdyn.geometry.EulerZXY(yaw=yaw, roll=roll, pitch=pitch)))

    ##
    # @brief 로봇의 이동 제어, 회전 제어에 대한 API command를 호출하는 함수
    def _velocity_cmd_helper(self, desc='', v_x=0.0, v_y=0.0, v_rot=0.0):
        self._start_robot_command(desc,
                                  RobotCommandBuilder.synchro_velocity_command(
                                      v_x=v_x, v_y=v_y, v_rot=v_rot),
                                  end_time_secs=time.time() + VELOCITY_CMD_DURATION)

    
    ## 
    # @brief 현재 로봇의 모터 상태를 파악하여, Power에 대한 On/Off 제어 API command를 호출하는 함수 
    def _toggle_power(self):
        power_state = self._power_state()
        if power_state is None:
            print('Could not toggle power because power state is unknown')
            return

        if power_state == robot_state_proto.PowerState.STATE_OFF:
            self._robot.logger.info('command robot to power on')
            self._try_grpc_async("powering-on", self._request_power_on)
        else:
            self._robot.logger.info('command robot to power off')
            self._try_grpc("powering-off", self._safe_power_off)

    ## 
    # @brief 로봇 모터 Power 상태를 ON 하는 API command를 호출하는 함수
    # @return 로봇 모터 Power를 ON하는 API command 호출
    def _request_power_on(self):
        request = PowerServiceProto.PowerCommandRequest.REQUEST_ON
        return self._power_client.power_command_async(request)

    ## 
    # @brief 로봇 모터 Power 상태를 OFF 하는 API command를 호출하는 함수
    def _safe_power_off(self):
        self._start_robot_command('safe_power_off', RobotCommandBuilder.safe_power_off_command())

    def _power_state(self):
        state = self.robot_state
        if not state:
            return None
        return state.power_state.motor_power_state

    ## 
    # @brief 로봇 모터의 Power 상태를 반환하는 함수, running_state 메시지에서 power.value로 사용
    # @return 로봇 모터 Power 상태에 대한 문자열
    def _power_state_str(self):
        power_state = self._power_state()
        if power_state is None:
            return ''
        state_str = robot_state_proto.PowerState.MotorPowerState.Name(power_state)
        return format(state_str[6:])  # get rid of STATE_ prefix
    
    ## 
    # @brief 로봇 Estop 상태를 반환하는 함수, running_state 메시지에서 estop.value로 사용
    # @return Estop 상태에 대한 문자열
    def _estop_str(self):
        if not self._estop_client:
            thread_status = 'NOT ESTOP'
        else:
            thread_status = 'RUNNING' if self._estop_keepalive else 'STOPPED'
        estop_status = '??'
        state = self.robot_state
        if state: 
            for estop_state in state.estop_states:
                if estop_state.type == estop_state.TYPE_SOFTWARE:
                    estop_status = estop_state.State.Name(estop_state.state)[6:]  # s/STATE_//
                    break 
        return estop_status

    ## 
    # @brief 로봇 배터리 충전 상태를 반환하는 함수, running_state 메시지에서 battery.value 로 사용
    # @return 로봇 배터리 충전 상태 값
    def _battery_str(self):
        if not self.robot_state:
            return ''
        battery_state = self.robot_state.battery_states[0]
        return battery_state.charge_percentage.value

    ## 
    # @brief 로봇 배터리 상태를 반환하는 함수
    # @return 로봇 배터리 상태에 대한 json 문자열 (id, 충전상태, 전압, 배터리셀 온도)
    def _battery_log_format(self):
        if not self.robot_state:
            return ''
        
        battery_state = self.robot_state.battery_states[0]
        battery_data = {'identifier' : battery_state.identifier, 'charge_percentage' : battery_state.charge_percentage.value , 'voltage' : battery_state.voltage.value , 'temperatures' : list(battery_state.temperatures)}
        battery_json = json.dumps(battery_data)
        
        return battery_json

# @author : Chulhee Lee
# @brief : Interface class to manipulate all features of spot cam
# @param : object ('Spot Cam Client' robot object of spot sdk)
class SpotCamControlInterface(object):

    def __init__(self, robot):
        """Interface class to manipulate all features of spot cam"""

        # 'Spot Cam Client' robot object of spot sdk
        self._robot = robot
        setup_logging(False)

        # A dict of classes for each function for spot cam operation
        self._command_dic = {}

        # Registers sub command argument required for spot cam operation
        self._parser = argparse.ArgumentParser(prog='bosdyn.api.spot_cam', description=main.__doc__)
        self._subparsers = self._parser.add_subparsers(title='commands', dest='command')
        self._subparsers.required = True

        # Register class for each function that can operate spot cam.(with. sub command)
        self._register_spot_cam_commands(self._subparsers, self._command_dic)

        # Parsing to the command namespace for spot cam operation using the command parameter received from the Robot Server.
        self._command_options = argparse.Namespace()

        # Parsing robot information commonly required for spot cam operation commands
        setattr(self._command_options, 'username', 'admin')
        setattr(self._command_options, 'password', 'uhkqr0sv0ko1')
        setattr(self._command_options, 'hostname', '192.168.80.3')
        setattr(self._command_options, 'verbose', False)

    def _register_spot_cam_commands(self, subparsers, command_dict):
        """Register function-specific functions and subcommand arguments for manipulating the spot cam"""

        COMMANDS = [
            AudioCommands,
            CompositorCommands,
            HealthCommands,
            LightingCommands,
            MediaLogCommands,
            NetworkCommands,
            PowerCommands,
            PtzCommands,
            StreamQualityCommands,
            UtilityCommands,
            VersionCommands,
            WebRTCCommands
        ]
        
        for register_command in COMMANDS:
            register_command(subparsers, command_dict)

    def _initialize_command(self, command_dict):
        """Initialize the command for new spot cam manipulations"""

        for key, value in command_dict.items():
            self._command_options.__delattr__(key)

    def _spot_cam_operate(self, command_dict):
        """Use the command to operate the spot cam."""

        # Perform spot cam operation command using information received from Robot Server
        # The result of the spot cam operation is returned.
        operated_spot_command = self._command_dic[self._command_options.command].run(self._robot, self._command_options)

        # Classification of spot cam operation functions that require data transmission to Robot Server
        # Send the name property data of the spot cam image to the Robot Server
        if 'list_logpoints_name' in command_dict.values():
            socket.emit('spot_cam_data_receive_uuid_list', operated_spot_command)
        
        # Transmits spot cam image data to Robot Server in byte array format
        if 'retrieve' in command_dict.values():
            retrieve_img = base64.b64encode(operated_spot_command)
            img_str = retrieve_img.decode("utf-8")
            socket.emit('spot_cam_data_receive_image', img_str)

        return operated_spot_command
        
    def _set_command_options(self, command_dict):
        """The parameters received from the robot server are divided into keys and values ​​and stored in the command dictionary."""

        for key, value in command_dict.items():        
            setattr(self._command_options, key, value)

        # Activate the spot cam with the setup command.
        return self._spot_cam_operate(command_dict)

def main(args=None):
    """Command-line interface."""
    urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

    # Create SPOT_PTZ robot object
    spot_cam_sdk = create_standard_sdk('Spot CAM Client')
    spot_cam.register_all_service_clients(spot_cam_sdk)
    spot_cam_robot = spot_cam_sdk.create_robot("192.168.80.3")
    spot_cam_robot.authenticate("admin", "uhkqr0sv0ko1")

    # Create SPOT CAM Control Interface class
    global cam_command_interface
    cam_command_interface = SpotCamControlInterface(spot_cam_robot)

    # Connect to Socket
    socket.connect('https://192.168.6.3:3458')
    
    wasd_sdk = create_standard_sdk('WASDClient')
    robot = wasd_sdk.create_robot("192.168.80.3")
    robot.authenticate("admin", "uhkqr0sv0ko1")

    global wasd_interface
    wasd_interface = WasdInterface(robot)

    try:
        wasd_interface.start()
    except (ResponseError, RpcError) as err:
        LOGGER.error("Failed to initialize robot communication: %s" % err)
        return False
    
    try:
        wasd_interface.drive()
    except Exception as e:
        LOGGER.error("WASD has thrown an error: %s" % repr(e))
    
    finally:
        socket.disconnect()
        wasd_interface.shutdown()

    return True

if __name__ == "__main__":
    if not main():
        os._exit(1)
    os._exit(0)
