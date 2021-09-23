# Copyright (c) 2021 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""WASD driving of robot."""

from __future__ import print_function
import logging
import signal
import sys
import threading
import time
import math
import io
import os

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
import base64
#add spot_cam_function

from PIL import Image, ImageEnhance

from bosdyn.api import geometry_pb2
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
from bosdyn.client.time_sync import TimeSyncError
import bosdyn.client.util
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
from bosdyn.util import duration_str, format_metric, secs_to_hms
import numpy as np

import socketio

socket = socketio.Client(ssl_verify=False)

@socket.event
def connect():
    print('spot socket server connected')
    socket.emit('spot_cam_init_position', get_spot_position())

@socket.event
def disconnect():
    print('disconnect from spot socket server')

@socket.event
def remote_client_connect(data):
    print("connect remote")
    wasd_interface.set_remote_client_id(data)
    socket.emit('spot_cam_init_position', get_spot_position())

@socket.event
def remote_client_disconnect(data):
    print("disconnect remote")
    remote_client_id = ""
    wasd_interface.set_remote_client_id(remote_client_id)
    wasd_interface._call_safe_power_off()

@socket.event
def remote_client_not_yet(data):
    print("not yet")
    remote_client_id = ""
    wasd_interface.set_remote_client_id(remote_client_id)

@socket.event
def spot_control_estop():
    print('toggle')
    wasd_interface._call_estop()

@socket.event
def spot_control_power_on():
    print("power on")
    wasd_interface._call_power_on()

@socket.event
def spot_control_power_off():
    print("power off")
    wasd_interface._call_safe_power_off()

@socket.event
def spot_control_sit():
    print("sit")
    wasd_interface._call_sit()

@socket.event
def spot_control_stand():
    print("stand")
    wasd_interface._call_stand()

@socket.event
def spot_control_cmd(data):
    wasd_interface._call_custom_drive(data[0], data[1], data[2])

@socket.event
def spot_cam_control(data):
    print('cam_control_receive!!')
    ptz_interface._set_command_options(data)
    ptz_interface._initialize_namespace(data)

LOGGER = logging.getLogger()

VELOCITY_BASE_SPEED = 0.5  # m/s
VELOCITY_BASE_ANGULAR = 0.8  # rad/sec
VELOCITY_CMD_DURATION = 0.8  # seconds
COMMAND_INPUT_RATE = 0.1

def _grpc_or_log(desc, thunk):
    try:
        return thunk()
    except (ResponseError, RpcError) as err:
        LOGGER.error("Failed %s: %s" % (desc, err))

def get_spot_position():
    print('get_spot_position')
    
    #spot ptz cam position
    get_spot_cam_posotion = ptz_interface._set_command_options(
                            {'command' : 'ptz', 'ptz_command' : 'get_position', 'ptz_name' : 'mech'})
    
    init_spot_cam_position = {}

    init_spot_cam_position['pan'] = get_spot_cam_posotion.pan.value
    init_spot_cam_position['tilt'] = get_spot_cam_posotion.tilt.value
    init_spot_cam_position['zoom'] = get_spot_cam_posotion.zoom.value
    
    return init_spot_cam_position 

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


class AsyncRobotState(AsyncPeriodicQuery):
    """Grab robot state."""

    def __init__(self, robot_state_client):
        super(AsyncRobotState, self).__init__("robot_state", robot_state_client, LOGGER,
                                              period_sec=0.2)
    def _start_query(self):
        return self._client.get_robot_state_async()


class WasdInterface(object):
    """A curses interface for driving the robot."""

    def __init__(self, robot):
        self._robot = robot
        # Create clients -- do not use the for communication yet.
        self._lease_client = robot.ensure_client(LeaseClient.default_service_name)
        try:
            self._estop_client = self._robot.ensure_client(EstopClient.default_service_name)
            self._estop_endpoint = EstopEndpoint(self._estop_client, 'GNClient', 9.0)
        except:
            # Not the estop.
            self._estop_client = None
            self._estop_endpoint = None


        self._power_client = robot.ensure_client(PowerClient.default_service_name)
        self._robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
        self._robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)


        #self._image_publishing = AsyncRobotImage(robot)

        self._robot_state_task = AsyncRobotState(self._robot_state_client)
        self._async_tasks = AsyncTasks([self._robot_state_task])
        self._lock = threading.Lock()

        self._locked_messages = ['', '', '']  # string: displayed message for user
        self._estop_keepalive = None
        self._exit_check = None

        # Stuff that is set in start()
        self._robot_id = None
        self._lease = None
        self._lease_keepalive = None
        

    def start(self):
        """Begin communication with the robot."""

        self._lease = self._lease_client.acquire()
        # Construct our lease keep-alive object, which begins RetainLease calls in a thread.
        self._lease_keepalive = LeaseKeepAlive(self._lease_client)

        self._robot_id = self._robot.get_id()

        # server에 robot nickname 전송
        socket.emit('spot_control_client_id', self._robot_id.nickname)

        if self._estop_endpoint is not None:
            self._estop_endpoint.force_simple_setup()  # Set this endpoint as the robot's sole estop.

    def shutdown(self):
        """Release control of robot as gracefully as posssible."""
        LOGGER.info("Shutting down WasdInterface.")
        if self._estop_keepalive:
            # This stops the check-in thread but does not stop the robot.
            self._estop_keepalive.shutdown()
        if self._lease:
            _grpc_or_log("returning lease", lambda: self._lease_client.return_lease(self._lease))
            self._lease = None

    def flush_and_estop_buffer(self, stdscr):
        """Manually flush the curses input buffer but trigger any estop requests (space)"""
        key = ''
        while key != -1:
            key = stdscr.getch()
            if key == ord(' '):
                self._toggle_estop()

    # custom function : wasd_interface object에 remote client id 등록
    def set_remote_client_id(self, remote_client_id):
        self._remote_client_id = remote_client_id

    @property
    def robot_state(self):
        """Get latest robot state proto."""
        return self._robot_state_task.proto

    def drive(self):

        """User interface to control the robot via the passed-in curses screen interface object."""
        image_client = self._robot.ensure_client(ImageClient.default_service_name)

        # for 문 돌려서 비쥬얼 이미지만 append 해도 가능함, Depth 까지 포함되서 반복하기 싫음
        camera_image_list = []
        camera_image_list.append("frontleft_fisheye_image")
        camera_image_list.append("frontright_fisheye_image")
        camera_image_list.append("left_fisheye_image")
        camera_image_list.append("right_fisheye_image")
        camera_image_list.append("back_fisheye_image")    
    
        with ExitCheck() as self._exit_check:
            try:
                while not self._exit_check.kill_now:
                                        
                    image_responses = image_client.get_image_from_sources(camera_image_list)
    
                    for image in image_responses:
                            test_image = io.BytesIO(image.shot.image.data)
                            img_str = base64.b64encode(test_image.getvalue())
                            test_str = img_str.decode("utf-8")
                            socket.emit(str(image.source.name), test_str)

                    # 이미지 보낸 이후에 async Update
                    self._async_tasks.update()

                    if self._remote_client_id != "":
                        dic = {"battery":self._battery_str(), "estop": self._estop_str(), "power": self._power_state_str()}
                        socket.emit("spot_running_state", dic)
            
                    else:
                        pass

                    try:
                        # Do not queue up commands on client
                        #time.sleep(COMMAND_INPUT_RATE)
                        time.sleep(0.01)

                        """"""
                    except Exception:
                        # On robot command fault, sit down safely before killing the program.
                        self._safe_power_off()
                        time.sleep(2.0)
                        raise

            finally:
                pass

    def add_message(self, msg_text):
        """Display the given message string to the user in the curses interface."""
        socket.emit('spot_error_message', msg_text)

        # with self._lock:
            # self._locked_messages = [msg_text] + self._locked_messages[:-1]

    def _try_grpc(self, desc, thunk):
        try:
            return thunk()
        except (ResponseError, RpcError, LeaseBaseError) as err:
            self.add_message("Failed {}: {}".format(desc, err))
            return None

    def _try_grpc_async(self, desc, thunk):
        def on_future_done(fut):
            try:
                fut.result()
            except (ResponseError, RpcError, LeaseBaseError) as err:
                self.add_message("Failed {}: {}".format(desc, err))
                return None
        future = thunk()
        future.add_done_callback(on_future_done)

    def _quit_program(self):
        self._sit()
        if self._exit_check is not None:
            self._exit_check.request_exit()

    def _toggle_time_sync(self):
        if self._robot.time_sync.stopped:
            self._robot.start_time_sync()
        else:
            self._robot.time_sync.stop()

    def _call_estop(self):
        self._toggle_estop()

    def _call_safe_power_off(self):
        self._try_grpc("powering-off", self._safe_power_off)
    
    def _call_power_on(self):
        self._toggle_power()

    def _call_stand(self):
        self._stand()

    def _call_sit(self):
        self._sit()
    
    def _call_battery_str(self):
        return self._battery_str()

    def _call_custom_drive(self, data0, data1, data2):
        self._velocity_cmd_helper('custom_drive_command', v_x=data0, v_y=data1, v_rot=data2)

    def _toggle_estop(self):
        """toggle estop on/off. Initial state is ON"""
        if self._estop_client is not None and self._estop_endpoint is not None:
            if not self._estop_keepalive:
                self._estop_keepalive = EstopKeepAlive(self._estop_endpoint)
            else:
                self._try_grpc("stopping estop", self._estop_keepalive.stop)
                self._estop_keepalive.shutdown()
                self._estop_keepalive = None

    def _toggle_lease(self):
        """toggle lease acquisition. Initial state is acquired"""
        if self._lease_client is not None:
            if self._lease_keepalive is None:
                self._lease = self._lease_client.acquire()
                self._lease_keepalive = LeaseKeepAlive(self._lease_client)
            else:
                self._lease_client.return_lease(self._lease)
                self._lease_keepalive.shutdown()
                self._lease_keepalive = None

    def _start_robot_command(self, desc, command_proto, end_time_secs=None):

        def _start_command():
            self._robot_command_client.robot_command(lease=None, command=command_proto,
                                                     end_time_secs=end_time_secs)

        self._try_grpc(desc, _start_command)

    def _self_right(self):
        self._start_robot_command('self_right', RobotCommandBuilder.selfright_command())

    def _sit(self):
        self._start_robot_command('sit', RobotCommandBuilder.synchro_sit_command())

    def _stand(self):
        self._start_robot_command('stand', RobotCommandBuilder.synchro_stand_command())

    def _move_forward(self):
        self._velocity_cmd_helper('move_forward', v_x=VELOCITY_BASE_SPEED)

    def _move_backward(self):
        self._velocity_cmd_helper('move_backward', v_x=-VELOCITY_BASE_SPEED)

    def _strafe_left(self):
        self._velocity_cmd_helper('strafe_left', v_y=VELOCITY_BASE_SPEED)

    def _strafe_right(self):
        self._velocity_cmd_helper('strafe_right', v_y=-VELOCITY_BASE_SPEED)

    def _turn_left(self):
        self._velocity_cmd_helper('turn_left', v_rot=VELOCITY_BASE_ANGULAR)

    def _turn_right(self):
        self._velocity_cmd_helper('turn_right', v_rot=-VELOCITY_BASE_ANGULAR)

    def _velocity_cmd_helper(self, desc='', v_x=0.0, v_y=0.0, v_rot=0.0):
        self._start_robot_command(desc,
                                  RobotCommandBuilder.synchro_velocity_command(
                                      v_x=v_x, v_y=v_y, v_rot=v_rot),
                                  end_time_secs=time.time() + VELOCITY_CMD_DURATION)

    def _toggle_power(self):
        power_state = self._power_state()
        if power_state is None:
            self.add_message('Could not toggle power because power state is unknown')
            return

        if power_state == robot_state_proto.PowerState.STATE_OFF:
            self._try_grpc_async("powering-on", self._request_power_on)
        else:
            self._try_grpc("powering-off", self._safe_power_off)

    def _request_power_on(self):
        request = PowerServiceProto.PowerCommandRequest.REQUEST_ON
        return self._power_client.power_command_async(request)

    def _safe_power_off(self):
        self._start_robot_command('safe_power_off', RobotCommandBuilder.safe_power_off_command())

    def _power_state(self):
        state = self.robot_state
        if not state:
            return None
        return state.power_state.motor_power_state

    def _lease_str(self, lease_keep_alive):
        alive = '??'
        lease = '??'
        if lease_keep_alive is None:
            alive = 'STOPPED'
            lease = 'RETURNED'
        else:
            if self._lease:
                lease = '{}:{}'.format(self._lease.lease_proto.resource,
                                       self._lease.lease_proto.sequence)
            else:
                lease = '...'
            if lease_keep_alive.is_alive():
                alive = 'RUNNING'
            else:
                alive = 'STOPPED'
        return 'Lease {} THREAD:{}'.format(lease, alive)

    def _power_state_str(self):
        power_state = self._power_state()
        if power_state is None:
            return ''
        state_str = robot_state_proto.PowerState.MotorPowerState.Name(power_state)
        # return 'Power: {}'.format(state_str[6:])  # get rid of STATE_ prefix
        return format(state_str[6:])  # get rid of STATE_ prefix

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
        # return 'Estop {} (thread: {})'.format(estop_status, thread_status)
        return estop_status

    def _time_sync_str(self):
        if not self._robot.time_sync:
            return 'Time sync: (none)'
        if self._robot.time_sync.stopped:
            status = 'STOPPED'
            exception = self._robot.time_sync.thread_exception
            if exception:
                status = '{} Exception: {}'.format(status, exception)
        else:
            status = 'RUNNING'
        try:
            skew = self._robot.time_sync.get_robot_clock_skew()
            if skew:
                skew_str = 'offset={}'.format(duration_str(skew))
            else:
                skew_str = "(Skew undetermined)"
        except (TimeSyncError, RpcError) as err:
            skew_str = '({})'.format(err)
        return 'Time sync: {} {}'.format(status, skew_str)

    def _battery_str(self):
        if not self.robot_state:
            return ''
        battery_state = self.robot_state.battery_states[0]
        status = battery_state.Status.Name(battery_state.status)
        status = status[7:]  # get rid of STATUS_ prefix
        if battery_state.charge_percentage.value:
            bar_len = int(battery_state.charge_percentage.value) // 10
            bat_bar = '|{}{}|'.format('=' * bar_len, ' ' * (10 - bar_len))
        else:
            bat_bar = ''
        time_left = ''
        if battery_state.estimated_runtime:
            time_left = ' ({})'.format(secs_to_hms(battery_state.estimated_runtime.seconds))
        #return 'Battery: {}{}{}'.format(status, bat_bar, time_left)
        return battery_state.charge_percentage.value

class ptzInterface(object):
    def __init__(self, robot):
        self._robot = robot
        self._command_dick = {}
        self._command_options = argparse.Namespace()
        setup_logging(False)

        self._parser = argparse.ArgumentParser(prog='bosdyn.api.spot_cam', description=main.__doc__)
        self._subparsers = self._parser.add_subparsers(title='commands', dest='command')
        self._subparsers.required = True

        self._register_all_commands(self._subparsers, self._command_dick)

        setattr(self._command_options, 'username', 'admin')
        setattr(self._command_options, 'password', 'uhkqr0sv0ko1')
        setattr(self._command_options, 'hostname', '192.168.80.3')
        setattr(self._command_options, 'verbose', False)


    def _register_all_commands(self, subparsers, command_dict):
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
            # print(type(register_command))
            register_command(subparsers, command_dict)

    def _initialize_namespace(self, command_dict):

        for key, value in command_dict.items():        
            self._command_options.__delattr__(key)

    def _call_command(self, command_dict):
        print('_call_command_innnnnn')
        print(self._command_options)
        call_command = self._command_dick[self._command_options.command].run(self._robot, self._command_options)

        return call_command

    def _set_command_options(self, command_dict):

        for key, value in command_dict.items():        
            setattr(self._command_options, key, value)

        return self._call_command(command_dict)

def main(args=None):
    """Command-line interface."""
    
    # socket connection
    # socket.connect('https://localhost:3458')
    socket.connect('https://192.168.6.3:3458')
    
    # Create WASD robot object & Interface class.
    wasd_sdk = create_standard_sdk('WASDClient')
    robot = wasd_sdk.create_robot("192.168.80.3")
    robot.authenticate("admin", "uhkqr0sv0ko1")

    global wasd_interface
    wasd_interface = WasdInterface(robot)

    # Create SPOT_PTZ robot object & Interface class
    ptz_sdk = create_standard_sdk('Spot CAM Client')
    spot_cam.register_all_service_clients(ptz_sdk)
    spot_cam_robot = ptz_sdk.create_robot("192.168.80.3")
    spot_cam_robot.authenticate("admin", "uhkqr0sv0ko1")

    global ptz_interface
    ptz_interface = ptzInterface(spot_cam_robot)

    try:
        wasd_interface.start()
    except (ResponseError, RpcError) as err:
        LOGGER.error("Failed to initialize robot communication: %s" % err)
        return False
    
    try:
        # Run wasd interface in curses mode, then restore terminal config.
        wasd_interface.drive()
    except Exception as e:
        LOGGER.error("WASD has thrown an error: %s" % repr(e))
    
    finally:
        # Restore stream handler after curses mode.
        # Do any final cleanup steps.
        socket.disconnect()
        wasd_interface.shutdown()

    return True


if __name__ == "__main__":
    if not main():
        os._exit(1)
    os._exit(0)