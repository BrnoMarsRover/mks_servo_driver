import rclpy
import rclpy.clock
import rclpy.executors
import rclpy.logging
from rclpy.node import Node
import asyncio
import threading
import can
from enum import Enum
from math import pi
from typing import List, Dict, Tuple
import traceback
import time
import math
from sensor_msgs.msg import JointState


from .mks_enums import MksCommands, Direction, WorkMode, EndStopLevel, Enable, GoHomeResult, RunMotorResult, MotorStatus, SuccessStatus
from manipulator_servo_driver_interfaces.srv import ChangeMode, ResetAxis, HomeAxis

# Globální slovník odpovědí CANu. Sdílený mezi všechny serva
#              (can_id, cmd_code) -> can response future
can_resp_dict: Dict[Tuple[int,int], asyncio.Future] = {}

async def free_loop_resources():
    while True:
        await asyncio.sleep(0)

def can_parse_response(msg:can.Message):
    servo_id = msg.arbitration_id
    cmd_code = msg.data[0]
    #print(f"Parsing message from servo id {servo_id} with code {cmd_code}")
    return (servo_id, cmd_code), msg.data    # jako klíč ve slovníku je nutné uchovávat ID serva i command


async def can_process_response(msg:can.Message): #vrací bytes
    global can_resp_dict
    key, data = can_parse_response(msg)
    #print(f"RX CAN: {(key, data)}")

    # Vytáhne danou future ze slovníku
    future = can_resp_dict.pop(key, None)

    if future:  # daná future je ve slovníku
        if future.cancelled():
            #print(f"Future {key} bylo zrušeno (doběhl timeout)")
            return

        if future.done():
            #print(f"Future {key} bylo dokončeno")
            return
        
        try:
            future.set_result(data) # naplnění future promise
        except Exception as e:
            print(f"Chyba future.set_result {key}: exc: {e}") 
    else:        
        noop = 1
        #print("Obdržená zprává není ve slovníku očekávaných odpovědí")


# V nekonečné smyčce poslouchá CAN a responses ukládá do slovníku can_resp_dict, ze kterého si je vyzvedávají jednotlivé funkce pomocí futures
async def can_listener(can_reader: can.AsyncBufferedReader):
    # can_reader (AsyncBufferedReader) přijímá asynchronně zprávy a ukládá je FIFO fronty a potom vydává JEDNU PO DRUHÉ - Blokující čekání
    while True:
        msg = await can_reader.get_message() #vytáhne nejstarší zprávu z FIFO fronty
        #asyncio.create_task(can_raw_queue.put(msg)) # a okamžitě ji vloží do async fronty
        #continue
        #can_reader.
        #can_reader (AsyncBufferedReader) přijímá asynchronně zprávy a ukládá je FIFO fronty 
        # a potom vydává JEDNU PO DRUHÉ - Když se jedna zpráva zpozdí tak zablokuje všechny ostatní
        #await can_process_response(msg)                                    # <- funguje, ale až po timeout, bez timeout error 
        asyncio.create_task(can_process_response(msg))

class JointDriver():
    def __init__(self, asyncio_loop, can_id, can_bus, motor_subdivisions, homing_speed, homing_dir, max_speed, adaptive_current, zero_offset_rad, joint_lim_rad, gear_ratio, invert_direction):
        self.DEFAULT_RESPONSE_LENGTH = 3
        self.MAX_HOMING_TIME = 30
        self.can_id = can_id
        self.can_bus: can.BusABC = can_bus
        self.adaptive_current = adaptive_current
        self.homing_speed = homing_speed
        self.homing_dir = homing_dir
        self.max_speed = max_speed
        self.zero_offset_rad = zero_offset_rad
        self.sw_limits_rad = joint_lim_rad
        self.speed = 0.0
        self.gear_ratio = gear_ratio
        self.dir = -1 if invert_direction == True else 1
        self.motor_subdivisions = motor_subdivisions
        self.asyncio_loop = asyncio_loop
        self.homing_status = GoHomeResult.Unkown
    
    @staticmethod
    def validate_direction(direction):
        if direction not in [Direction.CW, Direction.CCW]:
            raise Exception("Direction must be CW or CCW")
        
    @staticmethod
    def validate_speed(speed):
        if speed < 0 or speed > 3000:
            raise Exception(f"Speed mks_rpm must be between 0 and {3000}")
        
    @staticmethod
    def validate_acceleration(acceleration):
        if acceleration < 0 or acceleration > 255:
            raise Exception(f"Acceleration mks_value must be between 0 and {255}")
    @staticmethod
    def validate_current(current):
        if current < 0 or current > 5200:
            raise Exception("Current is outside the valid range from 0 to 5200")
    
    @staticmethod
    def build_can_msg(cmd_code, data, can_id):
        

        msg = [cmd_code] + data
        crc = (can_id + sum(msg)) & 0xFF
        can_msg = can.Message(
            arbitration_id=can_id,
            data=bytearray(msg) + bytes([crc]),
            is_extended_id=False)
        
        return can_msg

    # Odeslání zprávy přes CAN a uložení response future promise do slovníku
    async def can_query(self, expected_response_length: int, cmd_code: int|Enum, data: list[int] = []):    
        global can_resp_dict

        if isinstance(cmd_code, Enum):
            cmd_code = cmd_code.value

        if isinstance(data, int):
            data = [data]

        can_msg = self.build_can_msg(cmd_code,data,self.can_id)

        #key, data = can_parse_response(can_msg)
        #print(f"CAN TX: {key} -> {data}")
        #print(f"CAN TX raw: {can_msg}")

        # Vytvoření future promise a vložení do globálního dict pod klíčem (can_id, cmd_code). 
        # V paralelním vlákně se po přijetí odpovídající response naplní future
        future = asyncio.get_event_loop().create_future()
        can_resp_dict[(self.can_id, cmd_code)] = future
        # future se musí uložit před odesláním zprávy, abych náhodou nedostal odpověď a future ještě nebyla ve slovníku

        try:
            self.can_bus.send(can_msg)
            #print(f"sent {self.can_id}: {rclpy.clock.Clock().now().nanoseconds *1e-9} sec")
        except Exception as e:
            print(f"can_query send failed: {e}")
            can_resp_dict.pop((self.can_id, cmd_code), None)    #odstranění future ze slovníku při send failed
        #print(f"can msg sent: {can_msg}")
        
        
        
        try:
            # asynchronní čekání na naplnění future (tahle větev stojí, ale neblokuje ostatní větve asyncio executoru)
            response = await asyncio.wait_for(future, timeout=0.05)
            
            # wait_for z nějakého důvodu končí(KONČILO) přesně po timeoutu. Když je timeout 0.01sec, tak skončí po 0.01sec, když je 5sec, tak skončí po 5sec
            # Dělo se to proto:
            #   - asyncio executor při wait_for předal zdroje jiné úloze
            #   - ta nikde neměla await (to je ten problém. Asyncio executor drží zdroje v jedné úloze, dokud se neobjeví await)
            #   - takže executor nedokázal zdroje uvolnit a předat je zpět sem
            #   - těsně před timeoutem executor zdroje uvolnil a předal je sem, protože vzrostla priorita této úlohy (asi)
            #   - vyřešilo to cyklické volání await asyncio.sleep(0), které vynutí uvolnění zdrojů a přepnutí executoru 
            if len(response) != expected_response_length:
                print("can_query unexpected response length") 
                return None
                
            return response
        
        except asyncio.TimeoutError:
            #print(f"canDict_query: {can_resp_dict}")
            #print(f"canKey_query: {(self.can_id, cmd_code)}")
            cmd = MksCommands(cmd_code).name
            #print(f"can_query response timeout {cmd}, {self.can_id}. Future canceled")
            can_resp_dict.pop((self.can_id, cmd_code), None)    # tohle zároveň zruší future a zároveň ji smaže
            
            return None
        
        except Exception as e:
            print(f"can_query general exception: {e}")
        
    
    async def read_single_enc(self):
        cmd = MksCommands.READ_ENCODER_VALUE_CARRY
        expected_response_length = 8
        
        # odeslání CAN req a čekání na CAN resp je await - asynchronní
        data_bytes:bytearray = await self.can_query(expected_response_length, cmd, [cmd.value])

        if data_bytes is None:
            return None
        
        carry = int.from_bytes(data_bytes[1:5], byteorder='big', signed=True) # Revolutions
        value = int.from_bytes(data_bytes[5:7], byteorder='big', signed=True) # Position inside the revolution (0 - 0x4000)

        drive_pulse = carry * 0x4000 + value
        servo_rad = drive_pulse *2*pi /(2**14)

        joint_rad = float(servo_rad / self.gear_ratio) * self.dir

        return joint_rad

    
    async def read_single_vel(self):
        cmd = MksCommands.READ_MOTOR_SPEED
        expected_response_length = 4

        data_bytes:bytearray = await self.can_query(expected_response_length, cmd, [cmd.value])

        if data_bytes is None:
            return None

        servo_rpm = int.from_bytes(data_bytes[1:3], byteorder='big', signed=True) 
        joint_rpm = servo_rpm/self.gear_ratio * self.dir
        joint_rad_s = joint_rpm/60 * 2*pi
        return joint_rad_s

    async def servo_is_running(self):
        tmp = await self.can_query(self.DEFAULT_RESPONSE_LENGTH, MksCommands.QUERY_MOTOR_STATUS_COMMAND)
        status_int = int.from_bytes(tmp[1:2], byteorder='big')
        
        try:
            return MotorStatus(status_int) != MotorStatus.MotorStop
        except ValueError:
            raise Exception(f"servo_status joint{self.can_id}: Invalid status value {status_int}")


    async def write_single_pos(self, pos_rad, vel_rad, acc_rad=1*pi/180):
        if vel_rad > self.max_speed:
            print(f"Rychlost serva{self.can_id-1} omezena z {vel_rad:.4f} na {self.max_speed:.4f} rad/s")
            vel_rad = self.max_speed
        
        mks_pos = int(0x4000 * pos_rad/(2*pi) * self.gear_ratio * self.dir)
        mks_rpm = round((vel_rad * 60/(2*pi)) * self.gear_ratio)

        mks_acc = 0
        acc_rad_servo = 0
        if acc_rad != 0:  # MKS servo rampa neufnguje (první switchne směr a až pak klesá rychlost)
            acc_rad_servo = acc_rad * self.gear_ratio
            mks_acc = int(round(256-(pi/(acc_rad_servo*1.5)))) #\delta_t / (\delta_v_rpm*50e-6) = 256-acc_mks   
            
        mks_acc = int(acc_rad * 180/pi * self.gear_ratio)

        print(f"{self.can_id-1}: acc_rad: {acc_rad}, acc_rad_servo {acc_rad_servo}, mks_acc {mks_acc}")

        if await self.servo_is_running():
            raise Exception(f"Servo{self.can_id-1} is already running")    
        self.validate_speed(mks_rpm)
        self.validate_acceleration(mks_acc)
    
        cmd = [
            ((mks_rpm >> 8) & 0b1111),
            mks_rpm & 0xFF,
            mks_acc,        
            (mks_pos >> 16) & 0xFF,
            (mks_pos >> 8) & 0xFF,
            (mks_pos >> 0) & 0xFF,
        ]

        if mks_rpm <= 0:    # servo s rychlostí 0.0 se rozjede, ale nikdy nedojede
            return RunMotorResult.RunFail
        
        status_bytes = await self.can_query(self.DEFAULT_RESPONSE_LENGTH, MksCommands.RUN_MOTOR_ABSOLUTE_MOTION_BY_AXIS_COMMAND, cmd)
        
        # Status start motion
        status_start_int = int.from_bytes(status_bytes[1:2], byteorder='big')      
        
        try:
            status_start = RunMotorResult(status_start_int)
        except ValueError:
            raise Exception(f"set_pos status start joint{self.can_id-1}: Invalid status value {status_start}")         

        # Nepodařilo se odstartovat -> nečeká se na doběhnutí
        if status_start != RunMotorResult.RunStarting:
            return status_start
        
        # Status finished motion - čekání na dokončení pohybu
        future = asyncio.get_event_loop().create_future()
        cmd_code = MksCommands.RUN_MOTOR_ABSOLUTE_MOTION_BY_AXIS_COMMAND
        can_resp_dict[(self.can_id, cmd_code.value)] = future

        try:
            status_bytes = await asyncio.wait_for(future, timeout=30)
            status_int = int.from_bytes(status_bytes[1:2], byteorder='big')
            status_done = RunMotorResult(status_int)
        except (TimeoutError, asyncio.TimeoutError):
            status_done = RunMotorResult.RunFail
        except ValueError:
            raise Exception(f"set_pos_done joint{self.can_id-1}: Invalid status value {status_done}")

        return status_done     



    

    async def write_single_vel(self, vel_rad: float):
        cmd = MksCommands.RUN_MOTOR_SPEED_MODE_COMMAND
        expected_response_length = 3

        vel_rpm = int(round(vel_rad * 30/pi * self.gear_ratio * self.dir))

        #if (self.can_id-1) == 2 and True:   # Joint2 má obrácený směr pouze ve velocity režimu ¯\_(ツ)_/¯ don't ask
        #    vel_rpm = -vel_rpm
        #vel_rpm = -vel_rpm

        dir = Direction.CCW if vel_rpm < 0 else Direction.CW
        vel_rpm = min(abs(vel_rpm), 3000)
        
        acc_mks_value = 0
        #acc_rad_servo = 0
        #if acc_rad != 0:  # MKS servo rampa neufnguje (první switchne směr a až pak klesá rychlost)
        #    acc_rad_servo = acc_rad * self.gear_ratio
        #    acc_mks_value = int(round(256-(pi/(acc_rad_servo*1.5)))) #\delta_t / (\delta_v_rpm*50e-6) = 256-acc_mks        
        
        

        self.validate_direction(dir)
        self.validate_speed(vel_rpm)
        self.validate_acceleration(acc_mks_value)

        

        dir_mks_value = 0x80 if dir == Direction.CW else 0x00

        data = [
            dir_mks_value + ((vel_rpm >> 8) & 0b1111),
            vel_rpm & 0xFF,
            int(acc_mks_value)
        ]

        
        data_bytes:bytearray = await self.can_query(expected_response_length, cmd, data)

        if data_bytes:
            return int.from_bytes(data_bytes[1:2], byteorder='big') 
        return None
    

    async def set_current_axis_to_zero(self):
        status_bytes = await self.can_query(self.DEFAULT_RESPONSE_LENGTH, MksCommands.SET_CURRENT_AXIS_TO_ZERO_COMMAND)
        if status_bytes:
            status_int = int.from_bytes(status_bytes[1:2], byteorder='big') 
            print(f"set_current_axis_to_zero {self.can_id-1}: {status_int}")
            return SuccessStatus(status_int)
        return SuccessStatus.Fail


    async def set_work_mode(self, mode: WorkMode):
        return await self.can_query(self.DEFAULT_RESPONSE_LENGTH, MksCommands.SET_WORK_MODE_COMMAND, mode.value)
    

    async def set_working_current(self, current):
        self.validate_current(current)

        return await self.can_query(self.DEFAULT_RESPONSE_LENGTH, MksCommands.SET_WORKING_CURRENT_COMMAND, [(current >> 8) & 0xFF, current & 0xFF])
    

    async def config_home(self, homeTrig : EndStopLevel, homeDir: Direction, homeSpeed: int, endLimit: Enable):

        return await self.can_query(self.DEFAULT_RESPONSE_LENGTH, MksCommands.SET_HOME_COMMAND, [homeTrig.value, homeDir.value, (homeSpeed >> 8) & 0xF, homeSpeed & 0xFF, endLimit.value])


    async def set_subdivisions(self, mstep):

        return await self.can_query(self.DEFAULT_RESPONSE_LENGTH, MksCommands.SET_SUBDIVISIONS_COMMAND, mstep) 
    
    async def emergency_stop_motor(self):
        return await self.can_query(self.DEFAULT_RESPONSE_LENGTH, MksCommands.EMERGENCY_STOP_COMMAND)
    
    async def homing_non_blocking(self):
        tmp = await self.can_query(self.DEFAULT_RESPONSE_LENGTH, MksCommands.GO_HOME_COMMAND)
        status_int = int.from_bytes(tmp[1:2], byteorder='big')  
        try:
            result = GoHomeResult(status_int)
            self.homing_status = result
        except ValueError:
            raise Exception(f"Homing error: Invalid status value {status_int}")                     
        return result

    async def wait_for_home_done(self):
        if self.homing_status == GoHomeResult.Unkown:
            raise Exception("Homing not running")

        # Čekání na odpověď serva - musí se vložit nová future do slovníku, protože servo při startu odpoví "Start"
        print(f"dict1: {can_resp_dict}")
        future = asyncio.get_event_loop().create_future()
        cmd_code = MksCommands.GO_HOME_COMMAND
        can_resp_dict[(self.can_id, cmd_code.value)] = future
        print(f"dict2: {can_resp_dict}")

        try:
            tmp = await asyncio.wait_for(future, timeout=self.MAX_HOMING_TIME)
            status_int = int.from_bytes(tmp[1:2], byteorder='big')

        except TimeoutError:
            print("Homing timeout")
        
        self.homing_status = status_int
        return status_int


    async def homing_blocking(self):
        await self.homing_non_blocking()
        await self.wait_for_home_done()
        return self.homing_status

    def joint_setup(self):
        loop = self.asyncio_loop
        if self.adaptive_current:
            asyncio.run_coroutine_threadsafe(self.set_work_mode(WorkMode.SrvFoc),loop).result()
            asyncio.run_coroutine_threadsafe(self.set_working_current(1600),loop).result()
        else:
            asyncio.run_coroutine_threadsafe(self.set_work_mode(WorkMode.SrClose),loop).result()
            asyncio.run_coroutine_threadsafe(self.set_working_current(1000),loop).result()

        tasks = []
        asyncio.run_coroutine_threadsafe(self.config_home(EndStopLevel.Low, self.homing_dir, int(self.homing_speed * self.gear_ratio), Enable.Disable),loop).result()
        asyncio.run_coroutine_threadsafe(self.set_subdivisions(self.motor_subdivisions),loop).result()
        asyncio.run_coroutine_threadsafe(self.emergency_stop_motor(),loop).result()



class ManipulatorDriver(Node):
    def __init__(self):
        super().__init__('async_servo_node')

        self.JOINT_COUNT = 5
        self.HOMING_SPEEDS = [1, 1, 1, 1, 1]
        self.HOMING_DIRECTIONS = [Direction.CW, Direction.CW, Direction.CW, Direction.CW, Direction.CW]
        max_servo_speeds_deg = [180, 180, 180, 90, 90]
        self.MAX_SERVO_SPEEDS_RAD = [deg*pi/180 for deg in max_servo_speeds_deg] # v SW se racuje s RAD, ale zadává se jako deg pro lepší představu
        self.GEAR_RATIOS = [10, 10, 10, 10, 10]
        # teď je všude minimální zrychlení natvrdo (smazat acc_mks=1)!!
        ramp_deg = [900, 900, 900, 900, 900]
        self.RAMP_RAD = [deg*pi/180 for deg in ramp_deg]
        self.INVERT_DIRECTIONS = [False, True, True, True, True]
        self.ADAPTIVE_CURRENT = [True, True, True, True, True]
        self.MOTOR_SUBDIVISIONS = 64
        zero_offset_deg = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.ZERO_OFFSET_RAD = [deg*pi/180 for deg in zero_offset_deg]
        self.SAFETY_ANGLE_RAD = 0.0
        #self.JOINT_LIMIT_RAD = [{"lo": 100 * (-3.14) + self.SAFETY_ANGLE_RAD, "hi": 100 * (3.14) - self.SAFETY_ANGLE_RAD},
        #                        {"lo": 100 * (-3.14) + self.SAFETY_ANGLE_RAD, "hi": 100 * (3.14) - self.SAFETY_ANGLE_RAD},
        #                        {"lo": 100 * (-3.14) + self.SAFETY_ANGLE_RAD, "hi": 100 * (3.14) - self.SAFETY_ANGLE_RAD},
        #                        {"lo": 100 * (-3.14) + self.SAFETY_ANGLE_RAD, "hi": 100 * (3.14) - self.SAFETY_ANGLE_RAD},
        #                        {"lo": 100 * (-3.14) + self.SAFETY_ANGLE_RAD, "hi": 100 * (3.14) - self.SAFETY_ANGLE_RAD}]
        # Turn off limits for the competition
        self.JOINT_LIMIT_RAD = [{"lo": -math.inf, "hi": math.inf},
                                {"lo": -math.inf, "hi": math.inf},
                                {"lo": -math.inf, "hi": math.inf},
                                {"lo": -math.inf, "hi": math.inf},
                                {"lo": -math.inf, "hi": math.inf}]
        self.mode: int = 2 # 0 - standby, 1 - position, 2 - speed
        self.last_read_joint_pos = [0.0] * self.JOINT_COUNT
        self.last_read_joint_vel = [0.0] * self.JOINT_COUNT
        self.act_vel_lock = threading.Lock()
        self.slow_vel_dif = [0.0] * self.JOINT_COUNT

        self.act_pos_lock = threading.Lock()

        # Nastavení CAN sběrnice
        self.declare_parameter('can_bus', 'can0')
        self.can_channel = self.get_parameter('can_bus').get_parameter_value().string_value
        self.can_bus = can.Bus(interface='socketcan', channel=self.can_channel, bitrate=500000)
        
	# bitrate 1Mbit/s -> 125 kByte/sec
        # 1 CAN zpráva: 6*(8byte req + 8byte resp) = 100 byte
        # -> MAX 1250 dotazů za sec

        # Asynchronní přístup ke CAN
        self.can_reader = can.AsyncBufferedReader(queue_size=1000)
        can.Notifier(self.can_bus, [self.can_reader]) # Když obdrží zprávu přes bus, tak ji předá asynchronnímu readeru, který ji zpracuje
        
        # vytvoření executoru asyncio, který sbírá tasky a řídí jejich vykonávání/uspávání
        # asyncio executor neumí spolupracovat s ros2 executorem -> musí se spustit na jiném vlákně (start_can_handler)
        # Všechny asyncio operace běží na vlákně odděleném od ROS
        self.asyncio_loop = asyncio.new_event_loop()
        threading.Thread(target=self.start_can_handler, daemon=True).start()
        
        #self.create_timer(0.01, self.pos_timer_callback)
        self.pos_timer_lock = threading.Lock()

        self.create_timer(0.01, self.vel_timer_callback)
        self.vel_timer_lock = threading.Lock()

        self.servos: List[JointDriver] = []
        for i in range(self.JOINT_COUNT):
            self.servos.append(JointDriver(self.asyncio_loop, i+1, self.can_bus, self.MOTOR_SUBDIVISIONS, self.HOMING_SPEEDS[i],self.HOMING_DIRECTIONS[i],self.MAX_SERVO_SPEEDS_RAD[i],self.ADAPTIVE_CURRENT[i], self.ZERO_OFFSET_RAD[i], self.JOINT_LIMIT_RAD[i], self.GEAR_RATIOS[i],self.INVERT_DIRECTIONS[i]))

        self.pose_subscriber = self.create_subscription(JointState, "manipulator/set_joints_position", self.set_position_callback, 1)
        self.pos_set_lock = threading.Lock()
        self.velocity_subscriber = self.create_subscription(JointState, "manipulator/set_joints_velocity", self.set_velocity_callback, 1)
        self.vel_set_lock = threading.Lock()

        self.pos_publisher = self.create_publisher(JointState, "manipulator/state_joints_position", 1)
        self.vel_publisher = self.create_publisher(JointState, "manipulator/state_joints_velocity", 1)

        for servo in self.servos:
            servo.joint_setup()
        self.service_change_mode = self.create_service(ChangeMode, "manipulator/change_mode", self.on_change_mode)
        #self.service_reset_axis = self.create_service(ResetAxis, "manipulator/reset_single_joint", self.on_reset_axis)
        self.service_zero_axis = self.create_service(ResetAxis, "manipulator/zero_single_joint", self.on_zero_axis)
        self.service_home_axis = self.create_service(HomeAxis, "manipulator/home_single_joint", self.on_home_axis)
        self.get_logger().info("Manipulator driver ready - changed5")


    def start_can_handler(self):
        asyncio.set_event_loop(self.asyncio_loop) # Spuštění async executoru na jiném vlákně
        self.asyncio_loop.create_task(can_listener(self.can_reader))
        self.asyncio_loop.create_task(free_loop_resources())
        self.asyncio_loop.run_forever()
    

    def limit_positions(self, pos):
        # SW limit KLOUBŮ, ne serv:
        for i in range(self.JOINT_COUNT):
            if  pos[i] < (self.servos[i].sw_limits_rad['lo']):
                pos[i] = self.servos[i].sw_limits_rad['lo']
                self.get_logger().error(f"Požadavek joint{i} byl omezen na limit {pos[i]:.2f}rad")
            
            elif pos[i] > (self.servos[i].sw_limits_rad['hi']):
                pos[i] = self.servos[i].sw_limits_rad['hi']
                self.get_logger().error(f"Požadavek joint{i} byl omezen na SW limit {pos[i]:.2f}rad")

        return pos

    def set_position_callback(self, msg: JointState):
        if self.mode != 1:
            self.get_logger().error("Manipulátor není v režimu \"position\" a dostal požadavek")
            return
        
        if self.pos_set_lock.locked():
            self.get_logger().error("Manipulátor zpracovává předchozí set_position požadavek")
            return

        thread = threading.Thread(target=self.write_all_positions(msg))
        thread.start()
        
    


    def set_velocity_callback(self, msg: JointState):
        if self.mode != 2:
            self.get_logger().error("Manipulátor není v režimu \"velocity\" a dostal požadavek")
            return
        
        if self.vel_set_lock.locked():
            self.get_logger().error("Manipulátor zpracovává předchozí set_velocity požadavek")
            return
        
        thread = threading.Thread(target=self.write_all_velocity(msg))
        thread.start()

    def on_change_mode(self, request:ChangeMode.Request, response: ChangeMode.Response):
        if request.mode >= 3 or request.mode < 0:
            response.success = False
            return response
        
        futures = []
        for servo in self.servos:
            futures.append(asyncio.run_coroutine_threadsafe(servo.emergency_stop_motor(), self.asyncio_loop))
        
        results = [fut.result() for fut in futures]

        print(f"Mode changed to: {request.mode}, servos stopped: {results}")
        self.mode = request.mode
        response.success = True
        
        return response

    def on_zero_axis(self, request:ResetAxis.Request, response: ResetAxis.Response):
        servo = self.servos[request.index]
        success = False

        # Přepnutí do pozičního řízení
        modeReq = ChangeMode.Request()
        modeReq.mode = 1
        modeResp = ChangeMode.Response()     
        modeResp = self.on_change_mode(modeReq, modeResp)

        set_pos_status = SuccessStatus(0)
        set_pos_future = asyncio.run_coroutine_threadsafe(servo.write_single_pos(servo.zero_offset_rad, self.MAX_SERVO_SPEEDS_RAD[request.index]/2), self.asyncio_loop)    # vrací RunMotorResult
        try:
            move_status = set_pos_future.result(timeout=60)
            set_pos_status = SuccessStatus.Success if (move_status==RunMotorResult.RunComplete) else SuccessStatus.Fail
        except (asyncio.TimeoutError, TimeoutError):
            self.get_logger().error(f"Timeout zero joint{request.index}")
            return False
        
        if set_pos_status == SuccessStatus.Fail:
            self.get_logger().error(f"Failed zero joint{request.index}")
            return False

        zero_future = asyncio.run_coroutine_threadsafe(servo.set_current_axis_to_zero(), self.asyncio_loop)
        zero_success = zero_future.result(timeout=2)
        bool_success = (zero_success == SuccessStatus.Success)
        print(f"succ_1: {bool_success}")

        response.success = bool_success

        self.get_logger().warn(f"Zero joint{request.index} finished: {'Success' if bool_success else 'Failed'}")
        return response


    def on_home_axis(self, request:HomeAxis.Request, response:HomeAxis.Response):
        future = asyncio.run_coroutine_threadsafe(self.servos[request.index].homing_blocking(), self.asyncio_loop)   # blokující čekání na homing
        homing_status = future.result() 

        response.success = (homing_status == GoHomeResult.Success.value)
        if homing_status != GoHomeResult.Success.value:
            print(f"Homing of joint{new_index} failed")
        return response  


    def pos_timer_callback(self):
        if not self.pos_timer_lock.locked():
            # ROS2 Humble MultiThreadedExecutor neumí "Fair Scheduling". 
            # Když přijdou 2 callback požadavky současně tak spustí ten callback, který se dokáže připravit rychleji
            # To vede ke "Callback Starvation" 
            #  - kvůli race conditions se volá pořád dokola ten callback, který se dokáže zavolat a připravit rychleji
            # Pokud chceme skutečně multithreaded callbacky, tak si je musíme udělat sami přes threading
            # Pokud nechceme reentrantní callbacky, tak je musíme zamykat přes threading.Lock()
            thread = threading.Thread(target=self.read_all_positions)
            thread.start()
    
    def vel_timer_callback(self):
        if not self.vel_timer_lock.locked():
            thread = threading.Thread(target=self.read_all_velocity)
            thread.start()

    async def read_all_positions_async(self):
        tasks = []
        for i in range(self.JOINT_COUNT):
            tasks.append(self.servos[i].read_single_enc())  # registrace asynchronních tasků (1 task = čtení 1 enc)

        try:
            results = await asyncio.gather(*tasks)  # čekání na dokončení všech tasků paralelně. Potom naplní future, na kterou čeká ros executor (future.result)
        except Exception as e:
            raise e

        return results     

    def read_all_positions(self):
        with self.pos_timer_lock:
            # Při run_coroutine_threadsafe se funkce registruje do toho asyncio executoru z VEDLEJŠÍHO VLÁKNA - neblokuje ros2 executor
            # asyncio executor ji zavolá podle potřeby -> uvnitř už to je asynchronní funkce která může běžet paralelně
            # vrací future promise = příslib budoucího výsledku, který paralelně naplní až bude znát hodnoty. do té doby tohle vlákno normálně pokračuje dál
            future = asyncio.run_coroutine_threadsafe(self.read_all_positions_async(), self.asyncio_loop)

            # Co se stane po naplnění future promise - je to registrace callbacku, ne okamžité zavolání callbacku - neblokující
            # Ten calback se provede na vlákně asyncio executoru. Ros executor běží normálně dál
            # Tím že se volá paralelně na asyncio vlákně, tak není bezpečné používat ros funkce - vůbec neblokuje ROS, ale data se musí předávat atomicky přes threading.Lock
            
            # Blokující čekání na výsledek future - nevadí protože callbacky jsou paralelní
            time.sleep(0.01) # aby se ke slovu dostaly další callbacky
            act_pos = [None] * self.JOINT_COUNT
            try:
                act_pos = future.result(timeout=1)
                #print(f"{rclpy.clock.Clock().now().nanoseconds *1e-9} - read_pos[rad]: {act_pos}") 
            except asyncio.TimeoutError:
                self.get_logger().error("Timeout while reading position")
            except Exception as e:
                self.get_logger().error(f"Error reading position: {e}")
                self.get_logger().error(traceback.format_exc())        

            # kontrola chyby čtení - nahradí se poslední minulou
            with self.act_pos_lock:
                last_read_pos = self.last_read_joint_pos

            for i in range(self.JOINT_COUNT):
                if act_pos[i] is None:
                    act_pos[i] = last_read_pos[i]
            
            if self.JOINT_COUNT > 4:
                act_pos[-2:] = self.wrist_from_servo(act_pos[-2:])

            with self.act_pos_lock:
                self.last_read_joint_pos = act_pos

            # Tady už mám v ros2 threadu data z asyncio feature a můžu s ními pracovat bez threading.lockování
            # Publish
            msg = self.build_JointState_msg(act_pos, None)
            self.pos_publisher.publish(msg)            

    
    def build_JointState_msg(self, position:List[float]|None, velocity:List[float]|None):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f"joint{i}" for i in range(self.JOINT_COUNT)]

        if position:
            msg.position = position


        if velocity:
            msg.velocity = velocity

        return msg

        
    async def read_all_velocity_async(self):
        tasks = []
        for i in range(self.JOINT_COUNT):
            tasks.append(self.servos[i].read_single_vel())
        
        results = await asyncio.gather(*tasks)

        return results
    
    def read_all_velocity(self):
        with self.vel_timer_lock:

            future = asyncio.run_coroutine_threadsafe(self.read_all_velocity_async(), self.asyncio_loop)
            act_vel = [None] * self.JOINT_COUNT
            try:
                act_vel = future.result(timeout=1)
                #print(f"{rclpy.clock.Clock().now().nanoseconds *1e-9} - read_vel[rad/s]: {act_vel}")
            except asyncio.TimeoutError:
                self.get_logger().error("Timeout while reading velocity")
            except Exception as e:
                self.get_logger().error(f"Error reading velocity: {e}")
                self.get_logger().error(traceback.format_exc())
            

            # kontrola chyby čtení - nahradí se poslední minulou

            with self.act_vel_lock:
                last_vel = self.last_read_joint_vel

            for i in range(self.JOINT_COUNT):
                if act_vel[i] is None:
                    act_vel[i] = last_vel[i]

            with self.act_vel_lock:
                self.last_read_joint_vel = act_vel

            # Publish
            msg = self.build_JointState_msg(None, act_vel)
            self.vel_publisher.publish(msg)

    async def write_all_positions_async(self, position_rad, velocity_rad, acceleration_rad):
        tasks = []

        for i in range(self.JOINT_COUNT):
            tasks.append(self.servos[i].write_single_pos(position_rad[i], velocity_rad[i], acceleration_rad[i]))
        
        results = await asyncio.gather(*tasks)
        print(f"write_pos done: {[res.name for res in results]}")
        return results

    async def write_all_velocity_async(self, velocity_rad):
        tasks = []
        
        for i in range(self.JOINT_COUNT):
            tasks.append(self.servos[i].write_single_vel(velocity_rad[i]))
        
        results = await asyncio.gather(*tasks)

        return results

    @staticmethod
    def limit_velocity(req: List[float], max: List[float], logger=None):
        new_vel = req

        for i in range(len(req)):
            if abs(req[i]) > max[i]:
                sign = (req[i]>0)-(req[i]<0)
                new_speed = max[i] * sign
                if logger:
                    logger.error(f"Rychlost serva{i} omezena z {req[i]:.4f} rad/s na {new_speed} rad/s")
                else:
                    print(f"Rychlost serva{i} omezena z {req[i]} na {new_speed} rad/s")
                new_vel[i] = new_speed

        return new_vel

    @staticmethod
    def velocity_ramp(req_vel_list: list, act_vel_list: list, acceleration: list):

        min_vel_diff = []
        max_vel_diff = []
        for i in range(len(acceleration)):
            if acceleration[i] == 0:
                min_vel_diff.append(-999)
                max_vel_diff.append(999)
            else:
                min_vel_diff.append(-acceleration[i])
                max_vel_diff.append(acceleration[i])


        new_vel_list = []
        for i in range(len(req_vel_list)):
            if abs(act_vel_list[i]) < 0.01: # šum senzoru
                act_vel_list[i] = 0.0
            
            vel_diff = req_vel_list[i] - act_vel_list[i]
            new_vel = act_vel_list[i] + min(max(vel_diff, min_vel_diff[i]), max_vel_diff[i])
            new_vel_list.append(new_vel)
        
        return new_vel_list

    @staticmethod
    def servo_from_wrist(wristAngle: list[float]):
        servoAngle0 = (wristAngle[0] + wristAngle[1])
        servoAngle1 = (wristAngle[0] - wristAngle[1])
        
        #return wristAngle # <-- ignoruje kinematiku zápěstí a řídí serva samostatně jako předtím
        return [servoAngle0, servoAngle1]


    @staticmethod
    def wrist_from_servo(servoAngle :list[float]):
        wristAngle0 = (servoAngle[0] + servoAngle[1]) / 2
        wristAngle1 = (servoAngle[0] - servoAngle[1]) / 2

        #return servoAngle # <-- ignoruje kinematiku zápěstí a řídí serva samostatně jako předtím
        return [wristAngle0, wristAngle1]

    def write_all_positions(self, msg: JointState):
        with self.pos_set_lock:
            goal_position = list(msg.position)
            goal_velocity = list(msg.velocity)

            if not goal_position or any(pos is None for pos in goal_position):
                print("Neplatný požadavek na set_positon")
                return
            
            goal_position = self.limit_positions(goal_position)

            # Kinematika diferenciálního zápěstí - joint-->servo
            if self.JOINT_COUNT > 4:
                goal_position[-2:] = self.servo_from_wrist(goal_position[-2:])
                goal_velocity[-2:] = self.servo_from_wrist(goal_velocity[-2:])
                goal_velocity = [abs(vel) for vel in goal_velocity]

            #acc = [5*pi/180 for _ in range(self.JOINT_COUNT)] #?
            acc = [1*pi/180 for _ in range(self.JOINT_COUNT)]

            #self.get_logger().error(f"goal position: {goal_position}")
            future = asyncio.run_coroutine_threadsafe(self.write_all_positions_async(goal_position, goal_velocity, acc), self.asyncio_loop)
            print(future)
            try:
                results = future.result(timeout=20)
                self.get_logger().info(f"set_pos success: {[res.name for res in results]}")
            except (asyncio.TimeoutError, TimeoutError) as e:
                self.get_logger().error(f"Timeout while setting positions: {e}")
            except Exception as e:
                self.get_logger().error(f"Error setting position {e}")
                #self.get_logger().error(traceback.format_exc())



    def write_all_velocity(self, msg: JointState):
        with self.vel_set_lock:
            req_joint_vel_rad = list(msg.velocity)
            if any(vel is None for vel in req_joint_vel_rad):    # Když příjde neplatná rychlost, tak se serva zastaví
                req_joint_vel_rad = [0.0] * self.JOINT_COUNT
            else:
                req_joint_vel_rad = self.limit_velocity(req_joint_vel_rad, self.MAX_SERVO_SPEEDS_RAD, self.get_logger())
                #with self.act_vel_lock:
                #    act_joint_vel = self.last_read_joint_vel

                with self.act_pos_lock:
                    act_joint_pos = self.last_read_joint_pos
                # Limity kloubů
                for i in range(len(self.servos)):
                    if  (act_joint_pos[i] < (self.servos[i].sw_limits_rad['lo'])) and (req_joint_vel_rad[i] < 0):
                        req_joint_vel_rad[i] = 0.0
                        self.get_logger().error(f"VelCmd: Joint{i} narazil na SW limit {act_joint_pos[i]:.2f} rad")
                    
                    if (act_joint_pos[i] > (self.servos[i].sw_limits_rad['hi'])) and (req_joint_vel_rad[i] > 0):
                        req_joint_vel_rad[i] = 0.0
                        self.get_logger().error(f"VelCmd: Joint{i} narazil na SW limit {act_joint_pos[i]:.2f} rad")
                
                if self.JOINT_COUNT > 4:
                    req_joint_vel_rad[-2:] = self.servo_from_wrist(req_joint_vel_rad[-2:])

                # rampa nečte aktuální rychlost, která má velký delay, ale minulou astavenou rychlost
                req_joint_vel_rad = self.velocity_ramp(req_joint_vel_rad, self.slow_vel_dif, self.RAMP_RAD)

            self.slow_vel_dif = req_joint_vel_rad
            #self.get_logger().error(f"req_joint_vel_main: {req_joint_vel_rad}")

            future = asyncio.run_coroutine_threadsafe(self.write_all_velocity_async(req_joint_vel_rad), self.asyncio_loop)

            try:
                result = future.result(timeout=1)
                vel_succ_set = [f"{vel:.4}" if res==1 else "0.0" for res,vel in zip(result, req_joint_vel_rad)]
                #self.get_logger().info(f"set_vel: {vel_succ_set}")
            except asyncio.TimeoutError:
                self.get_logger().error("Timeout while setting velocity")
            except Exception as e:
                self.get_logger().error(f"Error setting velocity: {e}")
                self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    node = ManipulatorDriver()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
