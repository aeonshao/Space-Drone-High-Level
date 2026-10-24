'''
Mission node - the "brain" of the drone.

Flies the whole thing with no human input:
    1. connect to PX4 + preflight checks + arm
    2. take off to search altitude
    3. fly a lawnmower pattern over the search area while the aruco detector +
       tool registry build up a map of where the tools are
    4. go back to each tool in `visit_tools` (or every tool found if visit_all_found
       is true), drop down and hover over it  ("post deployment" part)
    5. fly home and land

After landing it stays up so you can call the go_to_tool service and it will take off,
fly back to that tool, hover, and come home again:

    ros2 service call /spacedrone/go_to_tool spacedrone_interfaces/srv/GoToTool "{tag_id: 3}"

Talks to PX4 with MAVSDK (same as the old scripts in drone_software/) and to the rest
of the stack with ROS2. MAVSDK is all asyncio and rclpy has its own executor so the
asyncio loop runs in a separate thread. Took a while to get that working, see the
commented out stuff in __init__.
'''

import asyncio
import math
import threading
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw

from spacedrone_interfaces.msg import ToolMap
from spacedrone_interfaces.srv import GoToTool
from spacedrone_mission.search_pattern import lawnmower, path_length


class State(Enum):
    IDLE = 0
    CONNECTING = 1
    PREFLIGHT = 2
    TAKEOFF = 3
    SEARCH = 4
    VISIT = 5
    RETURN_HOME = 6
    LANDING = 7
    DONE = 8
    FAILED = 9


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        # ---------------- params ----------------
        # udp://:14540 is deprecated in newer mavsdk_server, udpin is the new way
        self.declare_parameter('system_address', 'udpin://0.0.0.0:14540')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('search_altitude', 6.0)
        self.declare_parameter('search_speed', 2.0)
        self.declare_parameter('search_north_min', 0.0)
        self.declare_parameter('search_north_max', 22.0)
        self.declare_parameter('search_east_min', -9.0)
        self.declare_parameter('search_east_max', 9.0)
        self.declare_parameter('lane_spacing', 6.0)
        self.declare_parameter('visit_altitude', 3.0)
        self.declare_parameter('visit_hover_time', 5.0)
        # ros2 params cant be an empty int list (it becomes a string array??) so -1 = nothing
        self.declare_parameter('visit_tools', [-1])
        self.declare_parameter('visit_all_found', False)
        self.declare_parameter('waypoint_tolerance', 0.4)
        self.declare_parameter('settle_speed', 0.3)
        self.declare_parameter('min_battery', 0.25)

        self.address = self.get_parameter('system_address').value
        self.search_alt = self.get_parameter('search_altitude').value
        self.search_speed = self.get_parameter('search_speed').value
        self.visit_alt = self.get_parameter('visit_altitude').value
        self.visit_hover_time = self.get_parameter('visit_hover_time').value
        self.wp_tol = self.get_parameter('waypoint_tolerance').value
        self.settle_speed = self.get_parameter('settle_speed').value
        self.min_battery = self.get_parameter('min_battery').value
        self.visit_all_found = self.get_parameter('visit_all_found').value
        self.visit_queue = [i for i in self.get_parameter('visit_tools').value if i >= 0]

        # ---------------- ros stuff ----------------
        self.pose_pub = self.create_publisher(PoseStamped, '/spacedrone/vehicle_pose', qos_profile_sensor_data)
        self.state_pub = self.create_publisher(String, '/spacedrone/mission_state', 10)

        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_sub = self.create_subscription(ToolMap, '/spacedrone/tool_map', self.map_callback, map_qos)

        self.goto_srv = self.create_service(GoToTool, '/spacedrone/go_to_tool', self.go_to_tool_callback)
        self.start_srv = self.create_service(Trigger, '/spacedrone/start_mission', self.start_callback)

        self.pose_timer = self.create_timer(0.05, self.publish_pose)   # 20 hz
        self.state_timer = self.create_timer(1.0, self.publish_state)

        # ---------------- shared state (written by asyncio thread, read by ros thread) ----------------
        self.state = State.IDLE
        self.ned = None          # (n, e, d)
        self.speed = 0.0         # horizontal speed m/s
        self.quat = None         # (w, x, y, z) body FRD -> NED
        self.in_air = False
        self.battery = 1.0
        self.tool_map = {}       # tag_id -> Tool msg
        self.mission_started = False
        self.flight_lock = threading.Lock()

        self.drone = System()

        # asyncio loop for mavsdk lives in its own thread
        # first attempt was just asyncio.run() inside a timer callback... that blocks spin forever
        # second attempt was rclpy.spin_once() inside the asyncio loop, it kind of worked but the
        # detections lagged like 2 seconds behind
        # self.create_timer(0.1, lambda: asyncio.run(self.run_mission()))
        self.loop = asyncio.new_event_loop()
        self.loop_thread = threading.Thread(target=self.loop.run_forever, daemon=True)
        self.loop_thread.start()

        self.run_async(self.connect())

        if self.get_parameter('auto_start').value:
            self.mission_started = True
            self.run_async(self.run_mission())
        else:
            self.get_logger().info('auto_start is off, call /spacedrone/start_mission to go')

    # ===================== ros callbacks =====================

    def run_async(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self.loop)

    def set_state(self, s):
        if s != self.state:
            self.get_logger().info(f'STATE: {self.state.name} -> {s.name}')
            self.state = s
            self.publish_state()

    def publish_state(self):
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)

    def publish_pose(self):
        if self.ned is None or self.quat is None:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'local_ned'
        msg.pose.position.x = self.ned[0]
        msg.pose.position.y = self.ned[1]
        msg.pose.position.z = self.ned[2]
        msg.pose.orientation.w = self.quat[0]
        msg.pose.orientation.x = self.quat[1]
        msg.pose.orientation.y = self.quat[2]
        msg.pose.orientation.z = self.quat[3]
        self.pose_pub.publish(msg)

    def map_callback(self, msg):
        new_map = {}
        for t in msg.tools:
            new_map[t.tag_id] = t
        self.tool_map = new_map   # swap the whole dict so the other thread never sees half of it

    def start_callback(self, request, response):
        if self.mission_started:
            response.success = False
            response.message = 'mission already started'
            return response
        self.mission_started = True
        self.run_async(self.run_mission())
        response.success = True
        response.message = 'starting mission'
        return response

    def go_to_tool_callback(self, request, response):
        tag_id = request.tag_id
        if tag_id not in self.tool_map:
            response.accepted = False
            response.message = f'tool {tag_id} is not in the map (known: {sorted(self.tool_map.keys())})'
            return response

        name = self.tool_map[tag_id].tool_name
        if self.state in (State.DONE, State.IDLE) and not self.flight_lock.locked():
            # on the ground, do a whole new flight just for this tool
            self.run_async(self.fetch_flight(tag_id))
            response.message = f'taking off to go to {name}'
        elif self.state in (State.SEARCH, State.VISIT, State.TAKEOFF):
            if tag_id not in self.visit_queue:
                self.visit_queue.append(tag_id)
            response.message = f'{name} added to visit queue'
        else:
            response.accepted = False
            response.message = f'cant do that right now, state is {self.state.name}'
            return response

        response.accepted = True
        self.get_logger().info(response.message)
        return response

    # ===================== mavsdk side (asyncio thread) =====================

    async def connect(self):
        self.set_state(State.CONNECTING)
        self.get_logger().info(f'connecting to drone on {self.address}')
        await self.drone.connect(system_address=self.address)

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info('-- connected to drone!')
                break

        # faster telemetry so the pose lines up with the camera better
        try:
            await self.drone.telemetry.set_rate_position_velocity_ned(30.0)
            await self.drone.telemetry.set_rate_attitude_quaternion(30.0)
        except Exception as e:
            self.get_logger().warn(f'couldnt set telemetry rate: {e}')

        # background telemetry readers
        asyncio.ensure_future(self.watch_position())
        asyncio.ensure_future(self.watch_attitude())
        asyncio.ensure_future(self.watch_in_air())
        asyncio.ensure_future(self.watch_battery())
        asyncio.ensure_future(self.watch_status_text())

        if self.state == State.CONNECTING:
            self.set_state(State.IDLE)

    async def watch_position(self):
        async for pv in self.drone.telemetry.position_velocity_ned():
            self.ned = (pv.position.north_m, pv.position.east_m, pv.position.down_m)
            self.speed = math.hypot(pv.velocity.north_m_s, pv.velocity.east_m_s)

    async def watch_attitude(self):
        async for q in self.drone.telemetry.attitude_quaternion():
            self.quat = (q.w, q.x, q.y, q.z)

    async def watch_in_air(self):
        async for in_air in self.drone.telemetry.in_air():
            self.in_air = in_air

    async def watch_battery(self):
        async for b in self.drone.telemetry.battery():
            pct = b.remaining_percent
            # mavsdk changed this from 0-1 to 0-100 at some point?? handle both
            if pct > 1.0:
                pct = pct / 100.0
            self.battery = pct

    async def watch_status_text(self):
        try:
            async for status_text in self.drone.telemetry.status_text():
                self.get_logger().info(f'[PX4] {status_text.type}: {status_text.text}')
        except asyncio.CancelledError:
            return

    async def wait_for_connection(self):
        while self.state == State.CONNECTING or self.ned is None or self.quat is None:
            await asyncio.sleep(0.2)

    async def preflight_and_arm(self):
        # same as drone_software/flight_control/precheck.py
        self.set_state(State.PREFLIGHT)
        self.get_logger().info('waiting for drone to be armable (gps + home position)...')
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok and health.is_armable:
                self.get_logger().info('-- preflight checks ok')
                break

        self.get_logger().info('-- arming')
        await self.drone.action.arm()

    async def takeoff(self, altitude):
        self.set_state(State.TAKEOFF)
        await self.drone.action.set_takeoff_altitude(altitude)
        await self.drone.action.takeoff()

        # wait until we're actually up there (down is negative)
        t0 = time.time()
        while -self.ned[2] < altitude * 0.9:
            await asyncio.sleep(0.2)
            if time.time() - t0 > 30:
                self.get_logger().warn('takeoff taking forever, continuing anyway')
                break
        self.get_logger().info(f'-- reached {-self.ned[2]:.1f} m')

        # need to send a setpoint BEFORE starting offboard or PX4 rejects it
        n, e, d = self.ned
        await self.drone.offboard.set_position_ned(PositionNedYaw(n, e, -altitude, 0.0))
        try:
            await self.drone.offboard.start()
        except OffboardError as err:
            self.get_logger().error(f'offboard start failed: {err._result.result}')
            raise
        self.get_logger().info('-- offboard mode started')

    def dist_to(self, n, e, d):
        return math.sqrt((self.ned[0] - n) ** 2 + (self.ned[1] - e) ** 2 + (self.ned[2] - d) ** 2)

    async def fly_to(self, n, e, d, yaw=0.0, timeout=40.0, settle=False):
        '''
        position setpoint and wait till we get there.
        settle=True also waits until the drone has actually stopped. without this the land
        command went out while still doing ~3 m/s and it overshot home by like 4 meters
        '''
        await self.drone.offboard.set_position_ned(PositionNedYaw(n, e, d, yaw))
        t0 = time.time()
        while self.dist_to(n, e, d) > self.wp_tol or (settle and self.speed > self.settle_speed):
            if time.time() - t0 > timeout:
                self.get_logger().warn(f'timed out going to ({n:.1f}, {e:.1f}, {d:.1f}), '
                                       f'still {self.dist_to(n, e, d):.1f}m away')
                return False
            await asyncio.sleep(0.1)
        return True

    async def fly_line(self, start, end, d, speed, yaw=0.0):
        '''
        Fly a straight line at a constant speed. Just setting the end point as the setpoint
        made PX4 go full speed (like 10 m/s) and tilt a ton, which messed up the tag positions
        so instead we drag a "carrot" setpoint along the line with a velocity feedforward.
        '''
        dn = end[0] - start[0]
        de = end[1] - start[1]
        length = math.hypot(dn, de)
        if length < 0.01:
            return await self.fly_to(end[0], end[1], d, yaw)
        un, ue = dn / length, de / length
        duration = length / speed

        t0 = time.time()
        while True:
            if self.battery < self.min_battery:
                self.get_logger().warn(f'battery low ({self.battery*100:.0f}%), stopping line early')
                return False
            t = time.time() - t0
            s = min(speed * t, length)
            done = s >= length
            vn, ve = (0.0, 0.0) if done else (un * speed, ue * speed)
            await self.drone.offboard.set_position_velocity_ned(
                PositionNedYaw(start[0] + un * s, start[1] + ue * s, d, yaw),
                VelocityNedYaw(vn, ve, 0.0, yaw))
            if done:
                break
            await asyncio.sleep(0.05)

        # let it catch up to the end point
        return await self.fly_to(end[0], end[1], d, yaw, timeout=10.0)

    async def search(self):
        self.set_state(State.SEARCH)
        wps = lawnmower(self.get_parameter('search_north_min').value,
                        self.get_parameter('search_north_max').value,
                        self.get_parameter('search_east_min').value,
                        self.get_parameter('search_east_max').value,
                        self.get_parameter('lane_spacing').value)
        d = -self.search_alt
        self.get_logger().info(f'search pattern: {len(wps)} waypoints, ~{path_length(wps):.0f} m')

        # get to the first corner normally
        await self.fly_to(wps[0][0], wps[0][1], d)

        prev = wps[0]
        for i, wp in enumerate(wps[1:], start=1):
            self.get_logger().info(f'  search leg {i}/{len(wps)-1} -> N {wp[0]:.1f} E {wp[1]:.1f}  '
                                   f'(tools so far: {self.tool_summary()})')
            ok = await self.fly_line(prev, wp, d, self.search_speed)
            if not ok and self.battery < self.min_battery:
                self.get_logger().warn('aborting search, low battery')
                return
            prev = wp

        self.get_logger().info(f'search done! found {len(self.tool_map)} tools: {self.tool_summary()}')

    def tool_summary(self):
        if not self.tool_map:
            return 'none'
        return ', '.join(f'{t.tool_name}({t.tag_id})' for t in self.tool_map.values())

    async def visit_tool(self, tag_id):
        if tag_id not in self.tool_map:
            self.get_logger().warn(f'tool {tag_id} not in map, skipping')
            return False

        tool = self.tool_map[tag_id]
        self.get_logger().info(f'going to {tool.tool_name} at N {tool.position.x:.2f} E {tool.position.y:.2f}')

        # fly over at search altitude first so we dont clip anything, then go down
        cur_d = min(self.ned[2], -self.visit_alt)
        await self.fly_line((self.ned[0], self.ned[1]), (tool.position.x, tool.position.y), cur_d, self.search_speed)
        await self.fly_to(tool.position.x, tool.position.y, -self.visit_alt)

        # now that we're low and not moving the registry is getting really good sightings,
        # so wait a sec and correct to the updated estimate
        await asyncio.sleep(2.0)
        tool = self.tool_map.get(tag_id, tool)
        await self.fly_to(tool.position.x, tool.position.y, -self.visit_alt, settle=True)

        self.get_logger().info(f'-- hovering over {tool.tool_name} '
                               f'(N {tool.position.x:.2f} E {tool.position.y:.2f}, '
                               f'{tool.num_sightings} sightings, spread {tool.spread_m:.2f} m)')
        # TODO: this is where the gripper/drop mechanism would go for the real drone
        await asyncio.sleep(self.visit_hover_time)
        return True

    async def visit_tools(self):
        if self.visit_all_found:
            for tid in sorted(self.tool_map.keys()):
                if tid not in self.visit_queue:
                    self.visit_queue.append(tid)

        if not self.visit_queue:
            return
        self.set_state(State.VISIT)
        while self.visit_queue:
            if self.battery < self.min_battery:
                self.get_logger().warn('low battery, skipping the rest of the tools')
                break
            tid = self.visit_queue.pop(0)
            await self.visit_tool(tid)

    async def return_home_and_land(self):
        self.set_state(State.RETURN_HOME)
        # go back up first, then home
        await self.fly_to(self.ned[0], self.ned[1], -self.search_alt, timeout=15.0)
        await self.fly_line((self.ned[0], self.ned[1]), (0.0, 0.0), -self.search_alt, self.search_speed * 1.5)
        # await self.fly_to(0.0, 0.0, -self.search_alt)
        await self.fly_to(0.0, 0.0, -self.search_alt, settle=True)

        # await self.drone.action.return_to_launch()
        # ^ RTL climbs to RTL_RETURN_ALT (30m) first and takes forever in sim. we're already home anyway

        self.set_state(State.LANDING)
        try:
            await self.drone.offboard.stop()
        except OffboardError as err:
            self.get_logger().warn(f'offboard stop: {err._result.result}')
        await self.drone.action.land()

        # wait for touchdown
        t0 = time.time()
        while self.in_air and time.time() - t0 < 60:
            await asyncio.sleep(0.5)
        self.get_logger().info('-- landed')

    async def run_mission(self):
        if not self.flight_lock.acquire(blocking=False):
            self.get_logger().warn('already flying')
            return
        try:
            await self.wait_for_connection()
            await self.preflight_and_arm()
            await self.takeoff(self.search_alt)
            await self.search()
            await self.visit_tools()
            await self.return_home_and_land()
            self.set_state(State.DONE)
            self.get_logger().info(f'MISSION COMPLETE. tools found: {self.tool_summary()}')
            self.get_logger().info('call /spacedrone/go_to_tool to go back to one')
        except Exception as e:
            self.get_logger().error(f'mission failed: {e!r}')
            await self.emergency_land()
        finally:
            self.flight_lock.release()

    async def fetch_flight(self, tag_id):
        '''separate flight to go get one tool after the main mission'''
        if not self.flight_lock.acquire(blocking=False):
            return
        try:
            await self.preflight_and_arm()
            await self.takeoff(self.search_alt)
            self.set_state(State.VISIT)
            await self.visit_tool(tag_id)
            await self.return_home_and_land()
            self.set_state(State.DONE)
        except Exception as e:
            self.get_logger().error(f'fetch flight failed: {e!r}')
            await self.emergency_land()
        finally:
            self.flight_lock.release()

    async def emergency_land(self):
        self.set_state(State.FAILED)
        try:
            await self.drone.offboard.stop()
        except Exception:
            pass
        try:
            await self.drone.action.land()
        except Exception as e:
            self.get_logger().error(f'couldnt even land: {e!r}')

    def shutdown(self):
        self.loop.call_soon_threadsafe(self.loop.stop)


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
