import numpy as np
from time import time  # https://realpython.com/python-time-module/
from functools import partial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from crazyswarm_application.msg import AgentState, AgentsStateFeedback, UserCommand


class planner_ROS(Node):
    def __init__(
        self, policy_net, k_size, device="cpu", greedy=False, save_image=False
    ):
        super().__init__(
            "planner_ROS",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        #####
        # Example agent list
        #####

        self.agent_list = {
            "cf1": {
                "executing_waypoint": False,  # True if agent is in the process of executing waypoint, else False.
                "current_waypoint": None,  # Current wayoint being executed
                "flight_state": None,  # Current state of agent
                "actual_pos": self.create_posestamped(
                    x=0.0, y=0.0, z=0.0
                ),  # Current actual position
                "waypoint_queue": [
                    self.create_posestamped(x=1.0, y=0.0, z=1.0, yaw=0.0),
                    self.create_posestamped(x=2.0, y=0.0, z=1.0, yaw=0.0),
                    self.create_posestamped(x=2.0, y=1.0, z=1.0, yaw=0.0),
                    self.create_posestamped(x=1.0, y=1.0, z=1.0, yaw=0.0),
                    self.create_posestamped(x=1.0, y=0.0, z=1.0, yaw=0.707),
                ],  # List of waypoints to fulfill
                "time_at_last_cmd": time(),  # Time elapsed since last waypoint issued
            },
        }

        #####
        # Parameters
        #####
        self.agent_timeout = 10  # Timeout for agent before it is removed
        self.pub_wp_timer_period = 1.0  # Time period for publishing waypoint callback (seconds)
        self.check_agent_timer_period = 1.0  # Time period for checking if agent has fulfilled goals
        self.goal_tolerance = 0.1  # Distance between goal and actual pose for goal to be fulfilled (meters)

        #####
        # Create publishers
        #####
        self.usercommand_pub = self.create_publisher(UserCommand, "/user", 10)

        #####
        # Create subscribers
        #####
        self.agent_state_sub = self.create_subscription(
            AgentsStateFeedback, "/agents", self.agent_state_callback, 10
        )

        self.agent_pose_sub = []

        for agent_id in list(self.agent_list.keys()):
            self.agent_pose_sub.append(
                self.create_subscription(
                    PoseStamped,
                    agent_id + "/pose",
                    partial(self.agent_pose_callback, agent_id=agent_id),
                    10,
                )
            )

        # All drones takeoff
        # self.takeoff_all()

        #####
        # Create timers
        #####

        self.pub_wp_timer = self.create_timer(
            self.pub_wp_timer_period, self.publish_waypoints
        )

        self.check_agent_timer = self.create_timer(
            self.check_agent_timer_period, self.check_agent_callback
        )


    def agent_pose_callback(self, pose, agent_id):
        """Subscriber callback to save the actual pose of the agent 

        Parameters
        ----------
        pose : geometry_msgs.msg.PoseStamped
            PoseStamped message
        agent_id : String
            Id of Agent
        """
        if agent_id in self.agent_list:
            self.agent_list[agent_id]["actual_pos"] = pose

    def check_agent_callback(self):
        """Check if agents has fulfilled waypoint. IF so, prepare it for the next waypoint 
        """
        for item in self.agent_list.items():
            agent_id = item[0]
            agent = item[1]

            # Check if agent has fulfilled goal
            # If so, set "executing_waypoint" to FALSE so that agent is ready to accept new waypoints
            if agent["executing_waypoint"] and agent["current_waypoint"]:
                dist_to_goal = self.get_euclid_dist(agent["actual_pos"].pose.position, agent["current_waypoint"].goal)
                
                self.get_logger().info(f"Agent {agent_id} has {dist_to_goal}m left to goal")

                if dist_to_goal < self.goal_tolerance:
                    self.get_logger().info(f"Agent {agent_id} has fulfilled waypoint, ready to accept next waypoint")
                    agent["executing_waypoint"] = False
                    agent["current_waypoint"] = None

            # self.get_logger().info(
            #     f"""{agent_id} \n
            #     flight_state: {agent['flight_state']}
            #     actual_pos: {agent['actual_pos']}
            #     time_at_last_cmd: {agent['time_at_last_cmd']}
            #     connected: {agent['connected']}
            #     completed: {agent['completed']}
            #     mission_capable: {agent['mission_capable']}
            #     """
            # )

    def agent_state_callback(self, agent_states):
        time_now = time()

        for agent in agent_states.agents:
            if agent.id not in self.agent_list:
                self.get_logger().error(
                    f"Agent {agent.id} has been removed from agent list. Ignoring..."
                )
                continue

            self.agent_list[agent.id]["flight_state"] = agent.flight_state

            # # Remove agents not in the desired state
            # if agent.flight_state not in {
            #     AgentState.MOVE,
            #     AgentState.IDLE,
            #     AgentState.TAKEOFF,
            #     AgentState.HOVER
            # }:
            #     print(
            #         f"Agent {agent.id} state is not in desired state, currently {agent.flight_state}"
            #     )
            #     # self.remove_agent_from_list(agent.id)

            # Check if mission has timed out
            # execution_time_elapsed = time_now - self.agent_list[agent.id]["time_at_last_cmd"]
            # if execution_time_elapsed > self.agent_timeout:
            #     self.get_logger().error(f"Agent {agent.id} exceeded timeout of {self.agent_timeout}s in executing mission. Removing from agent list")
            #     self.remove_agent_from_list(agent.id)
            # else:
            #     self.get_logger().info(f"Agent {agent.id} is {execution_time_elapsed}s into executing mission")

    def publish_waypoints(self):

        for item in self.agent_list.items():  # Iterate through list of available agents
            agent_id = item[0]
            agent = item[1]

            # IF agent is still executing current waypoint, then keep publishing the current one
            if agent["executing_waypoint"]:
                # self.get_logger().info(
                #     f"Agent {agent_id} has not finished executing waypoints, no new waypoints will be sent."
                # )
                if not agent["current_waypoint"]:
                    self.get_logger().info("Agent {agent_id} has no current waypoint to follow!")
                
                self.usercommand_pub.publish(agent["current_waypoint"])

            # Send a new waypoint from the existing waypoint_queue
            elif (len(agent["waypoint_queue"]) > 0):
                current_wp = agent["waypoint_queue"][0]

                waypoint = self.create_usercommand(
                    cmd = "goto_velocity",
                    uav_id = [agent_id],
                    goal = Point(
                        x = current_wp.pose.position.x,
                        y = current_wp.pose.position.y,
                        z = current_wp.pose.position.z,
                    ),
                    yaw = self.quaternion_to_euler(current_wp.pose.orientation)[2],
                    is_external=True,
                )

                self.usercommand_pub.publish(waypoint)

                agent["executing_waypoint"] = True
                agent["current_waypoint"] = waypoint
                # Pop the first waypoint from the queue
                agent["waypoint_queue"].pop(0)
                # Refresh time since last waypoint was published
                agent["time_at_last_cmd"] = time()

                self.get_logger().info(
                    "Published new waypoint ({},{},{}) with yaw {} for agent {}".format(
                        waypoint.goal.x,
                        waypoint.goal.y,
                        waypoint.goal.z,
                        waypoint.yaw,
                        agent_id,
                    )
                )

    # def waypoint_pub(self, waypoints_list):

    #     for name, _ in self.agent_list.items():
    #         if (
    #             self.waypoint[name] is not None and self["actual_pos"][name] is not None
    #         ):  # only publish if waypoint is set and position is known

    #             if np.linalg.norm(self.waypoint[name] - self["actual_pos"][name]) <= 0.5:
    #                 waypoint = UserCommand()

    #                 waypoint.cmd = "goto_velocity"
    #                 waypoint.uav_id.append(name)  # cf1
    #                 waypoint.goal.x = (
    #                     (640 - self.waypoint[name][0][0]) * 5 / 640
    #                 )  # change the distance
    #                 waypoint.goal.y = (
    #                     (480 - self.waypoint[name][0][1]) * 5 / 480
    #                 )  # change the distance
    #                 waypoint.goal.z = 2
    #                 yaw = self.waypoint[name][1] * np.pi / 2 - np.pi / 4
    #                 waypoint.goal.yaw = yaw

    #                 self.publisher.publish(waypoint)

    #                 self.get_logger().info(
    #                     "Publishing waypoint: {},{},{} with yaw {} for crazyflie {}".format(
    #                         waypoint.goal.x,
    #                         waypoint.goal.y,
    #                         waypoint.goal.z,
    #                         waypoint.goal.yaw,
    #                         name,
    #                     )
    #                 )
    #                 self.waypoint[name] = ()
    #                 continue

    #     for name, _ in self.agent_list.items():  # Need a flag to send next waypoint
    #         if self.waypoint[name] is None:
    #             self.get_logger().info("Calculating next waypoint")
    #             (
    #                 next_position_list,
    #                 done,
    #             ) = self.get_next_position()  # To replace with fake setpoints
    #             for i, position in enumerate(self.all_robot_positions):
    #                 name = self.robot_list[i].cf_id
    #                 for node in self.env.map.nodes_list:
    #                     if (node.coords == position).all():
    #                         heading = node.current_heading
    #                 self.waypoint[name] = [position, heading]

    # if done:
    #     rclpy.shutdown()

    def get_euclid_dist(self, point_a, point_b):
        """Get the 3D euclidean distance between 2 geometry_msgs.msg.Point messages

        Parameters
        ----------
        point_a : geometry_msgs.msg.Point
            Origin Point
        point_b : geometry_msgs.msg.Point
            Goal Point

        Returns
        -------
        float
            Distance between point a and b
        """
        dx = point_a.x - point_b.x
        dy = point_a.y - point_b.y
        dz = point_a.z - point_b.z

        return np.sqrt(dx * dx + dy * dy + dz * dz)

    def remove_agent_from_list(self, agent_id):
        """Remove a crazyflie agent from the internal list

        Parameters
        ----------
        agent_id : String
            ID of crazyflie agent to be removed
        """
        try:
            self.agent_list.pop(agent_id)
        except KeyError:
            self.get_logger().info(f"Unable to remove agent {agent_id}")

    def takeoff_all(self):
        """Helper method to command all crazyflies to take off
        """
        waypoint = self.create_usercommand(
            cmd="takeoff_all",
            uav_id=[],
        )

        self.usercommand_pub.publish(waypoint)

        self.get_logger().info("Commanded all crazyflies to take off")

    def create_posestamped(self, x, y, z, yaw=0.0):
        """Helper method to create a geometry_msgs.msg.PoseStamped message
        """
        pose = PoseStamped()
        pose.header.frame_id = ""
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position= Point(
            x=x,
            y=y,
            z=z
        )

        pose.pose.orientation = self.quaternion_from_euler(
            0, 0, -np.radians(yaw)
        )

        return pose

    def create_usercommand(self, cmd, uav_id, goal=Point(), yaw=0.0, is_external=False):
        """Helper method to create a UserCommand message, used to send drone commands

        Parameters
        ----------
        cmd : String
            Command type
        uav_id : String
            Crazyflie agent ID
        goal : geometry_msgs.msg.Point()
            (x,y,z) goal
        yaw : float, optional
            yaw, by default 0.0
        is_external : bool, optional
            TODO, by default False

        Returns
        -------
        crazyswarm_application.msg.UserCommand()
            UserCommand message
        """
        usercommand = UserCommand()

        usercommand.cmd = cmd
        usercommand.uav_id = uav_id
        usercommand.goal = goal
        usercommand.yaw = yaw
        usercommand.is_external = is_external

        return usercommand

# Converts ROS2 Quaterion message to yaw

    def quaternion_to_euler(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return [roll, pitch, yaw]

    def quaternion_from_euler(self, ai, aj, ak):
        """Converts euler values to a ROS Quaternion message

        Parameters
        ----------
        ai : float
            roll (radians)
        aj : float
            pitch (radians)
        ak : float
            yaw (radians)

        Returns
        -------
        geometry_msgs.msg.Quaternion
            quaternion ROS message type
        """
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = np.cos(ai)
        si = np.sin(ai)
        cj = np.cos(aj)
        sj = np.sin(aj)
        ck = np.cos(ak)
        sk = np.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))

        return Quaternion(
            x = cj*sc - sj*cs,
            y = cj*ss + sj*cc,
            z = cj*cs - sj*sc,
            w = cj*cc + sj*ss,
        )

def main():
    rclpy.init(args=None)
    policy_net = None
    k_size = None
    device = None

    planner_ros = planner_ROS(
        policy_net=policy_net, k_size=k_size, device=device, greedy=True
    )

    rclpy.spin(planner_ros)  # Keep node alive

    planner_ros.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
