import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


# joint -> speed (rad/s, m/s for the rail)
SPEEDS = {
    'rail_joint': 0.05,
    'shoulder_joint': 0.3,
    'elbow_joint': 0.3,
    'forearm_roll_joint': 0.3,
    'wrist_pitch_joint': 0.3,
    'wrist_roll_joint': 0.3,
}
MOVE_TIME = 1.0   # seconds to move in each direction
PAUSE = 0.5       # seconds to rest between joints


def main():

    rclpy.init()
    node = Node('all_joints_up_down')

    # One publisher per joint, stored in a dict so we can look them up by name.
    pubs = {}
    for joint in SPEEDS:
        pubs[joint] = node.create_publisher(Float64, f'/arm/{joint}/cmd_vel', 10)

    print('Waiting for the bridge...')
    for joint, pub in pubs.items():
        while pub.get_subscription_count() == 0:
            time.sleep(0.1)
        print(f'  {joint} connected')

    for joint, speed in SPEEDS.items():
        pub = pubs[joint]

        print(f'{joint}: +')
        pub.publish(Float64(data=speed))
        time.sleep(MOVE_TIME)

        print(f'{joint}: -')
        pub.publish(Float64(data=-speed))
        time.sleep(MOVE_TIME)

        print(f'{joint}: stop')
        pub.publish(Float64(data=0.0))
        time.sleep(PAUSE)

    # Safety: make sure everything is stopped before we exit.
    for pub in pubs.values():
        pub.publish(Float64(data=0.0))
    time.sleep(0.2)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()