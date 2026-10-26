#!/usr/bin/env python3
"""Publish incremental forward speed and yaw rate as TwistStamped."""

import os
import select
import sys
import termios
import tty

ROS_VERSION = os.environ.get('ROS_VERSION')
if ROS_VERSION == '1':
    import rospy
    from geometry_msgs.msg import TwistStamped
elif ROS_VERSION == '2':
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import TwistStamped
else:
    sys.exit('Source a ROS1 or ROS2 environment first (ROS_VERSION must be 1 or 2).')


if __name__ == '__main__':
    if not sys.stdin.isatty():
        sys.exit('Run this keyboard publisher in an interactive terminal.')

    if ROS_VERSION == '1':
        rospy.init_node('aliengo_keyboard', disable_signals=True)
        topic = rospy.get_param('~cmd_topic', '/cmd_planner')
        publisher = rospy.Publisher(topic, TwistStamped, queue_size=1)
    else:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('aliengo_keyboard')
        topic = node.declare_parameter('cmd_topic', '/cmd_planner').value
        publisher = node.create_publisher(TwistStamped, topic, 1)

    # Integer steps avoid floating point drift after opposite key presses.
    forward_steps = 0
    yaw_steps = 0
    msg = TwistStamped()
    msg.header.frame_id = 'base_link'
    terminal_settings = termios.tcgetattr(sys.stdin)
    print('W/S: forward/back +/-0.05 m/s; A/D: left/right yaw +/-0.05 rad/s')
    print('Space: zero both; Q or Ctrl-C: zero and quit. Holding a key repeats steps.')
    print(f'Publishing to {topic} at 50 Hz. Command: 0.00,0.00', flush=True)
    try:
        tty.setcbreak(sys.stdin.fileno())
        # Publish the initial zero command before accepting input.
        if ROS_VERSION == '1':
            msg.header.stamp = rospy.Time.now()
        else:
            msg.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(msg)

        while True:
            if ROS_VERSION == '1':
                if rospy.is_shutdown():
                    break
            else:
                if not rclpy.ok():
                    break
                rclpy.spin_once(node, timeout_sec=0.0)

            # Wall-time polling keeps publishing when no keys are being pressed.
            readable, _, _ = select.select([sys.stdin], [], [], 0.02)
            if readable:
                key = os.read(sys.stdin.fileno(), 1).decode('ascii', errors='ignore').lower()
                if key in ('q', '\x03') or not key:
                    break
                if key == 'w':
                    forward_steps += 1
                elif key == 's':
                    forward_steps -= 1
                elif key == 'a':
                    yaw_steps += 1
                elif key == 'd':
                    yaw_steps -= 1
                elif key == ' ':
                    forward_steps = yaw_steps = 0
                print(f'Command: {forward_steps * 0.05:.2f},{yaw_steps * 0.05:.2f}', flush=True)

            msg.twist.linear.x = forward_steps * 0.05
            msg.twist.angular.z = yaw_steps * 0.05
            if ROS_VERSION == '1':
                msg.header.stamp = rospy.Time.now()
            else:
                msg.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(msg)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal_settings)
        # Best effort final stop; the controller's 100 ms watchdog covers disconnects.
        try:
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.0
            if ROS_VERSION == '1':
                if not rospy.is_shutdown():
                    msg.header.stamp = rospy.Time.now()
                    publisher.publish(msg)
            elif rclpy.ok():
                msg.header.stamp = node.get_clock().now().to_msg()
                publisher.publish(msg)
        finally:
            if ROS_VERSION == '1':
                rospy.signal_shutdown('Keyboard publisher stopped')
            else:
                node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
