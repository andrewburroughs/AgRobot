import rclpy
from rclpy.node import Node
import sys, select, tty, termios
from std_msgs.msg import Float32

key_map = {
    'A': {'topic': 'talon_2_speed', 'value':  1.0}, # Up arrow
    'B': {'topic': 'talon_2_speed', 'value': -1.0}, # Down arrow
    'C': {'topic': 'talon_1_speed', 'value':  1.0}, # Right arrow
    'D': {'topic': 'talon_1_speed', 'value': -1.0}, # Left arrow
    'a': {'topic': 'motor_speed', 'value': -1.0},
    'd': {'topic': 'motor_speed', 'value': 1.0}
}

def get_key(settings):
    """Function to get a single key press or an arrow key sequence from the terminal."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = ''
    if rlist:
        char = sys.stdin.read(1)
        if char == '\x1b':
            key = char + sys.stdin.read(2)
        else:
            key = char
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init(args=args)
    node = Node('terminal_keyboard_node') 
    
    pub1 = node.create_publisher(Float32, 'talon_1_speed', 10)
    pub2 = node.create_publisher(Float32, 'talon_2_speed', 10)
    pub3 = node.create_publisher(Float32, 'motor_speed', 10)
    
    node.get_logger().info("Terminal Keyboard Node Started...")
    node.get_logger().info("Use arrow keys to control. Press 'q' to quit.")

    try:
        while rclpy.ok():
            key = get_key(settings)
            
            talon_1_speed = 0.0
            talon_2_speed = 0.0
            motor_speed = 0.0

            if key:
                if key == 'q':
                    node.get_logger().info("'q' pressed, shutting down.")
                    break
                
                if key.startswith('\x1b') and len(key) == 3:
                    final_char = key[-1]
                elif len(key) == 1:
                    final_char = key

                if final_char in key_map:
                    mapping = key_map[final_char]
                    if mapping['topic'] == 'talon_1_speed':
                        talon_1_speed = mapping['value']
                    elif mapping['topic'] == 'talon_2_speed':
                        talon_2_speed = mapping['value']
                    elif mapping['topic'] == 'motor_speed':
                        motor_speed = mapping['value']

            msg1 = Float32()
            msg1.data = talon_1_speed
            pub1.publish(msg1)

            msg2 = Float32()
            msg2.data = talon_2_speed
            pub2.publish(msg2)

            msg3 = Float32()
            msg3.data = motor_speed
            pub3.publish(msg3)

            node.get_logger().info(f'Publishing: Talon1={talon_1_speed}, Talon2={talon_2_speed}, Motor={motor_speed}')

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.get_logger().info("Restoring terminal settings and shutting down.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()