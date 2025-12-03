import rclpy
from rclpy.node import Node 
from tf2_ros import Buffer, TransformListener 
import tf2_ros 
from geometry_msgs.msg import TransformStamped
import csv 
import sys 
import select
import termios 
import tty


class TF2DataLogger(Node):
    """A ROS2 node that listens for a TF2 transform, saves the translation vector on 's' key press,
    and writes all data to a csv file on 'q' key press.
    """
    def __init__(self, target_frame, source_frame, csv_file_path):
        super().__init__('tf2_data_logger')

        self.TARGET_FRAME = target_frame 
        self.SOURCE_FRAME = source_frame 
        self.CSV_FILE_PATH = csv_file_path

        # setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info(f'Listening for transform from {self.SOURCE_FRAME} to {self.TARGET_FRAME}.')

        # data storage 
        self.saved_translations = []
        self.point_index = 1 

        # Setup timer for periodic transform lookup
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Keyboard Input Setup 
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.get_logger().info("Ready. Press 's' to save, 'q' to quit.")

    def get_transform(self):
        """Looks up the transform between SOURCE_FRAME and TARGET_FRAME.
        Returns the TransformStamped message or None if lookup fails.
        
        """
        try:
            t = self.tf_buffer.lookup_transform(
                self.TARGET_FRAME,
                self.SOURCE_FRAME,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.01)
            )
            return t 
        except tf2_ros.TransformException as ex:
            return None 

    def save_data(self, t):
        """Extracts and saves the translation vector.
        
        Parameters
        ----------
        t: TransformStamped
            The  transformed stamped obntained from ``get_transform``.
        
        """
        x = t.transform.translation.x
        y = t.transform.translation.y
        z = t.transform.translation.z 

        self.saved_translations.append({
            'point': self.point_index,
            'x': x,
            'y': y,
            'z': z
        })
        self.get_logger().info(f"Point {self.point_index}: Saved translation (x, y, z) = ({x:.3f}, {y:.3f}, {z:.3f})")
        self.point_index += 1

    def write_to_csv(self):
        """Write all saved data to the csv file"""

        if not self.saved_translations:
            self.get_logger().info(f"No data points to save. CSV file '{self.CSV_FILE_PATH}' will not be created.")
            return 

        keys = ['point', 'x', 'y', 'z']

        try:
            with open(self.CSV_FILE_PATH, 'w', newline='') as output_file:
                dict_writer = csv.DictWriter(output_file, fieldnames=keys)
                dict_writer.writeheader()
                dict_writer.writerows(self.saved_translations)
            self.get_logger().info(f"Succesfully wrote {len(self.saved_translations)} points to '{self.CSV_FILE_PATH}'")
        except IOError as e:
            self.get_logger().error(f"Failed to write to CSV file: {e}")

    def timer_callback(self):
        """Runs on a timer to check for keyboard input."""
        key = self.get_key()
        
        if key is None:
            return

        if key == 's':
            # Save the current translation vector
            transform = self.get_transform()
            if transform is not None:
                self.save_data(transform)
            else:
                self.get_logger().warn("Could not find transform to save data.")

        if key == 'q':
            # Quit the programme
            self.get_logger().info("Terminating program. Writing data to CSV...")
            self.write_to_csv()

            # Clean up and shutdown 
            self.restore_terminal_settings()
            self.destroy_node()
            rclpy.shutdown()

    def get_key(self):
        """Checks if a key has been pressed without blocking."""
        if sys.stdin in select.select([sys.stdin],[],[], 0)[0]:
            return sys.stdin.read(1)
        return None 
    
    def restore_terminal_settings(self):
        """Restore the terminal settings on exit."""

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main():
    rclpy.init()

    if len(sys.argv) < 4:
        print("Usage: ros2 run tf2_data_logger tf2_data_saver <source_frame> <target_frame> <csv_file_path>")
        rclpy.shutdown()
        return 

    _, SOURCE_FRAME, TARGET_FRAME, CSV_FILE_PATH = sys.argv

    node = TF2DataLogger(TARGET_FRAME, SOURCE_FRAME, CSV_FILE_PATH)

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        if rclpy.ok():
            node.restore_terminal_settings()
            node.destroy_node()
            rclpy.shutdown()
        elif 'node' in locals() and node.settings is not None:
            node.restore_terminal_settings()
    
if __name__ == '__main__':
    main()
