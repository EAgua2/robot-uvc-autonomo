import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Header, Int32MultiArray
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class GoalPoseSequencer(Node):
    def __init__(self):
        super().__init__('goal_pose_sequencer')

        # Lista de objetivos
        self.goal_poses = []
        self.current_index = 0

        # Cliente de acción para NavigateToPose
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Subscripción a la secuencia de poses
        self.posearray_sub = self.create_subscription(
            PoseArray,
            '/goal_poses',
            self.posearray_callback,
            10
        )

        # Publisher para progreso
        self.progress_pub = self.create_publisher(Int32MultiArray, '/disinfection_progress', 10)

        self.get_logger().info('🎯 GoalPoseSequencer activo y esperando /goal_poses')

    def posearray_callback(self, msg):
        self.goal_poses = msg.poses
        self.current_index = 0

        if not self.client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('❌ Servidor /navigate_to_pose no disponible.')
            return

        self.get_logger().info(f'📬 Recibido PoseArray con {len(self.goal_poses)} objetivos')
        self.send_next_goal()

    def send_next_goal(self):
        # Publicar progreso antes de enviar objetivo
        self.publish_progress()

        if self.current_index >= len(self.goal_poses):
            self.get_logger().info('✅ Todos los objetivos fueron completados')
            self.publish_progress()  # asegura progreso final 100%
            return

        # Prepara el siguiente objetivo
        pose = self.goal_poses[self.current_index]
        goal_msg = NavigateToPose.Goal()
        pose_stamped = PoseStamped()
        pose_stamped.header = Header()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose
        goal_msg.pose = pose_stamped

        self.get_logger().info(f'🚩 Enviando objetivo {self.current_index + 1}/{len(self.goal_poses)}')
        send_future = self.client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('⚠️ Objetivo rechazado por el servidor')
            return

        self.get_logger().info('🟢 Objetivo aceptado. Esperando resultado...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == 4:
            self.get_logger().info(f'🏁 Objetivo alcanzado (estado: {status})')
        else:
            self.get_logger().warn('❌ El objetivo fue cancelado o fallido.')

        self.current_index += 1
        self.send_next_goal()

    def publish_progress(self):
        msg = Int32MultiArray()
        msg.data = [self.current_index, len(self.goal_poses)]
        self.progress_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseSequencer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
