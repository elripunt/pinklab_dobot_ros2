import sys
import rclpy as rp
from rclpy.node import Node
from dobot_ros2_interface.srv import Dashboard

class DashboardClient(Node):
    
    def __init__(self):
        super().__init__('dashboard_client')

        self.dsbd_cli = self.create_client(Dashboard, 'dashboard_server')

        while not self.dsbd_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = Dashboard.Request()

    def send_request(self):
        self.req.req_str = str(sys.argv[1])
        self.future = self.dsbd_cli.call_async(self.req)


def main(args=None):
    rp.init(args=args)
    dsbd_cli = DashboardClient()
    dsbd_cli.send_request()

    while rp.ok():
        rp.spin_once(dsbd_cli)
        if dsbd_cli.future.done():
            try:
                response = dsbd_cli.future.result()
            except Exception as e:
                dsbd_cli.get_logger().info(
                    'Service call failed'
                )
            else:
                dsbd_cli.get_logger().info(
                    'Result : {}'.format(response.res_str)
                )
            break
    
    dsbd_cli.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()