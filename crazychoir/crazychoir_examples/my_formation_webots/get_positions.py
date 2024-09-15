import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray  # Use Float64MultiArray for better precision
from geometry_msgs.msg import PointStamped  # Assuming your GPS data comes in as PointStamped
import numpy as np

class GetPositions(Node):
    def __init__(self):
        super().__init__('get_positions')
  
        self.declare_parameter('n_agents', 1)
        self.n_agents = self.get_parameter('n_agents').get_parameter_value().integer_value
       
        self.publisher_ = self.create_publisher(Float64MultiArray, 'my_positions_topic', 10)

        self.timer = self.create_timer(0.5, self.timer_callback)   #svakih 0.5 sekundi
        #pozicije svih idu u X matricu
        self.X = np.zeros((self.n_agents, 3))  

   
        for i in range(self.n_agents):
            topic_name = f'/agent_{i}/agent_{i}/gps'
            self.create_subscription(
                PointStamped,  
                topic_name,
                lambda msg, idx=i: self.gps_callback(msg, idx),
                10
            )

    def gps_callback(self, msg, idx):
        self.X[idx] = np.array([msg.point.x, msg.point.y, msg.point.z])


    def timer_callback(self):
        msg = Float64MultiArray()  
        msg.data = self.X.flatten().tolist() 
        #msg.data = self.ksi_matrix
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = GetPositions()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
