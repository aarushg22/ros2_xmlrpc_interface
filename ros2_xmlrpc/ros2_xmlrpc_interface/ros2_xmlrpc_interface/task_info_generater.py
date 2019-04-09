import json
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from sesto_api_msgs.msg import SestoApiInfo

from sesto_api_msgs.msg import ServerResponse

class Talker(Node):

    def __init__(self):
        super().__init__('talker')
        self.pub = self.create_publisher(SestoApiInfo, 'task_info')
        self.sub = self.create_subscription(ServerResponse, 'response', self.response_callback)
        # Publish the info repeatedly with the following time period(in seconds)
        timer_period = 5.0
        self.tmr = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        #Create a dummy msg with all the info
        msg = SestoApiInfo()
        msg.method="getUserReqStates"
        msg.user_id=1234
        msg.pickup="S1"
        msg.payloads=["BOX22M43"]
        msg.type=0
        msg.delivery="S2"
        msg.start_time="2018-08-31 12:07:00"
        msg.delivery_end=60
        msg.agv_id=0
        msg.requests=[]

        # publish the msg on task_info topic
        print(msg)
        self.pub.publish(msg)

    def response_callback(self, msg):
        global req_id
        res_msg={}
        print(msg.response)
        res_msg=json.loads(msg.response)
        print(msg.method)
        if msg.method=="createAdhocRequest" :
            req_id=res_msg.req_id
            print("Succesfully created task!!!!!!!!!!!!!")
            print("Request id",req_id)

def main(args=None):
    rclpy.init(args=args)

    node = Talker()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
