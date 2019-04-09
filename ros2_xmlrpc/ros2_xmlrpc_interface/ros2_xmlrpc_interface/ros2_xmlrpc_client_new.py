import rclpy
import json
import xmlrpc.client

from rclpy.node import Node

from std_msgs.msg import String

from sesto_api_msgs.msg import SestoApiInfo

from sesto_api_msgs.msg import ServerResponse

#from atm_msgs.msg import AgvCommand

from collections import OrderedDict

# Create a XMLRPC client using the server as XMLRPC host
proxy=xmlrpc.client.ServerProxy("http://10.233.29.151:20001/RPC2")

# TODO: Create a mechanism to publish each request with a time period a at least 1 sec, so the Sesto FM doesn't get throttled

class SestoAPI(Node):

    def __init__(self):
        super().__init__('ros2_xmlrpc_client')

        # A publisher to response, and a subscriber to recieve info for making API calls
        self.sub_sesto_api = self.create_subscription(SestoApiInfo, 'call_sesto_API', self.command_callback)
        #self.sub_agv_command = self.create_subscription(AgvCommand, 'command_to_agv', self.agv_command_callback)
        self.pub_resp = self.create_publisher(ServerResponse, 'response')
        self.pub_fault = self.create_publisher(String, 'sesto_xmlrpc_fault')


    def command_callback(self, msg):
        mod_instance = globals()['SestoAPI']()
        print(msg.method)
        # Goto the required method (we are mostly concerned with createAdhocRequest only)
        func = getattr(mod_instance, msg.method)
        func(msg)

    def getAllAgvStatuses(self, msg):
        res_msg=ServerResponse()
        param={}
        param['user_id']=msg.user_id
        param['agv_id']=msg.agv_id
        param_json=json.dumps(param)
        print(param_json)
        res_msg.method="getAllAgvStatuses"
        try:
            res_msg.response=proxy.getAllAgvStatuses(param_json)
            print(res_msg)
            self.pub_resp.publish(res_msg)
        except xmlrpc.client.Fault as err:
            self.print_xmlrpc_fault(err)


    def getUserReqStates(self, msg):
        res_msg=ServerResponse()
        param=OrderedDict()
        param['user_id']=msg.user_id
        param['requests']=[]
        param['requests'].append(msg.req_id)
        param_json=json.dumps(param)
        print(param_json)
        res_msg.method="getUserReqStates"
        try:
            res_msg.response=proxy.getUserReqStates(param_json)
            #print(res_msg)
            self.pub_resp.publish(res_msg)
        except xmlrpc.client.Fault as err:
            self.print_xmlrpc_fault(err)

    def createAdhocRequest(self, msg):
        res_msg = ServerResponse()
        param = OrderedDict()

        # Extract the requiredd info
        param['user_id']=msg.user_id
        param['pickup']=msg.pickup
        param['payloads']=msg.payloads
        param['type']=msg.type
        param['delivery']=msg.delivery
        param['start_time']=msg.start_time
        param['delivery_end']=msg.delivery_end
        param_json=json.dumps(param)


        #standard call format
        #param_json="{'user_id': 1278, 'pickup': 'S1', 'payloads': ['BOX22M46'], 'type': 0,'delivery': 'S2','start_time': '2019-02-20 10:00:00', 'delivery_end': 60}"
        print(param_json)
        res_msg.method="createAdhocRequest"
        # send XMLRPC call to the server
        try:
            res_msg.response=proxy.createAdhocRequest(param_json)
            print(res_msg.response)
            # For response, publishing the req_id
            self.pub_resp.publish(res_msg)
        except xmlrpc.client.Fault as err:
            self.print_xmlrpc_fault(err)


    def cancelUserRequest(self, msg):
        res_msg=ServerResponse()
        param={}
        param['user_id']=msg.user_id
        param['req_id']=msg.agv_id
        param_json=json.dumps(param)
        print(param_json)
        res_msg.method="cancelUserRequest"
        try:
            res_msg.response=proxy.cancelUserRequest(param_json)
            self.pub_resp.publish(res_msg)
        except xmlrpc.client.Fault as err:
            self.print_xmlrpc_fault(err)

    def print_xmlrpc_fault(self, err):
        print("FAULT")
        print(err.faultCode,err.faultString)
        fault_res=String()
        fault_res.data=str(err.faultString)
        self.pub_fault.publish(fault_res)

    '''
    def setAgvPausedState(self, msg):
        #Change acc to agv_ip
        res_msg=ServerResponse()
        param={}
        param['paused']=msg.user_id
        param_json=json.dumps(param)
        print(param_json)
        res_msg.method="setAgvPausedState"
        res_msg.response=pause_proxy.setAgvPausedState(param_json)
        self.pub_resp.publish(res_msg)
    '''

def main(args=None):
    rclpy.init(args=args)
    node = SestoAPI()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
