import argparse
import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetModelStateResponse, GetModelPropertiesResponse

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model_name", required=True)
    return parser.parse_args()


class ContinuousModelQueryNode:
    def __init__(self, model_name, rate=10):
        rospy.init_node(self.__class__.__name__)
        self.query_service_proxy = rospy.ServiceProxy("/gazebo/get_model_state",GetModelState)
        self.request = GetModelStateRequest()
        self.request.model_name = model_name
        self.request.relative_entity_name = ""
    
    def format_response(self, resp: GetModelStateResponse):
        px = resp.pose.position.x
        py = resp.pose.position.y
        pz = resp.pose.position.z
        return (px, py, pz)
     
    def run(self):
        poses = []
        while not rospy.is_shutdown():
            pose = self.format_response(self.query_service_proxy.call(self.request))
            if not pose in poses:
                poses.append(pose)
                print(poses, end="\r")

def main():
    args = get_args()
    model_name = args.model_name
    node = ContinuousModelQueryNode(model_name)
    node.run()

if __name__ == "__main__":
    main()