import rclpy 
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from some_pkg.control_gripper import GripperController
import pyrealsense2 as rs 
from diffusion_policy import DiffusionPolicy 

class ArmInterface(Node):
    def __init__(self, arm_name:str):
        super().__init__(f"{arm_name}_interface")
        self.arm_name = arm_name
        self.namespace = f"/{arm_name}_arm"
        self.fk_client = self.create_client(GetPositionFK, f"{self.namespace}/compute_fk")
        self.ik_client = self.create_client(GetPositionIK, f"{self.namespace}/compute_ik")
        self.joint_state = None
        self.force_data = None
        self.create_subscription(JointState, "/joint_states", self._joint_callback, 10)
        # for tactile you'd subscribe like
        self.create_subscription(Float32, f"{self.namespace}/tactile/finger_0", self._tactile_callback, 10)

    def _joint_callback(self, msg):
        self.joint_state = msg
    
    #Setup other subscribes like tactile and use to get data for tactile sensors

    def get_fk(self) -> Pose:
        #Get Current JointState
        #Request FK using GetPositionFK.request()
        #spin until you get fk result 
        #return position + quaternion
        pass

    def get_ik(self,target_pose: Pose)->JointState:
        #request IK using GetPositionIK.request()
        #define constraints if any and check if solution within constraints
        #spin until you get ik result
        #return joint_states
        pass

class GraspMonitor(Node):
    def __init__(self):
        super().__init__("grasp_monitor")
        # subscribe to /x_arm/tactile/finger_* topics
        # run simple slip/contact heuristics
        # publish modified gripper scalar if needed
    
class DiffusionExecutor:
    def __init__(self):
        self.left_arm = ArmInterface("left")
        self.right_arm = ArmInterface("right")
        self.left_pipeline, self.right_pipeline = self.init_cameras()
        self.left_gripper = GripperController("left")
        self.right_gripper = GripperController("right")
        self.policy = DiffusionPolicy()
        self.load_checkpoint()
        obs = self.get_obs() #first observation
    def init_cameras(self):
        #Initialize both the cameras to start collecting rgb data
        #return left_pipeline, right_pipeline
        pass

    def load_checkpoint(self):
        # load model weights from path
        pass

    def get_obs(self) -> dict:
        left_frames = self.left_pipeline.wait_for_frames()
        right_frames = self.right_pipeline.wait_for_frames()
        left_image = np.asanyarray(left_frames.get_data())
        right_image = np.asanyarray(right_frames.get_data())
        left_pose = self.left_arm.get_fk()
        right_pose = self.right_arm.get_fk()
        tactile_left = self.left_arm.tactile_data  
        tactile_right = self.right_arm.tactile_data   #get tactile observation from subscriber
        
        left_gripper_pos = self.left_gripper.get_state()
        right_gripper_pos = self.right_gripper.get_state()
        return {
            "left_image": self.process_image(left_image),
            "right_image": self.process_image(right_image),
            "left_pose": left_pose,
            "right_pose": right_pose,
            "left_gripper": left_gripper_pos,
            "right_gripper": right_gripper_pos,
            "tactile_left" : tactile_left,
            "tactile_right" : tactile_right
        }
    def process_image(self, image):
        # Crop, resize, normalize as needed
        # Return CHW format
        pass

    def execute_pose(self, action):
        #convert action to pose
        #call IK
        #send to controller
        #send gripper command
        #return new obs
        pass

    def run(self):
        obs = self.get_obs()
        for step in range(self.policy.horizon):
            action = self.policy.predict(obs)
            self.execute_pose(action)
            obs = self.get_obs()

def main():
    rclpy.init() 
    try:
        executor = DiffusionExecutor()
        executor.run()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()