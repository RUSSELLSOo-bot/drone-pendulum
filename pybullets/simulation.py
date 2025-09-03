import os
import time
import pybullet as p
import pybullet_data
import numpy as np
from scipy.spatial.transform import Rotation as R

# --- TUNE THIS FOR HOVER ---
HOVER_SPEED = 399.3   # target angular velocity (rad/s) per rotor, calculated from physics
MAX_FORCE   = 1000.0   # max torque each motor can apply (increased to allow motors to reach target speed)

# Rotor indices and their initial orientations
FRONT_ROTOR = 1  
LEFT_ROTOR = 3  
BACK_ROTOR = 5   
RIGHT_ROTOR = 7 

ROTOR_JOINTS = [FRONT_ROTOR, LEFT_ROTOR, BACK_ROTOR, RIGHT_ROTOR]

# Initial orientations in Euler angles (roll, pitch, yaw)
ROTOR_ORIENTATIONS = {
    FRONT_ROTOR: [0, 1.5708, 0],    # 90° around Y
    LEFT_ROTOR: [1.5708, 0, 0],     # 90° around X
    BACK_ROTOR: [0, 1.5708, 0],     # 90° around Y
    RIGHT_ROTOR: [1.5708, 0, 0]     # 
}
motor_constant = 9.9865e-06
rotor_drag_coefficient = 8.06428e-05

A_face = np.pi * 0.128 **2 # (m)
A_side = 2 * 0.128 * 0.01  # projected area of the rotor in side view

class DroneController:
    def __init__(self, drone_id):
        self.drone_id = drone_id

    def set_rotor_speeds(self, speeds):
        """Apply a list of target velocities to each rotor joint."""
        for joint_idx, speed in zip(ROTOR_JOINTS, speeds):
            p.changeDynamics(self.drone_id, joint_idx, maxJointVelocity=2000)
            p.setJointMotorControl2(
                bodyIndex=self.drone_id,
                jointIndex=joint_idx,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=speed,
                force=MAX_FORCE,
            )

    def apply_prop_force(self):
        for joint_idx in ROTOR_JOINTS:
            joint_state = p.getJointState(self.drone_id, joint_idx)
            omega = joint_state[1]
            thrust = motor_constant * omega**2

            arm_state = p.getLinkState(self.drone_id, joint_idx, computeForwardKinematics=True, computeLinkVelocity=True)
            pos_world = arm_state[0]
            orientation_world = p.getMatrixFromQuaternion(arm_state[1])
            thrust_dir = np.array([orientation_world[2], orientation_world[5], orientation_world[8]])
            force_thrust = thrust * thrust_dir

            lin_vel = arm_state[6]
            speed = np.linalg.norm(lin_vel)
            if speed > 1e-3:
                v_dir = lin_vel / speed
                cos_theta = np.dot(v_dir, thrust_dir)
                A_proj = abs(cos_theta)*A_face + np.sqrt(max(0,1 - cos_theta**2))*A_side
                drag_mag = rotor_drag_coefficient * A_proj * speed**2
                force_drag = -drag_mag * v_dir
            else:
                force_drag = np.zeros(3)

            force_total = force_thrust + force_drag
            p.applyExternalForce(
                objectUniqueId=self.drone_id,
                linkIndex=joint_idx,
                forceObj=force_total.tolist(),
                posObj=pos_world,
                flags=p.WORLD_FRAME
            )

    def stabilize_orientation(self, keepCurrentYaw = True):
        _, orientation = p.getBaseOrientation(self.drone_id)
        curr_rotation = R.from_quat(orientation) #creates rotation obj form quat

        roll, pitch, yaw = curr_rotation.as_euler('xyz', degrees = False) #radians

        if keepCurrentYaw:
            desired_rotation = R.from_euler('xyz', [0.0, 0.0, yaw]) #creates rot obj from euler
        else:
            desired_rotation = R.from_euler('xyz', [0.0, 0.0, 0.0])


def main():
    physicsClient = p.connect(p.GUI_SERVER)
    p.setPhysicsEngineParameter(fixedTimeStep=1.0/240.0)
    p.setPhysicsEngineParameter(numSolverIterations=50)
    p.setPhysicsEngineParameter(numSubSteps=4)

    p.resetDebugVisualizerCamera(
        cameraDistance=3.0,
        cameraYaw=45.0,
        cameraPitch=-30.0,
        cameraTargetPosition=[0, 0, 0]
    )

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    script_dir = os.path.dirname(os.path.realpath(__file__))
    urdf_dir = os.path.join(script_dir, "urdf")
    mesh_dir = os.path.join(script_dir, "meshes")
    p.setAdditionalSearchPath(mesh_dir)

    plane_urdf = os.path.join(pybullet_data.getDataPath(), "plane.urdf")
    plane_id = p.loadURDF(plane_urdf)

    pelican_urdf = os.path.join(urdf_dir, "pelican.urdf")
    start_pos = [0, 0, 1]
    start_ori = p.getQuaternionFromEuler([0, 0, 0])

    try:
        drone_id = p.loadURDF(pelican_urdf, start_pos, start_ori, useFixedBase=False)
    except p.error as e:
        print(f"PyBullet error loading URDF: {e}")
        return

    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)

    controller = DroneController(drone_id)

    try:
        print("Starting simulation loop...")
        while True:
            rotor_speeds = [
                -HOVER_SPEED,
                HOVER_SPEED,
                -HOVER_SPEED,
                HOVER_SPEED,
            ]
            controller.set_rotor_speeds(rotor_speeds)
            controller.apply_prop_force()

            p.stepSimulation()
            time.sleep(1/300.0)

            try:
                connection_info = p.getConnectionInfo()
                if not connection_info['isConnected']:
                    print("Simulation disconnected - thread terminated")
                    break

                if not p.isConnected():
                    print("GUI window closed - thread terminated")
                    break

                keys = p.getKeyboardEvents()
                cam_data = p.getDebugVisualizerCamera()
                dist = cam_data[10]
                yaw = cam_data[8]
                pitch = cam_data[9]
                target = list(cam_data[11])

                if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
                    p.resetDebugVisualizerCamera(
                        cameraDistance=3.0,
                        cameraYaw=45.0,
                        cameraPitch=-30.0,
                        cameraTargetPosition=[0, 0, 0]
                    )
                    print("Camera reset to default position")

                time.sleep(1.0/240.0)

            except p.error as e:
                print(f"PyBullet error: {e}")
                print("Thread terminated due to error")
                break
            except Exception as e:
                print(f"Unexpected error: {e}")
                print("Thread terminated due to error")
                break

    except KeyboardInterrupt:
        print("Simulation interrupted by user - thread terminated")
    finally:
        try:
            if p.isConnected():
                print("Cleaning up PyBullet connection...")
                p.disconnect()
                print("PyBullet disconnected successfully")
            else:
                print("PyBullet already disconnected")
        except Exception as e:
            print(f"Error during cleanup: {e}")
        print("Simulation thread fully terminated")

if __name__ == "__main__":
    main()