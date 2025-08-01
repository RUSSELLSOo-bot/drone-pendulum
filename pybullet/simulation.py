import os
import time
import pybullet as p
import pybullet_data
import numpy as np

# --- TUNE THIS FOR HOVER ---
HOVER_SPEED = 400.0   # target angular velocity (rad/s) per rotor, calculated from physics
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
    RIGHT_ROTOR: [1.5708, 0, 0]     # 90° around X
}
motor_constant = 9.9865e-06
rotor_drag_coefficient = 8.06428e-05

def set_rotor_speeds(drone_id, speeds):
    """Apply a list of target velocities to each rotor joint."""
    for joint_idx, speed in zip(ROTOR_JOINTS, speeds):
        # First disable the motor to remove any velocity constraint
        p.changeDynamics(drone_id, joint_idx, maxJointVelocity=2000)  

        # Then set the new target velocity with maximum force
        p.setJointMotorControl2(
            bodyIndex=drone_id,
            jointIndex=joint_idx,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=speed,
            force=MAX_FORCE,
        )


def apply_prop_force(drone_id):
    for joint_idx in ROTOR_JOINTS:
        joint_state = p.getJointState(drone_id, joint_idx) # Get current joint state
        omega = joint_state[1]  # Get angular velocity from joint state
        
   
        thrust = motor_constant * omega**2  # Calculate thrust force
       

        arm_state = p.getLinkState(drone_id, joint_idx, computeForwardKinematics=True)
        pos_world = arm_state[0]  # position within the link frame (x,y,z)
        orientation_world = p.getMatrixFromQuaternion(arm_state[1])  # orientation in world frame (3x3)
        
        thrust_dir = np.array([orientation_world[2], orientation_world[5], orientation_world[8]])  # z direction of the link local frame
        
        
        # Calculate force vector by multiplying thrust magnitude with direction
        force = thrust * thrust_dir  # force vector in world frame
        
        
        # Apply the force at the rotor position in world frame
        p.applyExternalForce(
            objectUniqueId=drone_id,
            linkIndex=joint_idx,
            forceObj=force.tolist(),  # Convert numpy array to list
            posObj=pos_world,
            flags=p.WORLD_FRAME)


def main():
    # 1. Connect to PyBullet physics server (GUI)
    physicsClient = p.connect(p.GUI_SERVER)  # Use GUI_SERVER for better mouse control
    
    # Configure physics simulation
    p.setPhysicsEngineParameter(fixedTimeStep=1.0/240.0)  # Higher frequency updates
    p.setPhysicsEngineParameter(numSolverIterations=50)   # More accurate physics
    p.setPhysicsEngineParameter(numSubSteps=4)            # More substeps for stability
    

    # Set camera position and parameters
    p.resetDebugVisualizerCamera(
        cameraDistance=3.0,  # Distance from camera to target
        cameraYaw=45.0,     # Rotation around the vertical axis in degrees
        cameraPitch=-30.0,  # Rotation around the horizontal axis in degrees
        cameraTargetPosition=[0, 0, 0]  # Point the camera is looking at
    )

    # 2. Add search paths for URDF and meshes
    #    plane.urdf comes from pybullet_data; meshes are in 'meshes/'
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # for plane.urdf
    script_dir = os.path.dirname(os.path.realpath(__file__))
    urdf_dir = os.path.join(script_dir, "urdf")
    mesh_dir = os.path.join(script_dir, "meshes")
    p.setAdditionalSearchPath(mesh_dir)

    # 3. Load ground plane (explicit full path)
    plane_urdf = os.path.join(pybullet_data.getDataPath(), "plane.urdf")
    plane_id = p.loadURDF(plane_urdf)

    # 4. Load the Pelican drone URDF
    pelican_urdf = os.path.join(urdf_dir, "pelican.urdf")
    
    # print(f"Looking for URDF at: {pelican_urdf}")
    # if not os.path.exists(pelican_urdf):
    #     print(f"Error: URDF file not found!")
    #     print(f"Contents of {urdf_dir}:")
    #     for file in os.listdir(urdf_dir):
    #         print(f"  - {file}")
    #     return
    
    start_pos = [0, 0, 1]
    start_ori = p.getQuaternionFromEuler([0, 0, 0])

    try:
        drone_id = p.loadURDF(pelican_urdf, start_pos, start_ori, useFixedBase=False)
    except p.error as e:
        print(f"PyBullet error loading URDF: {e}")
        return

    # # 5. Print joint & link information
    # print("=== Drone Joint and Link Info ===")
    # for i in range(p.getNumJoints(drone_id)):
    #     info = p.getJointInfo(drone_id, i)
    #     joint_name = info[1].decode('utf-8')
    #     link_name = info[12].decode('utf-8')
    #     print(f"Index {i}: joint='{joint_name}', link='{link_name}'")

    # 6. Configure gravity and real-time simulation
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)  # Disable real-time simulation for more control
     
 


    # 7. Main simulation loop
    try:
        print("Starting simulation loop...")
        while True:
            # Set rotor speeds and apply forces
            rotor_speeds = [
                -HOVER_SPEED,    # Front rotor (clockwise)
                HOVER_SPEED,     # Left rotor (counter-clockwise)
                -HOVER_SPEED,    # Back rotor (clockwise)
                HOVER_SPEED,     # Right rotor (counter-clockwise)
            ]
            set_rotor_speeds(drone_id, rotor_speeds)
            apply_prop_force(drone_id)

            p.stepSimulation()  
            time.sleep(1/240.0)
            
            try:
                # Check connection status
                connection_info = p.getConnectionInfo()
                if not connection_info['isConnected']:
                    print("Simulation disconnected - thread terminated")
                    break
                
                # Check if GUI is still running
                if not p.isConnected():
                    print("GUI window closed - thread terminated")
                    break
                
                # Handle keyboard events for camera control
                keys = p.getKeyboardEvents()
                
                # Get current camera info
                cam_data = p.getDebugVisualizerCamera()
                dist = cam_data[10]
                yaw = cam_data[8]
                pitch = cam_data[9]
                target = list(cam_data[11])
                
                # Camera controls
                if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:  # Reset camera
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