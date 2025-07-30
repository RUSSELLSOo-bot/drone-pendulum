import os
import time
import pybullet as p
import pybullet_data

# --- TUNE THIS FOR HOVER ---
HOVER_SPEED = 4000.0   # target angular velocity (rad/s) per rotor
MAX_FORCE   = 10.0     # max torque each motor can apply



ROTOR_JOINTS = [1, 3, 5, 7]
motor_constant = 9.9865e-06
rotor_drag_coefficient = 8.06428e-05

def set_rotor_speeds(drone_id, speeds):
    """Apply a list of target velocities to each rotor joint."""
    for joint_idx, speed in zip(ROTOR_JOINTS, speeds):
        p.setJointMotorControl2(
            bodyIndex=drone_id,
            jointIndex=joint_idx,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=speed,
            force=MAX_FORCE
        )


def apply_prop_force(drone_id):
    for joint_idx in ROTOR_JOINTS:
        omega = p.getJointState(drone_id, joint_idx) # Get current angular velocity


def main():
    # 1. Connect to PyBullet physics server (GUI)
    physicsClient = p.connect(p.GUI_SERVER)  # Use GUI_SERVER for better mouse control
    

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
    p.setRealTimeSimulation(1)

    # 7. Main simulation loop
    try:
        print("Starting simulation loop...")
        while True:
            # Front (0) and Back (2) rotors spin clockwise
            # Left (1) and Right (3) rotors spin counter-clockwise
            rotor_speeds = [
                -HOVER_SPEED,    # Front rotor (clockwise)
                HOVER_SPEED,   # Left rotor (counter-clockwise)
                -HOVER_SPEED,    # Back rotor (clockwise)
                HOVER_SPEED,   # Right rotor (counter-clockwise)
            ]
            set_rotor_speeds(drone_id, rotor_speeds)

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