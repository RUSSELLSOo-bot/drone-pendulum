import os
import time
import pybullet as p
import pybullet_data
import numpy as np
from scipy.spatial.transform import Rotation as R
import threading
import gymnasium as gym
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import EvalCallback
import matplotlib.pyplot as plt  # NEW: For graphing
from collections import defaultdict  # NEW: For data storage

# --- TUNE THIS FOR HOVER ---
HOVER_SPEED = 399   # target angular velocity (rad/s) per rotor, calculated from physics
MAX_FORCE   = 1000.0   # max torque each motor can apply (increased to allow motors to reach target speed)

FRONT_ROTOR = 1  
LEFT_ROTOR = 3  
BACK_ROTOR = 5   
RIGHT_ROTOR = 7 

ROTOR_JOINTS = [FRONT_ROTOR, LEFT_ROTOR, BACK_ROTOR, RIGHT_ROTOR]

ROTOR_ORIENTATIONS = {
    FRONT_ROTOR: [0, 1.5708, 0],
    LEFT_ROTOR: [1.5708, 0, 0],
    BACK_ROTOR: [0, 1.5708, 0],
    RIGHT_ROTOR: [1.5708, 0, 0]
}
motor_constant = 9.9865e-06
rotor_drag_coefficient = 8.06428e-05
A_face = np.pi * 0.128 **2
A_side = 2 * 0.128 * 0.01

class DroneController:
    def __init__(self, drone_id):
        self.drone_id = drone_id
        self.initial_pos = [0, 0, 1]
        self.initial_ori = p.getQuaternionFromEuler([0, 0, 0])
        self.Kp_orientation = np.array([0.1, 0.1, 0.1])
        self.Ki_orientation = np.array([0.0, 0.0, 0.0])
        self.Kd_orientation = np.array([0.05, 0.05, 0.05])
        self.orientation_integral = np.zeros(3)
        self.last_orientation_error = np.zeros(3)
        self.last_time = time.time()
        self.Kp_slider = p.addUserDebugParameter("Kp", 0, 50.0, 50.0)
        self.Ki_slider = p.addUserDebugParameter("Ki", 0, 50.0, 0.01)
        self.Kd_slider = p.addUserDebugParameter("Kd", 0, 20.0, 1.0)
        self.wind_toggle = p.addUserDebugParameter("Wind On/Off", 0, 1, 0)
        self.wind_strength_slider = p.addUserDebugParameter("Wind Strength", 0, 5.0, 1.0)
        self.wind_enabled = False
        self.wind_strength = 1.0
        self.wind_frequency = 0.5
        self.last_wind_update = time.time()
        self.current_wind = np.zeros(3)
        
        # NEW: Data recording variables for 10Hz recording
        self.recording_data = defaultdict(list)
        self.recording_start_time = time.time()
        self.last_record_time = 0
        self.record_interval = 0.1  # 10Hz = 0.1 second intervals

    def set_rotor_speeds(self, speeds):
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
            thrust = abs(motor_constant * omega**2)
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

    def apply_wind_forces(self):
        current_time = time.time()
        wind_enabled = p.readUserDebugParameter(self.wind_toggle) > 0.5
        wind_strength = p.readUserDebugParameter(self.wind_strength_slider)
        if not wind_enabled:
            return
        if current_time - self.last_wind_update > self.wind_frequency:
            self.current_wind = np.array([
                np.random.normal(0, wind_strength * 0.5),
                np.random.normal(0, wind_strength * 0.5),
                np.random.normal(0, wind_strength * 0.1)
            ])
            self.last_wind_update = current_time
        turbulence = np.random.normal(0, wind_strength * 0.1, 3)
        total_wind = self.current_wind + turbulence
        p.applyExternalForce(
            objectUniqueId=self.drone_id,
            linkIndex=-1,
            forceObj=total_wind.tolist(),
            posObj=[0, 0, 0],
            flags=p.LINK_FRAME
        )

    def record_data(self, roll, pitch, yaw, rotor_speeds):
        """Record data for graphing at 10Hz"""
        current_time = time.time() - self.recording_start_time
        
        # Only record if enough time has passed (10Hz = 0.1 second intervals)
        if current_time - self.last_record_time < self.record_interval:
            return
        
        self.last_record_time = current_time
        
        self.recording_data['time'].append(current_time)
        self.recording_data['roll'].append(roll)
        self.recording_data['pitch'].append(pitch)
        self.recording_data['yaw'].append(yaw)
        self.recording_data['front_rotor'].append(rotor_speeds[0])
        self.recording_data['left_rotor'].append(rotor_speeds[1])
        self.recording_data['back_rotor'].append(rotor_speeds[2])
        self.recording_data['right_rotor'].append(rotor_speeds[3])

    def generate_final_graphs(self):
        """Generate comprehensive graphs when simulation ends"""
        if len(self.recording_data['time']) < 10:
            print("Not enough data to generate graphs (need at least 10 data points)")
            return

        print(f"\nGenerating final performance graphs with {len(self.recording_data['time'])} data points...")

        # Create figure with subplots (2x2 layout)
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle('Drone Performance Analysis - Complete Session', fontsize=16, fontweight='bold')

        times = np.array(self.recording_data['time'])

        # Plot 1: Roll vs Time with Front/Back Rotor Speeds
        color_roll = 'tab:blue'
        ax1.set_xlabel('Time (seconds)')
        ax1.set_ylabel('Roll (degrees)', color=color_roll)
        ax1.plot(times, np.degrees(self.recording_data['roll']), color=color_roll, linewidth=2, label='Roll')
        ax1.tick_params(axis='y', labelcolor=color_roll)
        ax1.grid(True, alpha=0.3)
        ax1.set_title('Roll Control with Front/Back Rotor Response')

        # Secondary axis for front/back rotor speeds
        ax1_rotor = ax1.twinx()
        color_rotor = 'tab:red'
        ax1_rotor.set_ylabel('Rotor Speed (rad/s)', color=color_rotor)
        ax1_rotor.plot(times, self.recording_data['front_rotor'], '--', color='red', alpha=0.7, linewidth=1.5, label='Front Rotor')
        ax1_rotor.plot(times, self.recording_data['back_rotor'], '--', color='darkred', alpha=0.7, linewidth=1.5, label='Back Rotor')
        ax1_rotor.tick_params(axis='y', labelcolor=color_rotor)
        ax1_rotor.legend(loc='upper right')

        # Plot 2: Pitch vs Time with Left/Right Rotor Speeds
        color_pitch = 'tab:green'
        ax2.set_xlabel('Time (seconds)')
        ax2.set_ylabel('Pitch (degrees)', color=color_pitch)
        ax2.plot(times, np.degrees(self.recording_data['pitch']), color=color_pitch, linewidth=2, label='Pitch')
        ax2.tick_params(axis='y', labelcolor=color_pitch)
        ax2.grid(True, alpha=0.3)
        ax2.set_title('Pitch Control with Left/Right Rotor Response')

        # Secondary axis for left/right rotor speeds
        ax2_rotor = ax2.twinx()
        color_rotor2 = 'tab:orange'
        ax2_rotor.set_ylabel('Rotor Speed (rad/s)', color=color_rotor2)
        ax2_rotor.plot(times, self.recording_data['left_rotor'], '--', color='magenta', alpha=0.7, linewidth=1.5, label='Left Rotor')
        ax2_rotor.plot(times, self.recording_data['right_rotor'], '--', color='cyan', alpha=0.7, linewidth=1.5, label='Right Rotor')
        ax2_rotor.tick_params(axis='y', labelcolor=color_rotor2)
        ax2_rotor.legend(loc='upper right')

        # Plot 3: Yaw vs Time
        ax3.set_xlabel('Time (seconds)')
        ax3.set_ylabel('Yaw (degrees)')
        ax3.plot(times, np.degrees(self.recording_data['yaw']), 'tab:purple', linewidth=2)
        ax3.grid(True, alpha=0.3)
        ax3.set_title('Yaw Performance Over Time')

        # Plot 4: All Rotor Speeds Together
        ax4.set_xlabel('Time (seconds)')
        ax4.set_ylabel('Rotor Speed (rad/s)')
        ax4.plot(times, self.recording_data['front_rotor'], 'r-', linewidth=2, label='Front Rotor', alpha=0.8)
        ax4.plot(times, self.recording_data['left_rotor'], 'm-', linewidth=2, label='Left Rotor', alpha=0.8)
        ax4.plot(times, self.recording_data['back_rotor'], 'g-', linewidth=2, label='Back Rotor', alpha=0.8)
        ax4.plot(times, self.recording_data['right_rotor'], 'c-', linewidth=2, label='Right Rotor', alpha=0.8)
        ax4.grid(True, alpha=0.3)
        ax4.legend(loc='best')
        ax4.set_title('All Rotor Speeds')

        # Add statistics
        roll_std = np.std(np.degrees(self.recording_data['roll']))
        pitch_std = np.std(np.degrees(self.recording_data['pitch']))
        yaw_std = np.std(np.degrees(self.recording_data['yaw']))
        duration = times[-1] if len(times) > 0 else 0

        # Add comprehensive stats text
        stats_text = (f'Session Duration: {duration:.1f}s | Data Points: {len(times)} | '
                     f'Roll σ: {roll_std:.2f}° | Pitch σ: {pitch_std:.2f}° | Yaw σ: {yaw_std:.2f}°')
        fig.text(0.5, 0.02, stats_text, ha='center', fontsize=11, style='italic')

        plt.tight_layout()
        plt.subplots_adjust(bottom=0.08)  # Make room for stats

        # REMOVE: Do not save the graph as a file
        # plt.savefig(filename, dpi=300, bbox_inches='tight')
        # print(f"Complete session graph saved as '{filename}'")
        # print(f"Graph saved in directory: {os.getcwd()}")

        # Show the graph and keep it open
        plt.show(block=True)  # Block=True keeps the window open

        return

    def reset_recording(self):
        """Reset recording data for new run"""
        self.recording_data = defaultdict(list)
        self.recording_start_time = time.time()
        self.last_record_time = 0

    def stabilize_orientation(self, target_yaw=0.0, target_roll=0.0, target_pitch=0.0):
        current_time = time.time()
        dt = current_time - self.last_time
        _, orientation = p.getBasePositionAndOrientation(self.drone_id)
        curr_rotation = R.from_quat(orientation)
        roll, pitch, yaw = curr_rotation.as_euler('xyz', degrees=False)
        orientation_error = np.array([
            target_roll - roll,
            target_pitch - pitch,
            target_yaw - yaw
        ])
        self.orientation_integral += orientation_error * dt
        MAX_INTEGRAL = 2.0
        self.orientation_integral = np.clip(self.orientation_integral, -MAX_INTEGRAL, MAX_INTEGRAL)
        kp = p.readUserDebugParameter(self.Kp_slider)
        ki = p.readUserDebugParameter(self.Ki_slider)
        kd = p.readUserDebugParameter(self.Kd_slider)
        self.Kp_orientation = np.array([kp, kp, kp])
        self.Ki_orientation = np.array([ki, ki, ki])
        self.Kd_orientation = np.array([kd, kd, kd])
        deriv_error = -(np.array(p.getBaseVelocity(self.drone_id)[1]))
        pid_output = (self.Kp_orientation * orientation_error +
                     self.Ki_orientation * self.orientation_integral +
                     self.Kd_orientation * deriv_error)
        roll_correction = pid_output[0]
        pitch_correction = pid_output[1]
        yaw_correction = pid_output[2]
        base_speed = HOVER_SPEED

        rotor_speeds = [
            -(base_speed + pitch_correction + yaw_correction),  # Front (no roll)
            base_speed + roll_correction - yaw_correction,      # Left (no pitch)  
            -(base_speed + pitch_correction + yaw_correction),  # Back (no roll)
            base_speed + roll_correction - yaw_correction       # Right (no pitch)
        ]

        rotor_speeds = np.clip(rotor_speeds, -2000, 2000)
        self.set_rotor_speeds(rotor_speeds)
        self.last_orientation_error = orientation_error

        # NEW: Record data at 10Hz
        self.record_data(roll, pitch, yaw, rotor_speeds)

        # Print one updating line (no new lines)
        print(
            f"\rRoll: {roll:5.2f}, Pitch: {pitch:5.2f}, Yaw: {yaw:5.2f} | "
            f"Rotor Speeds: [{rotor_speeds[0]:6.1f}, {rotor_speeds[1]:6.1f}, {rotor_speeds[2]:6.1f}, {rotor_speeds[3]:6.1f}]",
            end="", flush=True
        )

        self.last_time = current_time

    def reset_drone(self):
        # NEW: Generate graphs before resetting
        print("\nGenerating performance graphs...")
        
        
        p.removeBody(self.drone_id)
        script_dir = os.path.dirname(os.path.realpath(__file__))
        urdf_dir = os.path.join(script_dir, "urdf")
        pelican_urdf = os.path.join(urdf_dir, "pelican.urdf")
        self.drone_id = p.loadURDF(pelican_urdf, self.initial_pos, self.initial_ori, useFixedBase=False)
        self.orientation_integral = np.zeros(3)
        self.last_orientation_error = np.zeros(3)
        self.last_time = time.time()
        
        # NEW: Reset recording for new run
        self.reset_recording()

    def train_rl_policy(self, total_timesteps=100000):
        """Train RL policy to find optimal PID parameters"""
        if not RL_AVAILABLE:
            print("Please install required packages: pip install stable-baselines3 gymnasium")
            return
            
        print("Starting RL training for PID parameter optimization...")
        print(f"Training for {total_timesteps} timesteps")
        print("Target orientation is FIXED to level (roll=0, pitch=0, yaw=0)")
        
        # Create training environment
        env = DroneStabilizationEnv(self)
        
        # Create PPO model
        model = PPO(
            "MlpPolicy", 
            env, 
            verbose=1,
            learning_rate=3e-4,
            n_steps=2048,
            batch_size=64,
            n_epochs=10,
            tensorboard_log="./drone_tensorboard/"
        )
        
        # Setup evaluation callback
        eval_env = DroneStabilizationEnv(self)
        eval_callback = EvalCallback(
            eval_env, 
            best_model_save_path="./drone_models/",
            log_path="./drone_logs/", 
            eval_freq=5000,
            deterministic=True, 
            render=False
        )
        
        # Train the model
        model.learn(total_timesteps=total_timesteps, callback=eval_callback)
        
        # Save the final model
        model.save("drone_pid_policy")
        
        print("Training completed!")
        print("Model saved as 'drone_pid_policy'")
        
        # Test the trained policy
        self.test_trained_policy(model)
        
        return model
    
    def test_trained_policy(self, model):
        """Test the trained policy"""
        print("Testing trained policy...")
        
        env = DroneStabilizationEnv(self)
        obs, _ = env.reset()
        
        episode_reward = 0
        step_count = 0
        
        for _ in range(3600):  # 15 second test (updated from 1200)
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, truncated, info = env.step(action)
            episode_reward += reward
            step_count += 1
            
            if done:
                break
        
        final_pid = info.get('current_pid', [0, 0, 0])
        print(f"Test completed!")
        print(f"Episode reward: {episode_reward:.2f}")
        print(f"Steps survived: {step_count}")
        print(f"Final PID gains: Kp={final_pid[0]:.3f}, Ki={final_pid[1]:.3f}, Kd={final_pid[2]:.3f}")


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
        print("Press 'R' to reset drone | Close window to generate final graphs")
        rotor_speeds = [
                -HOVER_SPEED,
                HOVER_SPEED,
                -HOVER_SPEED,
                HOVER_SPEED,
            ]
        controller.set_rotor_speeds(rotor_speeds)
        for j in range(p.getNumJoints(drone_id)):
            print(j, p.getJointInfo(drone_id, j)[12].decode())

        while True:
            controller.apply_prop_force()
            controller.apply_wind_forces()
            controller.stabilize_orientation()
            
            p.stepSimulation()
            time.sleep(1/240)
            
            try:
                connection_info = p.getConnectionInfo()
                if not connection_info['isConnected']:
                    print("\nSimulation window closed - generating final graphs...")
                    controller.generate_final_graphs()
                    break
                    
                if not p.isConnected():
                    print("\nPyBullet disconnected - generating final graphs...")
                    controller.generate_final_graphs()
                    break
                
                keys = p.getKeyboardEvents()
                
                if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
                    print("\nResetting drone...")
                    rotor_speeds = [
                        -HOVER_SPEED,
                        HOVER_SPEED,
                        -HOVER_SPEED,
                        HOVER_SPEED,
                    ]
                    controller.set_rotor_speeds(rotor_speeds)
                    controller.reset_drone()
                    p.resetDebugVisualizerCamera(
                        cameraDistance=3.0,
                        cameraYaw=45.0,
                        cameraPitch=-30.0,
                        cameraTargetPosition=[0, 0, 0]
                    )
                    print("Drone reset and recording restarted")
                    
                if ord('l') in keys and keys[ord('l')] & p.KEY_WAS_TRIGGERED:
                    print("\nStarting RL training... (This will take several minutes)")
                    model = controller.train_rl_policy(total_timesteps=50000)
                    print("RL training complete!")
                    
            except p.error as e:
                print(f"\nPyBullet error: {e}")
                print("Generating final graphs before exit...")
                controller.generate_final_graphs()
                break
            except Exception as e:
                print(f"\nUnexpected error: {e}")
                print("Generating final graphs before exit...")
                controller.generate_final_graphs()
                break
                
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
        print("Generating final graphs...")
        controller.generate_final_graphs()
        
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
        print("Simulation fully terminated")

if __name__ == "__main__":
    main()