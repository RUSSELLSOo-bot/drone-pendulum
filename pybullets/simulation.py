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

# --- TUNE THIS FOR HOVER ---
HOVER_SPEED = 399.3   # target angular velocity (rad/s) per rotor, calculated from physics
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
        self.Kp_slider = p.addUserDebugParameter("Kp", 0, 5.0, 1.0)
        self.Ki_slider = p.addUserDebugParameter("Ki", 0, 1.0, 0.01)
        self.Kd_slider = p.addUserDebugParameter("Kd", 0, 2.0, 0.5)
        self.wind_toggle = p.addUserDebugParameter("Wind On/Off", 0, 1, 0)
        self.wind_strength_slider = p.addUserDebugParameter("Wind Strength", 0, 5.0, 1.0)
        self.wind_enabled = False
        self.wind_strength = 1.0
        self.wind_frequency = 0.5
        self.last_wind_update = time.time()
        self.current_wind = np.zeros(3)

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

    def stabilize_orientation(self, target_yaw = 0.0, target_roll = 0.0, target_pitch = 0.0):
        current_time = time.time()
        dt = current_time - self.last_time
        _, orientation = p.getBasePositionAndOrientation(self.drone_id)
        curr_rotation = R.from_quat(orientation)
        roll, pitch, yaw = curr_rotation.as_euler('xyz', degrees = False)
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

    
        # Correction: For pitch, front and back should be opposite
        # So, front: base - pitch_correction, back: base + pitch_correction
        # For roll, left: base - roll_correction, right: base + roll_correction
        rotor_speeds = [
            -(base_speed - pitch_correction + roll_correction + yaw_correction),  # Front
            base_speed + pitch_correction - roll_correction + yaw_correction,  # Left
            -(base_speed + pitch_correction + roll_correction + yaw_correction),  # Back
            base_speed - pitch_correction - roll_correction + yaw_correction   # Right
        ]

        rotor_speeds = np.clip(rotor_speeds, -2000, 2000)
        self.set_rotor_speeds(rotor_speeds)
        self.last_orientation_error = orientation_error

        # Print one updating line (no new lines)
        print(
            f"\rRoll: {roll:5.2f}, Pitch: {pitch:5.2f}, Yaw: {yaw:5.2f} | "
            f"Rotor Speeds: [{rotor_speeds[0]:6.1f}, {rotor_speeds[1]:6.1f}, {rotor_speeds[2]:6.1f}, {rotor_speeds[3]:6.1f}]",
            end="", flush=True
        )

        self.last_time = current_time

    def reset_drone(self):
        p.removeBody(self.drone_id)
        script_dir = os.path.dirname(os.path.realpath(__file__))
        urdf_dir = os.path.join(script_dir, "urdf")
        pelican_urdf = os.path.join(urdf_dir, "pelican.urdf")
        self.drone_id = p.loadURDF(pelican_urdf, self.initial_pos, self.initial_ori, useFixedBase=False)
        self.orientation_integral = np.zeros(3)
        self.last_orientation_error = np.zeros(3)
        self.last_time = time.time()

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

    def constrain_to_pitch_rotation_only(self):
        """
        Constrain the drone to only rotate around the Y axis (pitch) and prevent translation.
        This function fixes the drone at [0, 0, 1] and zeroes roll/yaw, but does NOT perform stabilization.
        Call this in the main simulation loop before or after stabilize_orientation().
        """
        _, orientation = p.getBasePositionAndOrientation(self.drone_id)
        curr_rotation = R.from_quat(orientation)
        _, pitch, _ = curr_rotation.as_euler('xyz', degrees=False)
        # Set orientation to only pitch, fix position
        new_orientation = p.getQuaternionFromEuler([0, pitch, 0])
        p.resetBasePositionAndOrientation(self.drone_id, [0, 0, 1], new_orientation)

class DroneStabilizationEnv(gym.Env):
    def __init__(self, controller):
        super(DroneStabilizationEnv, self).__init__()
        
        self.controller = controller
        self.max_episode_steps = 3600  # 15 seconds at 240Hz (updated from 1200)
        self.step_count = 0
        
        # Define action space - PID parameter adjustments
        # Actions: [ΔKp, ΔKi, ΔKd] - same adjustment for all axes
        self.action_space = spaces.Box(
            low=np.array([-0.1, -0.01, -0.05]),   # Maximum decrease per step
            high=np.array([0.1, 0.01, 0.05]),     # Maximum increase per step
            dtype=np.float32
        )
        
        # Define observation space
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(21,),  # Increased from 15 to 21 (added 6 position/velocity terms)
            dtype=np.float32
        )
        
        # PID parameter bounds
        self.kp_bounds = [0.01, 2.0]
        self.ki_bounds = [0.0, 0.5]
        self.kd_bounds = [0.0, 1.0]
        
        # Episode tracking
        self.episode_reward = 0
        self.best_performance = -np.inf
        
        # Fixed target orientation - always level
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.target_yaw = 0.0
        
        # Starting orientation randomization
        self.max_start_angle = np.radians(5)  # 5 degrees in radians
        self.min_start_angle = np.radians(1)  # 1 degree minimum in radians
        
    def reset(self, seed=None):
        """Reset the environment"""
        super().reset(seed=seed)
        
        # Reset drone first
        self.controller.reset_drone()
        time.sleep(0.1)  # Let physics settle
        
        # Generate random starting orientation within ±1 to ±5 degrees
        def random_angle():
            # Generate random angle with minimum magnitude
            sign = np.random.choice([-1, 1])
            magnitude = np.random.uniform(self.min_start_angle, self.max_start_angle)
            return sign * magnitude
        
        random_roll = random_angle()
        random_pitch = random_angle()
        random_yaw = random_angle()
        
        # Apply random starting orientation
        random_orientation = p.getQuaternionFromEuler([random_roll, random_pitch, random_yaw])
        p.resetBasePositionAndOrientation(
            self.controller.drone_id, 
            self.controller.initial_pos, 
            random_orientation
        )
        
        print(f"\nEpisode start - Roll: {np.degrees(random_roll):.1f}°, Pitch: {np.degrees(random_pitch):.1f}°, Yaw: {np.degrees(random_yaw):.1f}°")
        
        # Initialize PID parameters randomly within reasonable bounds
        self.controller.Kp_orientation = np.array([
            np.random.uniform(0.05, 0.3), 
            np.random.uniform(0.05, 0.3), 
            np.random.uniform(0.05, 0.3)
        ])
        self.controller.Ki_orientation = np.array([0.0, 0.0, 0.0])  # Start with no integral
        self.controller.Kd_orientation = np.array([
            np.random.uniform(0.01, 0.1), 
            np.random.uniform(0.01, 0.1), 
            np.random.uniform(0.01, 0.1)
        ])
        
        # Reset controller state
        self.controller.orientation_integral = np.zeros(3)
        self.controller.last_orientation_error = np.zeros(3)
        self.controller.last_time = time.time()
        
        self.step_count = 0
        self.episode_reward = 0
        
        return self._get_observation(), {}
    
    def _get_observation(self):
        """Get current state observation - INCLUDING POSITION"""
        # Get drone state
        pos, orientation = p.getBasePositionAndOrientation(self.controller.drone_id)
        lin_vel, ang_vel = p.getBaseVelocity(self.controller.drone_id)
        
        # Convert orientation to euler angles
        curr_rotation = R.from_quat(orientation)
        roll, pitch, yaw = curr_rotation.as_euler('xyz', degrees=False)
        
        # Calculate orientation errors (target is always level - fixed)
        roll_error = self.target_roll - roll
        pitch_error = self.target_pitch - pitch
        yaw_error = self.target_yaw - yaw
        
        # Calculate position errors
        target_pos = [0.0, 0.0, 1.0]  # Stay at origin, 1m height
        pos_error_x = target_pos[0] - pos[0]
        pos_error_y = target_pos[1] - pos[1] 
        pos_error_z = target_pos[2] - pos[2]
        
        # Get current PID parameters (average across axes)
        current_kp = np.mean(self.controller.Kp_orientation)
        current_ki = np.mean(self.controller.Ki_orientation)  
        current_kd = np.mean(self.controller.Kd_orientation)
        
        observation = np.array([
            # Orientation state (9 values)
            roll, pitch, yaw,                                    # Current orientation
            ang_vel[0], ang_vel[1], ang_vel[2],                 # Angular velocities
            roll_error, pitch_error, yaw_error,                 # Orientation errors
            
            # Position state (6 values) - NEW
            pos[0], pos[1], pos[2],                             # Current position
            pos_error_x, pos_error_y, pos_error_z,              # Position errors
            
            # Controller state (6 values)
            self.controller.orientation_integral[0],             # Integral terms
            self.controller.orientation_integral[1],
            self.controller.orientation_integral[2],
            current_kp, current_ki, current_kd                  # Current PID gains
        ], dtype=np.float32)
        
        return observation
    
    def step(self, action):
        """Execute one environment step"""
        # Apply PID parameter adjustments
        delta_kp, delta_ki, delta_kd = action
        
        # Update PID parameters (same adjustment for all axes)
        new_kp = np.clip(
            self.controller.Kp_orientation + delta_kp,
            self.kp_bounds[0], self.kp_bounds[1]
        )
        new_ki = np.clip(
            self.controller.Ki_orientation + delta_ki,
            self.ki_bounds[0], self.ki_bounds[1]
        )
        new_kd = np.clip(
            self.controller.Kd_orientation + delta_kd,
            self.kd_bounds[0], self.kd_bounds[1]
        )
        
        self.controller.Kp_orientation = new_kp
        self.controller.Ki_orientation = new_ki
        self.controller.Kd_orientation = new_kd
        
        # Run simulation for multiple physics steps per RL step
        steps_per_action = 12  # 0.05 seconds at 240Hz
        
        total_reward = 0
        for _ in range(steps_per_action):
            # Apply drone control with FIXED target orientation (always level)
            self.controller.apply_prop_force()
            self.controller.apply_wind_forces()
            self.controller.stabilize_orientation(
                target_roll=self.target_roll,
                target_pitch=self.target_pitch, 
                target_yaw=self.target_yaw
            )
            p.stepSimulation()
            
            # Calculate reward for this physics step
            step_reward = self._calculate_reward()
            total_reward += step_reward
            
            # Check if episode should terminate
            terminated = self._check_termination()
            if terminated:
                break
        
        self.step_count += 1
        self.episode_reward += total_reward
        
        # Check if episode is done
        truncated = self.step_count >= self.max_episode_steps
        done = terminated or truncated
        
        observation = self._get_observation()
        
        info = {
            'episode_reward': self.episode_reward,
            'step_count': self.step_count,
            'current_pid': [np.mean(new_kp), np.mean(new_ki), np.mean(new_kd)]
        }
        
        return observation, total_reward, done, truncated, info
    
    def _calculate_reward(self):
        """Calculate reward for current state"""
        # Get current state
        pos, orientation = p.getBasePositionAndOrientation(self.controller.drone_id)
        _, ang_vel = p.getBaseVelocity(self.controller.drone_id)
        
        # Convert to euler angles
        curr_rotation = R.from_quat(orientation)
        roll, pitch, yaw = curr_rotation.as_euler('xyz', degrees=False)
        
        # Primary reward: Orientation stability (average roll and pitch as requested)
        avg_tilt = (abs(roll) + abs(pitch)) / 2.0
        orientation_reward = -avg_tilt * 10  # Negative because we want to minimize tilt
        
        # Secondary rewards
        yaw_stability = -abs(yaw) * 2  # Less weight on yaw
        altitude_maintenance = -abs(pos[2] - 1.0) * 5  # Stay at 1m height
        horizontal_drift = -(pos[0]**2 + pos[1]**2) * 2  # Stay centered
        
        # Control smoothness (penalize excessive angular velocities)
        angular_velocity_penalty = -(abs(ang_vel[0]) + abs(ang_vel[1]) + abs(ang_vel[2])) * 0.5
        
        # Efficiency bonus (reward for using reasonable PID gains)
        kp_avg = np.mean(self.controller.Kp_orientation)
        efficiency_bonus = -abs(kp_avg - 0.2) * 0.1  # Encourage gains around 0.2
        
        # Combine all rewards
        total_reward = (orientation_reward + yaw_stability + altitude_maintenance + 
                       horizontal_drift + angular_velocity_penalty + efficiency_bonus)
        
        return total_reward
    
    def _check_termination(self):
        """Check if episode should terminate early"""
        pos, orientation = p.getBasePositionAndOrientation(self.controller.drone_id)
        
        # Termination conditions
        crashed = pos[2] < 0.1  # Too low
        flew_away_vertical = pos[2] > 4.0  # Too high
        flew_away_horizontal = abs(pos[0]) > 5.0 or abs(pos[1]) > 5.0  # Too far horizontally
        
        # Severe orientation failure
        curr_rotation = R.from_quat(orientation)
        roll, pitch, yaw = curr_rotation.as_euler('xyz', degrees=False)
        severe_tilt = abs(roll) > 1.0 or abs(pitch) > 1.0  # More than ~60 degrees
        
        return crashed or flew_away_vertical or flew_away_horizontal or severe_tilt

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
        rotor_speeds = [
                -HOVER_SPEED,
                HOVER_SPEED,
                -HOVER_SPEED,
                HOVER_SPEED,
            ]
        controller.set_rotor_speeds(rotor_speeds)

        while True:
            controller.apply_prop_force()
            controller.apply_wind_forces()
            controller.constrain_to_pitch_rotation_only()
            controller.stabilize_orientation()
            
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
                target = list(cam_data[11]);
                if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
                    time.sleep(1.0)
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
                    print("Drone and camera reset to initial position")
                    print(f"Drone position: {controller.initial_pos}")
                if ord('l') in keys and keys[ord('l')] & p.KEY_WAS_TRIGGERED:
                    print("\nStarting RL training... (This will take several minutes)")
                    model = controller.train_rl_policy(total_timesteps=50000)
                    print("RL training complete!")
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