# Testing Visible Noise in Robot Simulation

## What I've Implemented

1. **Sinusoidal Noise**: Added a new `applySinusoidalNoise()` method that creates visible oscillations in the robot's joints
   - Base frequency: 2 Hz with variations per joint
   - Amplitude: ±0.5 rad/s for visible shaking
   - Different phase offsets for each joint to create complex motion

2. **Noise Without Movement**: Modified the actuator and controller code so that:
   - When noise level > 0 and robot is not commanded to move, joints will still oscillate
   - This allows you to see the noise effect clearly without any control input

3. **Time-based Noise**: The noise is now time-dependent (sinusoidal) rather than random, making it more visible and predictable

## How to Test

1. Run the simulation: `./vsg_ode_robot`

2. **Test pure noise (no movement)**:
   - Press `N` several times to increase noise level (you'll see "Noise level increased to: X.X")
   - Watch the robot's legs start to shake/oscillate
   - The shaking should get more intense as noise increases

3. **Test noise with movement**:
   - Press `Space` to enable manual control
   - Press `N` to increase noise
   - Use `WASD` to move - you'll see noisy motion on top of the commanded movement

4. **Test stabilization**:
   - With noise active (press `N` a few times)
   - The robot controller will automatically try to compensate
   - At higher noise levels, the PID gains are reduced to prevent overcorrection

## What You Should See

- At noise level 0.0: Robot stands still, no shaking
- At noise level 0.3: Visible oscillation in leg joints
- At noise level 0.6: Strong shaking, robot struggles to maintain balance
- At noise level 1.0: Maximum oscillation amplitude

The noise affects:
- Joint velocities (visible as shaking)
- Sensor readings (affects balance control)
- Contact detection (may cause intermittent ground contact)

## Stabilization Features

When noise is active, the controller:
1. Uses filtered sensor data (exponential moving average)
2. Requires stable contact from 3+ feet over multiple frames
3. Reduces PID gains proportionally to noise level
4. Only applies corrective forces when stable ground contact is detected

This simulates a real robot dealing with sensor noise and trying to maintain stability!