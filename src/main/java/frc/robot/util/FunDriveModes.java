package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj.drive.DifferentialDrive.*;

public class FunDriveModes {
    /**
     * <p>Calculates {@link SwerveModuleState}s to make swerve act like a differential drive train.</p>
     *
     * <p>Code Example</p>
     * <pre><code>runStates(new differentialDrive(tankDriveIK(leftThrottle, rightThrottle, bool)));</code></pre>
     *
     * @param wheelSpeeds the wheelSpeeds to transform.
     * @param maxSpeed max {@link LinearVelocity} to multiply wheel speeds by. Wheel speeds are -1 to 1, states are velocity.
     * @return the states to run.
     */
    public static SwerveModuleState[] differentialDrive(WheelSpeeds wheelSpeeds, LinearVelocity maxSpeed) {
        // Module number for kinematics, usually 0 to 3. Front left -> front right -> back left -> back right
        SwerveModuleState[] states = new SwerveModuleState[4];
        // Applies the max speed, converting speeds to velocity.
        double leftSpeeds = wheelSpeeds.left * maxSpeed.in(MetersPerSecond);
        double rightSpeeds = wheelSpeeds.right * maxSpeed.in(MetersPerSecond);
        // Apply the new speeds to module states.
        states[0] = new SwerveModuleState(leftSpeeds, Rotation2d.kZero);
        states[2] = new SwerveModuleState(leftSpeeds, Rotation2d.kZero);
        states[1] = new SwerveModuleState(rightSpeeds, Rotation2d.kZero);
        states[3] = new SwerveModuleState(rightSpeeds, Rotation2d.kZero);
        // Return our new states
        return states;
    }

    /**
     * <p>Calculates {@link SwerveModuleState}s to make swerve act like a car.</p>
     * <p>This method just uses a speed input, add a CarSpeedsConfig to get accelerator and brake options.</p>
     * <P>The Swerve Drive should have brake mode disabled.</P>
     *
     * <p>Code Example</p>
     * <pre><code>runStates(new carDrive(CarType.FWD, controller.getRightTrigger(), controller.getRightX));</code></pre>
     *
     * <p>Code Example with a {@link CarSpeedsController}</p>
     * <pre><code>
     *     runStates(new carDrive(CarType.FWD, controller.calculateNormalizedSpeeds(), controller.getRightX))
     *                  .beforeStarting(controller.reset());
     * </code></pre>
     *
     * @param type basic FWD and RWD options.
     * @param speed basic throttle speeds.
     * @param rotation rotation input.
     * @param maxSpeed max {@link LinearVelocity} to abide by.
     * @return the states to run.
     */
    public static SwerveModuleState[] carDrive(CarType type, double speed, double rotation, LinearVelocity maxSpeed) {
        // Module number for kinematics, usually 0 to 3. Front left -> front right -> back left -> back right
        SwerveModuleState[] states = new SwerveModuleState[4];
        // Car wheels can only turn so far.
        Angle turningLimit = Degrees.of(50);
        Rotation2d turn = Rotation2d.fromDegrees(turningLimit.times(rotation).in(Degrees));
        // Change our speeds depending on drive style.
        switch (type) {
            case FWD -> {
                states[0] = new SwerveModuleState(speed, turn);
                states[1] = new SwerveModuleState(speed, turn);
                states[2] = new SwerveModuleState(0, Rotation2d.kZero);
                states[3] = new SwerveModuleState(0, Rotation2d.kZero);
            }
            case RWD -> {
                double speedToTurn = Math.abs(turn.getDegrees()) > 2 ? Math.abs(turn.times(speed).div(2).getRotations()) : 0; // Seems to need speed to move wheel to angle, may be a sim bug.
                states[0] = new SwerveModuleState(speedToTurn, turn);
                states[1] = new SwerveModuleState(speedToTurn, turn);
                states[2] = new SwerveModuleState(speed, Rotation2d.kZero);
                states[3] = new SwerveModuleState(speed, Rotation2d.kZero);
            }
        }
        // Return our new states.
        return states;
    }

    public enum CarType {
        FWD,
        RWD
    }

    // TODO TEST on real bot
    public static class CarSpeedsController {
        // Speed inputs
        private final DoubleSupplier gas;
        private final DoubleSupplier brake;
        // How long since we last checked.
        private long lastUpdateTime = System.currentTimeMillis();
        // Current velocity in m/s
        private double currentSpeed = 0.0;
        // Controller parameters
        private final LinearAcceleration MAX_ACCELERATION;
        private final LinearAcceleration MAX_DECELERATION;
        private final LinearVelocity MAX_SPEED;
        private final LinearAcceleration FRICTION_DECEL;
        private final double DEADZONE;

        /**
         * A speed controller that imitates the acceleration and braking of a real car.
         *
         * @param gas the gas pedal inputs [-1 to 1], uses negative instead of a reverse gear for simplicity.
         *            Can be mapped to two separate triggers for a forward and reverse.
         * @param brake the brake pedal inputs [0 to 1]
         * @param maxAccel the max acceleration of the car.
         * @param maxDeccel the max deceleration of the car. Braking force.
         * @param maxSpeed the max velocity of the drive train.
         * @param decelFriction the engine braking friction when not accelerating or braking.
         * @param deadzone the speed threshold (m/s) below which the robot stops completely.
         */
        public CarSpeedsController(DoubleSupplier gas, DoubleSupplier brake,
                                   LinearAcceleration maxAccel, LinearAcceleration maxDeccel,
                                   LinearVelocity maxSpeed, LinearAcceleration decelFriction,
                                   double deadzone)
        {
            this.gas = gas;
            this.brake = brake;
            this.MAX_ACCELERATION = maxAccel;
            this.MAX_DECELERATION = maxDeccel;
            this.MAX_SPEED = maxSpeed;
            this.FRICTION_DECEL = decelFriction;
            this.DEADZONE = deadzone;
        }

        /**
         * Calculates the desired speed with realistic acceleration/deceleration curves.
         *
         * @return desired speed in m/s (positive = forward, negative = reverse)
         */
        public double calculateSpeed() {
            // Time delta since last call
            long currentTime = System.currentTimeMillis();
            double dt = (currentTime - lastUpdateTime) / 1000.0; // Convert to seconds
            lastUpdateTime = currentTime;

            // Guard against massive dt jumps
            dt = Math.min(dt, 0.020);

            // Input range: -1 to 1 (negative = reverse, positive = forward)
            double gasInput = Math.max(-1, Math.min(1, gas.getAsDouble()));
            double brakeInput = Math.max(0, Math.min(1, brake.getAsDouble()));

            // Brake takes priority over gas
            double throttleInput = brakeInput > 0 ? 0 : gasInput;

            // Initialize our values to make code a little nicer to read.
            double acceleration = 0.0;
            double maxAccelMs2 = MAX_ACCELERATION.in(MetersPerSecondPerSecond);
            double maxDecelMs2 = MAX_DECELERATION.in(MetersPerSecondPerSecond);
            double maxSpeedMs = MAX_SPEED.in(MetersPerSecond);
            double frictionMs2 = FRICTION_DECEL.in(MetersPerSecondPerSecond);

            if (brakeInput > 0) {
                // Braking - decelerate toward zero regardless of direction
                acceleration = -Math.copySign(maxDecelMs2 * brakeInput, currentSpeed);
            } else if (Math.abs(throttleInput) > 0.01) {
                // Accelerating (forward or reverse)
                double speedRatio = Math.abs(currentSpeed) / maxSpeedMs;
                // Quadratic falloff as we approach max speed (realistic motor behavior)
                double availableAccel = maxAccelMs2 * (1.0 - Math.pow(speedRatio, 2));
                acceleration = Math.copySign(availableAccel * Math.abs(throttleInput), throttleInput);
            } else {
                // Coasting - friction/drag toward zero
                if (Math.abs(currentSpeed) > DEADZONE) {
                    acceleration = -Math.copySign(frictionMs2, currentSpeed);
                } else {
                    currentSpeed = 0.0;
                    acceleration = 0.0;
                }
            }
            // Integrate acceleration to get new velocity
            currentSpeed += acceleration * dt;
            // Clamp speed to max range [-MAX_SPEED, MAX_SPEED]
            currentSpeed = Math.max(-maxSpeedMs, Math.min(maxSpeedMs, currentSpeed));
            return currentSpeed;
        }

        /**
         * Calculate the desired speed as a normalized value [-1 to 1].
         *
         * @return normalized desired speed relative to max speed
         */
        public double calculateNormalizedSpeed() {
            return calculateSpeed() / MAX_SPEED.in(MetersPerSecond);
        }

        /**
         * Reset the controller's internal state (useful when switching modes or on disable).
         */
        public void reset() {
            currentSpeed = 0.0;
            lastUpdateTime = System.currentTimeMillis();
        }
    }
}