package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

    final SwerveMotors kMotors;
    final Translation2d kPosition;

    /**
     * @return The physical location of the swerve module relative to the center of the robot.
     */
    public Translation2d getPosition() {
        return kPosition;
    }

    /**
     * Construct a single swerve module
     * @param Motors Group of motors and encoder to use
     */
    public SwerveModule(SwerveMotors Motors, Translation2d Position) {
        this.kMotors = Motors;
        this.kPosition = Position;
    }

    /**
     * Drive the current swerve module using optimization
     * @param State Un-Optimized state
     */
    public void drive(SwerveModuleState State) {
        SwerveModuleState OptimizedState = optimizeState(
            State, 
            // Assume reading is degrees
            new Rotation2d(Math.toRadians(kMotors.TurnEncoder.getAbsolutePosition()))
        );

        // Set drive motor

        double realSpeed = kMotors.DriveMotor.getEncoder().getVelocity();
        double speedDifference = (OptimizedState.speedMetersPerSecond / Constants.Swerve.DriveGearRatioFactor) - realSpeed;
        // Give the input to the pid controller to allow for speed compensation
        kMotors.SpeedDriveController.setInput(speedDifference);

        kMotors.DriveMotor.set(
            // Clamp to avoid overload
            RobotContainer.Clamp(
                (OptimizedState.speedMetersPerSecond / Constants.Swerve.DriveGearRatioFactor)
                    // Add the PID output to the input
                    // TODO may need to be reversed
                    + kMotors.SpeedDriveController.calculate(1, -1),
                1, -1
            )
        );

        // Set turn motor
        kMotors.PositionTurnController.setTarget(OptimizedState.angle.getDegrees());
        kMotors.PositionTurnController.setInput(kMotors.TurnEncoder.getAbsolutePosition());
        // TODO may need to be reversed
        kMotors.TurnMotor.set(kMotors.PositionTurnController.calculate(1, -1));
    }

    /**
     * Optimizes the provided state to not turn more than 90 degrees from the current position
     * @param State
     * @param CurrentRotation
     * @return
     */
    static SwerveModuleState optimizeState(SwerveModuleState State, Rotation2d CurrentRotation) {
        Rotation2d RotationDifference = State.angle.minus(CurrentRotation);

        SwerveModuleState OptimizedState = State;

        if (Math.abs(RotationDifference.getDegrees()) > 90) {
            // Reverse wheel direction and reverse wheel speed
            OptimizedState = new SwerveModuleState(-State.speedMetersPerSecond, State.angle.plus(new Rotation2d(180)));
        }
    
        return OptimizedState;
    }

    /**
     * @return SwerveModulePosition containing turn encoder degrees and drive motor position
     */
    public SwerveModulePosition getEncoderPositions() {
        return new SwerveModulePosition(
            kMotors.DriveMotor.getEncoder().getPosition(), 
            new Rotation2d(Math.toRadians(kMotors.TurnEncoder.getAbsolutePosition()))
        );
    }
}
