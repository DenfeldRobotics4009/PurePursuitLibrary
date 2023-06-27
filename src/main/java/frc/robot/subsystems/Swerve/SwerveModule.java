package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SwerveModule {

    final SwerveMotors kMotors;
    final Translation2d kPosition;

    final ShuffleboardTab kSwerveTab;
    final GenericEntry calibrationAngle;

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
    public SwerveModule(SwerveMotors Motors, Translation2d Position, ShuffleboardTab SwerveTab) {
        this.kSwerveTab = SwerveTab;

        this.kMotors = Motors;
        this.kPosition = Position;

        calibrationAngle = SwerveTab.addPersistent(
            Motors.Name +  " Calibration Angle"
            , 0
        ).getEntry();
    }

    public void updateCalibration() {
        kMotors.configureCANCoder(
            new Rotation2d(Math.toRadians(calibrationAngle.getDouble(0)))
        );
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

        kMotors.DriveMotor.set((OptimizedState.speedMetersPerSecond / Constants.Swerve.MaxMetersPerSecond));

        // Set turn motor
        kMotors.PositionTurnController.setTarget(0);

        kMotors.PositionTurnController.setInput(
            calcDistCorrection(OptimizedState.angle.getDegrees(), kMotors.TurnEncoder.getAbsolutePosition())
        );

        kMotors.TurnMotor.set(-kMotors.PositionTurnController.calculate(1, -1));
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

        if (Math.abs(RotationDifference.getRadians()) > (Math.PI / 2)) {
            // Reverse wheel direction and reverse wheel speed
            OptimizedState = new SwerveModuleState(
                -State.speedMetersPerSecond, 
                State.angle.plus(new Rotation2d(Math.PI))
            );
        }
    
        return OptimizedState;
    }

    /**
     * Set PIDController goal to zero
     * @param target PID goal
     * @param pos PID input
     * @return target relative to pos on a circle
     */
    double calcDistCorrection(double target, double pos) {
        if (Math.abs(target + (360) - pos) < Math.abs(target - pos)) {
        return target + (360) - pos;
        } else if (Math.abs(target - (360) - pos) < Math.abs(target - pos)) {
        return target - (360) - pos;
        } else {return target - pos;}
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
