package frc.robot.subsystems.Swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.Swerve;

public class SwerveModule {

    final SwerveMotors kMotors;
    final Translation2d kPosition;

    final ShuffleboardTab kSwerveTab;
    final GenericEntry calibrationAngle;

    public SwerveModulePosition swerveModulePosition;

    private double lastAccumulatedDriveDistance = 0;

    private final AHRS navxGyro;

    /**
     * Initially set by odometry source constructor,
     * if not set, begin at zero
     */
    private Translation2d AccumulatedRelativePositionMeters = new Translation2d();

    /**
     * @return The physical location of the swerve module relative to the center of the robot.
     */
    public Translation2d getPosition() {return kPosition;}

    /**
     * @return The physical location in meters of the swerve module relative
     * to its starting location and rotation. Updated by calling updateMovementVector()
     */
    public Translation2d getFieldRelativePosition() {return AccumulatedRelativePositionMeters;}

    /**
     * 
     * @param Position
     */
    public void setFieldRelativePosition(Translation2d Position) {AccumulatedRelativePositionMeters = Position;}


    // public static SwerveModule getInstance(int instance) {

    //     return null;
    // }

    /**
     * Construct a single swerve module
     * @param Motors Group of motors and encoder to use
     */
    public SwerveModule(
        SwerveMotors Motors, 
        Translation2d Position, 
        ShuffleboardTab SwerveTab,
        AHRS NAVXGyro
    ) {
        this.kSwerveTab = SwerveTab;

        this.kMotors = Motors;
        this.kPosition = Position;

        this.navxGyro = NAVXGyro;

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
            new Rotation2d(
                Math.toRadians(kMotors.TurnEncoder.getAbsolutePosition())
            )
        );

        // Set drive motor

        kMotors.DriveMotor.set(
            OptimizedState.speedMetersPerSecond
        );

        // Set turn motor
        kMotors.PositionTurnController.setTarget(0);

        kMotors.PositionTurnController.setInput(
            calculateDistanceCorrection(
                OptimizedState.angle.getDegrees(), 
                kMotors.TurnEncoder.getAbsolutePosition()
            )
        );

        kMotors.TurnMotor.set(
            -kMotors.PositionTurnController.calculate(1, -1)
        );
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

    static double rotationsToMeters(double rotations) {
        return rotations * Swerve.rotationsToMeters;
    }

    /**
     * This function calculates the difference between the target and
     * position on a circle, allowing a PID controller to travel over 
     * the point where 0 degrees and 360 degrees meet.
     * @param target PID goal
     * @param pos PID input
     * @return target relative to pos on a circle
     */
    public static double calculateDistanceCorrection(
        double targetDegrees, 
        double positionDegrees
    ) {
        if (Math.abs(targetDegrees + (360) - positionDegrees) 
            < Math.abs(targetDegrees - positionDegrees)
        ) {
            return targetDegrees + (360) - positionDegrees;
        } else if (Math.abs(targetDegrees - (360) - positionDegrees) 
            < Math.abs(targetDegrees - positionDegrees)
        ) {
            return targetDegrees - (360) - positionDegrees;
        } else {
            return targetDegrees - positionDegrees;
        }
    }

    /**
     * @return SwerveModulePosition containing turn encoder degrees and drive motor position
     */
    public SwerveModulePosition getSwerveModulePosition() {
        return swerveModulePosition = new SwerveModulePosition(
            rotationsToMeters(
                kMotors.DriveMotor.getEncoder().getPosition()
            ), 
            new Rotation2d(
                Math.toRadians(kMotors.TurnEncoder.getAbsolutePosition())
            )
        );
    }

    /**
     * Updates and constructs the field position of this swerve module.
     * This should be ran periodically, and as frequently as possible.
     */
    public Translation2d updateFieldRelativePosition() {

        Translation2d movementVectorMeters = new Translation2d(
            rotationsToMeters(
                kMotors.DriveMotor.getEncoder().getPosition() - lastAccumulatedDriveDistance
            ),
            // Sum is bounded by -pi to pi
            kMotors.getRotation2d().plus(navxGyro.getRotation2d())
        );

        lastAccumulatedDriveDistance = kMotors.DriveMotor.getEncoder().getPosition();

        // Update single module tracking
        // Add last vector and current vector
        return AccumulatedRelativePositionMeters = AccumulatedRelativePositionMeters.plus(movementVectorMeters);
    }

    public void setFieldRelativePositionFromRobotPosition(Pose2d Position) {
        AccumulatedRelativePositionMeters = Position.getTranslation().plus(getPosition());
    }
}
