package frc.robot.subsystems.swerve;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SwerveModule {

    public final SwerveMotors swerveMotors;
    public final Translation2d robotTrackPosition;
    public final String name;

    static double maxMetersPerSecond = 5.05; // Default
    static double steerProportion = 0.3; // Default

    // Units in meters
    private double lastAccumulatedDriveDistance = 0;

    /**
     * Initially set by odometry source constructor,
     * if not set, begin at zero
     */
    private Translation2d AccumulatedRelativePositionMeters = new Translation2d();

    final GenericEntry calibrationEntry;

    public static ShuffleboardTab swerveModuleTab = Shuffleboard.getTab("Swerve Modules");

    public static ShuffleboardLayout calibrationAngleEntryGroup = 
        swerveModuleTab.getLayout("Calibration Angle (Degrees)", BuiltInLayouts.kList)
            .withPosition(0, 0)
            .withSize(5, 4);

    public static ArrayList<SwerveModule> instances = new ArrayList<SwerveModule>();

    public SwerveModule(SwerveMotors swerveMotors, Translation2d robotTrackPosition, String name) {
        instances.add(this);
        this.name = name;
        this.swerveMotors = swerveMotors;
        this.robotTrackPosition = robotTrackPosition;

        // Add calibration entry, persistent for safety
        calibrationEntry = calibrationAngleEntryGroup
            .add(name + " Calibration", swerveMotors.defaultAngleOffset.getDegrees()).getEntry();
    }

    public static void setMaxMetersPerSecond(double maxMetersPerSecond) {
        SwerveModule.maxMetersPerSecond = maxMetersPerSecond;
    }

    public static void setSteerPIDProportion(double steerProportion) {
        SwerveModule.steerProportion = steerProportion;
    }

    /**
     * Gets array of robot track positions of modules in the
     * order they were initialized in
     * @return
     */
    public static Translation2d[] getTrackPositions() {
        ArrayList<Translation2d> posList = new ArrayList<Translation2d>();
        for (SwerveModule swerveModule : instances) {
            posList.add(swerveModule.robotTrackPosition);
        }
        return posList.toArray(new Translation2d[posList.size()]);
    }
    
    /**
     * @return The physical location in meters of the swerve module relative
     * to its starting location. Updated by calling updateMovementVector()
     */
    public Translation2d getFieldRelativePosition() {return AccumulatedRelativePositionMeters;}

    /**
     * @return The assumed physical location in meters of the robot relative to
     * its starting location calculated from the field relative position of this
     * individual swerve module.
     */
    public Translation2d getAssumedRobotFieldRelativePosition() {
        return AccumulatedRelativePositionMeters.minus(robotTrackPosition);
    }

    /**
     * 
     * @param Position
     */
    public void setFieldRelativePosition(Translation2d Position) {AccumulatedRelativePositionMeters = Position;}

    /**
     * Drive the current swerve module using optimization.
     * Inputs are assumed to be on a scale from 0 to 1
     * @param State Un-Optimized state
     */
    public void drive(SwerveModuleState State) {

        // Update calibration from entries
        swerveMotors.configureCANCoder(
            new Rotation2d(Math.toRadians(calibrationEntry.getDouble(0)))
        );

        SwerveModuleState OptimizedState = optimizeState(
            State, 
            // Assume reading is degrees
            new Rotation2d(
                Math.toRadians(swerveMotors.SteerEncoder.getAbsolutePosition())
            )
        );

        // Set drive motor

        swerveMotors.DriveMotor.set(
            OptimizedState.speedMetersPerSecond / maxMetersPerSecond
        );

        // Set turn motor
        swerveMotors.SteerMotor.set(
            SwerveMotors.signedAngleBetween(
                OptimizedState.angle, swerveMotors.getRotation2d()
            ).getRadians() * steerProportion
        );
    }

    /**
     * Optimizes the provided state to not turn more than 90 degrees from the current position
     * @param State
     * @param CurrentRotation
     * @return SwerveModuleState
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
     * Updates and constructs the field position of this swerve module.
     * This should be ran periodically, and as frequently as possible.
     * 
     * @return AccumulatedRelativePositionMeters
     */
    public Translation2d updateFieldRelativePosition(Rotation2d robotOrientation) {
        // Handoff previous value and update
        double lastAccumulatedDriveDistance_h = lastAccumulatedDriveDistance;
        lastAccumulatedDriveDistance = swerveMotors.getDriveDistanceMeters();
        // This velocity is not for an accurate velocity reading, rather to catch a large
        // jump in drive distance. Grabbing velocity from drive motor will not catch this
        // error.
        double velocityFromDriveDistance = lastAccumulatedDriveDistance - lastAccumulatedDriveDistance_h;
        // For an accurate velocity reading, kMotors.DriveMotor.getEncoder().getVelocity()

        // Make sure this error does not exist!
        // Catch velocity error, and reset position with current robot pos
        // Assume a polling rate of 0.2 seconds.
        if (SwerveMotors.metersToRotations(velocityFromDriveDistance * 0.2) > maxMetersPerSecond) {
            // notify on driver station
            System.out.println(
                "Swerve module " + this.toString() + " has encountered a drive motor encoder failure. " +
                "Attempting recalibration from sibling modules"
            );
            // If this is ran, the swerve module needs to be reset
            // Calculate position from all swerve module instances.
            Translation2d posSum = new Translation2d();
            for (SwerveModule swerveModule : instances) {
                // Average from all, as this module hasnt updated yet
                posSum = posSum.plus(
                    swerveModule.getAssumedRobotFieldRelativePosition()
                );
            }

            // Return corrected position.
            return AccumulatedRelativePositionMeters = posSum.div(instances.size() - 1);
            // End function
        }
        
        // Else no velocity error
        // Calculate delta to add to last accumulated position
        Translation2d movementVectorMeters = new Translation2d(
            velocityFromDriveDistance, // Delta of drive distance
            // Sum is bounded by -pi to pi
            swerveMotors.getRotation2d().plus(robotOrientation)
        );
        // Update single module tracking
        // Add last vector and current vector
        return AccumulatedRelativePositionMeters = AccumulatedRelativePositionMeters.plus(movementVectorMeters);
    }

    /**
     * Updates position of swerve module from position of robot
     * 
     * @param Position Field relative position of robot.
     */
    public void setFieldRelativePositionFromRobotPosition(Pose2d Position) {
        AccumulatedRelativePositionMeters = Position.getTranslation().plus(robotTrackPosition);
    }
}
