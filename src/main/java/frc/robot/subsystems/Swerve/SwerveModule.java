package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Swerve;


public class SwerveModule {

    final SwerveModuleInstance instance;

    public final SwerveMotors motors;

    public final GenericEntry calibrationAngle;

    // Units in meters
    private double lastAccumulatedDriveDistance = 0;

    private final AHRS navxGyro;

    /**
     * TODO Move to top of class
     * 
     * Construct a single swerve module
     * @param Motors Group of motors and encoder to use
     * 
     * Private constructor for multiton
     */
    private SwerveModule(
        SwerveModuleInstance ModulePosition, 
        GenericEntry CalibrationAngleEntry,
        AHRS NAVXGyro
    ) {
        this.motors = ModulePosition.getMotors();
        this.instance = ModulePosition;

        this.navxGyro = NAVXGyro;

        calibrationAngle = CalibrationAngleEntry;
    }

    /**
     * Constructs or returns a pre-existing swerve module
     * within the boundaries of the SwerveModulePosition enum.
     * 
     * @param ModulePosition SwerveModuleInstance enum item
     * @param Motors Group of motors and encoder to match with module.
     * @param Position Position of module (in meters) relative to
     * the center of the robot.
     * @param SwerveTab ShuffleboardTab instance to store data on.
     * @param NAVXGyro AHRS NAVX gyro for position tracking.
     * 
     * @return Constructed, or already present, swerve module instance.
     */
    public static SwerveModule getInstance(

        SwerveModuleInstance ModulePosition,
        // Params for initialization, allow
        // no default construction
        GenericEntry CalibrationAngleEntry,
        AHRS NAVXGyro
    ) {
        SwerveModule swerveModule;

        if (!Instances.containsKey(ModulePosition)) {
            // Construct new module if not present
            swerveModule = new SwerveModule(ModulePosition, CalibrationAngleEntry, NAVXGyro);

            // Insert into hash map
            Instances.put(ModulePosition, swerveModule);
        } else {
            // Module already constructed, return module
            // Parameters will be ignored
            swerveModule = Instances.get(ModulePosition);
        }

        return swerveModule;
    }

    /**
     * Returns an already constructed swerve module instance.
     * This will error if the instance has not been initialized.
     * 
     * @param ModulePosition SwerveModuleInstance enum item
     * @return Instance
     */
    public static SwerveModule getInstance(SwerveModuleInstance ModulePosition) {
        return Instances.get(ModulePosition);
    }

    /**
     * Converts instance hash map to array and returns constructed array.
     * Order of array DOES NOT match enum, order should not be attempted 
     * to predict
     * 
     * @return Array of currently initialized SwerveModule instances
     */
    public static SwerveModule[] getInstances() {
        return Instances.values().toArray(new SwerveModule[0]);
    }

    public void updateCalibration() {
        motors.configureCANCoder(
            new Rotation2d(Math.toRadians(calibrationAngle.getDouble(0)))
        );
    }

    // /**
    //  * Sets the current encoder position to zero
    //  */
    // public void calibrate() {
    //     // Set the genericEntry calibration angle to the modules current angle
    //     calibrationAngle.setDouble(
    //         // Add the current adjusted rotation2d to the last calibration
    //         // angle to properly offset
    //         (motors.SteerEncoder.getAbsolutePosition() - motors.SteerEncoder.configGetMagnetOffset()) % 360
    //     );
    // }

    /**
     * Initially set by odometry source constructor,
     * if not set, begin at zero
     */
    private Translation2d AccumulatedRelativePositionMeters = new Translation2d();

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
        return AccumulatedRelativePositionMeters.minus(getRobotRelativePosition());
    }

    /**
     * 
     * @param Position
     */
    public void setFieldRelativePosition(Translation2d Position) {AccumulatedRelativePositionMeters = Position;}

    /**
     * @return Robot relative position of swerve module/
     */
    public Translation2d getRobotRelativePosition() {return instance.getPosition();}

    /**
     * Multiton instances
     */
    private static final Map<SwerveModuleInstance, SwerveModule> Instances = new HashMap<>();

    /**
     * Runs the provided lambda expression for every initialized
     * swerve module. This function is typically unnecessary, though
     * provides a more efficient foreach operation than looping
     * through getInstances array.
     * 
     * @param arg0 (SwerveModuleInstance, SwerveModule) -> {...}
     * 
     * @see java.util.Map java.util.Map.forEach
     */
    public static void forEach(
        BiConsumer<? super SwerveModuleInstance, ? super SwerveModule> arg0
    ) {
        Instances.forEach(arg0);
    }

    /**
     * @return Array of robot relative module positions
     */
    public static Translation2d[] getRobotRelativePositions() {
        List<Translation2d> modulePositionList = new ArrayList<Translation2d>();

        // Construct array via looping through hash map
        Instances.forEach (

            (k, swerveModule) -> {
                modulePositionList.add(
                    swerveModule.getRobotRelativePosition()
                );
            }

        );

        return modulePositionList.toArray(new Translation2d[0]);
    }

    /**
     * Drive the current swerve module using optimization.
     * Inputs are assumed to be on a scale from 0 to 1
     * @param State Un-Optimized state
     */
    public void drive(SwerveModuleState State) {
        SwerveModuleState OptimizedState = optimizeState(
            State, 
            // Assume reading is degrees
            new Rotation2d(
                Math.toRadians(motors.SteerEncoder.getAbsolutePosition())
            )
        );

        // Set drive motor

        motors.DriveMotor.set(
            OptimizedState.speedMetersPerSecond
        );

        // Set turn motor
        motors.SteerMotor.set(
            SwerveMotors.signedAngleBetween(OptimizedState.angle, motors.getRotation2d()).getRadians() * Swerve.turningkP
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

    /**
     * Updates and constructs the field position of this swerve module.
     * This should be ran periodically, and as frequently as possible.
     * 
     * @return AccumulatedRelativePositionMeters
     */
    public Translation2d updateFieldRelativePosition() {
        // Handoff previous value and update
        double lastAccumulatedDriveDistance_h = lastAccumulatedDriveDistance;
        lastAccumulatedDriveDistance = motors.getDriveDistanceMeters();
        // This velocity is not for an accurate velocity reading, rather to catch a large
        // jump in drive distance. Grabbing velocity from drive motor will not catch this
        // error.
        double velocityFromDriveDistance = lastAccumulatedDriveDistance - lastAccumulatedDriveDistance_h;
        // For an accurate velocity reading, kMotors.DriveMotor.getEncoder().getVelocity()

        // Make sure this error does not exist!
        // Catch velocity error, and reset position with current robot pos
        // Assume a polling rate of 0.2 seconds.
        if (SwerveMotors.metersToRotations(velocityFromDriveDistance * 0.2) > Swerve.MaxMetersPerSecond) {
            // notify on driver station
            System.out.println(
                "Swerve module " + instance.name() + " has encountered a drive motor encoder failure. " +
                "Attempting recalibration from sibling modules"
            );
            // If this is ran, the swerve module needs to be reset
            // Calculate position from all swerve module instances.
            Translation2d posSum = new Translation2d();
            for (SwerveModule swerveModule : getInstances()) {
                // Average from all other 3
                if (swerveModule.instance != instance) {

                    posSum = posSum.plus(
                        swerveModule.getAssumedRobotFieldRelativePosition()
                    );
                }
            }

            // Return corrected position.
            return AccumulatedRelativePositionMeters = posSum.div(Instances.size() - 1);
            // End function
        }
        
        // Else no velocity error
        // Calculate delta to add to last accumulated position
        Translation2d movementVectorMeters = new Translation2d(
            velocityFromDriveDistance, // Delta of drive distance
            // Sum is bounded by -pi to pi
            motors.getRotation2d().plus(navxGyro.getRotation2d())
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
        AccumulatedRelativePositionMeters = Position.getTranslation().plus(instance.getPosition());
    }
}
