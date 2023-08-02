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


public class SwerveModule {

    final SwerveModuleInstance Instance;

    final SwerveMotors kMotors;

    final ShuffleboardTab kSwerveTab;
    final GenericEntry calibrationAngle;
    final GenericEntry xPos;
    final GenericEntry yPos;
    final GenericEntry theta;

    private double lastAccumulatedDriveDistance = 0;

    private final AHRS navxGyro;

    /**
     * Initially set by odometry source constructor,
     * if not set, begin at zero
     */
    private Translation2d AccumulatedRelativePositionMeters = new Translation2d();

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

    /**
     * @return Robot relative position of swerve module
     */
    public Translation2d getRobotRelativePosition() {return Instance.getPosition();}

    // public static SwerveModule getInstance(int instance) {

    //     return null;
    // }

    /**
     * Multiton instances
     */
    private static final Map<SwerveModuleInstance, SwerveModule> Instances = new HashMap<>();

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
        ShuffleboardTab SwerveTab,
        AHRS NAVXGyro
    ) {
        SwerveModule swerveModule;

        if (!Instances.containsKey(ModulePosition)) {
            // Construct new module if not present
            swerveModule = new SwerveModule(ModulePosition, SwerveTab, NAVXGyro);

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
     * Construct a single swerve module
     * @param Motors Group of motors and encoder to use
     * 
     * Private constructor for multiton
     */
    private SwerveModule(
        SwerveModuleInstance ModulePosition, 
        ShuffleboardTab SwerveTab,
        AHRS NAVXGyro
    ) {
        this.kSwerveTab = SwerveTab;

        this.kMotors = ModulePosition.getMotors();
        this.Instance = ModulePosition;

        this.navxGyro = NAVXGyro;

        calibrationAngle = SwerveTab.addPersistent(
            kMotors.Name +  " Calibration Angle"
            , 0
        ).getEntry();

        xPos = SwerveTab.add(
            kMotors.Name + " PosX", 0
        ).getEntry();

        yPos = SwerveTab.add(
            kMotors.Name + " PosY", 0
        ).getEntry();

        theta = SwerveTab.add(
            kMotors.Name + " theta", 0
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
            kMotors.PositionTurnController.calculate(1, -1)
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
     * Updates and constructs the field position of this swerve module.
     * This should be ran periodically, and as frequently as possible.
     */
    public Translation2d updateFieldRelativePosition() {

        Translation2d movementVectorMeters = new Translation2d(
            kMotors.getDriveDistanceMeters() - lastAccumulatedDriveDistance,
            // Sum is bounded by -pi to pi
            kMotors.getRotation2d().plus(navxGyro.getRotation2d())
        );

        lastAccumulatedDriveDistance = kMotors.getDriveDistanceMeters();

        xPos.setDouble(AccumulatedRelativePositionMeters.getX());
        yPos.setDouble(AccumulatedRelativePositionMeters.getY());
        theta.setDouble(
            kMotors.getRotation2d().plus(navxGyro.getRotation2d()).getDegrees()
        );

        // Update single module tracking
        // Add last vector and current vector
        return AccumulatedRelativePositionMeters = AccumulatedRelativePositionMeters.plus(movementVectorMeters);
    }

    public void setFieldRelativePositionFromRobotPosition(Pose2d Position) {
        AccumulatedRelativePositionMeters = Position.getTranslation().plus(Instance.getPosition());
    }
}
