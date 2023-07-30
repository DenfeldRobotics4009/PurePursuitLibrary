package frc.robot.odometry;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.swerve.SwerveModule;

public class SwerveDriveInverseKinematics extends OdometrySource {

    /**
     * Zero degrees on the gyroscope represents facing the opposite
     * alliance driver station. As the robot turns left, the gyro
     * scope value should increase.
     * 
     * @see https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#field-coordinate-system
     * for further reading on the field coordinate system standard.
     */

    // Shuffleboard display
    final ShuffleboardTab swerveTab;
    final GenericEntry xPosition, yPosition;

    final AHRS navxGyro;

    Pose2d currentPose;

    private static SwerveDriveInverseKinematics Instance;

    public static SwerveDriveInverseKinematics getInstance(
        AHRS navxGyro, ShuffleboardTab Tab
    ) {
        if (Instance == null) {
            Instance = new SwerveDriveInverseKinematics(navxGyro, Tab);
        }

        return Instance;
    }

    /**
     * @param SwerveModules 4 Swerve modules to calculate position from
     */
    private SwerveDriveInverseKinematics(
        AHRS navxGyro,
        ShuffleboardTab Tab
    ) {
        this.navxGyro = navxGyro;
        this.swerveTab = Tab;

        xPosition = Tab.add("XPosition", 0).getEntry();

        yPosition = Tab.add("YPosition", 0).getEntry();
    }
 
    /**
     * Updates current readings from swerve modules to calculate
     * position, should be ran from the drive train's periodic
     * function.
     */
    public void Update() {

        Translation2d wheelPosSum = new Translation2d();
        
        for (SwerveModule swerveModule : SwerveModule.getInstances()) {

            swerveModule.updateFieldRelativePosition();

            wheelPosSum = wheelPosSum.plus(swerveModule.getFieldRelativePosition());
        }

        currentPose =  new Pose2d (
            wheelPosSum.div(
                SwerveModule.getInstances().length
            ), 
            navxGyro.getRotation2d()
        );
    }

    @Override
    public Pose2d getPosition() {
        return currentPose;
    }

    @Override
    public void setPosition(Pose2d Position) {
        for (SwerveModule swerveModule : SwerveModule.getInstances()) {
            swerveModule.setFieldRelativePositionFromRobotPosition(Position);
        }
    }

    /**
     * Returning null just tells the odometry handler that this source cannot supply it
     */

    @Override
    public Pose2d getVelocity() {
        return null;
    }

    @Override
    public Pose2d getAcceleration() {
        return null;
    }
    
}
