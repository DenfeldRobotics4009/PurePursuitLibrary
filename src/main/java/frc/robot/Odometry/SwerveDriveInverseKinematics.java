package frc.robot.Odometry;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Swerve.SwerveModule;
import frc.robot.subsystems.Swerve.SwerveTranslationFrame;

public class SwerveDriveInverseKinematics extends OdometrySource {

    /**
     * @see https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html
     * for further documentation on the SwerveDriveOdometry class.
     * 
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

    final SwerveModule[] swerveModules;

    final AHRS navxGyro;

    Pose2d currentPose;

    /**
     * @param SwerveModules 4 Swerve modules to calculate position from
     */
    public SwerveDriveInverseKinematics(
        SwerveModule[] SwerveModules, 
        AHRS navxGyro,
        ShuffleboardTab Tab
    ) {
        this.swerveModules = SwerveModules;
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
        for (SwerveModule swerveModule : swerveModules) {
            swerveModule.updateFieldRelativePosition();
        }

        Translation2d wheelPosSum = new Translation2d();

        for (SwerveModule swerveModule : swerveModules) {
            wheelPosSum = wheelPosSum.plus(swerveModule.getFieldRelativePosition());
        }

        currentPose =  new Pose2d (
            wheelPosSum.div(swerveModules.length), 
            navxGyro.getRotation2d()
        );
    }

    @Override
    public Pose2d getPosition() {
        return currentPose;
    }

    @Override
    public void setPosition(Pose2d Position) {
        for (SwerveModule swerveModule : swerveModules) {
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
