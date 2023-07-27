package frc.robot.Odometry;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
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

    final SwerveDriveOdometry m_odometry;

    final AHRS navxGyro;

    /**
     * @param SwerveModules 4 Swerve modules to calculate position from
     */
    public SwerveDriveInverseKinematics(
        SwerveDriveKinematics Kinematics, 
        SwerveModule[] SwerveModules, 
        AHRS navxGyro,
        ShuffleboardTab Tab
    ) {
        this.swerveModules = SwerveModules;
        this.navxGyro = navxGyro;
        this.swerveTab = Tab;

        m_odometry = new SwerveDriveOdometry(
            Kinematics, 
            navxGyro.getRotation2d().unaryMinus(),
            getModulePositions(),
            new Pose2d() // TODO Starting pos must be grabbed from odometry handler
        );

        xPosition = Tab.add("XPosition", 0).getEntry();
        yPosition = Tab.add("YPosition", 0).getEntry();
    }

    /**
     * @return Array of all swerve modules positions
     */
    SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            swerveModules[0].getSwerveModulePosition(),
            swerveModules[1].getSwerveModulePosition(),
            swerveModules[2].getSwerveModulePosition(),
            swerveModules[3].getSwerveModulePosition()
        };
    }

    SwerveTranslationFrame[] updateSwerveTranslationFrames() {
        return new SwerveTranslationFrame[] {
            swerveModules[0].updateMovementVector(),
            swerveModules[1].updateMovementVector(),
            swerveModules[2].updateMovementVector(),
            swerveModules[3].updateMovementVector()
        };
    }
 
    /**
     * Updates current readings from swerve modules to calculate
     * position, should be ran from the drive train's periodic
     * function.
     */
    public void Update() {

        m_odometry.update(
            navxGyro.getRotation2d().unaryMinus(), 
            getModulePositions()
        );

        xPosition.setDouble(
            m_odometry.getPoseMeters().getX()
        );

        yPosition.setDouble(
            m_odometry.getPoseMeters().getY()
        );
    }

    @Override
    public Pose2d getPosition() {
        // update method automatically calculates period, this
        // should be ran as fast as possible for more
        // accurate position tracking
        m_odometry.update(
            navxGyro.getRotation2d().unaryMinus(), 
            getModulePositions()
        );

        return m_odometry.getPoseMeters();
    }

    @Override
    public void setPosition(Pose2d Position) {
        m_odometry.resetPosition(
            navxGyro.getRotation2d().unaryMinus(), 
            getModulePositions(), 
            Position
        );
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
