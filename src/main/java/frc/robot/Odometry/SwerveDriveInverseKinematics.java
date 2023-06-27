package frc.robot.Odometry;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Swerve.SwerveModule;

public class SwerveDriveInverseKinematics extends OdometrySource {

    final ShuffleboardTab swerveTab;

    final SwerveModule[] swerveModules;

    final SwerveDriveOdometry m_odometry;
    final GenericEntry xPosition, yPosition;

    final AHRS navxGryo;

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
        this.navxGryo = navxGyro;
        this.swerveTab = Tab;

        m_odometry = new SwerveDriveOdometry(
            Kinematics, 
            navxGryo.getRotation2d().times(-1),
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
            swerveModules[0].getEncoderPositions(),
            swerveModules[1].getEncoderPositions(),
            swerveModules[2].getEncoderPositions(),
            swerveModules[3].getEncoderPositions()
        };
    }

    /**
     * Updates current readings from swerve modules to calculate
     * position, should be ran from the drive train's periodic
     * function.
     */
    public void Update() {
        m_odometry.update(
            navxGryo.getRotation2d().times(-1), 
            getModulePositions()
        );

        xPosition.setDouble(m_odometry.getPoseMeters().getX());
        yPosition.setDouble(m_odometry.getPoseMeters().getY());
    }

    @Override
    public Pose2d getPosition() {
        // TODO will need to be tested
        // update method automatically calculates period, so
        // this likely wont need to be ran periodically if the period
        // is calculated every frame
        m_odometry.update(
            navxGryo.getRotation2d(), 
            getModulePositions()
        );

        return m_odometry.getPoseMeters();
    }

    @Override
    public void setPosition(Pose2d Position) {
        m_odometry.resetPosition(
            navxGryo.getRotation2d(), 
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
