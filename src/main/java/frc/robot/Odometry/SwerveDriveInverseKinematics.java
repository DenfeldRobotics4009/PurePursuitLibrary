package frc.robot.Odometry;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.Swerve.SwerveModule;

public class SwerveDriveInverseKinematics extends OdometrySource {

    final SwerveModule[] swerveModules;

    final SwerveDriveOdometry m_odometry;

    final AHRS navxGryo;

    /**
     * @param SwerveModules 4 Swerve modules to calculate position from
     */
    public SwerveDriveInverseKinematics(
        SwerveDriveKinematics Kinematics, 
        SwerveModule[] SwerveModules, 
        AHRS navxGyro
    ) {
        this.swerveModules = SwerveModules;
        this.navxGryo = navxGyro;

        m_odometry = new SwerveDriveOdometry(
            Kinematics, 
            null,
            new SwerveModulePosition[] {
                swerveModules[0].getEncoderPositions(),
                swerveModules[1].getEncoderPositions(),
                swerveModules[2].getEncoderPositions(),
                swerveModules[3].getEncoderPositions()
            },
            null // TODO Starting pos must be grabbed from odometry handler
        );
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
            navxGryo.getRotation2d(), 
            getModulePositions()
        );
    }

    @Override
    Pose2d getPosition() {
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
    void setPosition(Pose2d Position) {
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
    Pose2d getVelocity() {
        return null;
    }

    @Override
    Pose2d getAcceleration() {
        return null;
    }
    
}
