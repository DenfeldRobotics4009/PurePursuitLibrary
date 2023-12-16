// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Swerve;

/**
 * Multiton pattern map for SwerveModule
 */
public enum SwerveModuleInstance {

    FRONT_LEFT {
        public final SwerveMotors motors = new SwerveMotors(Swerve.FrontLeft);

        @Override
        public Translation2d getPosition() {
            return new Translation2d(-Swerve.TrackYMeters/2, Swerve.TrackXMeters/2);
        }

        @Override
        public SwerveMotors getMotors() {
            return motors;
        }
    }, 
    FRONT_RIGHT {
        public final SwerveMotors motors = new SwerveMotors(Swerve.FrontRight);

        @Override
        public Translation2d getPosition() {
            return new Translation2d(Swerve.TrackYMeters/2, Swerve.TrackXMeters/2);
        }

        @Override
        public SwerveMotors getMotors() {
            return motors;
        }
    }, 
    BACK_LEFT {
        public final SwerveMotors motors = new SwerveMotors(Swerve.BackLeft);

        @Override
        public Translation2d getPosition() {
            return new Translation2d(-Swerve.TrackYMeters/2, -Swerve.TrackXMeters/2);
        }

        @Override
        public SwerveMotors getMotors() {
            return motors;
        }
    }, 
    BACK_RIGHT {
        public final SwerveMotors motors = new SwerveMotors(Swerve.BackRight);

        @Override
        public Translation2d getPosition() {
            return new Translation2d(Swerve.TrackYMeters/2, -Swerve.TrackXMeters/2);
        }

        @Override
        public SwerveMotors getMotors() {
            return motors;
        }
    };

    public abstract Translation2d getPosition();

    public abstract SwerveMotors getMotors();
}