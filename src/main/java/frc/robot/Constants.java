// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static int NEOMaxRPM = 5676;

    public static final class Swerve {
        // Meters from wheel center to wheel center
        public static double TrackWidthMeters = 0.57785;
        public static double TrackLengthMeters = 0.65405;

        public static double MaxMetersPerSecond = 3.6576;

        public static double MaxRadPerSecond = MaxMetersPerSecond /
            Math.hypot(TrackWidthMeters / 2.0, TrackLengthMeters / 2.0);

        public static int FrontLeftDriveMotor = 21;
        public static int FrontLeftSteerMotor = 31;
        public static int FrontLeftCANCoder = 0;
        public static Rotation2d FrontLeftCANCoderOffset = new Rotation2d(Math.toRadians(-5));

        public static int FrontRightDriveMotor = 23;
        public static int FrontRightSteerMotor = 33;
        public static int FrontRightCANCoder = 2;
        public static final Rotation2d FrontRightCANCoderOffset = new Rotation2d(Math.toRadians(-120));

        public static int BackLeftDriveMotor = 22;
        public static int BackLeftSteerMotor = 32;
        public static int BackLeftCANCoder = 1;
        public static final Rotation2d BackLeftCANCoderOffset = new Rotation2d(Math.toRadians(109.5));

        public static int BackRightDriveMotor = 24;
        public static int BackRightSteerMotor = 34;
        public static int BackRightCANCoder = 3;
        public static final Rotation2d BackRightCANCoderOffset = new Rotation2d(Math.toRadians(-59));
    }
}
