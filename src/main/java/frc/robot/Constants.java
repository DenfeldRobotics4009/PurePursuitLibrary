// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class Swerve {

        public static final class FrontLeft {
            public static int driveID = 21;
            public static int steerID = 31;
            public static int CANCoderID = 1;
            public static Rotation2d defaultCalibration = 
                new Rotation2d(Math.toRadians(49.5));
            public static Translation2d trackPosition =
                new Translation2d(-Swerve.TrackYMeters/2, Swerve.TrackXMeters/2);
        }

        public static final class FrontRight {
            public static int driveID = 23;
            public static int steerID = 33;
            public static int CANCoderID = 2;
            public static Rotation2d defaultCalibration = 
                new Rotation2d(Math.toRadians(200));
            public static Translation2d trackPosition =
                new Translation2d(Swerve.TrackYMeters/2, Swerve.TrackXMeters/2);
        }

        public static final class BackLeft {
            public static int driveID = 22;
            public static int steerID = 32;
            public static int CANCoderID = 3;
            public static Rotation2d defaultCalibration = 
                new Rotation2d(Math.toRadians(2));
            public static Translation2d trackPosition =
                new Translation2d(-Swerve.TrackYMeters/2, -Swerve.TrackXMeters/2);
        }

        public static final class BackRight {
            public static int driveID = 24;
            public static int steerID = 34;
            public static int CANCoderID = 0;
            public static Rotation2d defaultCalibration = 
                new Rotation2d(Math.toRadians(54));
            public static Translation2d trackPosition =
                new Translation2d(Swerve.TrackYMeters/2, -Swerve.TrackXMeters/2);
        }

        // Meters from wheel center to wheel center
        public static double TrackYMeters = 0.65405;
        public static double TrackXMeters = 0.57785;

        // public static double MaxMetersPerSecond = 3.6576
        public static double MaxMetersPerSecond = 5.06;
        public static double MaxRadPerSecond = MaxMetersPerSecond /
            Math.hypot(TrackYMeters / 2.0, TrackXMeters / 2.0);

        public static double wheelDiameterMeters = 0.10308;
        public static double swerveGearRatio = 1 / 6.2; // Rotations of motor per rotations of wheel
        public static double swerveSteerGearRatio = 150 / 7; // Rotations of motor per rotations of wheel
        public static double rotationsToMeters = (Math.PI * wheelDiameterMeters * swerveGearRatio); 

        public static double MaxRotationsPerSecond = 5676;

        // Time from 0% to 100% speed
        public static double driveRampRateSeconds = 0.2;
        public static double steerRampRateSeconds = 0.2; 

        public static Rotation2d forwardAngle = new Rotation2d(Math.toRadians(90));
    }
}
