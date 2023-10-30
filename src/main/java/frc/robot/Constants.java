// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

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

        public static class SwerveModuleConstants {
            public final int DriveMotorID, SteerMotorID, CANCoderID;
            public final Rotation2d DefaultOffset;
            public final String Name;

            public SwerveModuleConstants (
                int DriveMotorID, 
                int SteerMotorID, 
                int CANCoderID, 
                Rotation2d DefaultOffset,
                String Name
            ) {
                this.DriveMotorID = DriveMotorID;
                this.SteerMotorID = SteerMotorID;
                this.CANCoderID = CANCoderID;
                this.DefaultOffset = DefaultOffset;
                this.Name = Name;
            }
        }

        // Meters from wheel center to wheel center
        public static double TrackWidthMeters = 0.57785;
        public static double TrackLengthMeters = 0.65405;

        public static double MaxMetersPerSecond = 3.6576; // TODO not accurate bozo

        public static double MaxRadPerSecond = MaxMetersPerSecond /
            Math.hypot(TrackWidthMeters / 2.0, TrackLengthMeters / 2.0);

        public static double wheelDiameterMeters = 0.10308;
        public static double swerveGearRatio = 1 / 6.2; // Rotations of motor per rotations of wheel
        public static double rotationsToMeters = (Math.PI * wheelDiameterMeters * swerveGearRatio); 

        public static double MaxRotationsPerSecond = 5676;

        public static double MaxAccelerationMetersPerSecondPerSecond = 1; // TODO Calculate

        public static SwerveModuleConstants FrontLeft = new SwerveModuleConstants(
            21, 31, 1, new Rotation2d(Math.toRadians(48)), "FrontLeft"
        );

        public static SwerveModuleConstants FrontRight = new SwerveModuleConstants(
            23, 33, 2, new Rotation2d(Math.toRadians(200)), "FrontRight"
        );
       public static SwerveModuleConstants BackLeft = new SwerveModuleConstants(
          22, 32, 3, new Rotation2d(Math.toRadians(4)), "BackLeft"
        );

        public static SwerveModuleConstants BackRight = new SwerveModuleConstants(
            24, 34, 0, new Rotation2d(Math.toRadians(54)), "BackRight"
        );
    }
}
