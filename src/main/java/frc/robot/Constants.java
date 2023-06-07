// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

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
        public static double TrackWidthMeters = 1;
        public static double TrackLengthMeters = 1;

        public static double MaxMetersPerSecond = 1;
        public static double MaxRadPerSecond = 1;

        public static int FrontLeftDriveMotor = 0;
        public static int FrontLeftSteerMotor = 0;
        public static int FrontLeftCANCoder = 0;

        public static int FrontRightDriveMotor = 0;
        public static int FrontRightSteerMotor = 0;
        public static int FrontRightCANCoder = 0;

        public static int BackLeftDriveMotor = 0;
        public static int BackLeftSteerMotor = 0;
        public static int BackLeftCANCoder = 0;

        public static int BackRightDriveMotor = 0;
        public static int BackRightSteerMotor = 0;
        public static int BackRightCANCoder = 0;
    }
}
