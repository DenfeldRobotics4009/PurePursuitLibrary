// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.odometry.sources.SwerveDriveInverseKinematics;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveModuleInstance;

public class SwerveDrive extends SubsystemBase {

  final ShuffleboardTab SwerveTab = Shuffleboard.getTab("Swerve");
  GenericEntry gyroAngle;

  // Construct swerve modules
  final SwerveModule
    FrontLeftModule = SwerveModule.getInstance(SwerveModuleInstance.FRONT_LEFT, SwerveTab, navxGyro),
    FrontRightModule = SwerveModule.getInstance(SwerveModuleInstance.FRONT_RIGHT, SwerveTab, navxGyro),
    BackLeftModule = SwerveModule.getInstance(SwerveModuleInstance.BACK_LEFT, SwerveTab, navxGyro),
    BackRightModule = SwerveModule.getInstance(SwerveModuleInstance.BACK_RIGHT, SwerveTab, navxGyro);

  SwerveDriveKinematics kinematics;

  public static AHRS navxGyro = new AHRS();

  /**
   * Object to track the robots position via inverse kinematics
   */
  public static SwerveDriveInverseKinematics inverseKinematics;

  static SwerveDrive Instance;

  public static SwerveDrive GetInstance() {
    if (Instance == null) {
      Instance = new SwerveDrive();
    }
    return Instance;
  }

  /** Creates a new SwerveDrive. */
  private SwerveDrive() {

    // Initialize during constructor to avoid building kinematics
    // object with uninitialized swerve modules.
    kinematics = new SwerveDriveKinematics(
      // Parse through initialized hash map values
      SwerveModule.getRobotRelativePositions()
    );

    inverseKinematics = SwerveDriveInverseKinematics.getInstance(navxGyro, SwerveTab);

    navxGyro.calibrate();
  
    gyroAngle = SwerveTab.add("Gyro Angle", 0).getEntry();
  }

  @Override
  public void periodic() {
    gyroAngle.setDouble(navxGyro.getRotation2d().getDegrees());

    // Update swerve module calibration from shuffleboard
    SwerveModule.forEach(
      (instance, swerveModule) -> {
        swerveModule.updateCalibration();
      }
    );

    inverseKinematics.Update();
  }

  public void drive(ChassisSpeeds Speeds) {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(Speeds);
    for (int i = 0; i < 4; i++) {
      SwerveModule.getInstances()[i].drive(states[i]);
    }
  }
}
