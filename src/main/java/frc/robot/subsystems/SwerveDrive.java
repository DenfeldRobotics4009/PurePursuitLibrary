// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.CalibrateDrive;
import frc.robot.odometry.sources.SwerveDriveInverseKinematics;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveModuleInstance;

public class SwerveDrive extends SubsystemBase {

  final ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");

  // Construct swerve modules
  final SwerveModule
    // Pass swerve tab into modules to allow each of them to display relevant data, whatever that may be
    FrontLeftModule = SwerveModule.getInstance(SwerveModuleInstance.FRONT_LEFT, swerveTab, navxGyro),
    FrontRightModule = SwerveModule.getInstance(SwerveModuleInstance.FRONT_RIGHT, swerveTab, navxGyro),
    BackLeftModule = SwerveModule.getInstance(SwerveModuleInstance.BACK_LEFT, swerveTab, navxGyro),
    BackRightModule = SwerveModule.getInstance(SwerveModuleInstance.BACK_RIGHT, swerveTab, navxGyro);

  SwerveDriveKinematics kinematics;

  public static AHRS navxGyro = new AHRS();

  // Entries for motion graphing
  GenericEntry 
    xPositionEntry = swerveTab.add("xPosition", 0).getEntry(), 
    yPositionEntry = swerveTab.add("yPosition", 0).getEntry(),
    rotationEntry = swerveTab.add("Rotation", 0).getEntry();

  /**
   * Object to track the robots position via inverse kinematics
   */
  public static SwerveDriveInverseKinematics inverseKinematics;

  static SwerveDrive instance;

  public static SwerveDrive GetInstance() {
    if (instance == null) {
      instance = new SwerveDrive();
    }
    return instance;
  }

  /** Creates a new SwerveDrive. */
  private SwerveDrive() {

    // Initialize during constructor to avoid building kinematics
    // object with uninitialized swerve modules.
    kinematics = new SwerveDriveKinematics(
      // Parse through initialized hash map values
      SwerveModule.getRobotRelativePositions()
    );

    inverseKinematics = SwerveDriveInverseKinematics.getInstance(navxGyro, swerveTab);

    navxGyro.calibrate();

    // Add calibrate command to shuffleboard
    swerveTab.add("Calibrate", new CalibrateDrive(instance));
  }

  @Override
  public void periodic() {

    // Update swerve module calibration from shuffleboard
    SwerveModule.forEach(
      (instance, swerveModule) -> {
        swerveModule.updateCalibration();
      }
    );

    inverseKinematics.Update();

    // Displaying position values
    xPositionEntry.setDouble(inverseKinematics.getPosition().getX());
    yPositionEntry.setDouble(inverseKinematics.getPosition().getY());
    rotationEntry.setDouble(navxGyro.getRotation2d().getDegrees());
  }

  /**
   * Drives the robot in a robot oriented manner, if field oriented is
   * desired, inputs must be rotated by calling function accordingly.
   * @param Speeds Robot relative chassis speeds on a scale from 0 to 1.
   */
  public void drive(ChassisSpeeds Speeds) {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(Speeds);
    for (int i = 0; i < 4; i++) {
      SwerveModule.getInstances()[i].drive(states[i]);
    }
  }

  public Pose2d getPosition() {
    return inverseKinematics.getPosition();
  }
}
