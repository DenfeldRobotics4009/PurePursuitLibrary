// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.SwerveModule;
import frc.robot.SwerveMotors;
import frc.robot.Constants.Swerve;
import frc.robot.Odometry.SwerveDriveInverseKinematics;

public class SwerveDrive extends SubsystemBase {

  SwerveModule 
    FrontLeftModule = new SwerveModule(
        new SwerveMotors(Swerve.FrontLeftDriveMotor, Swerve.FrontLeftSteerMotor, Swerve.FrontLeftCANCoder),
        new Translation2d(-Swerve.TrackWidthMeters/2, Swerve.TrackLengthMeters/2)
      ),
    FrontRightModule = new SwerveModule(
        new SwerveMotors(Swerve.FrontRightDriveMotor, Swerve.FrontRightSteerMotor, Swerve.FrontRightCANCoder),
        new Translation2d(Swerve.TrackWidthMeters/2, Swerve.TrackLengthMeters/2)
      ),
    BackLeftModule = new SwerveModule(
        new SwerveMotors(Swerve.BackLeftDriveMotor, Swerve.BackLeftSteerMotor, Swerve.BackLeftCANCoder),
        new Translation2d(-Swerve.TrackWidthMeters/2, -Swerve.TrackLengthMeters/2)
      ),
    BackRightModule = new SwerveModule(
        new SwerveMotors(Swerve.BackRightDriveMotor, Swerve.BackRightSteerMotor, Swerve.BackRightCANCoder),
        new Translation2d(Swerve.TrackWidthMeters/2, -Swerve.TrackLengthMeters/2)
      );
  
  final SwerveModule[] swerveModules = {FrontLeftModule, FrontRightModule, BackLeftModule, BackRightModule};

  SwerveDriveKinematics kinematics;

  AHRS navxGyro = new AHRS();

  /**
   * Object to tract the robots position via inverse kinematics
   */
  SwerveDriveInverseKinematics inverseKinematics;

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {

    inverseKinematics = new SwerveDriveInverseKinematics(kinematics, swerveModules, navxGyro);

    kinematics = new SwerveDriveKinematics(
      FrontLeftModule.getPosition(), FrontRightModule.getPosition(),
      BackLeftModule.getPosition(), BackRightModule.getPosition()
    );
  }

  @Override
  public void periodic() {}

  public void drive(ChassisSpeeds Speeds) {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(Speeds);
    for (int i = 0; i < 4; i++) {swerveModules[i].drive(states[i]);}
  }
}
