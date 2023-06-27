// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;
import frc.robot.Odometry.SwerveDriveInverseKinematics;
import frc.robot.libraries.PIDController;
import frc.robot.subsystems.Swerve.SwerveModule;
import frc.robot.subsystems.Swerve.SwerveMotors;

public class SwerveDrive extends SubsystemBase {

  final ShuffleboardTab SwerveTab = Shuffleboard.getTab("Swerve");

  final PIDController 
    xController = new PIDController(0.05, 0, 0, 0), 
    yController = new PIDController(0.05, 0, 0, 0);

  SwerveModule 
    FrontLeftModule = new SwerveModule(
        new SwerveMotors(Swerve.FrontLeft),
        new Translation2d(Swerve.TrackWidthMeters/2, -Swerve.TrackLengthMeters/2),
        SwerveTab
      ),
    FrontRightModule = new SwerveModule(
        new SwerveMotors((Swerve.FrontRight)),
        new Translation2d(Swerve.TrackWidthMeters/2, Swerve.TrackLengthMeters/2),
        SwerveTab
      ),
    BackLeftModule = new SwerveModule(
        new SwerveMotors(Swerve.BackLeft),
        new Translation2d(-Swerve.TrackWidthMeters/2, -Swerve.TrackLengthMeters/2),
        SwerveTab
      ),
    BackRightModule = new SwerveModule(
        new SwerveMotors(Swerve.BackRight),
        new Translation2d(-Swerve.TrackWidthMeters/2, Swerve.TrackLengthMeters/2),
        SwerveTab
      );
  
  final SwerveModule[] swerveModules = {FrontLeftModule, FrontRightModule, BackLeftModule, BackRightModule};

  SwerveDriveKinematics kinematics;

  public static AHRS navxGyro = new AHRS();

  GenericEntry gyroAngle;

  /**
   * Object to tract the robots position via inverse kinematics
   */
  public static SwerveDriveInverseKinematics inverseKinematics;

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {

    kinematics = new SwerveDriveKinematics(
      swerveModules[0].getPosition(), swerveModules[1].getPosition(),
      swerveModules[2].getPosition(), swerveModules[3].getPosition()
    );

    inverseKinematics = new SwerveDriveInverseKinematics(
      kinematics, swerveModules, navxGyro, SwerveTab
    );
  
    gyroAngle = SwerveTab.add("Gyro Angle", 0).getEntry();
  }

  @Override
  public void periodic() {
    gyroAngle.setDouble(navxGyro.getRotation2d().getDegrees());

    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.updateCalibration();
    }

    inverseKinematics.Update();
  }

  public void drive(ChassisSpeeds Speeds) {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(Speeds);
    for (int i = 0; i < 4; i++) {swerveModules[i].drive(states[i]);}
  }

  /**
   * Drives the robot to the given position via PControllers
   * @param Position goal position
   * @return Units from destination
   */
  public double goTo(Pose2d Position) {
    xController.setTarget(Position.getX());
    yController.setTarget(Position.getY());

    xController.setInput(inverseKinematics.getPosition().getX());
    yController.setInput(inverseKinematics.getPosition().getY());

    Transform2d difference = Position.minus(inverseKinematics.getPosition());

    drive(
      new ChassisSpeeds(
        xController.calculate(1, -1) * Swerve.MaxMetersPerSecond, 
        yController.calculate(1, -1) * Swerve.MaxMetersPerSecond,
        0
      )
    );

    return Math.hypot(difference.getX(), difference.getY());
  }
}
