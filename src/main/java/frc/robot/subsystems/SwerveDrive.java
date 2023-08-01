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
import frc.robot.Constants.Swerve;
import frc.robot.libraries.PIDController;
import frc.robot.odometry.sources.SwerveDriveInverseKinematics;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveModuleInstance;

public class SwerveDrive extends SubsystemBase {

  final ShuffleboardTab SwerveTab = Shuffleboard.getTab("Swerve");

  final PIDController 
    xController = new PIDController(0.05, 0, 0, 0), 
    yController = new PIDController(0.05, 0, 0, 0),
    thetaController = new PIDController(0.05, 0, 0, 0);

  // Construct swerve modules
  final SwerveModule 
    FrontLeftModule = SwerveModule.getInstance(SwerveModuleInstance.FRONT_LEFT, SwerveTab, navxGyro),
    FrontRightModule = SwerveModule.getInstance(SwerveModuleInstance.FRONT_RIGHT, SwerveTab, navxGyro),
    BackLeftModule = SwerveModule.getInstance(SwerveModuleInstance.BACK_LEFT, SwerveTab, navxGyro),
    BackRightModule = SwerveModule.getInstance(SwerveModuleInstance.BACK_RIGHT, SwerveTab, navxGyro);

  SwerveDriveKinematics kinematics;

  public static AHRS navxGyro = new AHRS();

  GenericEntry gyroAngle;

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
      // Lambda boundary
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

  /**
   * Drives the robot to the given position via PControllers
   * @param Position goal position
   * @return Units from destination
   */
  public double goTo(Pose2d Position) {


    xController.setTarget(Position.getX());
    yController.setTarget(Position.getY());

    // Zero is being set for CalculateDistCorrection function
    thetaController.setTarget(0);

    xController.setInput(inverseKinematics.getPosition().getX());
    yController.setInput(inverseKinematics.getPosition().getY());

    thetaController.setInput(
      // See function documentation
      SwerveModule.calculateDistanceCorrection(
        Position.getRotation().getDegrees(), 
        navxGyro.getRotation2d().getDegrees()
      )
    );

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
