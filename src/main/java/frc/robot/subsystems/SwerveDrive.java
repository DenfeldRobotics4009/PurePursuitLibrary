// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;
import frc.robot.auto.DriveSubsystem;
import frc.robot.commands.CalibrateDrive;
import frc.robot.commands.CalibrateGyro;
import frc.robot.odometry.sources.SwerveDriveInverseKinematics;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveModuleInstance;

public class SwerveDrive extends SubsystemBase implements DriveSubsystem {

  final ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
  final Field2d feildWidget = new Field2d();

  // Construct swerve modules
  final SwerveModule
    // Pass swerve tab into modules to allow each of them to display relevant data, whatever that may be
    FrontLeftModule = SwerveModule.getInstance(
      SwerveModuleInstance.FRONT_LEFT, 
      createCalibrationWidget(SwerveModuleInstance.FRONT_LEFT, 0, 6, 3, 1),
      navxGyro
    ),
    FrontRightModule = SwerveModule.getInstance(
      SwerveModuleInstance.FRONT_RIGHT,
      createCalibrationWidget(SwerveModuleInstance.FRONT_RIGHT, 0, 7, 3, 1),
      navxGyro
    ),
    BackLeftModule = SwerveModule.getInstance(
      SwerveModuleInstance.BACK_LEFT, 
      createCalibrationWidget(SwerveModuleInstance.BACK_LEFT, 0, 8, 3, 1),
      navxGyro
    ),
    BackRightModule = SwerveModule.getInstance(
      SwerveModuleInstance.BACK_RIGHT, 
      createCalibrationWidget(SwerveModuleInstance.BACK_RIGHT, 0, 9, 3, 1),
      navxGyro
    );

  SwerveDriveKinematics kinematics;

  public static AHRS navxGyro = new AHRS();

  // Entries for motion graphing
  GenericEntry 
    xPositionEntry = swerveTab.add("xPosition", 0
      ).withPosition(3, 4).withSize(2, 1).getEntry(), 
    yPositionEntry = swerveTab.add("yPosition", 0
      ).withPosition(5, 4).withSize(2, 1).getEntry(), 
    rotationEntry = swerveTab.add("Rotation", 0
      ).withPosition(3, 0).withSize(4, 4).withWidget("Gyro").getEntry(),

    xVelocityEntry = swerveTab.add("xVelocity", 0
      ).withPosition(0, 4).withSize(3, 1).getEntry(), 
    yVelocityEntry = swerveTab.add("yVelocity", 0
      ).withPosition(0, 5).withSize(3, 1).getEntry(), 
    rotationVelocityEntry = swerveTab.add("rotationVelocity", 0
      ).withPosition(0, 0).withSize(3, 4).withWidget("Gyro").getEntry();

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

    inverseKinematics = SwerveDriveInverseKinematics.getInstance(navxGyro);

    navxGyro.setAngleAdjustment(-Swerve.forwardAngle.getDegrees());
    navxGyro.calibrate();

    // Add calibrate commands to shuffleboard
    swerveTab.add("Calibrate Position", new CalibrateDrive(this, new Pose2d())
      ).withPosition(3, 5).withSize(4, 1);
    swerveTab.add("Calibrate Gyro", new CalibrateGyro()
      ).withPosition(3, 6).withSize(4, 1);

    // Construct feild widget
    swerveTab.add("Robot Position", feildWidget
      ).withPosition(7, 0).withSize(18, 10);
  }

  /**
   * Creates calibration angle generic entry, just for clarity 
   */
  GenericEntry createCalibrationWidget(
    SwerveModuleInstance instance, int posX, int posY, int sizeX, int sizeY
  ) {
    return swerveTab.add(
      instance.name() +  " Calibration Angle", 0
    ).withPosition(posX, posY).withSize(sizeX, sizeY).getEntry();
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

    feildWidget.setRobotPose(
      new Pose2d(
        getPosition().getTranslation().plus(new Translation2d(1.8, 0.4)), new Rotation2d()
      )
    );
  }

  /**
   * Drives the robot in a robot oriented manner, if field oriented is
   * desired, inputs must be rotated by calling function accordingly.
   * @param Speeds Robot relative chassis speeds on a scale from 0 to 1.
   */
  public void drive(ChassisSpeeds Speeds) {

    xVelocityEntry.setDouble(Speeds.vxMetersPerSecond);
    yVelocityEntry.setDouble(Speeds.vyMetersPerSecond);
    rotationVelocityEntry.setDouble(Math.toDegrees(Speeds.omegaRadiansPerSecond));

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(Speeds);
    for (int i = 0; i < 4; i++) {
      SwerveModule.getInstances()[i].drive(states[i]);
    }
  }

  public Pose2d getPosition() {
    return inverseKinematics.getPosition();
  }

  /**
   * Grabs velocity from generic entry table,
   * not from sensor collections
   * @return A Pose2d with translation values bounded from -1 to 1
   */
  public Pose2d getVelocity() {
    return new Pose2d(
      new Translation2d(
        xVelocityEntry.getDouble(0), yVelocityEntry.getDouble(0)
      ), 
      new Rotation2d(
        Math.toRadians(
          rotationVelocityEntry.getDouble(0)
        )
      )
    );
  }

  public void setPosition(Pose2d position) {
    inverseKinematics.setPosition(position);
  }
}
