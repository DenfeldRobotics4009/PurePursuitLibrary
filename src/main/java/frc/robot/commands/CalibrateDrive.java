// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveModule;

public class CalibrateDrive extends CommandBase {

  SwerveDrive m_Drive;

  /**
   * Calibrates the swerve module steer directions when executed
   * The current position of the swerve wheels will become the new 0
   * @param Drivetrain
   */
  public CalibrateDrive(SwerveDrive Drivetrain) {
    m_Drive = Drivetrain;
    addRequirements(Drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    for (SwerveModule swerveModule : SwerveModule.getInstances()) {
      // Set the genericEntry calibration angle to the modules current angle
      swerveModule.calibrationAngle.setDouble(
        // Add the current adjusted rotation2d to the last calibration
        // angle to properly offset
        swerveModule.calibrationAngle.getDouble(0) + 
        swerveModule.motors.getRotation2d().getDegrees()
      );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true; // Run once
  }
}
