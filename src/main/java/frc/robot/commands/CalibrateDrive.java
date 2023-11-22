// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class CalibrateDrive extends CommandBase {

  SwerveDrive m_Drive;
  Pose2d position;

  /**
   * Calibrates the swerve module steer directions when executed
   * The current position of the swerve wheels will become the new 0
   * @param Drivetrain
   */
  public CalibrateDrive(SwerveDrive Drivetrain, Pose2d Position) {
    m_Drive = Drivetrain;
    addRequirements(Drivetrain);
    position = Position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Calibrating Drivetrain to " + position);
    // Reset position
    SwerveDrive.inverseKinematics.setPosition(position);
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
