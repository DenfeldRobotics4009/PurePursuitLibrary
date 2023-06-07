// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Swerve.DriveControls;

public class Drive extends CommandBase {
  SwerveDrive m_drivetrain;
  DriveControls m_controls;

  /** Creates a new Drive. */
  public Drive(SwerveDrive Drivetrain, DriveControls Controls) {
    addRequirements(Drivetrain);

    m_drivetrain = Drivetrain;
    m_controls = Controls;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      new ChassisSpeeds(m_controls.getForward(), m_controls.getLateral(), m_controls.getTurn()), 
      m_drivetrain.navxGyro.getRotation2d()
    );

    m_drivetrain.drive(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}