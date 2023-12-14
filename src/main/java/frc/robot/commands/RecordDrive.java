// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveDrive;

public class RecordDrive extends CommandBase {

  // For position and velocity grabbing
  final SwerveDrive m_drive = SwerveDrive.GetInstance();

  // Entries to send to outside program
  final NetworkTableEntry pXMeters, pYMeters, thetaRad, vMeters, status, sent;

  GenericEntry recordingBoolEntry;

  NetworkTableEntry[] dEntries;

  // Global for velocity calculation
  Pose2d position = new Pose2d();

  /** Creates a new RecordDrive. */
  public RecordDrive(NetworkTable table, GenericEntry recordingBoolEntry) {
    // Initialize entries within table
    pXMeters = table.getEntry("pXMeters");
    pYMeters = table.getEntry("pYMeters");
    thetaRad = table.getEntry("thetaRad");
    vMeters = table.getEntry("vMeters");
    status = table.getEntry("status");

    // Will be periodically set to true,
    // when recording is active
    sent = table.getEntry("sent");

    dEntries = new NetworkTableEntry[] {
      pXMeters, pYMeters, thetaRad, vMeters
    };

    this.recordingBoolEntry = recordingBoolEntry;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    status.setString("Activating");
    recordingBoolEntry.setBoolean(true);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Extract from drivetrain
    double speed = m_drive.getVelocity().getTranslation().getNorm() * Swerve.MaxMetersPerSecond;
    position = m_drive.getPosition();

    // Put current values into networkTable
    pXMeters.setDouble(position.getX());
    pYMeters.setDouble(position.getY());
    thetaRad.setDouble(position.getRotation().getRadians());
    vMeters.setDouble(speed);

    // Acknowledge sent packet
    sent.setBoolean(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    status.setString("Deactivating");
    sent.setBoolean(true);

    // Set all packets to 0
    for (NetworkTableEntry doubleEntry : dEntries) {
      doubleEntry.setDouble(0);
    }

    recordingBoolEntry.setBoolean(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Loop until cancelled
    return false;
  }
}
