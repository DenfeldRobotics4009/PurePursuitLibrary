// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class RecordDrive extends CommandBase {

  static RecordDrive instance = null;

  // For position and velocity grabbing
  final SwerveDrive m_drive = SwerveDrive.GetInstance();

  // Entries to send to outside program
  final NetworkTableEntry pXMeters, pYMeters, thetaRad, vMeters, active, sent;

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
    active = table.getEntry("active");

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
    // Cancel the command if its already running
    System.out.println("recordDrive obj: " + instance);
    if (instance != null) {
      active.setBoolean(false);

      // Set all packets to 0
      for (NetworkTableEntry doubleEntry : dEntries) {
        doubleEntry.setDouble(0);
      }

      recordingBoolEntry.setBoolean(false);
      // Cancel both currently running
      // command, and this
      instance.cancel();
      instance = null;
      cancel();
    } else {
      // If not, add to the instance
      instance = this;
      active.setBoolean(true);
      recordingBoolEntry.setBoolean(true);
    } 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Assume polling rate if 0.2 seconds
    // Extract from drivetrain
    double speed = position.minus(m_drive.getPosition()).getTranslation().getNorm() / 0.2;
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
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Loop until cancelled
    return false;
  }

  public static boolean isRunning() {
    return instance != null;
  }
}
