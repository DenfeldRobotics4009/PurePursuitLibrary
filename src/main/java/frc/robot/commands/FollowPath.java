// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Swerve;
import frc.robot.auto.Path;
import frc.robot.auto.PathFollower;
import frc.robot.auto.PathState;
import frc.robot.libraries.PController;
import frc.robot.subsystems.SwerveDrive;

public class FollowPath extends CommandBase {

  SwerveDrive m_drivetrain;

  Path path;
  PathFollower m_pathFollower;

  // TODO Tune kP
  PController rotationController = new PController(0.01, -1, 1);

  /** Creates a new FollowPath. */
  public FollowPath(SwerveDrive DriveTrain, Path Path) {
    // Assume drive control when path following
    addRequirements(DriveTrain);

    m_drivetrain = DriveTrain;
    path = Path;

    // Construct pathFollower from provided path
    m_pathFollower = new PathFollower(path);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d robotPose = m_drivetrain.getPosition();
    PathState state = m_pathFollower.getPathState(robotPose);

    // Assign target to PController
    rotationController.setTarget(state.goalPose.getRotation().getRadians());

    // The target relative to the robots current position
    Transform2d deltaLocation = robotPose.minus(state.goalPose);
    // Scale to goal speed. Speed input is in meters per second, while drive accepts normal values.
    Transform2d axisSpeeds = deltaLocation.times(state.speedMetersPerSecond / Swerve.MaxMetersPerSecond);

    // Construct chassis speeds from state values
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      new ChassisSpeeds(
        axisSpeeds.getX(),
        axisSpeeds.getY(),
        rotationController.calculate(
          SwerveDrive.navxGyro.getRotation2d().getRadians()
        )
      ), 
      SwerveDrive.navxGyro.getRotation2d()
    );
    // Drive
    m_drivetrain.drive(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
