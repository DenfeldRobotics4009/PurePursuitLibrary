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
import frc.robot.auto.PathPoint;
import frc.robot.auto.PathState;
import frc.robot.libraries.PController;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveModule;

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
    m_pathFollower = new PathFollower(path, 0.5);

    // Assign target to PController
    rotationController.setTarget(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("--- Following path of points: ---");
    for (PathPoint point : path.points) {
      System.out.print(point.posMeters);
      System.out.println();
    }
    System.out.println("--- --- --- -- --- -- --- --- ---");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d robotPose = m_drivetrain.getPosition();
    System.out.println("Grabbing path state from position " + robotPose);
    PathState state = m_pathFollower.getPathState(robotPose);
    System.out.println(" ");

    // The target relative to the robots current position
    Transform2d deltaLocation = state.goalPose.minus(robotPose);
    System.out.println("Distance from goal position is " + deltaLocation + " meters");
    // Scale to goal speed. Speed input is in meters per second, while drive accepts normal values.
    Transform2d axisSpeeds = deltaLocation.times(state.speedMetersPerSecond / Swerve.MaxMetersPerSecond);
    System.out.println("Traveling to goal position at " + axisSpeeds + " m/s");

    // Construct chassis speeds from state values
    // Convert field oriented to robot oriented
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      // Field oriented chassisSpeeds
      new ChassisSpeeds(
        axisSpeeds.getX(),
        axisSpeeds.getY(),
        rotationController.calculate(
          SwerveModule.calculateDistanceCorrection(
            state.goalPose.getRotation().getRadians(), 
            robotPose.getRotation().getDegrees()
          )
        ) / 360 // Measure in rotations, 360 degrees is 1 rotation
      ), 
      SwerveDrive.navxGyro.getRotation2d()
    );
    // Drive
    m_drivetrain.drive(speeds);

    // Recurse until called to end
    System.out.println(" ");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Schedule last command in path.
    m_pathFollower.getLastPoint().triggeredCommand.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // calculate distance to last point
    double distanceToLastPointMeters = m_pathFollower.getLastPoint().posMeters.getDistance(
      m_drivetrain.getPosition().getTranslation()
    );

    return (
      // If we have passed the second to last point
      m_pathFollower.lastCrossedPointIndex >= (path.points.size() - 2) && 
      // And are within half a meter
      distanceToLastPointMeters < 0.2
    );
  }
}
