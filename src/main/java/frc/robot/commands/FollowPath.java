// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.PathFollowing;
import frc.robot.Constants.Swerve;
import frc.robot.auto.Path;
import frc.robot.auto.PathFollower;
import frc.robot.auto.PathPoint;
import frc.robot.auto.PathState;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveMotors;

public class FollowPath extends CommandBase {

  SwerveDrive m_drivetrain;

  Path path;
  PathFollower m_pathFollower;

  /** Creates a new FollowPath. */
  public FollowPath(Path Path) {
    // Assume drive control when path following
    m_drivetrain = SwerveDrive.GetInstance();
    addRequirements(m_drivetrain);

    path = Path;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Construct pathFollower from provided path
    m_pathFollower = new PathFollower(path, 0.1);

    PathFollower.println("--- Following path of points: ---");
    for (PathPoint point : path.points) {
      PathFollower.print(point.posMeters);
      PathFollower.println(" ");
    }
    PathFollower.println("--- --- --- -- --- -- --- --- ---");

    System.out.println("Scheduling path command: " + m_pathFollower.getFirstPoint().triggeredCommand);
    m_pathFollower.getFirstPoint().triggeredCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d robotPose = m_drivetrain.getPosition();
    
    PathState state = m_pathFollower.getPathState(robotPose);

    //PathFollower.println("Current pos is " + robotPose);
    //PathFollower.println("Goal pos is " + state.goalPose);
    // The target relative to the robots current position
    Translation2d deltaLocation = state.goalPose.getTranslation().minus(robotPose.getTranslation());

    // Clamp state speed so the end of the path can be consistently reached
    // Clamped between [Const Max, 5 cm/s]
    double clampedSpeed = RobotContainer.Clamp(state.speedMetersPerSecond, Swerve.MaxMetersPerSecond, 0.05);

    RobotContainer.distanceFromGoalEntry.setDouble(deltaLocation.getNorm());
    RobotContainer.speedEntry.setDouble(clampedSpeed);
    RobotContainer.lookAheadEntry.setDouble(m_pathFollower.lookAheadMeters);

    // Scale to goal speed. Speed input is in meters per second, while drive accepts normal values.
    Translation2d axisSpeeds = new Translation2d(clampedSpeed, deltaLocation.getAngle());
    
    //PathFollower.println("Traveling to goal position at " + axisSpeeds + " m/s");

    // Set lookahead based upon speed of next point
    m_pathFollower.setLookAheadDistance(RobotContainer.Clamp(
      PathFollowing.lookAheadScalar * clampedSpeed,
      1, 0.1
    ));

    // Construct chassis speeds from state values
    // Convert field oriented to robot oriented
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      // Field oriented chassisSpeeds
      new ChassisSpeeds(
        axisSpeeds.getX(),
        axisSpeeds.getY(),
        // Rotate by the angle between
        SwerveMotors.signedAngleBetween(
          // Angle from current to goal
          state.goalPose.getRotation(),
          SwerveDrive.navxGyro.getRotation2d()
        ).getRadians() // Units in radians
      ), 
      SwerveDrive.navxGyro.getRotation2d()
    );
    // Drive
    // PathFollower.print("\r");
    // PathFollower.print("Driving with robot relative speed of " + speeds + " m/s");
    m_drivetrain.drive(speeds);

    // Recurse until called to end
    //PathFollower.println(" ");
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    PathFollower.println("End of path reached");
    // Schedule last command in path.

    RobotContainer.distanceFromGoalEntry.setDouble(0);
    RobotContainer.speedEntry.setDouble(0);
    RobotContainer.lookAheadEntry.setDouble(0);

    System.out.println("Scheduling path command: " + m_pathFollower.getLastPoint().triggeredCommand);
    m_pathFollower.getLastPoint().triggeredCommand.schedule();
    m_pathFollower = null;
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
      distanceToLastPointMeters < path.lastPointTolerance
    );
  }
}
