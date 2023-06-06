package frc.robot.Odometry;

import edu.wpi.first.math.geometry.Pose2d;

public abstract class OdometrySource {
    /**
     * @return Pose2d of the calculated robot position
     */
    abstract Pose2d getPosition();

    /**
     * @param Position Pose2d in meters
     */
    abstract void setPosition(Pose2d Position);

    /**
     * @return Pose2d of the calculated robot velocities
     */
    abstract Pose2d getVelocity();

    /**
     * @return Pose2d of the calculated robot accelerations
     */
    abstract Pose2d getAcceleration();
}
