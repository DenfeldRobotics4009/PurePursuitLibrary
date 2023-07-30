package frc.robot.odometry;

import edu.wpi.first.math.geometry.Pose2d;

public abstract class OdometrySource {
    /**
     * @return Pose2d of the calculated robot position
     */
    public abstract Pose2d getPosition();

    /**
     * @param Position Pose2d in meters
     */
    public abstract void setPosition(Pose2d Position);

    /**
     * @return Pose2d of the calculated robot velocities
     */
    public abstract Pose2d getVelocity();

    /**
     * @return Pose2d of the calculated robot accelerations
     */
    public abstract Pose2d getAcceleration();
}
