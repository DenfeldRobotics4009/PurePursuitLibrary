package frc.robot.odometry.sources;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.libraries.geometry.Quadrilateral;

public abstract class OdometrySource {

    /**
     * Calculates and returns robot field position from tacked or absolute
     * values. This returns the "most likely" position, while
     * getPositionBoundary() returns the range of possible field positions.
     * 
     * @return Pose2d of the calculated most likely robot field position.
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

    /**
     * Calculates and returns the range of all possible robot
     * field positions.
     *  
     * @return Quadrilateral representing the boundary of
     * possible robot positions.
     */
    public abstract Quadrilateral getPositionBoundary();

    /**
     * Some OdometrySources may require a periodic function, this
     * should be handled by each specific OdometrySource, and called
     * from its associated subsystem, or robot periodic.
     */
}
