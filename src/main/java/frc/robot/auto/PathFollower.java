// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.odometry.sources.OdometrySource;

public class PathFollower {

    final Path path;
    
    final OdometrySource positionSource;

    int lastCrossedPointIndex;

    double positionAlongPath;

    double lookAheadMeters; // Calculate-able?

    /**
     * 
     * @param FollowedPath
     */
    public PathFollower(Path FollowedPath, OdometrySource PositionSource) {
        path = FollowedPath;
        positionSource = PositionSource;
    }

    public void setLookAheadDistance(double distanceMeters) {
        lookAheadMeters = distanceMeters;
    }

    /**
     * @return State for robot to travel to from
     * recorded position along path.
     * 
     * TODO
     */
    PathState getPathState() {
        PathPoint lastPoint = path.points.get(lastCrossedPointIndex);
        PathPoint nextPoint = path.points.get(lastCrossedPointIndex + 1);

        double distanceBetweenPoints = lastPoint.getPosMeters().getDistance(
            nextPoint.getPosMeters()
        );

        double percentFromLastPoint = 
            (positionAlongPath - lastPoint.getDistanceAlongPathMeters()) / distanceBetweenPoints;

        // PathState state = new PathState(
        //     , 
        //     , 
        //     )
        return null;
    }

    /**
     * Updates position in meters along path from
     * given robot position, and updates the last
     * recorded point.
     * 
     * TODO
     */
    void updatePositionAlongPath() {
        // update path on position

        // TODO the meat and potatoes
    };

    
}
