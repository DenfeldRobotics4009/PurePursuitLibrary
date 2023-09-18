// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotContainer;
import frc.robot.libraries.geometry.Line2d;

public class PathFollower {

    final Path path;
    
    int lastCrossedPointIndex = 0;

    double lookAheadMeters; // Calculate-able?

    /**
     * 
     * @param FollowedPath
     */
    public PathFollower(Path FollowedPath) {
        path = FollowedPath;
    }

    public void setLookAheadDistance(double distanceMeters) {
        lookAheadMeters = distanceMeters;
    }

    /**
     * Calculates a PathState indicating where the robot should
     * drive to, and how fast it should be traveling to that point.
     * @param robotPosition current position of robot in meters
     * @return State for robot to travel to from grabbed position along path.
     */
    public PathState getPathState(Pose2d robotPosition) {
        PathPoint lastPoint = path.points.get(lastCrossedPointIndex);
        PathPoint nextPoint; // Avoid init, though declare access
        // Catch index out of bounds, thus we know its the end of the path.
        try {
            nextPoint = path.points.get(lastCrossedPointIndex + 1);
        } catch (IndexOutOfBoundsException e) {
            e.printStackTrace();
            // lastCrossedPointIndex should never reach
            // this, if it does, decrement index and recurse            
            lastCrossedPointIndex --;
            return getPathState(robotPosition);
        }
        
        Translation2d lastPointPos = lastPoint.getPosMeters(), nextPointPos = nextPoint.getPosMeters();
        // Line class to allow for the math to be outsourced, just making this code cleaner.
        Line2d pathLine = new Line2d(lastPointPos, nextPointPos); // Line in original path coords
        Line2d lineFromOrigin = pathLine.adjustToOrigin(); // Line adjusted to origin

    
        // Calculates the coordinates of the closest point upon the path to the robot
        Translation2d robotPathIntersection = lineFromOrigin.findPerpendicularIntersection(
            // Adjust robot position according to line adjustment
            robotPosition.getTranslation().minus(lastPointPos)
        );

        // Construct distance and percent variables for both position, and lookahead
        double robotDistanceAlongLineMeters = robotPathIntersection.getNorm();
        double lookAheadDistanceAlongLineMeters = robotDistanceAlongLineMeters + lookAheadMeters;

        // Lines of a length of 0 are invalid
        // Normalize meters values
        double percentFromLastPoint = robotDistanceAlongLineMeters / lineFromOrigin.getLength();
        double lookAheadPercentFromLastPoint = lookAheadDistanceAlongLineMeters / lineFromOrigin.getLength();

        // Check if lookahead is past this line
        if (
            lookAheadPercentFromLastPoint >= 1 &&               // If lookahead has passed the next point
            lastCrossedPointIndex < (path.points.size() - 1)    // If the point after exists
        ) {
            // update last point, and continue driving along next line
            lastCrossedPointIndex ++;
        }

        /**
         * If the robot passes its goal point, its interpolated speed may reach the negatives,
         * and the drive direction is reversed by the goto function. The if the robot passes its goal
         * point it may speed up while driving away from the point.
         * 
         * This can be solved by clamping the speed of the robot to the speed boundaries of both points.
         */

        // Construct path state from calculations
        return new PathState(
            // Goto position
            new Translation2d(
                // Calculate X from lookahead
                pathLine.getAtLinearInterpolationOfX(lookAheadPercentFromLastPoint),

                // Calculate Y from lookahead
                pathLine.getAtLinearInterpolationOfY(lookAheadPercentFromLastPoint)
            ),

            // Rotation goal
            new Rotation2d(
                PathPoint.getAtLinearInterpolation(
                    lastPoint.orientation.getRadians(), nextPoint.orientation.getRadians(), 
                    percentFromLastPoint
                )
            ), 

            // TODO DO FUNNY INTERPOLATION
            // Travel Speed
            PathPoint.getAtLinearInterpolation(

                lastPoint.speedMetersPerSecond, nextPoint.speedMetersPerSecond, 

                percentFromLastPoint
            )
        );
    }

    /**
     * Clamps the input between A and B, it does not matter wether A is the min or max
     */
    public static double nonSpecificClamp(double input, double boundA, double boundB) {
        if (boundA > boundB) return RobotContainer.Clamp(input, boundA, boundB);
        else return RobotContainer.Clamp(input, boundB, boundA);
    }
}
