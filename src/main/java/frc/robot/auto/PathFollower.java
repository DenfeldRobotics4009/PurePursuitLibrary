// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

public class PathFollower {

    final Path path;
    
    public int lastCrossedPointIndex = 0;

    double lookAheadMeters; // Calculate-able?

    Timer calculationTimer = new Timer();

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

    public PathState getPathState(Pose2d robotPosition) {
        calculationTimer.start();
        // Store 3 points, the first point is the one most recently
        // passed by the robots closest perpendicular intersection (pI)
        PathPoint relevantPoints[] = packageRelevantPoints();
        // grab robot position
        Translation2d robotTranslation = robotPosition.getTranslation();

        // Calculate point distances
        double[] distancesBetween = {
            relevantPoints[0].getDistance(relevantPoints[1]),
            relevantPoints[1].getDistance(relevantPoints[2])
        };

        // Calculate perpendicularIntersection between both points
        Translation2d[] perpendicularIntersection = {
            PathPoint.findPerpendicularIntersection(
                relevantPoints[0].getPosMeters(), relevantPoints[1].getPosMeters(), robotTranslation),
            PathPoint.findPerpendicularIntersection(
                relevantPoints[0].getPosMeters(), relevantPoints[1].getPosMeters(), robotTranslation)
        };

        // Find the distance from both intersections
        double distanceToAB = perpendicularIntersection[0].getDistance(robotTranslation);
        double distanceToBC = perpendicularIntersection[1].getDistance(robotTranslation);

        int mRPI = 0; // Most Relevant Point Index, abbreviated for neatness
        // Grab closest intersection, and proceed with interpolation
        if (distanceToAB > distanceToBC) {
            mRPI = 1; // Look at BC, else look at AB
        }
        // Find distance from most relevant point
        double distanceFromMRP = relevantPoints[mRPI].posMeters.getDistance(perpendicularIntersection[mRPI]);

        // grab mRPI for lookahead
        int lMRPI = mRPI;
        double lookaheadDistanceFromMRP = distanceFromMRP + lookAheadMeters;
        if (lookaheadDistanceFromMRP > distancesBetween[mRPI]) {
            lMRPI = mRPI + 1; // Look at line beyond current
        }

        if (lMRPI > 1) {
            // Test if the next point exists by catching an indexOutOfBoundsException
            try {
                path.points.get(lastCrossedPointIndex + 4);
            } catch (IndexOutOfBoundsException e) {
                // If the point does not exist, move lookahead to distancesBetween[mRPI]
                lMRPI = mRPI;
                lookaheadDistanceFromMRP = distancesBetween[mRPI];
            }
            // no exception, increment and recurse
            lastCrossedPointIndex ++;
            getPathState(robotPosition);
        } // Else continue

        PathState state = new PathState(
            // Interpolate goto position from lookahead distance
            relevantPoints[lMRPI].getPosMeters().interpolate(
                relevantPoints[lMRPI+1].getPosMeters(), lookaheadDistanceFromMRP / distancesBetween[mRPI]
            ),
            // Interpolate orientation
            new Rotation2d(
                PathPoint.getAtLinearInterpolation(
                    relevantPoints[mRPI].orientation.getRadians(), relevantPoints[mRPI+1].orientation.getRadians(), 
                    distancesBetween[mRPI], distanceFromMRP
                )
            ),
            // Interpolate speed, TODO the funny
            PathPoint.getAtLinearInterpolation(
                relevantPoints[mRPI].orientation.getRadians(), relevantPoints[mRPI+1].orientation.getRadians(), 
                distancesBetween[mRPI], distanceFromMRP
            )
        );

        calculationTimer.stop();
        System.out.println("Calculated PathState in " + calculationTimer.get() + " seconds");
        calculationTimer.reset();

        return state;
    }

    /**
     * Construct and return group of 3 relevant points
     * @return Array of PathPoints with length 3
     */
    PathPoint[] packageRelevantPoints() {
        try {
            return new PathPoint[] {
                path.points.get(lastCrossedPointIndex),
                path.points.get(lastCrossedPointIndex + 1),
                path.points.get(lastCrossedPointIndex + 2)
            };
        } catch (IndexOutOfBoundsException e) {
            // Decrement and recurse
            lastCrossedPointIndex --;
            return packageRelevantPoints();
        }
    }
}