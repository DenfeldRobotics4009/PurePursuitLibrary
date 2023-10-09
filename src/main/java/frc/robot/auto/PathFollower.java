// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
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

        Translation2d robotTranslation = robotPosition.getTranslation();

        // Store 3 points, the first point is the one most recently
        // passed by the robots closest perpendicular intersection (pI)
        ArrayList<PathPoint> relevantPoints = packageRelevantPoints();

        // Assume at least 2 are grabbed
        double lengthAB = relevantPoints.get(0).getDistance(relevantPoints.get(1));
        // grab perpendicular intersection
        Translation2d perpendicularIntersectionAB = PathPoint.findPerpendicularIntersection(
            relevantPoints.get(0).posMeters, relevantPoints.get(1).posMeters, robotTranslation
        );
        
        // Calculate position along line AB
        double distanceMetersAlongAB = Math.copySign(
            relevantPoints.get(0).posMeters.getDistance(perpendicularIntersectionAB), 
            perpendicularIntersectionAB.getX() - relevantPoints.get(0).posMeters.getX() 
        );

        // Clamp distance along AB
        if (distanceMetersAlongAB > lengthAB) {
            distanceMetersAlongAB = lengthAB;
        } else if (distanceMetersAlongAB < 0) {
            distanceMetersAlongAB = 0;
        }

        // Calculate look ahead distance from ab, if its over the length, look to BC
        double lookAheadDistanceMetersAlongAB = distanceMetersAlongAB + lookAheadMeters;

        // Look on AB, correct after if cases allow
        Translation2d gotoGoal = relevantPoints.get(0).posMeters.interpolate(
            relevantPoints.get(1).posMeters,
            lookAheadDistanceMetersAlongAB / lengthAB   
        );;

        // If our scope is a standard size, check to increment from path
        if (relevantPoints.size() == 3) {
            // Calculates distance from both lines
            double distanceToLineAB = perpendicularIntersectionAB.getDistance(robotTranslation);

            double distanceToLineBC = PathPoint.findPerpendicularIntersection(
                relevantPoints.get(1).posMeters, relevantPoints.get(2).posMeters, robotTranslation
            ).getDistance(robotTranslation);

            // increment index if distance to BC is less 
            if (distanceToLineBC < distanceToLineAB) {lastCrossedPointIndex ++;}

            // TODO
            // Handle looking past line AB if line BC exists
            if (lookAheadDistanceMetersAlongAB > lengthAB) {

                double lookAheadDistanceMetersAlongBC = lookAheadDistanceMetersAlongAB - lengthAB;
                double lengthBC = relevantPoints.get(1).getDistance(relevantPoints.get(2));

                // Look upon next line
                gotoGoal = relevantPoints.get(1).posMeters.interpolate(
                    relevantPoints.get(2).posMeters,
                    lookAheadDistanceMetersAlongBC / lengthBC    
                );
            }
        } else {
            // TODO
            // Handle looking past line AB if line BC does not exist
            // Look to point B
            if (lookAheadDistanceMetersAlongAB > lengthAB) {
                gotoGoal = relevantPoints.get(1).posMeters;
            }
        }

        // Construct state
        PathState state = new PathState(
            gotoGoal, 
            new Rotation2d(
                PathPoint.getAtLinearInterpolation(
                    relevantPoints.get(0).orientation.getRadians(), 
                    relevantPoints.get(1).orientation.getRadians(), 
                    distanceMetersAlongAB / lengthAB
                )
            ), 
            
            // TODO, do properly
            PathPoint.getAtLinearInterpolation(
                relevantPoints.get(0).speedMetersPerSecond, 
                relevantPoints.get(1).speedMetersPerSecond, 
                distanceMetersAlongAB / lengthAB
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
    ArrayList<PathPoint> packageRelevantPoints() {
        ArrayList<PathPoint> packagedPoints = new ArrayList<PathPoint>();
        // Try to grab 3 points
        for (int i = 0; i < 3; i++) {
            try {
                packagedPoints.add(
                    path.points.get(lastCrossedPointIndex + i)
                );
            } catch (IndexOutOfBoundsException e) {
                DriverStation.reportWarning("Could not grab point at index " + i, e.getStackTrace());
            }
        }

        // If we couldn't grab 2 points, decrement.
        if (packagedPoints.size() < 2) {
            lastCrossedPointIndex --;
            return packageRelevantPoints();
        }

        // if our lastCrossPoint index is below zero,
        // our path doesn't contain enough points to fit the definition of a path.
        if (lastCrossedPointIndex < 0) {throw new IndexOutOfBoundsException();}

        return packagedPoints;
    }
}