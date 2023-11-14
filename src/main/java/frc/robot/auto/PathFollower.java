// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;
import frc.robot.Constants.Swerve;

public class PathFollower {

    final Path path;
    
    public int lastCrossedPointIndex = 0;

    public double lookAheadMeters; // Calculate-able?

    public static final boolean debug = true;

    /**
     * 
     * @param FollowedPath
     */
    public PathFollower(Path FollowedPath, double lookAheadMeters) {
        path = FollowedPath;
        this.lookAheadMeters = lookAheadMeters;
    }

    public void setLookAheadDistance(double distanceMeters) {
        lookAheadMeters = distanceMeters;
    } 

    public PathState getPathState(Pose2d robotPosition) {
        //println("Observing line " + lastCrossedPointIndex + " to " + (lastCrossedPointIndex + 1));

        // TODO split logic blocks into function

        Translation2d robotTranslation = robotPosition.getTranslation();

        // Store 2 points
        ArrayList<PathPoint> relevantPoints = packageRelevantPoints();

        // Assume at least 2 are grabbed
        double lengthAB = relevantPoints.get(0).getDistance(relevantPoints.get(1));
        //println("Length of current line = " + lengthAB);

        // grab perpendicular intersection
        Translation2d perpendicularIntersectionAB = PathPoint.findPerpendicularIntersection(
            relevantPoints.get(0).posMeters, relevantPoints.get(1).posMeters, robotTranslation
        );

        // Calculate position along line AB via finding difference between line length, and distance to B
        double distanceMetersAlongAB = lengthAB - relevantPoints.get(1).posMeters.getDistance(perpendicularIntersectionAB);

        // Clamp distance along AB
        if (distanceMetersAlongAB < 0) {
            distanceMetersAlongAB = 0;
        }

        // Calculate look ahead distance from ab, if its over the length, look to BC
        double lookAheadDistanceMetersAlongPoints = distanceMetersAlongAB + lookAheadMeters;

        Translation2d gotoGoal;
        double predictedSpeed = 0;

        // double lookedLineLength = 0;
        int pointsLookingAhead = 0;
        double distanceAlongLookaheadPoints = lookAheadDistanceMetersAlongPoints;

        // Parse lookahead
        while (true) {
            // Check to make sure points are accessible
            if (lastCrossedPointIndex + pointsLookingAhead + 1 >= path.points.size()) {
                //print("Looking towards end of path at point ");
                // We are looking to the end of path
                gotoGoal = path.points.get(path.points.size()-1).posMeters;
                predictedSpeed = path.points.get(path.points.size()-1).speedMetersPerSecond;
                //println(gotoGoal);
                break;
            }
            
            // Grab 2 points, and grab the length between them
            PathPoint lookAheadPointA = path.points.get(lastCrossedPointIndex + pointsLookingAhead);
            PathPoint lookAheadPointB = path.points.get(lastCrossedPointIndex + pointsLookingAhead + 1);

            double lookAheadLineLength = lookAheadPointA.getDistance(lookAheadPointB);

            // If we are not looking past this line
            if (distanceAlongLookaheadPoints < lookAheadLineLength) {
                // Stop looping, interpolate goto
                gotoGoal = lookAheadPointA.posMeters.interpolate(
                    lookAheadPointB.posMeters, 
                    distanceAlongLookaheadPoints / lookAheadLineLength // Normalized
                );

                predictedSpeed = PathPoint.getAtLinearInterpolation(
                    relevantPoints.get(0).speedMetersPerSecond, 
                    relevantPoints.get(1).speedMetersPerSecond, 
                    distanceAlongLookaheadPoints / lookAheadLineLength
                );

                //println(gotoGoal);
                break;
            }

            distanceAlongLookaheadPoints -= lookAheadLineLength;
            // Look 1 line ahead, and subtract length of last line
            pointsLookingAhead ++;
            // Continue
        }

        // Set lookahead distance based upon predicted speed
        lookAheadMeters = RobotContainer.Clamp(
            // Arbriturary scalar
            1200 * Math.sqrt(predictedSpeed / Swerve.MaxSteerAccelerationRad),
            1, 0.05
        );

        //println("Constructing path state");
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

        // Check to increment index
        if (compareWithNextLine(perpendicularIntersectionAB, robotTranslation)) {
            // Schedule associated command
            relevantPoints.get(0).triggeredCommand.schedule();
            lastCrossedPointIndex ++;
            //println("Increment last crossed point index to " + lastCrossedPointIndex);
        }

        return state;
    }

    public PathPoint getLastPoint() {
        return path.points.get(path.points.size() - 1);
    }

    /**
     * Returns true if the robot is closer to the next line
     * @param positionAlongLine
     * @param robotTranslation
     * @return false if robot is closer to current line, or next line doesn't exist
     */
    boolean compareWithNextLine(Translation2d positionAlongLine, Translation2d robotTranslation) {
        // Calculate the perpendicularIntersection of the next line in path, if it exists
        // Check if the line exists first
        if (lastCrossedPointIndex + 2 < path.points.size()) {
            //println("Next line found");
            // if the line exists, grab points
            PathPoint pointB = path.points.get(lastCrossedPointIndex + 1);
            PathPoint pointC = path.points.get(lastCrossedPointIndex + 2);
            // grab perpendicular intersection
            Translation2d perpendicularIntersectionBC = PathPoint.findPerpendicularIntersection(
                pointB.posMeters, pointC.posMeters, robotTranslation
            );

            //println("Found perpendicular intersection at " + perpendicularIntersectionBC);
            
            // Find distance from lines
            double distanceToAB = positionAlongLine.getDistance(robotTranslation);
            double distanceToBC = perpendicularIntersectionBC.getDistance(robotTranslation);

            // Compare distances to each intersection
            if (distanceToAB > distanceToBC) {
                //println("Distance to line AB is greater than distance to BC");
                return true;
            }
        } else {
            //println("Next line not found");
        }

        return false;
    }

    /**
     * Construct and return group of 2 relevant points
     * @return Array of PathPoints with length 2
     */
    ArrayList<PathPoint> packageRelevantPoints() {
        // if our lastCrossPoint index is below zero,
        // our path doesn't contain enough points to fit the definition of a path.
        if (lastCrossedPointIndex < 0) {throw new IndexOutOfBoundsException();}

        ArrayList<PathPoint> packagedPoints = new ArrayList<PathPoint>();
        // Try to grab 2 points
        for (int i = 0; i < 2; i++) {
            try {
                packagedPoints.add(
                    path.points.get(lastCrossedPointIndex + i)
                );
            } catch (IndexOutOfBoundsException e) {
                DriverStation.reportWarning("Could not grab point at index " + lastCrossedPointIndex + i, e.getStackTrace());
            }
        }

        // If we couldn't grab 2 points, decrement.
        if (packagedPoints.size() < 2) {
            lastCrossedPointIndex --;
            return packageRelevantPoints();
        }

        return packagedPoints;
    }

    /**
     * Disable-able print function
     * @param x
     */
    public static void print(Object x) {
        if (debug) {
            System.out.print(" > " + x);
        }
    }

    /**
     * Disable-able println function
     * @param x
     */
    public static void println(Object x) {
        print(x + "\n");
    }
}