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
import frc.robot.RobotContainer;

public class PathFollower {

    final Path path;
    
    public int lastCrossedPointIndex = 0;

    double lookAheadMeters; // Calculate-able?

    Timer calculationTimer = new Timer();

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
        calculationTimer.start();
        println("Began pathState calculation timer");

        // TODO split logic blocks into function

        Translation2d robotTranslation = robotPosition.getTranslation();

        // Store 2 points
        ArrayList<PathPoint> relevantPoints = packageRelevantPoints();

        // Assume at least 2 are grabbed
        double lengthAB = relevantPoints.get(0).getDistance(relevantPoints.get(1));
        println("Length of current line = " + lengthAB);

        // grab perpendicular intersection
        Translation2d perpendicularIntersectionAB = PathPoint.findPerpendicularIntersection(
            relevantPoints.get(0).posMeters, relevantPoints.get(1).posMeters, robotTranslation
        );
        println("Found perpendicular intersection at " + perpendicularIntersectionAB);

        // Calculate position along line AB
        double distanceMetersAlongAB = Math.copySign(
            relevantPoints.get(0).posMeters.getDistance(perpendicularIntersectionAB), 
            perpendicularIntersectionAB.getX() - relevantPoints.get(0).posMeters.getX() 
        );

        println("Robot is " + distanceMetersAlongAB + " meters along current line");

        // Clamp distance along AB
        if (distanceMetersAlongAB > lengthAB) {

            println("Clamping distance along line");
            distanceMetersAlongAB = lengthAB;
            
        } else if (distanceMetersAlongAB < 0) {
            distanceMetersAlongAB = 0;
        }

        // Calculate look ahead distance from ab, if its over the length, look to BC
        double lookAheadDistanceMetersAlongPoints = distanceMetersAlongAB + lookAheadMeters;
        println("Looking " + lookAheadDistanceMetersAlongPoints + " meters along line");

        Translation2d gotoGoal;

        // double lookedLineLength = 0;
        int pointsLookingAhead = 0;
        double distanceAlongLookaheadPoints = lookAheadDistanceMetersAlongPoints;

        // Parse lookahead
        println("Calculating lookAhead point");
        while (true) {
            println("Looking " + distanceAlongLookaheadPoints + " meters from last seen point");
            // Check to make sure points are accessible
            if (lastCrossedPointIndex + pointsLookingAhead + 1 >= path.points.size()) {
                print("Looking towards end of path at point ");
                // We are looking to the end of path
                gotoGoal = path.points.get(path.points.size()-1).posMeters;
                println(gotoGoal);
                break;
            }
            
            println("Looking " + pointsLookingAhead + " points ahead of last crossed point");
            // Grab 2 points, and grab the length between them
            PathPoint lookAheadPointA = path.points.get(lastCrossedPointIndex + pointsLookingAhead);
            PathPoint lookAheadPointB = path.points.get(lastCrossedPointIndex + pointsLookingAhead + 1);

            double lookAheadLineLength = lookAheadPointA.getDistance(lookAheadPointB);
            println("Length of observed line is " + lookAheadLineLength + " meters");

            if (distanceAlongLookaheadPoints < lookAheadLineLength) {
                println("Proper observed line found");
                println("Interpolating on line looking " + pointsLookingAhead + " points ahead");
                print("Looking towards path at point ");
                // Stop looping, interpolate goto
                gotoGoal = lookAheadPointA.posMeters.interpolate(
                    lookAheadPointB.posMeters, 
                    distanceAlongLookaheadPoints / lookAheadLineLength // Normalized
                );
                println(gotoGoal);
                break;
            }

            distanceAlongLookaheadPoints -= lookAheadLineLength;
            println("Valid line not found, looking further ... ");
            // Look 1 line ahead, and subtract length of last line
            pointsLookingAhead ++;
            // Continue
        }

        println("Constructing path state");
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

        println("Checking distance to next line");
        // Check to increment index
        if (compareWithNextLine(perpendicularIntersectionAB, robotTranslation)) {
            // Schedule associated command
            println("Scheduling command associated with point " + lastCrossedPointIndex);
            relevantPoints.get(0).triggeredCommand.schedule();
            lastCrossedPointIndex ++;
            println("Increment last crossed point index to " + lastCrossedPointIndex);
        }

        calculationTimer.stop();
        println(" --- --- --- Calculated PathState in " + calculationTimer.get() + " seconds --- --- ---");
        calculationTimer.reset();

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
            println("Next line found");
            // if the line exists, grab points
            PathPoint pointB = path.points.get(lastCrossedPointIndex + 1);
            PathPoint pointC = path.points.get(lastCrossedPointIndex + 2);
            // grab perpendicular intersection
            Translation2d perpendicularIntersectionBC = PathPoint.findPerpendicularIntersection(
                pointB.posMeters, pointC.posMeters, robotTranslation
            );

            println("Found perpendicular intersection at " + perpendicularIntersectionBC);
            
            // Find distance from lines
            double distanceToAB = positionAlongLine.getDistance(robotTranslation);
            double distanceToBC = perpendicularIntersectionBC.getDistance(robotTranslation);

            // Compare distances to each intersection
            if (distanceToAB > distanceToBC) {
                println("Distance to line AB is greater than distance to BC");
                return true;
            }
        } else {
            println("Next line not found");
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
            System.out.print(" > " + RobotContainer.trace(2) + " > " + x);
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