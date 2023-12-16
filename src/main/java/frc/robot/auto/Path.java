// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.ArrayList;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.Swerve;

public class Path {

    public final ArrayList<PathPoint> points;

    public final double lastPointTolerance;

    /**
     * Constructs a path from multiple points
     * @param Points
     */
    public Path(ArrayList<PathPoint> Points, double lastPointTolerance, String displayName) {
        System.out.println("Processing path " + displayName);

        this.lastPointTolerance = lastPointTolerance;
        points = Points;

        PathPoint firstPoint = points.get(0);

        // Process point data
        // Correct or implement orientation for each point.
        // Handle first point
        if (firstPoint.orientation == null) {
            // Default as 0, if the robot is not at 0 when 
            // the path begins, it will turn.
            firstPoint.orientation = new Rotation2d();
        }

        // Define others rotation
        for (int i = 1; i < points.size(); i++) {
            PathPoint point = points.get(i);
            PathPoint lastPoint = points.get(i-1);
            // If no orientation was initialized
            if (point.orientation == null) {
                // Set to last
                point.orientation = lastPoint.orientation;
            }
        }

        // Parse through a copy, as the original is being edited
        ArrayList<PathPoint> pointsCopy = Points;
        // Parse backward to correct speed of points
        // Parse from back, end at the first
        for (int i = pointsCopy.size()-1; i > 0; i--) {
            PathPoint point = pointsCopy.get(i);
            PathPoint previousPoint = pointsCopy.get(i-1);
            double deltaS = point.speedMetersPerSecond - previousPoint.speedMetersPerSecond;
            double deltaD = point.getDistance(previousPoint);
            // in this case, acceleration is negative, deceleration is positive
            double deceleration = -(deltaS / deltaD);
            // Pull max deceleration from constants
            if (deceleration > Swerve.MaxAccelerationMeters) {
                // Clamp speed
                double previousSpeed = previousPoint.speedMetersPerSecond;
                // This index will remain unaffected
                Points.get(i-1).speedMetersPerSecond = 
                    point.speedMetersPerSecond + deltaD*Swerve.MaxAccelerationMeters;
                System.out.println(
                    "Clamped speed from " + previousSpeed + " to " + 
                    previousPoint.speedMetersPerSecond
                );
            } else if (deceleration < Swerve.MaxAccelerationMeters && deltaS < 0) {
                // Insert new point
                // Normalized, deltaS / Swerve.MaxAccelerationMeters is negative
                double percentFromLastPoint =  1 + (deltaS / (Swerve.MaxAccelerationMeters * deltaD));
                System.out.println(
                    "Inserted new point at index " + i + " at " + percentFromLastPoint*100 + "%");
                // Interpolate between, and set speed to last speed
                PathPoint insertedPoint = previousPoint.interpolate(
                    point, percentFromLastPoint, 
                    new PrintCommand("Passed interpolated speed point at index " + i)
                );
                insertedPoint.speedMetersPerSecond = previousPoint.speedMetersPerSecond;
                points.add(i, insertedPoint);
            }
        }

        // Parse through point and print data
        for (int i = 0; i < points.size(); i++) {
            System.out.println("Point " + i);
            System.out.println("- Speed: " + points.get(i).speedMetersPerSecond);
            System.out.println("- Rotation: " + points.get(i).orientation);
            System.out.println("- Position: " + points.get(i).posMeters);
        }
    }
} 
