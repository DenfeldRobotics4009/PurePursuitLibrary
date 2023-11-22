// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.SwerveDrive;

public class Path {

    public final ArrayList<PathPoint> points;

    public final double lastPointTolerance;

    /**
     * Constructs a path from multiple points
     * @param Points
     */
    public Path(ArrayList<PathPoint> Points, double lastPointTolerance) {
        this.lastPointTolerance = lastPointTolerance;
        points = Points;

        PathPoint firstPoint = points.get(0);

        // Process point data
        // Correct or implement orientation for each point.
        // Handle first point
        if (firstPoint.orientation == null) {
            firstPoint.orientation = new Rotation2d();
        }
        // Define others
        for (int i = 1; i < points.size(); i++) {
            PathPoint point = points.get(i);
            PathPoint lastPoint = points.get(i-1);

            // If no orientation was initialized
            if (point.orientation == null) {
                // Set to last
                point.orientation = lastPoint.orientation;
            }
        }

        // // Correct or implement speed for each point,
        // // starting from last in path.
        // for (int i = points.size() - 1; i > 0; i--) {
        //     PathPoint point = points.get(i);
        //     PathPoint previousPoint = points.get(i-1); // Next in processing

        //     double distanceBetween = 
        //         point.distanceAlongPath - previousPoint.distanceAlongPath;

        //     double validSpeed = distanceBetween * Swerve.MaxAccelerationMeters;

        //     // handle next point from current point
        //     if (
        //         // Test if robot will be decelerating too fast
        //         validSpeed < previousPoint.speedMetersPerSecond
        //     ) { // Slope is too steep
        //         // Set speed of previous point
        //         previousPoint.speedMetersPerSecond = validSpeed;
        //     }
        // }
    }
} 
