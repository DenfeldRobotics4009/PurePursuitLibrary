// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libraries.geometry;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class Line2d {
    public final Translation2d PointA, PointB;

    public final double Slope;

    public Line2d(Translation2d PointA, Translation2d PointB) {
        this.PointA = PointA;
        this.PointB = PointB;

        this.Slope = (PointB.getY() - PointA.getY()) / (PointB.getX() - PointA.getX());
    }

    public double interpolateFromX(double X) {
        return Slope * (X - PointA.getX()) + PointA.getY();
    }

    /**
     * Returns the coordinates of the intersection between this line,
     * and a perpendicular line that coincides with point Source.
     * @param Source Line to attach perpendicular line to
     * @return Coordinates of intersection between lines
     */
    public Translation2d findPerpendicularIntersection(Translation2d Source) {
        Translation2d adjustedSource = Source.minus(PointA);

        double numerator = Slope * adjustedSource.getY() + adjustedSource.getX();
        double intersectionX = numerator / (1 + Slope * Slope);

        return new Translation2d(intersectionX, interpolateFromX(intersectionX)).plus(PointA);
    }

    /**
     * @return a new line with the first point as the origin
     */
    public Line2d adjustToOrigin() {
        return adjustToOrigin(PointA, PointB);
    }

    /**
     * @return a new line with the first point as the origin
     */
    public static Line2d adjustToOrigin(Translation2d PointA, Translation2d PointB) {
        return new Line2d(new Translation2d(), PointB.minus(PointA));
    }

    public double getLength() {
        return adjustToOrigin().PointB.getNorm();
    }

    /**
     * @param PercentBetween Position (from 0 to 1) along line
     * 
     * @return Calculated value between Initial and Final 
     * that is on the interpolated line function.
     */
    public double getAtLinearInterpolationOfY(double PercentBetween) {
        return (PointB.getY() - PointA.getY()) * PercentBetween + PointA.getY();
    }

    /**
     * @param PercentBetween Position (from 0 to 1) along line
     * 
     * @return Calculated value between Initial and Final 
     * that is on the interpolated line function.
     */
    public double getAtLinearInterpolationOfX(double PercentBetween) {
        return (PointB.getX() - PointA.getX()) * PercentBetween + PointA.getX();
    }
}
