// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class PathPoint {

    final Translation2d posMeters;
    double distanceAlongPath; // Handled by path constructor
    Rotation2d orientation; // Implemented by path constructor

    double speedMetersPerSecond; // Corrected by path constructor
    //double accelerationDistance; // For deceleration

    final Command triggeredCommand;

    public PathPoint(
        Translation2d PosMeters,
        Rotation2d Orientation,
        double SpeedMetersPerSecond,
        Command TriggeredCommand
    ) {
        posMeters = PosMeters;
        triggeredCommand = TriggeredCommand;

        // May be overridden
        speedMetersPerSecond = SpeedMetersPerSecond;
        orientation = Orientation;
    }

    public void defineDistanceAlongPath(double distanceAlongPath) {
        this.distanceAlongPath = distanceAlongPath;
    }

    public Translation2d getPosMeters() {return posMeters;}

    public double getDistanceAlongPathMeters() {return distanceAlongPath;}

    public void setOrientation(Rotation2d orientation) {this.orientation = orientation;}

    public Rotation2d getOrientation() {return orientation;}

    public double getSpeedMetersPerSecond() {return speedMetersPerSecond;}

    public void setSpeedMetersPerSecond(double SpeedMetersPerSecond) {speedMetersPerSecond = SpeedMetersPerSecond;}

    //public void setAccelerationDistance(double AccelerationDistance) {accelerationDistance = AccelerationDistance;}

    /**
     * 
     * @param Initial Initial Value
     * @param Final Final Value
     * @param PercentBetween Position (In Percent) along line starting from 0
     * 
     * @return Calculated value between Initial and Final 
     * that is on the interpolated line function.
     */
    public static double getAtLinearInterpolation(
        double Initial, double Final, double PercentBetween
    ) {
        return (Final - Initial) * PercentBetween + Initial;
    }

    public static double getAtLinearInterpolation(
        double Initial, double Final, double Length, double DistanceBetween
    ) {
        return getAtLinearInterpolation(Initial, Final, DistanceBetween / Length);
    }

    /**
     * 
     * @param Initial Initial Value
     * @param Final Final Value
     * @param Distance Distance between
     * 
     * @return Slope
     */
    public static double getSlopeOfLinearInterpolation(
        double Initial, double Final, double Distance
    ) {
        return (Final - Initial) / Distance;
    }

    /**
     * Returns the coordinates of the intersection between this line,
     * and a perpendicular line that coincides with point Source.
     * @param Source Line to attach perpendicular line to
     * @return Coordinates of intersection between lines
     */
    public static Translation2d findPerpendicularIntersection(Translation2d PointA, Translation2d PointB, Translation2d Source) {

        double distance = PointB.getX() - PointA.getX();
        double hight = PointB.getY() - PointA.getY();

        double Slope = distance / hight; 

        Translation2d adjustedSource = Source.minus(PointA);

        double numerator = Slope * adjustedSource.getY() + adjustedSource.getX();
        double intersectionX = numerator / (1 + Slope * Slope);

        return new Translation2d(
            intersectionX, getAtLinearInterpolation(0, hight, intersectionX / Math.abs(distance))
        ).plus(PointA);
    }

    public double getDistance(PathPoint Point) {
        return posMeters.getDistance(Point.getPosMeters());
    }
} 
