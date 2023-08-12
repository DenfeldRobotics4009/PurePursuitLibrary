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

    // public static double getDistanceBetweenPoints(
    //     PathPoint Initial, PathPoint Final
    // ) {
    //     return Initial.getPosMeters().getDistance(
    //         Final.getPosMeters()
    //     );
    // }
} 
