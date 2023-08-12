// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class PathState {

    public final Translation2d posMeters;
    public final Rotation2d orientation;
    public final double speedMetersPerSecond; 
    
    /**
     * Contains a single state within a robot path
     * @param posMeters
     * @param orientation
     * @param speedMetersPerSecond
     */
    public PathState (
        Translation2d posMeters,
        Rotation2d orientation,
        double speedMetersPerSecond
    ) {
        this.posMeters = posMeters;
        this.orientation = orientation;
        this.speedMetersPerSecond = speedMetersPerSecond;
    }
}
