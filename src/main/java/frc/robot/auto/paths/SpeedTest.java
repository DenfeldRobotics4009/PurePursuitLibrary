// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.paths;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.auto.Path;
import frc.robot.Constants.Swerve;
import frc.robot.auto.PathPoint;

/**
 * Constructs a TestPath instance
 */
public class SpeedTest extends Path {

    public SpeedTest() {
        super( // Path constructor
            new PathPoint(
                new Translation2d(0, 0), 
                Swerve.forwardAngle, 
                Swerve.MaxMetersPerSecond, 
                new PrintCommand("Past point 0")
            ),
            new PathPoint(
                new Translation2d(1.5, 0), 
                Swerve.forwardAngle, 
                0, 
                new PrintCommand("Past point 1")
            )
        );
    }

}
