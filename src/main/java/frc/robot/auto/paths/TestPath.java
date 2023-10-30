// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.paths;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auto.Path;
import frc.robot.auto.PathPoint;

/**
 * Constructs a TestPath instance
 */
public class TestPath extends Path {

    public TestPath() {
        // Construct path from arrayList of points
        super( // Path constructor
            new ArrayList<PathPoint>( // Create arraylist from list
                Arrays.asList( // Generate list from content

                    // Point A
                    new PathPoint(
                        new Translation2d(0, 0), 
                        new Rotation2d(0), 
                        0.1, 
                        new InstantCommand()
                    ),

                    // Point B
                    new PathPoint(
                        new Translation2d(-1, 0), 
                        new Rotation2d(0), 
                        // Last point should have a speed of 0, unless you know what you're doing
                        0,
                        new InstantCommand()
                    )
                )  
            )
        );
    }

}
