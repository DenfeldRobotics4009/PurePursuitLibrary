// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.paths;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.auto.Path;
import frc.robot.auto.PathPoint;
import frc.robot.commands.DummyCommand;

/**
 * Constructs a TestPath instance
 */
public class Trial extends Path {

    public Trial() {
        // Construct path from arrayList of points
        super( // Path constructor
            new ArrayList<PathPoint>( // Create arraylist from list
                Arrays.asList( // Generate list from content

                    new PathPoint(
                        new Translation2d(0, 0), 
                        new Rotation2d(0), 
                        0.4, 
                        new DummyCommand("Past point 1")
                    ),
                    new PathPoint(
                        new Translation2d(4, 0), 
                        new Rotation2d(Math.toRadians(-90)), 
                        0, 
                        new DummyCommand("You made it (:")
                    )
                )  

            ),

            // Last point tolerance
            0.02
        );
    }

}
