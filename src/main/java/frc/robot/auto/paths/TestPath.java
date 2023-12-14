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
public class TestPath extends Path {

    public TestPath() {
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
                        new Translation2d(1.6576, 0), 
                        new Rotation2d(0), 
                        0.4,
                        new DummyCommand("Past point 2")
                    ),

                    new PathPoint(
                        new Translation2d(3.6576, 0), 
                        new Rotation2d(0), 
                        0.4,
                        new DummyCommand("Past point 3")
                    ),

                    new PathPoint(
                        new Translation2d(3.6576, 1), 
                        new Rotation2d(0), 
                        1.5,
                        new DummyCommand("Past point 4")
                    ),

                    new PathPoint(
                        new Translation2d(3.6576, 2), 
                        new Rotation2d(0), 
                        0.8,
                        new DummyCommand("Past point 5")
                    ),

                    new PathPoint(
                        new Translation2d(3.6576, 4.1656), 
                        new Rotation2d(0), 
                        0.4,
                        new DummyCommand("Past point 6")
                    ),

                    new PathPoint(
                        new Translation2d(1, 4.1656), 
                        new Rotation2d(0), 
                        0.4,
                        new DummyCommand("Past point 7")
                    ),

                    new PathPoint(
                        new Translation2d(0, 4.1656), 
                        new Rotation2d(0), 
                        0.4,
                        new DummyCommand("Past point 8")
                    ),

                    new PathPoint(
                        new Translation2d(0, 2), 
                        new Rotation2d(0), 
                        1,
                        new DummyCommand("Past point 9")
                    ),

                    new PathPoint(
                        new Translation2d(0, 0), 
                        new Rotation2d(0), 
                        0,
                        new DummyCommand("Past point 10")
                    )
                )  
            ),

            // Last point tolerance
            0.02,
            "TestPath"
        );
    }

}
