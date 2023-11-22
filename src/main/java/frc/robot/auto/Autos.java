// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.paths.ConeCircle;
import frc.robot.auto.paths.TestPath;
import frc.robot.auto.paths.TestPathB;
import frc.robot.auto.paths.TestPathBReverse;
import frc.robot.auto.paths.Trial;
import frc.robot.commands.CalibrateDrive;
import frc.robot.commands.FollowPath;
import frc.robot.subsystems.SwerveDrive;

public enum Autos {

    /**
     * Define autos here
     * 
     */

    NONE(new SequentialCommandGroup()),

    TEST(new SequentialCommandGroup(
        new CalibrateDrive(SwerveDrive.GetInstance(), new Pose2d()),
        new FollowPath(new TestPath())
    )),
    
    TESTB(new SequentialCommandGroup(
        new CalibrateDrive(SwerveDrive.GetInstance(), new Pose2d()),
        new FollowPath(new TestPathB()),
        new FollowPath(new TestPathBReverse())
    )),

    TRIAL(new SequentialCommandGroup(new FollowPath(new Trial()))),

    CIRCLECONE(new SequentialCommandGroup(new FollowPath(new ConeCircle())));

    final SequentialCommandGroup autoSequence;

    Autos(SequentialCommandGroup autoSequence) {
        this.autoSequence = autoSequence;
    }

    public SequentialCommandGroup getSequence() {return autoSequence;}
}
