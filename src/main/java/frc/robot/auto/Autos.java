// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.paths.SpeedTest;
import frc.robot.commands.FollowPath;

public enum Autos {

    /**
     * Define autos here
     * 
     */

    SPEEDTEST(
        new SequentialCommandGroup(
            new FollowPath(new SpeedTest())
        )
    );

    final SequentialCommandGroup autoSequence;

    Autos(SequentialCommandGroup autoSequence) {
        this.autoSequence = autoSequence;
    }

    public SequentialCommandGroup getSequence() {return autoSequence;}
}
