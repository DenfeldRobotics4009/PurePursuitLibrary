// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Contains frame info for a swerve module,
 * holding the distance traveled over the frame, and
 * the framerate when the frame was captured.
 */
public class SwerveTranslationFrame {

    private final Translation2d vector;
    private final double frameRate;

    SwerveTranslationFrame(Translation2d Vector, double FrameRateSeconds) {
        this.vector = Vector;
        this.frameRate = FrameRateSeconds;
    }

    public Translation2d getVector() {
        return vector;
    }

    public double getFrameRate() {
        return frameRate;
    }

    public Translation2d getVelocityVector() {
        return vector.div(frameRate);
    }
}
