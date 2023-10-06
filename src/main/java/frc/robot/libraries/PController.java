// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libraries;

/** Add your docs here. */
public class PController {

    double kP = 0; // Const, non-final to allow setting
    public void setP(double kP) {this.kP = kP;}

    double target; // The value for the PController to approach
    public void setTarget(double target) {this.target = target;}

    double max, min; // Output clamp limits
    public void setBoundaries(double max, double min) {
        this.max = max; this.min = min;
    }

    /**
     * A simple proportional controller implementation
     * @param kP Proportion of the error
     */
    public PController(double kP, double minBoundary, double maxBoundary) {
        this.kP = kP;
        max = maxBoundary;
        min = minBoundary;
    }

    /**
     * Returns the proportion of the error from the target and
     * given feedbackward value.
     * @param feedBackward
     * @return (feedBackward - target) * kP , Clamped to boundaries
     */
    public double calculate(double feedBackward) {
        double deltaProportion = (target - feedBackward) * kP;
        return clamp(deltaProportion, max, min);
    }

    static double clamp(double input, double max, double min) {
        if (input > max) {return max;}
        else if (input < min) {return min;}
        else {return input;}
    }
}
