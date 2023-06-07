package frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj.Joystick;

public class DriveControls {

    final Joystick drive, steer;

    public DriveControls(Joystick Drive, Joystick Steer) {
        drive = Drive;
        steer = Steer;
    }

    public double getForward() {
        return modifyAxis(-drive.getY(), 0.05);
    }

    public double getLateral() {
        return modifyAxis(drive.getX(), 0.05);
    }

    public double getTurn() {
        return modifyAxis(drive.getZ(), 0.05);
    }

    public boolean getPrecisionMode() {
        return drive.getTrigger();
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
            } else {
            return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value, double deadband) {
        // Deadband
        value = deadband(value, deadband);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }
}
