package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Controls {

    final Joystick drive, steer;

    public JoystickButton GotoZero;

    public Controls(Joystick Drive, Joystick Steer) {
        drive = Drive;
        steer = Steer;

        GotoZero = new JoystickButton(drive, 2);
    }


    public double getForward() {
        return modifyAxis(drive.getY(), 0.15);
    }

    public double getLateral() {
        return modifyAxis(-drive.getX(), 0.15);
    }

    public double getTurn() {
        return modifyAxis(drive.getZ(), 0.15);
    }

    public boolean getPrecisionMode() {
        return drive.getTrigger();
    }

    public JoystickButton getDriverButton(int id) {
        return new JoystickButton(drive, id);
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
