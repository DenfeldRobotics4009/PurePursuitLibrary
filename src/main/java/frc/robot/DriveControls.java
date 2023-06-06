package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class DriveControls {

    final Joystick drive, steer;

    public DriveControls(Joystick Drive, Joystick Steer) {
        drive = Drive;
        steer = Steer;
    }

    public double getForward() {
        return -drive.getY();
    }

    public double getLateral() {
        return drive.getX();
    }

    public double getTurn() {
        return steer.getZ();
    }

    public boolean getPrecisionMode() {
        return drive.getTrigger();
    }
}
