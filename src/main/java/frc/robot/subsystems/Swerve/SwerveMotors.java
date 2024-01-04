package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveMotors {
    public CANSparkMax DriveMotor, SteerMotor;
    public CANCoder SteerEncoder;
    public final Rotation2d defaultAngleOffset;
    
    public static double rotationsToMeters(double rotations) {
        return rotations * SwerveModule.rotationsToMeters;
    }
    
    public static double metersToRotations(double meters) {
        return meters / SwerveModule.rotationsToMeters;
    }

    public SwerveMotors(
        int driveMotorID, 
        int steerMotorID, 
        int CANCoderID, 
        Rotation2d defaultAngleOffset
    ) {
        this.DriveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        this.SteerMotor = new CANSparkMax(steerMotorID, MotorType.kBrushless);
        this.SteerEncoder = new CANCoder(CANCoderID);

        this.defaultAngleOffset = defaultAngleOffset;
        configureCANCoder(defaultAngleOffset);
    }

    /**
     * @param A Rotation2d A
     * @param B Rotation2d B
     * @return the angle from A to B
     * on the interval [pi, -pi), in radians
     */
    public static Rotation2d signedAngleBetween(Rotation2d A, Rotation2d B) {
        return new Rotation2d(
            (B.getRadians() - A.getRadians() + Math.PI) % (Math.PI * 2) - Math.PI
        );
    }


    // Configures default CANCoder settings for swerve
    public void configureCANCoder(Rotation2d CANCoderOffset) {
        SteerEncoder.configMagnetOffset(CANCoderOffset.getDegrees());

        // CCW Standard
        SteerEncoder.configSensorDirection(false); 
    }

    /**
     * @return A Rotation2d of the swerve module direction.
     */
    public Rotation2d getRotation2d() {
        return new Rotation2d(
            Math.toRadians(
                SteerEncoder.getAbsolutePosition()
            )
        );
    }

    /**
     * @return A double of the total drive distance in meters
     */
    public double getDriveDistanceMeters() {
        return rotationsToMeters(
            DriveMotor.getEncoder().getPosition()
        );
    }

    /**
     * @return Current speed of drive motors
     */
    public double getDriveSpeedMeters() {
        return rotationsToMeters
            (DriveMotor.getEncoder().getVelocity()
        );
    }
}