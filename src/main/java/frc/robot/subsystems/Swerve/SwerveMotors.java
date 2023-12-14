package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Swerve.SwerveModuleConstants;

public class SwerveMotors {
    CANSparkMax DriveMotor, SteerMotor;
    public CANCoder SteerEncoder;
    final String Name;
    
    public static double rotationsToMeters(double rotations) {
        return rotations * Swerve.rotationsToMeters;
    }
    
    public static double metersToRotations(double meters) {
        return meters / Swerve.rotationsToMeters;
    }

    /**
     * 
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

    public void configureCANSparkMAXs() {
        DriveMotor.setOpenLoopRampRate(Swerve.driveRampRateSeconds); // TODO: Tune for physical maxima
        SteerMotor.setOpenLoopRampRate(Swerve.steerRampRateSeconds);
    }

    public SwerveMotors(SwerveModuleConstants Constants) {
        this.DriveMotor = new CANSparkMax(Constants.DriveMotorID, MotorType.kBrushless);
        this.SteerMotor = new CANSparkMax(Constants.SteerMotorID, MotorType.kBrushless);
        this.SteerEncoder = new CANCoder(Constants.CANCoderID);
        this.Name = Constants.Name;

        configureCANCoder(Constants.DefaultOffset);
        configureCANSparkMAXs();
    }

    public SwerveMotors(
        CANSparkMax DriveMotor, 
        CANSparkMax TurnMotor, 
        CANCoder TurnEncoder, 
        Rotation2d CANCoderOffset, 
        String Name
    ) {
        this.DriveMotor = DriveMotor;
        this.SteerMotor = TurnMotor;
        this.SteerEncoder = TurnEncoder;
        this.Name = Name;

        configureCANCoder(CANCoderOffset);
        configureCANSparkMAXs();
    }

    public SwerveMotors(
        int DriveMotorId, 
        int TurnMotorId, 
        int CANCoderId, 
        Rotation2d CANCoderOffset, 
        String Name
    ) {
        this.DriveMotor = new CANSparkMax(DriveMotorId, MotorType.kBrushless);
        this.SteerMotor = new CANSparkMax(TurnMotorId, MotorType.kBrushless);
        this.SteerEncoder = new CANCoder(CANCoderId);
        this.Name = Name;

        configureCANCoder(CANCoderOffset);
        configureCANSparkMAXs();
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