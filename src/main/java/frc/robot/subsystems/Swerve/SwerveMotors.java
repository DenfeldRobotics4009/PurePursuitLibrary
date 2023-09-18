package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Swerve.SwerveModuleConstants;
import frc.robot.libraries.PIDController;


public class SwerveMotors {
    CANSparkMax DriveMotor, TurnMotor;
    CANCoder TurnEncoder;
    final String Name;

    PIDController PositionTurnController = new PIDController(0.01, 0, 0, 0); // TODO Tune kP
    
    public static double rotationsToMeters(double rotations) {
        return rotations * Swerve.rotationsToMeters;
    }
    
    public static double metersToRotations(double meters) {
        return meters / Swerve.rotationsToMeters;
    }

    // Configures default CANCOder settings for swerve
    public void configureCANCoder(Rotation2d CANCoderOffset) {
        TurnEncoder.configMagnetOffset(CANCoderOffset.getDegrees());

        // CCW Standard
        TurnEncoder.configSensorDirection(false); 
    }

    public void configureCANSparkMAXs() {
        DriveMotor.setOpenLoopRampRate(0.2);
        TurnMotor.setOpenLoopRampRate(0);
    }

    public SwerveMotors(SwerveModuleConstants Constants) {
        this.DriveMotor = new CANSparkMax(Constants.DriveMotorID, MotorType.kBrushless);
        this.TurnMotor = new CANSparkMax(Constants.SteerMotorID, MotorType.kBrushless);
        this.TurnEncoder = new CANCoder(Constants.CANCoderID);
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
        this.TurnMotor = TurnMotor;
        this.TurnEncoder = TurnEncoder;
        this.Name = Name;

        //SpeedDriveController.setTarget(0);

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
        this.TurnMotor = new CANSparkMax(TurnMotorId, MotorType.kBrushless);
        this.TurnEncoder = new CANCoder(CANCoderId);
        this.Name = Name;
        //SpeedDriveController.setTarget(0);

        configureCANCoder(CANCoderOffset);
        configureCANSparkMAXs();
    }

    /**
     * @return A Rotation2d of the swerve module direction.
     */
    public Rotation2d getRotation2d() {
        return new Rotation2d(
            Math.toRadians(
                TurnEncoder.getAbsolutePosition()
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
}