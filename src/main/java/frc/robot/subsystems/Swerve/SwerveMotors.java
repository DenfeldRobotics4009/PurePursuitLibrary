package frc.robot.subsystems.Swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.libraries.PIDController;


public class SwerveMotors {
    public CANSparkMax DriveMotor, TurnMotor;
    public CANCoder TurnEncoder;

    PIDController 
        SpeedDriveController = new PIDController(0, 0.1, 0, 0), // TODO
        PositionTurnController = new PIDController(0.1, 0, 0, 0); // TODO
    
    // Configures default CANCOder settings for swerve
    void configureCANCoder(Rotation2d CANCoderOffset) {
        TurnEncoder.configMagnetOffset(CANCoderOffset.getDegrees());
    }

    public SwerveMotors(CANSparkMax DriveMotor, CANSparkMax TurnMotor, CANCoder TurnEncoder, Rotation2d CANCoderOffset) {
        this.DriveMotor = DriveMotor;
        this.TurnMotor = TurnMotor;
        this.TurnEncoder = TurnEncoder;

        SpeedDriveController.setTarget(0);

        configureCANCoder(CANCoderOffset);
        
    }

    public SwerveMotors(int DriveMotorId, int TurnMotorId, int CANCoderId, Rotation2d CANCoderOffset) {

        this.DriveMotor = new CANSparkMax(DriveMotorId, MotorType.kBrushless);
        this.TurnMotor = new CANSparkMax(TurnMotorId, MotorType.kBrushless);
        this.TurnEncoder = new CANCoder(CANCoderId);

        SpeedDriveController.setTarget(0);

        configureCANCoder(CANCoderOffset);
    }
}