package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.libraries.PIDController;

public class SwerveModule {

    public class SwerveMotors {
        public CANSparkMax DriveMotor, TurnMotor;
        public CANCoder TurnEncoder;

        PIDController 
            SpeedDriveController = new PIDController(0, 0.1, 0, 0), // TODO
            PositionTurnController = new PIDController(0.1, 0, 0, 0); // TODO
        

        public SwerveMotors(CANSparkMax DriveMotor, CANSparkMax TurnMotor, CANCoder TurnEncoder) {
            this.DriveMotor = DriveMotor;
            this.TurnMotor = TurnMotor;
            this.TurnEncoder = TurnEncoder;

            SpeedDriveController.setTarget(0);

            // TODO Configure turn encoder
            
        }
    }

    final SwerveMotors kMotors;

    /**
     * Construct a single swerve module
     * @param Motors Group of motors and encoder to use
     */
    public SwerveModule(SwerveMotors Motors) {
        this.kMotors = Motors;
    }

    /**
     * Drive the current swerve module using optimization
     * @param State Un-Optimized state
     */
    public void Drive(SwerveModuleState State) {
        SwerveModuleState OptimizedState = OptimizeState(
            State, 
            // Assume reading is degrees
            new Rotation2d(Math.toRadians(kMotors.TurnEncoder.getAbsolutePosition()))
        );

        // Set drive motor

        double realSpeed = kMotors.DriveMotor.getEncoder().getVelocity();
        double speedDifference = (OptimizedState.speedMetersPerSecond / Constants.Swerve.DriveGearRatioFactor) - realSpeed;
        // Give the input to the pid controller to allow for speed compensation
        kMotors.SpeedDriveController.setInput(speedDifference);

        kMotors.DriveMotor.set(
            // Clamp to avoid overload
            RobotContainer.Clamp(
                (OptimizedState.speedMetersPerSecond / Constants.Swerve.DriveGearRatioFactor)
                    // Add the PID output to the input
                    // TODO may need to be reversed
                    + kMotors.SpeedDriveController.calculate(1, -1),
                1, -1
            )
        );

        // Set turn motor
        kMotors.PositionTurnController.setTarget(OptimizedState.angle.getDegrees());
        kMotors.PositionTurnController.setInput(kMotors.TurnEncoder.getAbsolutePosition());
        // TODO may need to be reversed
        kMotors.TurnMotor.set(kMotors.PositionTurnController.calculate(1, -1));
    }

    /**
     * Optimizes the provided state to not turn more than 90 degrees from the current position
     * @param State
     * @param CurrentRotation
     * @return
     */
    public static SwerveModuleState OptimizeState(SwerveModuleState State, Rotation2d CurrentRotation) {
        Rotation2d RotationDifference = State.angle.minus(CurrentRotation);


        SwerveModuleState OptimizedState = State;

        if (Math.abs(RotationDifference.getDegrees()) > 90) {
            // Reverse wheel direction and reverse wheel speed
            OptimizedState = new SwerveModuleState(-State.speedMetersPerSecond, State.angle.plus(new Rotation2d(180)));
        }
    
        return OptimizedState;
    }
}
