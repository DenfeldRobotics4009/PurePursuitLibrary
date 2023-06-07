// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Drive;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Swerve.DriveControls;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final Joystick m_jsDriver = new Joystick(0);
  private final Joystick m_jsDriverSteer = new Joystick(1);

  private final SwerveDrive m_driveTrain = new SwerveDrive();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_driveTrain.setDefaultCommand(
      new Drive(m_driveTrain, new DriveControls(m_jsDriver, m_jsDriverSteer))
    );

    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    // m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
    //         m_drivetrainSubsystem,
    //         () -> -modifyAxis(m_jsDriver.getX()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> -modifyAxis(m_jsDriver.getY()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> -modifyAxis(m_jsDriver.getZ()) * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    // ));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }

  public static double Clamp(double input, double max, double min) {
    if (input > max) {return max;}
    else if (input < min) {return min;}
    else {return input;}
  }
}
