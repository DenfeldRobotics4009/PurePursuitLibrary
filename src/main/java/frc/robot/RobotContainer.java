// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.Autos;
import frc.robot.commands.Drive;
import frc.robot.subsystems.SwerveDrive;
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

  final Controls controls = new Controls(m_jsDriver, m_jsDriverSteer);

  private final SwerveDrive m_driveTrain = SwerveDrive.GetInstance();

  SendableChooser<SequentialCommandGroup> autoChooser = new SendableChooser<>();

  // Tab to display autonous data
  public static final ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
  // Entries for auto data
  public static final GenericEntry 
    lookAheadEntry = autoTab.add("Look Ahead", 0)
      .withPosition(0, 4).withSize(20, 4).withWidget("Graph").getEntry(), 
    speedEntry = autoTab.add("State Speed", 0)
      .withPosition(0,0).withSize(10, 4).withWidget("Graph").getEntry(),
    distanceFromGoalEntry = autoTab.add("Distance to Goal", 0)
      .withPosition(10,0).withSize(10, 4).withWidget("Graph").getEntry(),
    lastCrossedPointEntry = autoTab.add("Last Crossed Point Index", 0)
      .withPosition(20, 2).withSize(5, 1).getEntry();
    

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_driveTrain.setDefaultCommand(
      new Drive(m_driveTrain, controls)
    );

    // Iterate through enum of autos
    for (Autos autoEnum : Autos.values()) {
      // Add all enum objects to autoChooser, with name given by enum type
      // The last added option will remain as default
      autoChooser.addOption(autoEnum.toString(), autoEnum.getSequence());
    }
    
    autoTab.add("Autonomous", autoChooser).withPosition(20, 0).withSize(5, 2);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Schedule chosen command
    return autoChooser.getSelected();
  }

  /**
   * 
   * @param input
   * @param max return max if input > max
   * @param min return min is input < min
   * @return clamped input
   */
  public static double Clamp(double input, double max, double min) {
    if (input > max) {return max;}
    else if (input < min) {return min;}
    else {return input;}
  }
  
}
