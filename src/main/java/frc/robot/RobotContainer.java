// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.hopper.HopperBack;
import frc.robot.commands.hopper.HopperDefault;
import frc.robot.commands.hopper.HopperFire;
import frc.robot.commands.shooter.ShooterDefault;
import frc.robot.commands.shooter.ShooterFire;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Joystick driveStick = new Joystick(0);

  private Shooter subShooter = new Shooter();
  private ShooterFire shooterFireCommand = new ShooterFire(subShooter, driveStick);
  private ShooterDefault shooterDefaultCommand = new ShooterDefault(subShooter);


  private Hopper subHopper = new Hopper();
  private HopperFire hopperFireCommand = new HopperFire(subHopper);
  private HopperBack hopperBackCommand = new HopperBack(subHopper);
  private HopperDefault hopperDefaultCommand = new HopperDefault(subHopper);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    subShooter.setDefaultCommand(shooterDefaultCommand);
    subHopper.setDefaultCommand(hopperDefaultCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driveStick, 1).whenHeld(shooterFireCommand);
    new JoystickButton(driveStick, 3).whenHeld(hopperFireCommand);
    new JoystickButton(driveStick, 4).whenHeld(hopperBackCommand);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   return m_autoCommand;
  // }
}
