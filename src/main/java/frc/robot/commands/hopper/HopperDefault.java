// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.hopper.Hopper;

public class HopperDefault extends CommandBase {
  private Hopper subsystem;
  private double clock;

  /** Creates a new HopperDefault. */
  public HopperDefault(Hopper subsystem) {
    this.subsystem = subsystem;
    addRequirements(subsystem); // here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    clock = (clock + 1) % 40;

    if (clock <= 30) {
      this.subsystem.setMototrSpeed(Constants.HopperConstants.HOPPER_DEFAULT_SPEED);
    } else if (clock > 10) {
      this.subsystem.setMototrSpeed(-Constants.HopperConstants.HOPPER_DEFAULT_SPEED);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.subsystem.setMototrSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}