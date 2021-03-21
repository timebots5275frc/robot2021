// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterFire extends CommandBase {
  private Shooter subsystemShooter;
  /** Creates a new ShooterFire. */

  public ShooterFire(Shooter subsystem) {
    this.subsystemShooter = subsystem;
    addRequirements(subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.subsystemShooter.setMototrSpeed(Constants.ShooterConstants.SHOOTER_FIRE_RPM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
