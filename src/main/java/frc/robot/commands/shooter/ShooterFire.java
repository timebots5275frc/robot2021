// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterFire extends CommandBase {
  private Shooter subsystemShooter;
  private Joystick joy;
  
  /** Creates a new ShooterFire. */

  public ShooterFire(Shooter subsystem, Joystick joy) {
    this.subsystemShooter = subsystem;
    this.joy = joy;
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
    // System.out.println("Constants.ShooterConstants.SHOOTER_FIRE_RPM" + Constants.ShooterConstants.SHOOTER_FIRE_RPM);
    // this.subsystemShooter.mo

    
    SmartDashboard.putNumber("m_encoder", this.subsystemShooter.shooterMotor.getEncoder().getVelocity());

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
