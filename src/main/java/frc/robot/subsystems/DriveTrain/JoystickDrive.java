/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.driveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.Joystick;

public class JoystickDrive extends CommandBase {
  private final DriveTrain m_drive;
  private final Joystick joystick;

  /**
   * Creates a new DefaultDrive.
   *
   * @param subsystem The drive subsystem this command wil run on.
   * @param joystick  The control input for driving 
   */
  public JoystickDrive(DriveTrain subsystem, Joystick joystick) {
    this.m_drive = subsystem;
    this.joystick = joystick;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double throt = (joystick.getRawAxis(3) * -1 + 1) / 2;

    double m_forward = joystick.getRawAxis(1) * throt;
    double m_rotation = joystick.getRawAxis(2) * throt * -1;

    // m_drive.arcadeDrive(m_forward, m_rotation);
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
