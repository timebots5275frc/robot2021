/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.driveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class JoystickDrive extends CommandBase {
  private final DriveTrain m_drive;
  // private final Joystick joystick;
  public Joystick driveStick;
  public Joystick auxStick;
  public boolean fieldRelative;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(1);

  /**
   * Creates a new DefaultDrive.
   *
   * @param subsystem The drive subsystem this command wil run on.
   * @param joystick  The control input for driving
   */
  public JoystickDrive(boolean fieldRelative, DriveTrain subsystem, Joystick _driveStick, Joystick _auxstick) {
    this.m_drive = subsystem;
    this.driveStick = driveStick;
    this.auxStick = auxStick;
    this.fieldRelative = fieldRelative;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // double throt = (joystick.getRawAxis(3) * -1 + 1) / 2;

    // double m_forward = joystick.getRawAxis(1) * throt;
    // double m_rotation = joystick.getRawAxis(2) * throt * -1;

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    double xSpeed = this.smartJoystick(driveStick.getY(), Constants.ControllerConstants.DEADZONE_DRIVE)
        * Constants.DriveConstants.MAX_DRIVE_SPEED;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    double ySpeed = this.smartJoystick(driveStick.getX(), Constants.ControllerConstants.DEADZONE_DRIVE)
        * Constants.DriveConstants.MAX_DRIVE_SPEED;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    double rotRate = this.smartJoystick(driveStick.getTwist(), Constants.ControllerConstants.DEADZONE_STEER)
        * Constants.DriveConstants.MAX_TWIST_RATE;

    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("rotRate", rotRate);

    m_drive.drive(xSpeed, ySpeed, rotRate, fieldRelative);
  }

  /**
   * 
   * @param _in
   * @param deadZoneSize between -1 and 1
   * @return
   */

  public double smartJoystick(double _in, double deadZoneSize) {
    if (Math.abs(_in) < deadZoneSize) {
      return 0;
    }

    if (_in > 0) {
      return _in - deadZoneSize;
    } else if (_in < 0) {
      return _in + deadZoneSize;
    }
    return 0;
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
