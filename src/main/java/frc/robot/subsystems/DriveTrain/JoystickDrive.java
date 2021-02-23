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
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class JoystickDrive extends CommandBase {
  private final DriveTrain m_drive;
  // private final Joystick joystick;
  private Joystick driveStick;
  private Joystick auxStick;
  private boolean fieldRelative;

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
  public JoystickDrive(DriveTrain subsystem, Joystick _driveStick, Joystick _auxstick, boolean fieldRelative) {
    this.m_drive = subsystem;
    this.driveStick = _driveStick;
    this.auxStick = _auxstick;
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
    System.out.println(" JoystickDrive Running");

    if (driveStick.getRawButtonPressed(12)) {
      m_drive.imu.reset();
      System.out.println("m_drive.imu.reset();");
    }

    if (driveStick.getRawButtonPressed(10)) {
      System.out.println("m_drive.m_odometry.resetPosition");
      m_drive.m_odometry.resetPosition( new Pose2d(), new Rotation2d(0)  );
    }

    double xSpeed = this.smartJoystick(driveStick.getY(), Constants.ControllerConstants.DEADZONE_DRIVE)
        * Constants.DriveConstants.MAX_DRIVE_SPEED;

    double ySpeed = this.smartJoystick(driveStick.getX(), Constants.ControllerConstants.DEADZONE_DRIVE)
        * Constants.DriveConstants.MAX_DRIVE_SPEED;

    double rotRate = this.smartJoystick(driveStick.getTwist(), Constants.ControllerConstants.DEADZONE_STEER)
        * Constants.DriveConstants.MAX_TWIST_RATE;

    double throttle = (-driveStick.getThrottle() + 1) / 2; // between 0 and 1 = 0% and 100%

    xSpeed *= throttle;
    ySpeed *= throttle;
    rotRate *= throttle;

    SmartDashboard.putNumber("throttle", throttle);

    SmartDashboard.putNumber("xSpeed", driveStick.getY());
    SmartDashboard.putNumber("ySpeed", driveStick.getX());
    SmartDashboard.putNumber("rotRate", driveStick.getTwist());

    SmartDashboard.putNumber("smart xSpeed", xSpeed);
    SmartDashboard.putNumber("smart ySpeed", ySpeed);
    SmartDashboard.putNumber("smart rotRate", rotRate);

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
      return (_in - deadZoneSize) / (1 - deadZoneSize);
    } else if (_in < 0) {
      return (_in + deadZoneSize) / (1 - deadZoneSize);
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
