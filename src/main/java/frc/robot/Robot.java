// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.DriveConstants;
import frc.robot.subsystems.driveTrain.DriveTrain;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  // public Joystick driveStick = new Joystick(Constants.ControllerConstants.DRIVER_STICK_CHANNEL);
  // private final DriveTrain m_swerve = new DriveTrain();

  // // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  // private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  // private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  // private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    // double xSpeed = m_xspeedLimiter.calculate( driveStick.getY() ) * DriveConstants.MAX_DRIVE_SPEED ;


    // // Get the y speed or sideways/strafe speed. We are inverting this because
    // // we want a positive value when we pull to the left. Xbox controllers
    // // return positive values when you pull to the right by default.
    // double ySpeed = m_yspeedLimiter.calculate( driveStick.getX() ) * DriveConstants.MAX_DRIVE_SPEED; 


    // // Get the rate of angular rotation. We are inverting this because we want a
    // // positive value when we pull to the left (remember, CCW is positive in
    // // mathematics). Xbox controllers return positive values when you pull to
    // // the right by default.
    // double rotRate = m_rotLimiter.calculate(driveStick.getTwist()) * DriveConstants.MAX_TWIST_RATE;


    // SmartDashboard.putNumber("xSpeed", xSpeed);
    // SmartDashboard.putNumber("ySpeed", ySpeed);
    // SmartDashboard.putNumber("Rotation", rotRate);

    // m_swerve.drive(xSpeed, ySpeed, rotRate, false);

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
