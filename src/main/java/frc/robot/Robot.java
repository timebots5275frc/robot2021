// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.driveTrain.AutoNav;

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
    m_robotContainer.driveTrain.resetOdometryWithPose2d(m_robotContainer.trajectory.getInitialPose());

    // SequentialCommandGroup autoCommandGroup = new SequentialCommandGroup(new
    // AutoNav(m_robotContainer.driveTrain),
    // m_autonomousCommand);

    ParallelCommandGroup autoParallelCommandGroup = new ParallelCommandGroup(m_autonomousCommand,
        m_robotContainer.intakeOn, m_robotContainer.hopperDefaultCommand);

    SequentialCommandGroup autoCommandGroup = new SequentialCommandGroup(m_robotContainer.intakeExtend, m_robotContainer.intakeRetract,
    autoParallelCommandGroup);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      // m_autonomousCommand.schedule();
      // m_robotContainer.intakeExtend.schedule();
      // m_robotContainer.intakeRetract.schedule();
      autoCommandGroup.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    this.loggerPeriodic();
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
    this.loggerPeriodic();
  }

  public void loggerPeriodic() {
    Pose2d loc = m_robotContainer.driveTrain.getPose();

    SmartDashboard.putNumber("odometry getX", loc.getX());
    SmartDashboard.putNumber("odometry getY", loc.getY());
    SmartDashboard.putString("odometry getRotation", loc.getRotation().toString());
    SmartDashboard.putNumber("getRotation", m_robotContainer.driveTrain.getHeading().getDegrees());

    SmartDashboard.putNumber("Shooter m_encoder",
    m_robotContainer.subShooter.shooterMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter OutputCurrent",
    m_robotContainer.subShooter.shooterMotor.getOutputCurrent());
    SmartDashboard.putNumber("hoodCanCoder",
    m_robotContainer.subShooter.hoodCanCoder.getAbsolutePosition());
    SmartDashboard.putNumber("Hood getSupplyCurrent",
    m_robotContainer.subShooter.hoodMotor.getSupplyCurrent());
    SmartDashboard.putNumber("Hood getClosedLoopError",
        m_robotContainer.subShooter.hoodMotor.getClosedLoopError());
    

    SmartDashboard.putNumber("Drive 1 Current",
        m_robotContainer.driveTrain.leftFrontSwerveModule.driveMotor.getOutputCurrent());

    // SmartDashboard.putNumber("Drive X Error",
    // m_robotContainer.xController.getPositionError());
    SmartDashboard.putNumber("Drive Y Error", m_robotContainer.yController.getPositionError());
    SmartDashboard.putNumber("Drive getSetpoint", m_robotContainer.yController.getSetpoint());

    SmartDashboard.putNumber("getAccelInstantX", m_robotContainer.driveTrain.imuADIS16470.getAccelInstantX());
    SmartDashboard.putNumber("getAccelInstantY", m_robotContainer.driveTrain.imuADIS16470.getAccelInstantY());
    SmartDashboard.putNumber("getAccelInstantZ", m_robotContainer.driveTrain.imuADIS16470.getAccelInstantZ());

    SmartDashboard.putNumber("getRate", m_robotContainer.driveTrain.imuADIS16470.getRate());

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
