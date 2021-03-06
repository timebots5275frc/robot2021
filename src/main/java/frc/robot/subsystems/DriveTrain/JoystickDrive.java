/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.driveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.DriveConstants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.controller.PIDController;

public class JoystickDrive extends CommandBase {
	private final DriveTrain driveTrain;
	// private final Joystick joystick;
	private Joystick driveStick;
	private Joystick auxStick;
	private boolean fieldRelative;

	// Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
	private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(4);
	private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(4);
	private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(1);

	// Two Joystick Drive
	private PIDController steerAnglePID;
	private boolean twoJoystickDrive;

	/**
	 * Creates a new DefaultDrive.
	 *
	 * @param subsystem The drive subsystem this command wil run on.
	 * @param joystick  The control input for driving
	 */
	public JoystickDrive(DriveTrain subsystem, Joystick _driveStick, Joystick _auxstick, boolean fieldRelative,
			boolean twoJoystickDrive) {
		this.driveTrain = subsystem;
		this.driveStick = _driveStick;
		this.auxStick = _auxstick;
		this.fieldRelative = fieldRelative;
		this.twoJoystickDrive = twoJoystickDrive;

		addRequirements(driveTrain);

		steerAnglePID = new PIDController(1, 0.1, 0);
		steerAnglePID.enableContinuousInput(-Math.PI, Math.PI);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// System.out.println(" JoystickDrive Running");

		if (twoJoystickDrive) {
			twoJoystickDrive();
		} else {
			oneJoystickDrive();
		}

	}

	public void twoJoystickDrive() {
		double xSpeed = this.joystickDeadZone(driveStick.getY() * -1, Constants.ControllerConstants.DEADZONE_DRIVE)
				* Constants.DriveConstants.MAX_DRIVE_SPEED;

		double ySpeed = this.joystickDeadZone(driveStick.getX() * -1, Constants.ControllerConstants.DEADZONE_DRIVE)
				* Constants.DriveConstants.MAX_DRIVE_SPEED;

		double throttle = (-driveStick.getThrottle() + 1) / 2; // between 0 and 1 = 0% and 100%

		xSpeed *= throttle;
		ySpeed *= throttle;

		xSpeed = m_xspeedLimiter.calculate(xSpeed);
		ySpeed = m_yspeedLimiter.calculate(ySpeed);
		double auxX = auxStick.getY() * -1;
		double auxY = auxStick.getX() * -1;

		Rotation2d setpoint = new Rotation2d(auxX, auxY);

		double rotRate = steerAnglePID.calculate(driveTrain.getHeading().getRadians(), setpoint.getRadians());

		SmartDashboard.putNumber("smart xSpeed", xSpeed);
		SmartDashboard.putNumber("smart ySpeed", ySpeed);
		SmartDashboard.putNumber("smart rotRate", rotRate);
		SmartDashboard.putNumber("smart setpoint", setpoint.getDegrees());
		SmartDashboard.putNumber("smart auxX", auxX);
		SmartDashboard.putNumber("smart auxY", auxY);
		SmartDashboard.putNumber("driveStick.getPOV()", driveStick.getPOV());

		driveTrain.drive(xSpeed, ySpeed, rotRate, fieldRelative);

	}

	public void oneJoystickDrive() {

		double xSpeed = this.joystickDeadZone(driveStick.getY() * -1, Constants.ControllerConstants.DEADZONE_DRIVE)
				* Constants.DriveConstants.MAX_DRIVE_SPEED;

		double ySpeed = this.joystickDeadZone(driveStick.getX() * -1, Constants.ControllerConstants.DEADZONE_DRIVE)
				* Constants.DriveConstants.MAX_DRIVE_SPEED;

		double rotRate = this.joystickDeadZone(driveStick.getTwist() * -1, Constants.ControllerConstants.DEADZONE_STEER)
				* Constants.DriveConstants.MAX_TWIST_RATE;

		double throttle = (-driveStick.getThrottle() + 1) / 2; // between 0 and 1 = 0% and 100%

		xSpeed *= throttle;
		ySpeed *= throttle;
		rotRate *= throttle;

		xSpeed = m_xspeedLimiter.calculate(xSpeed);
		ySpeed = m_yspeedLimiter.calculate(ySpeed);
		// m_xspeedLimiter.calculate(rotRate);

		SmartDashboard.putNumber("throttle", throttle);

		SmartDashboard.putNumber("xSpeed", driveStick.getY());
		SmartDashboard.putNumber("ySpeed", driveStick.getX());
		SmartDashboard.putNumber("rotRate", driveStick.getTwist());

		SmartDashboard.putNumber("smart xSpeed", xSpeed);
		SmartDashboard.putNumber("smart ySpeed", ySpeed);
		SmartDashboard.putNumber("smart rotRate", rotRate);

		driveTrain.drive(xSpeed, ySpeed, rotRate, fieldRelative);
	}

	/**
	 * 
	 * @param _in
	 * @param deadZoneSize between -1 and 1
	 * @return
	 */

	public double joystickDeadZone(double _in, double deadZoneSize) {
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

	public void setFieldRelative(boolean _fieldRelative) {
		this.fieldRelative = _fieldRelative;
	}

	public void setTwoJoystickDrive(boolean _twoJoystickDrive) {
		this.twoJoystickDrive = _twoJoystickDrive;
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
