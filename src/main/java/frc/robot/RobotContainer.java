// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.driveTrain.DriveTrain;

import frc.robot.constants.Constants;

import frc.robot.commands.driveTrain.JoystickDrive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.hopper.HopperBack;
import frc.robot.commands.hopper.HopperDefault;
import frc.robot.commands.hopper.HopperFire;
import frc.robot.commands.shooter.ShooterDefault;
import frc.robot.commands.shooter.ShooterFire;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;

import java.io.IOException;
import java.nio.file.Path;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	public Joystick driveStick = new Joystick(Constants.ControllerConstants.DRIVER_STICK_CHANNEL);
	public Joystick auxStick = new Joystick(Constants.ControllerConstants.AUX_STICK_CHANNEL);

	public final DriveTrain driveTrain = new DriveTrain();
	private final JoystickDrive driveJoyCommand = new JoystickDrive(driveTrain, driveStick, auxStick, false);
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

		driveTrain.setDefaultCommand(driveJoyCommand);

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		new JoystickButton(driveStick, 7).whenPressed(() -> driveJoyCommand.setFieldRelative(false) );
		new JoystickButton(driveStick, 8).whenPressed(() -> driveJoyCommand.setFieldRelative(true) );
		
		new JoystickButton(driveStick, 9).whenPressed(() -> driveTrain.resetADIS16470() );
		new JoystickButton(driveStick, 10).whenPressed(() -> driveTrain.resetOdometry());
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		System.out.println("getAutonomousCommand");
		// Create config for trajectory
		TrajectoryConfig config = new TrajectoryConfig(Constants.AutoConstants.MAX_Speed_MetersPerSecond,
		Constants.AutoConstants.MAX_Acceleration_MetersPerSecondSquared)
						// Add kinematics to ensure max speed is actually obeyed
						.setKinematics(driveTrain.kinematics);

		String trajectoryJSON = "paths/slalom-path.wpilib.json";
		Trajectory trajectory = new Trajectory();
		try {
			Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
			trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
		} catch (IOException ex) {
			DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
			// System.out.println("Unable to open trajectory: " + trajectoryJSON);
		}

		/**
		 * For clamped cubic splines, this method accepts two Pose2d objects, one for
		 * the starting waypoint and one for the ending waypoint. The method takes in a
		 * vector of Translation2d objects which represent the interior waypoints. The
		 * headings at these interior waypoints are determined automatically to ensure
		 * continuous curvature. For quintic splines, the method simply takes in a list
		 * of Pose2d objects, with each Pose2d representing a point and heading on the
		 * field.
		 */

		var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
		Constants.AutoConstants.kThetaControllerConstraints);
		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, driveTrain::getPose, // Functional
																														// interface
																														// to
																														// feed
																														// supplier
				driveTrain.kinematics,
				// Position controllers
				new PIDController(Constants.AutoConstants.kPXController, 0, 0),
				new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController, driveTrain::setModuleStates,
				driveTrain);

		// Reset odometry to the starting pose of the trajectory.
		driveTrain.resetOdometryWithPose2d(trajectory.getInitialPose());

		// Run path following command, then stop at the end.
		return swerveControllerCommand.andThen(() -> driveTrain.drive(0, 0, 0, false));

		// https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervecontrollercommand/subsystems/DriveSubsystem.java
	}
}
