// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import frc.robot.subsystems.driveTrain.DriveTrain;

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
import frc.robot.commands.hopper.HopperBack;
import frc.robot.commands.hopper.HopperDefault;
import frc.robot.commands.hopper.HopperFire;
import frc.robot.commands.shooter.SetHoodAngle;
import frc.robot.commands.shooter.ShooterDefault;
import frc.robot.commands.shooter.ShooterFire;
import frc.robot.subsystems.photonvision.Photonvision;
import frc.robot.subsystems.driveTrain.DriveTrain;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.commands.intake.IntakeExtend;
import frc.robot.commands.intake.IntakeOff;
import frc.robot.commands.intake.IntakeOn;
import frc.robot.commands.intake.IntakeRetract;
import frc.robot.commands.intake.IntakeToggle;
import frc.robot.commands.intake.PhotonvisionIntakeAuto;
import frc.robot.subsystems.intake.Intake;

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
	public XboxController xboxController = new XboxController(Constants.ControllerConstants.XBOXCONTROLLER_CHANNEL);

	public final DriveTrain driveTrain = new DriveTrain();
	private final JoystickDrive driveJoyCommand = new JoystickDrive(driveTrain, driveStick, auxStick, true);

	private Intake intakeSubsystem = new Intake();
	public IntakeExtend intakeExtend = new IntakeExtend(intakeSubsystem);
	public IntakeRetract intakeRetract = new IntakeRetract(intakeSubsystem);
	public IntakeOn intakeOn = new IntakeOn(intakeSubsystem);
	private IntakeOff intakeOff = new IntakeOff(intakeSubsystem);
	private IntakeToggle intakeToggle = new IntakeToggle(intakeSubsystem);

	// private Pneumatics pneumaticsSubsystem = new Pneumatics();

	public Shooter subShooter = new Shooter();
	private ShooterFire shooterFireCommand = new ShooterFire(subShooter, Constants.ShooterConstants.SHOOTER_FIRE_RPM);
	private ShooterFire shooterSlowFireCommand = new ShooterFire(subShooter, 4000);
	private ShooterDefault shooterDefaultCommand = new ShooterDefault(subShooter);
	private SetHoodAngle hoodForward = new SetHoodAngle(subShooter, 120);
	private SetHoodAngle hoodReverse = new SetHoodAngle(subShooter, -120);
	private SetHoodAngle hoodZero = new SetHoodAngle(subShooter, 0);

	private Hopper subHopper = new Hopper();
	private HopperFire hopperFireCommand = new HopperFire(subHopper);
	private HopperBack hopperBackCommand = new HopperBack(subHopper);
	public HopperDefault hopperDefaultCommand = new HopperDefault(subHopper);

	private Photonvision subPhotonvision = new Photonvision();
	private PhotonvisionIntakeAuto intakeAuto = new PhotonvisionIntakeAuto(intakeSubsystem, driveTrain,
			subPhotonvision);

	public Trajectory trajectory;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the button bindings
		configureButtonBindings();
		driveTrain.setDefaultCommand(driveJoyCommand);
		subShooter.setDefaultCommand(shooterDefaultCommand);
		// subHopper.setDefaultCommand(hopperDefaultCommand);
		// intakeSubsystem.setDefaultCommand(intakeOff);

		String trajectoryJSON = "paths/output/turnTest.wpilib.json";
		trajectory = new Trajectory();
		try {
			Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
			trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
		} catch (IOException ex) {
			DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
			// System.out.println("Unable to open trajectory: " + trajectoryJSON);
		}
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		// new JoystickButton(driveStick, 7).whenPressed(() ->
		// driveJoyCommand.setFieldRelative(false));
		// new JoystickButton(driveStick, 8).whenPressed(() ->
		// driveJoyCommand.setFieldRelative(true));

		new JoystickButton(driveStick, 7).whenPressed(() -> driveTrain.resetADIS16470());
		new JoystickButton(driveStick, 8).whenPressed(() -> driveTrain.resetOdometry());
		new JoystickButton(driveStick, 10).whenPressed(() -> driveTrain.calibrateADIS16470());

		new JoystickButton(driveStick, 1).whenHeld(shooterFireCommand);
		new JoystickButton(driveStick, 2).whenHeld(shooterSlowFireCommand );
		new JoystickButton(driveStick, 3).whenHeld(hopperBackCommand);
		new JoystickButton(driveStick, 4).whenHeld(hopperFireCommand);

		new JoystickButton(driveStick, 5).whenHeld(intakeOff);
		new JoystickButton(driveStick, 6).whenHeld(intakeOn);
		new JoystickButton(driveStick, 6).whenHeld(hopperDefaultCommand);

		// new JoystickButton(driveStick, 2).whenPressed(intakeToggle);
		new JoystickButton(driveStick, 11).whenPressed(intakeRetract);
		new JoystickButton(driveStick, 12).whenPressed(intakeExtend);
		// new JoystickButton(driveStick, 5).whenPressed(intakeOff);

		// new JoystickButton(driveStick, 11).whenPressed(() -> {
		// subPhotonvision.toggleLightIntake();
		// });
		// new JoystickButton(driveStick, 11).toggleWhenPressed(intakeAuto);
		// new JoystickButton(driveStick, 12).whenPressed(() -> {
		// 	subPhotonvision.toggleLightShooter();
		// });

		// new JoystickButton(driveStick, 3).whenHeld(forward);
		// new JoystickButton(driveStick, 4).whenHeld(reverse);
		// new JoystickButton(driveStick, 6).whenHeld(zero);

		new JoystickButton(auxStick, 7).whenHeld(hoodReverse);
		new JoystickButton(auxStick, 9).whenHeld(hoodZero);
		new JoystickButton(auxStick, 11).whenHeld(hoodForward);

	}

	public PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
	public PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
	public ProfiledPIDController thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
	Constants.AutoConstants.kThetaControllerConstraints);

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		System.out.println("getAutonomousCommand");

		/**
		 * For clamped cubic splines, this method accepts two Pose2d objects, one for
		 * the starting waypoint and one for the ending waypoint. The method takes in a
		 * vector of Translation2d objects which represent the interior waypoints. The
		 * headings at these interior waypoints are determined automatically to ensure
		 * continuous curvature. For quintic splines, the method simply takes in a list
		 * of Pose2d objects, with each Pose2d representing a point and heading on the
		 * field.
		 */

		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		thetaController.getSetpoint();

		// PIDController xController = new
		// PIDController(Constants.AutoConstants.kPXController, 0, 0);
		// PIDController yController = new
		// PIDController(Constants.AutoConstants.kPYController, 0, 0);

		SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, driveTrain::getPose,
				driveTrain.kinematics,
				// Position controllers
				xController, yController, thetaController, driveTrain::setModuleStates, driveTrain);

		// Reset odometry to the starting pose of the trajectory.
		// driveTrain.resetOdometryWithPose2d(trajectory.getInitialPose());

		// Run path following command, then stop at the end.
		return swerveControllerCommand.andThen(() -> driveTrain.drive(0, 0, 0, false));

		// https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervecontrollercommand/subsystems/DriveSubsystem.java
	}
}
