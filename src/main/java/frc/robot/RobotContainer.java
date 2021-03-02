// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.constants.Constants;
import frc.robot.constants.Constants.AutoConstants;
import frc.robot.constants.Constants.DriveConstants;
import frc.robot.subsystems.driveTrain.DriveHome;
import frc.robot.subsystems.driveTrain.DriveTrain;
import frc.robot.subsystems.driveTrain.JoystickDrive;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
// import edu.wpi.first.wpilibj.examples.swervecontrollercommand.Constants.AutoConstants;
// import edu.wpi.first.wpilibj.examples.swervecontrollercommand.Constants.DriveConstants;
// import edu.wpi.first.wpilibj.examples.swervecontrollercommand.Constants.OIConstants;
// import edu.wpi.first.wpilibj.examples.swervecontrollercommand.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DriveTrain driveTrain = new DriveTrain();
  private final DriveHome driveHomeCommand = new DriveHome(driveTrain);

  public Joystick driveStick = new Joystick(Constants.ControllerConstants.DRIVER_STICK_CHANNEL);
  public Joystick auxStick = new Joystick(Constants.ControllerConstants.AUX_STICK_CHANNEL);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    driveTrain.setDefaultCommand(new JoystickDrive(driveTrain, driveStick, auxStick, true)); // fieldRelative = false
  }

  // public DriveTrain getDriveTrain(){
  // return m_driveTrain ;
  // }
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driveStick, 1).whenHeld(driveHomeCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    System.out.println("getAutonomousCommand");
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.MAX_Speed_MetersPerSecond,
        AutoConstants.MAX_Acceleration_MetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(driveTrain.kinematics);
    // cubic splines
    // cubic splines
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)), config);

    List<Translation2d> listTranslation2d = List.of(new Translation2d(1.8, .5), new Translation2d(2.5, 1.5),
        new Translation2d(3.5, 2.5), new Translation2d(7.5, 2.5), new Translation2d(8.5, 1.5),
        new Translation2d(9.5, 0.5), new Translation2d(10.5, 1.5), // far point
        new Translation2d(9.5, 2.5), new Translation2d(8.5, 1.5), new Translation2d(7.7, 0.5),
        new Translation2d(5.5, 0.5), new Translation2d(3.5, 0.5), new Translation2d(2.5, 1.5),
        new Translation2d(1.5, 2.5));

    Rotation2d zero = Rotation2d.fromDegrees(0);
    List<Pose2d> listPose2d = List.of(new Pose2d(1.8, 0.5, zero), new Pose2d(2.5, 1.5, zero),
        new Pose2d(3.5, 2.5, zero), new Pose2d(7.5, 2.5, zero), new Pose2d(8.5, 1.5, zero), new Pose2d(9.5, 0.5, zero),
        new Pose2d(10.5, 1.5, zero), new Pose2d(9.5, 2.5, zero), new Pose2d(8.5, 1.5, zero), new Pose2d(7.7, 0.5, zero),
        new Pose2d(5.5, 0.5, zero), new Pose2d(3.5, 0.5, zero), new Pose2d(2.5, 10.5, zero),
        new Pose2d(1.5, 2.5, zero));

    Pose2d offset = new Pose2d(.5, .5, new Rotation2d(0));
    Trajectory exampleTrajectory = DriveTrain.generateTrajectory(config, offset, listPose2d);
    // Trajectory exampleTrajectory =
    // TrajectoryGenerator.generateTrajectory(listTranslation2d, config, offset,
    // listPose2d);

    /**
     * For clamped cubic splines, this method accepts two Pose2d objects, one for
     * the starting waypoint and one for the ending waypoint. The method takes in a
     * vector of Translation2d objects which represent the interior waypoints. The
     * headings at these interior waypoints are determined automatically to ensure
     * continuous curvature. For quintic splines, the method simply takes in a list
     * of Pose2d objects, with each Pose2d representing a point and heading on the
     * field.
     */

    var thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
        AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(exampleTrajectory,
        driveTrain::getPose, // Functional interface to feed supplier
        driveTrain.kinematics,
        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0), new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController, driveTrain::setModuleStates, driveTrain);

    // Reset odometry to the starting pose of the trajectory.
    driveTrain.resetOdometryWithPose2d(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> driveTrain.drive(0, 0, 0, false));

    // https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervecontrollercommand/subsystems/DriveSubsystem.java
  }
}

/**
 * y x 0 0 .5 .5
 * 
 * 
 * 
 */