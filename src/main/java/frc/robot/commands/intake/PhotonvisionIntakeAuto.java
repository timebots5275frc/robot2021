/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import org.photonvision.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.driveTrain.DriveTrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.photonvision.Photonvision;

public class PhotonvisionIntakeAuto extends CommandBase {

  private Intake intake;
  private DriveTrain drivetrain;
  private Photonvision photonvision;
  private PIDController turnPID;

  /**
   * Creates a new PhotonvisionIntakeAuto.
   */
  public PhotonvisionIntakeAuto(Intake _intake, DriveTrain _drivetrain, Photonvision _photonvision) {
    this.intake = _intake;
    this.drivetrain = _drivetrain;
    this.photonvision = _photonvision;
    this.turnPID = new PIDController(0.01, 0, 0);
    addRequirements(this.intake);
    addRequirements(this.drivetrain);
    addRequirements(this.photonvision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.photonvision.setLightIntake(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    PhotonTrackedTarget ball = photonvision.getIntakeLatestResult();

    if (ball != null) {
      double yaw = ball.getYaw();
      double rotRate = turnPID.calculate(yaw, 0);
      // System.out.println("");
      if (Math.abs(rotRate) < 0.3) {
        this.drivetrain.drive(0.1, 0, rotRate, false);
      }
    } else {
      this.drivetrain.drive(0, 0, 0, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.photonvision.setLightIntake(false);
    this.drivetrain.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
