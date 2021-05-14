/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.driveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.driveTrain.DriveTrain;

public class AutoTurnMinnesotaChallenge extends CommandBase {
  private long startTimeMillis;
  private DriveTrain driveTrain;

  /**
   * Creates a new AutoTurnMinnesotaChallenge.
   */
  public AutoTurnMinnesotaChallenge(DriveTrain subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = subsystem;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.startTimeMillis = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    long timeElapsed = System.currentTimeMillis() - this.startTimeMillis;
    System.out.println("timeElapsed = " + timeElapsed);

    if (timeElapsed > 9000) { // seconds
      driveTrain.setAutoTurnOffsetRadians(-Math.PI / 2);
    } else if (timeElapsed > 6000) { // seconds
      driveTrain.setAutoTurnOffsetRadians(0);
    } else if (timeElapsed > 3000) { // seconds
      driveTrain.setAutoTurnOffsetRadians(Math.PI / 2);
    }

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
