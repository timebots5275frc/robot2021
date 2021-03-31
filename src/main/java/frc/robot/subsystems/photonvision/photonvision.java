/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.photonvision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Photonvision extends SubsystemBase {
  PhotonCamera cameraShooter = new PhotonCamera("MyCamera");
  PhotonCamera cameraIntake = new PhotonCamera("MyCamera");

  /**
   * Creates a new photonvision.
   */
  public Photonvision() {
  }

  public void getLatestResult() {
    cameraShooter.getLatestResult();
    cameraIntake.getLatestResult();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
