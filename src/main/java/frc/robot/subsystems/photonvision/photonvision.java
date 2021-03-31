/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.photonvision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineResult;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Photonvision extends SubsystemBase {
  PhotonCamera cameraShooter = new PhotonCamera("MyCamera");
  PhotonCamera cameraIntake = new PhotonCamera("MyCamera");

  Solenoid lightShooter = new Solenoid(6);
  Solenoid lightIntake = new Solenoid(7);

  /**
   * Creates a new photonvision.
   */
  public Photonvision() {
  }

  public void getLatestResult() {
    PhotonPipelineResult result = cameraShooter.getLatestResult();
    cameraIntake.getLatestResult();

    if (result.hasTargets()) {
      SmartDashboard.putString("cameraShooter", "" + result.getTargets().get(0).getYaw());
    }
  }

  public void setLightShooter(boolean set) {
    lightShooter.set(set);
    System.out.println("setLightShooter");
  }

  public void setLightIntake(boolean set) {
    lightIntake.set(set);
    System.out.println("setLightIntake");

  }

  public void toggleLightShooter() {
    lightShooter.toggle();
    System.out.println("toggleLightShooter");
  }

  public void toggleLightIntake() {
    lightIntake.toggle();
    System.out.println("toggleLightIntake");

  }
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
