/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.photonvision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineResult;
import org.photonvision.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Photonvision extends SubsystemBase {
  PhotonCamera cameraShooter = new PhotonCamera("MyCam");
  PhotonCamera cameraIntake = new PhotonCamera("IntakeWebcam");

  Solenoid lightShooter = new Solenoid(7);
  Solenoid lightIntake = new Solenoid(6);

  /**
   * Creates a new photonvision.
   */
  public Photonvision() {}

  public PhotonTrackedTarget getIntakeLatestResult() {
    PhotonPipelineResult result = cameraIntake.getLatestResult();
    cameraIntake.getLatestResult();

    if (result.hasTargets()) {
      SmartDashboard.putString("cameraIntake Yaw", "" + result.getTargets().get(0).getYaw());
      return result.getTargets().get(0);
    }
    return null;
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
