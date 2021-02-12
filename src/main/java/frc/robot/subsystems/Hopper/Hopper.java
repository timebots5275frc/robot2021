package frc.robot.subsystems.Hopper;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;

/**
 * Description:
 */
public class Hopper extends SubsystemBase {
    TalonSRX motor1 = new TalonSRX(Constants.HopperConstants.HOPPER_MOTOR_ID);
    public Hopper (){

    }
}
