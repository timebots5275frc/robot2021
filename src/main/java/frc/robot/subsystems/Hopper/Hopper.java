package frc.robot.subsystems.hopper;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

/**
 * Description:
 */
public class Hopper extends SubsystemBase {
    TalonSRX motor1 = new TalonSRX(Constants.HopperConstants.HOPPER_MOTOR_ID);
    public Hopper (){

    }
    public void setMototrSpeed(double input){
motor1.set(ControlMode.PercentOutput, input);
    }
}
