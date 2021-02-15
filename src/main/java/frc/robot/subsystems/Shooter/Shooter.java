package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

/**
 * Description:
 */
public class Shooter extends SubsystemBase {
    TalonSRX motor1 = new TalonSRX(Constants.ShooterConstants.Shooter_MOTOR_ID);

    public Shooter() {
    }

    public void setMototrSpeed(double input) {
        motor1.set(ControlMode.PercentOutput, input);
    }
}
