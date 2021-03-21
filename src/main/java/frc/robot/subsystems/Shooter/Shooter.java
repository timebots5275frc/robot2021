package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import com.revrobotics.*;

/**
 * Description:
 */
public class Shooter extends SubsystemBase {
    // TalonSRX motor1 = new TalonSRX(Constants.ShooterConstants.Shooter_MOTOR_ID);
    private CANSparkMax shooterMotor = new CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless);

    public Shooter() {

    }

    public void setMototrSpeed(double input) {
        shooterMotor.set(input);
        // motor1.set(ControlMode.PercentOutput, input);
    }
}
