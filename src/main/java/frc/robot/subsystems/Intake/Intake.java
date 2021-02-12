package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import edu.wpi.first.wpilibj.Solenoid;


/**
 * Description:
 */
public class Intake extends SubsystemBase {
    Solenoid solenoid1 = new Solenoid(1);
    // IMPORTANT
    TalonSRX intakeMotor1 = new TalonSRX(Constants.IntakeConstants.motorCanId);
    
    //Setting Talon Speed
    public void setMotorSpeed(double input) {
        intakeMotor1.set(ControlMode.PercentOutput, input);
    }
    // Setting solenoid to true
    public void setSolenoidTrue() {
        solenoid1.set(true);
    }

    // Setting solenoid to false
    public void setSolenoidFalse() {
        solenoid1.set(false);
    }

    // Class toggleSolinoid will toggle it
    public void toggleSolinoid() {
        solenoid1.toggle();

    }
    
}
