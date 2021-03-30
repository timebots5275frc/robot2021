package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;


/**
 * Description:
 */
public class Intake extends SubsystemBase {

    // exampleDouble.set(kOff);
    // exampleDouble.set(kForward);
    // exampleDouble.set(kReverse);
    
    // private DoubleSolenoid doubleSolenoid1 = new DoubleSolenoid(1, 2);
    // private DoubleSolenoid doubleSolenoid2 = new DoubleSolenoid(3, 4);


    // IMPORTANT
    private TalonSRX intakeMotor1 = new TalonSRX(Constants.IntakeConstants.MOTOR_CAN_ID);

    public Intake() {
        
    }
    
    //Setting Talon Speed
    public void setMotorSpeed(double input) {
        intakeMotor1.set(ControlMode.PercentOutput, input);
    }

    // public void setSolenoidExtend() {
    //     doubleSolenoid1.set(kForward);
    //     doubleSolenoid2.set(kForward);
    // }

    // public void setSolenoidRetract() {
    //     doubleSolenoid1.set(kForward);
    //     doubleSolenoid2.set(kReverse);
    // }

    // public void setSolenoidNoAirPressure() {
    //     doubleSolenoid1.set(kReverse); // NoAirPressure
    //     doubleSolenoid2.set(kOff); // NoAirPressure
    // }

    
}
