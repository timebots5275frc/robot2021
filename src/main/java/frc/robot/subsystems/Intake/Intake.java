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
    
    public DoubleSolenoid doubleSolenoid1 = new DoubleSolenoid(0, 1);
    // private DoubleSolenoid doubleSolenoid2 = new DoubleSolenoid(3, 4);


    // IMPORTANT
    private TalonSRX intakeMotor1 = new TalonSRX(Constants.IntakeConstants.MOTOR_CAN_ID);

    public Intake() {
        
    }
    
    //Setting Talon Speed
    public void setMotorSpeed(double input) {
        intakeMotor1.set(ControlMode.PercentOutput, input);
    }

    public void setSolenoidToggle() {
        doubleSolenoid1.toggle();
        System.out.println("setSolenoidToggle");

        // doubleSolenoid1.set(kForward);
        // doubleSolenoid2.set(kForward);
    }

    public void setSolenoidExtend() {
        System.out.println("setSolenoidExtend");

        doubleSolenoid1.set(kForward);
        // doubleSolenoid2.set(kForward);
    }

    public void setSolenoidRetract() {
        System.out.println("setSolenoidRetract");
        doubleSolenoid1.set(kReverse);
        // doubleSolenoid2.set(kReverse);
    }

    public void setSolenoidNoAirPressure() {
        // doubleSolenoid1.set(kReverse); // NoAirPressure
        doubleSolenoid1.set(kOff); // NoAirPressure
    }

    
}
