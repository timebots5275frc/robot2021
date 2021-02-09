package frc.robot.subsystems.Intake;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * Description:
 */
public class Intake extends SubsystemBase {
    Solenoid solenoid1 = new Solenoid(1);
    // IMPORTANT
    int Bob = 1;

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
