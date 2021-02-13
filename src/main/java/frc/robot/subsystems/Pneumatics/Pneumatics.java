package frc.robot.subsystems.pneumatics;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import edu.wpi.first.wpilibj.Compressor;

public class Pneumatics extends SubsystemBase {
    // Creating Solenoid

    private Compressor compressor1 = new Compressor(Constants.IntakeConstants.COMPRESSOR);

    public Pneumatics() {
        boolean enabled = compressor1.enabled();
        boolean pressureSwitch = compressor1.getPressureSwitchValue();
        double current = compressor1.getCompressorCurrent();
    }

}
