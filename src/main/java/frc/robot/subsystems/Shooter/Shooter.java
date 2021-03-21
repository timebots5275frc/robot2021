package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import com.revrobotics.*;

/**
 * Description:
 */
public class Shooter extends SubsystemBase {
    private CANSparkMax shooterMotor;
    private CANPIDController m_pidController;
    private CANEncoder m_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    public Shooter() {

        // initialize motor
        shooterMotor = new CANSparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_ID,
                CANSparkMaxLowLevel.MotorType.kBrushless);

        /**
         * In order to use PID functionality for a controller, a CANPIDController object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        m_pidController = shooterMotor.getPIDController();

        // Encoder object created to display position values
        m_encoder = shooterMotor.getEncoder();

        // PID coefficients
        kP = 6e-5;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0.000015;
        kMaxOutput = 1;
        kMinOutput = -1;

        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);
    }

    public void setMototrSpeed(double input) {
        if (input < Constants.ShooterConstants.SHOOTER_MAX_RPM) {
            m_pidController.setReference(input, ControlType.kVelocity);
        } else {
            System.out.println("Shooter Motor Warning - Cannot Set Motor RPM Over Limit Of " + Constants.ShooterConstants.SHOOTER_MAX_RPM);
        }
    }
}
