package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.*;

/**
 * Description:
 */
public class Shooter extends SubsystemBase {
    public CANSparkMax shooterMotor;
    private CANPIDController m_pidController;
    private CANEncoder m_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    private TalonSRX hoodMotor;
    // private ProfiledPIDController hoodMotorPID;
    private CANCoder hoodCanCoder;


    public Shooter() {
        // hoodMotor = new TalonSRX(Constants.ShooterConstants.SHOOTER_MOTOR_HOOD_ID);

        // hoodMotorPID = new ProfiledPIDController(kD, kD, kD, null, kD);
        
        // hoodCanCoder = new CANCoder(Constants.ShooterConstants.SHOOTER_HOOD_CODER_ID);
        // hoodMotor.configSelectedFeedbackSensor(hoodCanCoder);
        // hoodMotor.configSelectedFeedbackSensor(feedbackDevice)
        
         
        // initialize motor
        shooterMotor = new CANSparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_ID,
                CANSparkMaxLowLevel.MotorType.kBrushless);

        shooterMotor.setClosedLoopRampRate(1);

        /**
         * In order to use PID functionality for a controller, a CANPIDController object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        m_pidController = shooterMotor.getPIDController();

        // Encoder object created to display position values
        m_encoder = shooterMotor.getEncoder();

        // PID coefficients
        kP = 0.0004;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0.00017; // .000015
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
        // SmartDashboard.putNumber("P Gain", kP);
        // SmartDashboard.putNumber("I Gain", kI);
        // SmartDashboard.putNumber("D Gain", kD);
        // SmartDashboard.putNumber("I Zone", kIz);
        // SmartDashboard.putNumber("Feed Forward", kFF);
        // SmartDashboard.putNumber("Max Output", kMaxOutput);
        // SmartDashboard.putNumber("Min Output", kMinOutput);
    }

    public void setShooterMotorSpeed(double input) {
        if (input < Constants.ShooterConstants.SHOOTER_MAX_RPM) {
            m_pidController.setReference(-input, ControlType.kVelocity);
        } else {
            System.out.println("Shooter Motor Warning - Cannot Set Motor RPM Over Limit Of "
                    + Constants.ShooterConstants.SHOOTER_MAX_RPM);
        }
    }

    // private void setHoodMotorSpeed(double input) {
    //     hoodMotor.set(ControlMode.PercentOutput, input);
    // }

    // public void setHoodMotorAngle(double angle) {
    //     double speed = hoodMotorPID.calculate(hoodCanCoder.getAbsolutePosition(), angle);
    //     this.setHoodMotorSpeed(speed);
    // }


}
