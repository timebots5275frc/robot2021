package frc.robot.subsystems.DriveTrain;

import com.revrobotics.*;
import com.ctre.phoenix.sensors.*;
import edu.wpi.first.wpilibj.controller.PIDController;

public class SwerveModule {

    private CANSparkMax m_driveMotor;
    private CANSparkMax m_steerMotor;
    private CANEncoder m_driveMotorEncoder;
    private CANEncoder m_steerMotorEncoder;

    private CANCoder m_steerEncoder;

    private PIDController m_pidController;

    public SwerveModule(int driveMotorID, int steerMotorID, int steerEncoderId) {

        m_driveMotor = new CANSparkMax(driveMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_steerMotor = new CANSparkMax(steerMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

        m_steerEncoder = new CANCoder(steerEncoderId);

        m_driveMotorEncoder = m_driveMotor.getEncoder();
        m_steerMotorEncoder = m_steerMotor.getEncoder();

        m_pidController = new PIDController(20, 10, 0);

        CANPIDController m_SparkMaxpidController = m_steerMotor.getPIDController();

        double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

        // PID coefficients
        kP = 6e-5;
        kI = 6e-7;
        kD = 0;
        kIz = 0;
        kFF = 0.000015;
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 5700 / 2;

        // set PID coefficients
        m_SparkMaxpidController.setP(kP);
        m_SparkMaxpidController.setI(kI);
        m_SparkMaxpidController.setD(kD);
        m_SparkMaxpidController.setIZone(kIz);
        m_SparkMaxpidController.setFF(kFF);
        m_SparkMaxpidController.setOutputRange(kMinOutput, kMaxOutput);

    }

    // double setVel = m_pidController.calculate(m_steerEncoder.getPosition(),
    // angle_IN);

    // double setPoint = m_stick.getY() * maxRPM;
    // m_pidController.setReference(setVel, ControlType.kVelocity);

}
