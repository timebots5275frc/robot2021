package frc.robot.subsystems.DriveTrain;

import com.revrobotics.*;
import com.ctre.phoenix.sensors.*;
import edu.wpi.first.wpilibj.controller.PIDController;

import frc.robot.Constants.Constants.DriveConstants;


public class SwerveModule {

    private CANSparkMax m_driveMotor;
    private CANSparkMax m_steerMotor;
    private CANEncoder m_driveMotorEncoder;
    private CANEncoder m_steerMotorEncoder;

    private CANCoder m_steerEncoder;

    private PIDController PID_Encoder_Steer; 
    private CANPIDController PID_SparkMax_Steer; 

    public SwerveModule(int driveMotorID, int steerMotorID, int steerEncoderId) {

        m_driveMotor = new CANSparkMax(driveMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_steerMotor = new CANSparkMax(steerMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

        m_steerEncoder = new CANCoder(steerEncoderId);

        m_driveMotorEncoder = m_driveMotor.getEncoder();
        m_steerMotorEncoder = m_steerMotor.getEncoder();

        /// PID Controllers ///

        double m_kP, m_kI, m_kD;

        m_kP = DriveConstants.PID_Encoder_Steer.P;
        m_kI = DriveConstants.PID_Encoder_Steer.I;
        m_kD = DriveConstants.PID_Encoder_Steer.D;

        PID_Encoder_Steer = new PIDController(m_kP, m_kI, m_kD);


        PID_SparkMax_Steer = m_steerMotor.getPIDController();

        double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
        maxRPM = 5700 / 2;

        // PID coefficients
        // kP = ; // 6e-5;
        // kI = ; // 6e-7;
        // kD = ; // 0
        kIz = 0;
        kFF = 0.000015;
        kMaxOutput = 1;
        kMinOutput = -1;

        // set PID coefficients
        PID_SparkMax_Steer.setP(DriveConstants.PID_SparkMax_Steer.P);
        PID_SparkMax_Steer.setI(DriveConstants.PID_SparkMax_Steer.I);
        PID_SparkMax_Steer.setD(DriveConstants.PID_SparkMax_Steer.D);
        PID_SparkMax_Steer.setIZone(kIz);
        PID_SparkMax_Steer.setFF(kFF);
        PID_SparkMax_Steer.setOutputRange(kMinOutput, kMaxOutput);

    }

    // double setVel = m_pidController.calculate(m_steerEncoder.getPosition(),
    // angle_IN);

    // double setPoint = m_stick.getY() * maxRPM;
    // m_pidController.setReference(setVel, ControlType.kVelocity);

}
