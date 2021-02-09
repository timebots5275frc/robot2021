package frc.robot.subsystems.driveTrain;

import com.revrobotics.CANSparkMax;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

import com.revrobotics.*;
import com.ctre.phoenix.sensors.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.constants.Constants.DriveConstants;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.constants.Constants.*;

public class SwerveModule {

    private CANSparkMax m_driveMotor;
    private CANSparkMax m_steerMotor;
    private CANEncoder m_driveMotorEncoder;
    private CANEncoder m_steerMotorEncoder;

    private CANCoder m_steerEncoder;

    private PIDController m_drivePIDController = new PIDController(1, 0, 0);
    private PIDController PID_Encoder_Steer;
    private CANPIDController PID_SparkMax_Steer;
    private CANPIDController PID_SparkMax_Drive;

    // Gains are for example purposes only - must be determined for your own

    //private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
    //private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

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
        PID_Encoder_Steer.enableContinuousInput(-180, 180);

        PID_SparkMax_Steer = m_steerMotor.getPIDController();
        PID_SparkMax_Drive = m_driveMotor.getPIDController();

        double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
        // maxRPM = 5700 / 2;

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
        // PID_SparkMax_Steer.setOutputRange(kMinOutput, kMaxOutput);

    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        double driveSpeedMetersPerSecond = m_driveMotorEncoder.getVelocity() * DriveConstants.DRIVE_GEAR_RATIO * 2.0 * Math.PI * DriveConstants.WHEEL_RADIUS / 60.0 ;
        double steerAngleRadians = Math.toRadians(m_steerEncoder.getAbsolutePosition()) ;

        return new SwerveModuleState(driveSpeedMetersPerSecond, new Rotation2d(steerAngleRadians) );
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        double curSteerAngleRadians = Math.toRadians(m_steerEncoder.getAbsolutePosition()) ;

        SwerveModuleState state = SwerveModuleState.optimize( desiredState, new Rotation2d(curSteerAngleRadians) );

        // Calculate the drive output from the drive PID controller.

        //final double driveOutput = m_drivePIDController.calculate(m_driveMotorEncoder.getVelocity(),
        //        state.speedMetersPerSecond);

        //final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        double turnOutput = PID_Encoder_Steer.calculate(m_steerEncoder.getAbsolutePosition(), state.angle.getDegrees());
        PID_SparkMax_Steer.setReference(turnOutput, ControlType.kVelocity);
        
        //final double turnFeedforward = m_turnFeedforward.calculate(PID_Encoder_Steer.getSetpoint());

        // m_driveMotor.setVoltage(driveOutput + driveFeedforward);
        // m_turningMotor.setVoltage(turnOutput + turnFeedforward);
        // m_steerMotor.set


        // PID_SparkMax_Drive.setReference(driveOutput + turnFeedforward,

    }

    // double setVel = m_pidController.calculate(m_steerEncoder.getPosition(),
    // angle_IN);

    // double setPoint = m_stick.getY() * maxRPM;
    // m_pidController.setReference(setVel, ControlType.kVelocity);

}
