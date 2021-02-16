package frc.robot.subsystems.driveTrain;

// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.*;
import com.ctre.phoenix.sensors.*;
import frc.robot.constants.Constants.DriveConstants;

public class SwerveModule {

    private CANSparkMax driveMotor;
    private CANSparkMax steerMotor;
    private CANEncoder driveMotorEncoder;

    private CANCoder steerAngleEncoder;

    private PIDController steerAnglePID;
    private CANPIDController steerMotorVelocityPID;
    private CANPIDController driveMotorVelocityPID;

    public SwerveModule(int driveMotorID, int steerMotorID, int steerEncoderId) {

        driveMotor = new CANSparkMax(driveMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
        steerMotor = new CANSparkMax(steerMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

        steerAngleEncoder = new CANCoder(steerEncoderId);

        driveMotorEncoder = driveMotor.getEncoder();

        /// PID Controllers ///

        steerAnglePID = new PIDController(DriveConstants.PID_Encoder_Steer.P, DriveConstants.PID_Encoder_Steer.I,
                DriveConstants.PID_Encoder_Steer.D);
        steerAnglePID.enableContinuousInput(-180, 180);

        // Get the motor controller PIDs
        steerMotorVelocityPID = steerMotor.getPIDController();
        driveMotorVelocityPID = driveMotor.getPIDController();

        // set PID coefficients
        steerMotorVelocityPID.setP(DriveConstants.PID_SparkMax_Steer.P);
        steerMotorVelocityPID.setI(DriveConstants.PID_SparkMax_Steer.I);
        steerMotorVelocityPID.setD(DriveConstants.PID_SparkMax_Steer.D);
        steerMotorVelocityPID.setIZone(DriveConstants.PID_SparkMax_Steer.Iz);
        steerMotorVelocityPID.setFF(DriveConstants.PID_SparkMax_Steer.kFF);
        steerMotorVelocityPID.setOutputRange(-1, 1);
        // set PID coefficients
        driveMotorVelocityPID.setP(DriveConstants.PID_SparkMax_Drive.P);
        driveMotorVelocityPID.setI(DriveConstants.PID_SparkMax_Drive.I);
        driveMotorVelocityPID.setD(DriveConstants.PID_SparkMax_Drive.D);
        driveMotorVelocityPID.setIZone(DriveConstants.PID_SparkMax_Drive.Iz);
        driveMotorVelocityPID.setFF(DriveConstants.PID_SparkMax_Drive.kFF);
        driveMotorVelocityPID.setOutputRange(-1, 1);

    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        double driveSpeed = speedFromDriveRpm(driveMotorEncoder.getVelocity());
        double steerAngleRadians = Math.toRadians(steerAngleEncoder.getAbsolutePosition());

        return new SwerveModuleState(driveSpeed, new Rotation2d(steerAngleRadians));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean logit) {

        double curSteerAngleRadians = Math.toRadians(steerAngleEncoder.getAbsolutePosition());

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(curSteerAngleRadians));

        // The output of the steerAnglePID becomes the steer motor rpm reference.
        double steerMotorRpm = steerAnglePID.calculate(steerAngleEncoder.getAbsolutePosition(),
                state.angle.getDegrees());

        if (logit) {
            SmartDashboard.putNumber("SteerMotorRpmCommand", steerMotorRpm);
        }

        steerMotorVelocityPID.setReference(steerMotorRpm, ControlType.kVelocity);

        double driveMotorRpm = driveRpmFromSpeed(state.speedMetersPerSecond);

        if (logit) {
            double driveSpeed = driveMotorEncoder.getVelocity();
            SmartDashboard.putNumber("DriveSpeedMetersPerSecond", state.speedMetersPerSecond);
            SmartDashboard.putNumber("DriveMotorRpmCommand", driveMotorRpm);
            SmartDashboard.putNumber("DriveMotorSpeed", driveSpeed);

        }

        driveMotorVelocityPID.setReference(driveMotorRpm, ControlType.kVelocity);
    }

    /**
     * Returns the required motor rpm from the desired wheel speed in meters/second
     * 
     * @param speedMetersPerSecond
     * @return rpm of the motor
     */
    public double driveRpmFromSpeed(double speedMetersPerSecond) {
        var rpm = speedMetersPerSecond * 60.0 / DriveConstants.WHEEL_CIRCUMFERENCE / DriveConstants.DRIVE_GEAR_RATIO;
        return rpm;
    }

    /**
     * Returns the wheel speed in meters/second calculated from the drive motor rpm.
     * 
     * @param rpm
     * @return wheelSpeed
     */
    public double speedFromDriveRpm(double rpm) {
        var speedMetersPerSecond = rpm * DriveConstants.DRIVE_GEAR_RATIO * DriveConstants.WHEEL_CIRCUMFERENCE / 60.0;
        return speedMetersPerSecond;
    }

}
