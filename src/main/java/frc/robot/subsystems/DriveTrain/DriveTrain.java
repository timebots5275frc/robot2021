package frc.robot.subsystems;
import frc.robot.commands.*;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.geometry.Translation2d ;
import com.revrobotics.*;
import com.ctre.phoenix.sensors.CANCoder;
import frc.robot.Constants.DriveConstants;


public class DriveTrain extends SubsystemBase {

    private final Translation2d m_leftFrontLocation = new Translation2d(DriveConstants.kLeftFrontWheel_X, DriveConstants.kLeftFrontWheel_X);
    private final Translation2d m_rightFrontLocation = new Translation2d(DriveConstants.kRightFrontWheel_X, DriveConstants.kRightFrontWheel_X);
    private final Translation2d m_rightRearLocation = new Translation2d(DriveConstants.kRightRearWheel_X, DriveConstants.kRightRearWheel_X);
    private final Translation2d m_leftRearLocation = new Translation2d(DriveConstants.kLeftRearWheel_X, DriveConstants.kLeftRearWheel_X);
  
    private final SwerveModule m_leftFrontSwerve = 
        new SwerveModule(DriveConstants.kLeftFrontDriveMotorID, DriveConstants.kLeftFrontSteerMotorID, DriveConstants.kLeftFrontSteerEncoderID);

    private final SwerveModule m_rightFrontSwerve = 
        new SwerveModule(DriveConstants.kRightFrontDriveMotorID, DriveConstants.kRightFrontSteerMotorID, DriveConstants.kRightFrontSteerEncoderID);

    private final SwerveModule m_rightRearSwerve = 
        new SwerveModule(DriveConstants.kRightRearDriveMotorID, DriveConstants.kRightRearSteerMotorID, DriveConstants.kRightRearSteerEncoderID);

    private final SwerveModule m_leftRearSwerve = 
        new SwerveModule(DriveConstants.kLeftRearDriveMotorID, DriveConstants.kLeftRearSteerMotorID, DriveConstants.kLeftRearSteerEncoderID);



    public DriveTrain(){



    }


}
