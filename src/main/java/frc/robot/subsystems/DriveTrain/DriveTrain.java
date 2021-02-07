package frc.robot.subsystems.driveTrain;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.*;
/**
 * Description:
 */
public class DriveTrain extends SubsystemBase {

    private final Translation2d m_leftFrontLocation = 
        new Translation2d( DriveConstants.LEFT_FRONT_WHEEL_X, DriveConstants.LEFT_FRONT_WHEEL_Y);
    private final Translation2d m_rightFrontLocation = 
        new Translation2d( DriveConstants.RIGHT_FRONT_WHEEL_X, DriveConstants.RIGHT_FRONT_WHEEL_Y);
    private final Translation2d m_rightRearLocation = 
        new Translation2d( DriveConstants.RIGHT_REAR_WHEEL_X, DriveConstants.RIGHT_REAR_WHEEL_Y);
    private final Translation2d m_leftRearLocation = 
        new Translation2d( DriveConstants.LEFT_REAR_WHEEL_X, DriveConstants.LEFT_REAR_WHEEL_Y);

    private final SwerveModule m_leftFrontSwerve = 
        new SwerveModule( DriveConstants.LEFT_FRONT_DRIVE_MOTOR_ID, DriveConstants.LEFT_FRONT_STEER_MOTOR_ID, DriveConstants.LEFT_FRONT_STEER_ENCODER_ID);
    private final SwerveModule m_rightFrontSwerve = 
        new SwerveModule(DriveConstants.RIGHT_FRONT_DRIVE_MOTOR_ID, DriveConstants.RIGHT_FRONT_STEER_MOTOR_ID, DriveConstants.RIGHT_FRONT_STEER_ENCODER_ID);
    private final SwerveModule m_rightRearSwerve = 
        new SwerveModule(DriveConstants.RIGHT_REAR_DRIVE_MOTOR_ID, DriveConstants.RIGHT_READ_STEER_MOTOR_ID, DriveConstants.RIGHT_REAR_STEER_ENCODER_ID);
    private final SwerveModule m_leftRearSwerve = 
        new SwerveModule(DriveConstants.LEFT_REAR_DRIVE_MOTOR_ID, DriveConstants.LEFT_REAR_STEER_MOTOR_ID, DriveConstants.LEFT_REAR_STEER_ENCODER_ID);

    public DriveTrain() {

    }

}
