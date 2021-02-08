package frc.robot.subsystems.driveTrain;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.*;

import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;

import com.revrobotics.*;
import com.analog.adis16470.frc.ADIS16470_IMU;
import com.ctre.phoenix.sensors.CANCoder;

/** Represents a swerve drive style drivetrain. */
public class DriveTrain extends SubsystemBase {
	public static final double kMaxSpeed = 3.0; // 3 meters per second
	public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

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


	// private final AnalogGyro m_gyro = new AnalogGyro(0);

	public static final ADIS16470_IMU imu = new ADIS16470_IMU();

	private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_leftFrontLocation,
			m_rightFrontLocation, m_rightRearLocation, m_leftRearLocation);

	private final SwerveDriveOdometry m_odometry = new
	SwerveDriveOdometry(m_kinematics, imu.getRotation2d());

	public DriveTrain() {
		System.out.println("DriveTrain (:");
	} 

	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed        Speed of the robot in the x direction (forward).
	 * @param ySpeed        Speed of the robot in the y direction (sideways).
	 * @param rot           Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the
	 *                      field.
	 */
	@SuppressWarnings("ParameterName")
	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
		// var swerveModuleStates = m_kinematics.toSwerveModuleStates(
		// fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
		// imu.getRotation2d())
		// : new ChassisSpeeds(xSpeed, ySpeed, rot));
		var swerveModuleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
		SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);
		LeftFrontSwerveModule.setDesiredState(swerveModuleStates[0]);
		RightFrontSwerveModule.setDesiredState(swerveModuleStates[1]);
		LeftRearSwerveModule.setDesiredState(swerveModuleStates[2]);
		RightRearSwerveModule.setDesiredState(swerveModuleStates[3]);
	}

	/** Updates the field relative position of the robot. */
	public void updateOdometry() {
		m_odometry.update(imu.getRotation2d(), LeftFrontSwerveModule.getState(), RightFrontSwerveModule.getState(),
		LeftRearSwerveModule.getState(), RightRearSwerveModule.getState());
	}
}

/**
 * Description:
 */
// public class DriveTrain extends SubsystemBase {


// private final Translation2d m_leftFrontLocation = new
// Translation2d(DriveConstants.kLeftFrontWheel_X,
// DriveConstants.kLeftFrontWheel_X);
// private final Translation2d m_rightFrontLocation = new
// Translation2d(DriveConstants.kRightFrontWheel_X,
// DriveConstants.kRightFrontWheel_X);
// private final Translation2d m_rightRearLocation = new
// Translation2d(DriveConstants.kRightRearWheel_X,
// DriveConstants.kRightRearWheel_X);
// private final Translation2d m_leftRearLocation = new
// Translation2d(DriveConstants.kLeftRearWheel_X,
// DriveConstants.kLeftRearWheel_X);

// // Creating my kinematics object using the module locations
// //
// https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
// private SwerveDriveKinematics m_kinematics = new
// SwerveDriveKinematics(m_leftFrontLocation, m_rightFrontLocation,
// m_rightRearLocation, m_leftRearLocation);

// private final SwerveModule m_leftFrontSwerve = new
// SwerveModule(DriveConstants.kLeftFrontDriveMotorID,
// DriveConstants.kLeftFrontSteerMotorID,
// DriveConstants.kLeftFrontSteerEncoderID);

// private final SwerveModule m_rightFrontSwerve = new
// SwerveModule(DriveConstants.kRightFrontDriveMotorID,
// DriveConstants.kRightFrontSteerMotorID,
// DriveConstants.kRightFrontSteerEncoderID);

// private final SwerveModule m_rightRearSwerve = new
// SwerveModule(DriveConstants.kRightRearDriveMotorID,
// DriveConstants.kRightRearSteerMotorID,
// DriveConstants.kRightRearSteerEncoderID);
>>>>>>> Stashed changes

// private final SwerveModule m_leftRearSwerve = new
// SwerveModule(DriveConstants.kLeftRearDriveMotorID,
// DriveConstants.kLeftRearSteerMotorID,
// DriveConstants.kLeftRearSteerEncoderID);

// // private final Gyroscope gyroscope = new NavX(SPI.Port.kMXP);

// public DriveTrain() {
// // m_leftFrontSwerve.\

// // gyroscope.
// }

// }
