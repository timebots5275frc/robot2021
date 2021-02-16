package frc.robot.subsystems.driveTrain;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.*;

import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.analog.adis16470.frc.ADIS16470_IMU;

/** Represents a swerve drive style drivetrain. */
public class DriveTrain extends SubsystemBase {

	private final Translation2d leftFrontWheelLoc = new Translation2d(DriveConstants.LEFT_FRONT_WHEEL_X,
			DriveConstants.LEFT_FRONT_WHEEL_Y);
	private final Translation2d rightFrontWheelLoc = new Translation2d(DriveConstants.RIGHT_FRONT_WHEEL_X,
			DriveConstants.RIGHT_FRONT_WHEEL_Y);
	private final Translation2d rightRearWheelLoc = new Translation2d(DriveConstants.RIGHT_REAR_WHEEL_X,
			DriveConstants.RIGHT_REAR_WHEEL_Y);
	private final Translation2d leftRearWheelLoc = new Translation2d(DriveConstants.LEFT_REAR_WHEEL_X,
			DriveConstants.LEFT_REAR_WHEEL_Y);

	private final SwerveModule leftFrontSwerveModule = new SwerveModule(DriveConstants.LEFT_FRONT_DRIVE_MOTOR_ID,
			DriveConstants.LEFT_FRONT_STEER_MOTOR_ID, DriveConstants.LEFT_FRONT_STEER_ENCODER_ID);
	private final SwerveModule rightFrontSwerveModule = new SwerveModule(DriveConstants.RIGHT_FRONT_DRIVE_MOTOR_ID,
			DriveConstants.RIGHT_FRONT_STEER_MOTOR_ID, DriveConstants.RIGHT_FRONT_STEER_ENCODER_ID);
	private final SwerveModule rightRearSwerveModule = new SwerveModule(DriveConstants.RIGHT_REAR_DRIVE_MOTOR_ID,
			DriveConstants.RIGHT_READ_STEER_MOTOR_ID, DriveConstants.RIGHT_REAR_STEER_ENCODER_ID);
	private final SwerveModule leftRearSwerveModule = new SwerveModule(DriveConstants.LEFT_REAR_DRIVE_MOTOR_ID,
			DriveConstants.LEFT_REAR_STEER_MOTOR_ID, DriveConstants.LEFT_REAR_STEER_ENCODER_ID);

	// private final AnalogGyro m_gyro = new AnalogGyro(0);

	public static final ADIS16470_IMU imu = new ADIS16470_IMU();

	private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(leftFrontWheelLoc, rightFrontWheelLoc,
			rightRearWheelLoc, leftRearWheelLoc);

	private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, imu.getRotation2d());

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

		var swerveModuleStates = m_kinematics.toSwerveModuleStates(fieldRelative
				? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, imu.getRotation2d().times(-1))
				: new ChassisSpeeds(xSpeed, ySpeed, rot));

		SmartDashboard.putNumber("IMU Angle", imu.getRotation2d().getDegrees());
		// SmartDashboard.putNumber("LeftFrontSpeed",
		// swerveModuleStates[0].speedMetersPerSecond );
		// SmartDashboard.putNumber("LeftFrontAngle",
		// swerveModuleStates[0].angle.getDegrees() );

		SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, DriveConstants.MAX_DRIVE_SPEED);

		// SmartDashboard.putNumber("LeftFrontSpeedNorm",
		// swerveModuleStates[0].speedMetersPerSecond );

		leftFrontSwerveModule.setDesiredState(swerveModuleStates[0], false);
		rightFrontSwerveModule.setDesiredState(swerveModuleStates[1], false);
		rightRearSwerveModule.setDesiredState(swerveModuleStates[2], false);
		leftRearSwerveModule.setDesiredState(swerveModuleStates[3], false);

	}

	/** Updates the field relative position of the robot. */
	public void updateOdometry() {
		m_odometry.update(imu.getRotation2d(), leftFrontSwerveModule.getState(), rightFrontSwerveModule.getState(),
				rightRearSwerveModule.getState(), leftRearSwerveModule.getState());
	}
}