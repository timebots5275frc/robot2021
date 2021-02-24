package frc.robot.subsystems.driveTrain;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.*;

import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.sql.Time;

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

	public final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, imu.getRotation2d());
	public Pose2d m_pose;

	public long oldTime = System.currentTimeMillis();
	public double velX = 0;
	public double velY = 0;

	public double posX = 0;
	public double posY = 0;

	public DriveTrain() {
		System.out.println("DriveTrain (:");
		this.m_odometry.resetPosition(new Pose2d(), new Rotation2d(0));

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

		long newTime = System.currentTimeMillis();

		long deltaTime = newTime - this.oldTime; // deltaTime in ms
		deltaTime = (deltaTime / 1000);
		this.oldTime = newTime;

		this.velX = velX + imu.getAccelInstantX() * deltaTime; 
		this.velY = velY + (imu.getAccelInstantY() + 0.05) * deltaTime;

		this.posX = posX + velX * deltaTime;
		this.posY = posY + velY * deltaTime;
		SmartDashboard.putNumber("IMU Angle", imu.getRotation2d().getDegrees());
		SmartDashboard.putNumber("IMU Acc X", imu.getAccelInstantX());
		SmartDashboard.putNumber("IMU Acc Y", imu.getAccelInstantY());
		SmartDashboard.putNumber("IMU Acc Z", imu.getAccelInstantZ());
		SmartDashboard.putNumber("deltaTime", deltaTime);
		;
		SmartDashboard.putNumber("IMU Vel X", velX);
		SmartDashboard.putNumber("IMU Vel Y", velY);
		SmartDashboard.putNumber("IMU Pos X", posX);
		SmartDashboard.putNumber("IMU Pos Y", posY);

		SmartDashboard.putNumber("getX", m_odometry.getPoseMeters().getX());
		SmartDashboard.putNumber("getY", m_odometry.getPoseMeters().getY());
		SmartDashboard.putString("getRotation", m_odometry.getPoseMeters().getRotation().toString());

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

		// this.updateOdometry();

	}

	/** Updates the field relative position of the robot. */
	public void updateOdometry() {
		m_odometry.update(imu.getRotation2d(), leftFrontSwerveModule.getState(), rightFrontSwerveModule.getState(),
				rightRearSwerveModule.getState(), rightRearSwerveModule.getState());

	}

	@Override
	public void periodic() {
		// Get my gyro angle. We are negating the value because gyros return positive
		// values as the robot turns clockwise. This is not standard convention that is
		// used by the WPILib classes.
		var gyroAngle = Rotation2d.fromDegrees(-imu.getAngle());

		// Update the pose
		m_pose = m_odometry.update(gyroAngle, leftFrontSwerveModule.getState(), rightFrontSwerveModule.getState(),
				rightRearSwerveModule.getState(), rightRearSwerveModule.getState());
	}

	/** Zeroes the heading of the robot. */
	public void zeroHeading() {
		imu.reset();
	}

	/**
	 * Returns the heading of the robot.
	 *
	 * @return the robot's heading in degrees, from -180 to 180
	 */
	public double getHeading() {
		return imu.getRotation2d().getDegrees();
	}
}