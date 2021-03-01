package frc.robot.subsystems.driveTrain;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.*;

import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
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
			DriveConstants.RIGHT_REAR_STEER_MOTOR_ID, DriveConstants.RIGHT_REAR_STEER_ENCODER_ID);
	private final SwerveModule leftRearSwerveModule = new SwerveModule(DriveConstants.LEFT_REAR_DRIVE_MOTOR_ID,
			DriveConstants.LEFT_REAR_STEER_MOTOR_ID, DriveConstants.LEFT_REAR_STEER_ENCODER_ID);

	// private final AnalogGyro m_gyro = new AnalogGyro(0);

	private final ADIS16470_IMU imuADIS16470 = new ADIS16470_IMU();

	public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(leftFrontWheelLoc, rightFrontWheelLoc,
			rightRearWheelLoc, leftRearWheelLoc);

	private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(kinematics, this.getHeading() );
	private Pose2d m_pose;

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

		var swerveModuleStates = kinematics.toSwerveModuleStates(fieldRelative
				? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, this.getHeading() )
				: new ChassisSpeeds(xSpeed, ySpeed, rot));

		// SmartDashboard.putNumber("odometry getX", m_odometry.getPoseMeters().getX());
		// SmartDashboard.putNumber("odometry getY", m_odometry.getPoseMeters().getY());
		// SmartDashboard.putString("odometry getRotation", m_odometry.getPoseMeters().getRotation().toString());

		// SmartDashboard.putNumber("LeftFrontSpeed",
		// swerveModuleStates[0].speedMetersPerSecond );
		// SmartDashboard.putNumber("LeftFrontAngle",
		// swerveModuleStates[0].angle.getDegrees() );

		SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, DriveConstants.MAX_DRIVE_SPEED);

		// SmartDashboard.putNumber("LeftFrontSpeedNorm",
		// swerveModuleStates[0].speedMetersPerSecond );

		leftFrontSwerveModule.setDesiredState(swerveModuleStates[0], true);
		rightFrontSwerveModule.setDesiredState(swerveModuleStates[1], false);
		rightRearSwerveModule.setDesiredState(swerveModuleStates[2], false);
		leftRearSwerveModule.setDesiredState(swerveModuleStates[3], false);
	}

	@Override
	public void periodic() {
		this.updateOdometry();
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	/** Updates the field relative position of the robot. */
	public void updateOdometry() {
		m_odometry.update(this.getHeading(), leftFrontSwerveModule.getState(),
				rightFrontSwerveModule.getState(), rightRearSwerveModule.getState(), leftRearSwerveModule.getState());

	}

	/**
	 * Resets the odometry Position and Angle to 0.
	 */
	public void resetOdometry() {
		m_odometry.resetPosition(new Pose2d(), new Rotation2d(0));
	}

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometryWithPose2d(Pose2d pose) {
		m_odometry.resetPosition(pose, imuADIS16470.getRotation2d());
	}

	/** Zeroes the heading of the robot. */
	public void resetADIS16470() {
		imuADIS16470.reset();
	}

	/**
	 * Returns the heading of the robot.
	 *
	 * @return the robot's heading in degrees, from -180 to 180
	 */
	public Rotation2d getHeading() {
		return imuADIS16470.getRotation2d().times(-1);
	}

	/**
	 * Sets the swerve ModuleStates.
	 *
	 * @param desiredStates The desired SwerveModule states.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, DriveConstants.MAX_DRIVE_SPEED);

		leftFrontSwerveModule.setDesiredState(desiredStates[0], false);
		rightFrontSwerveModule.setDesiredState(desiredStates[1], false);
		rightRearSwerveModule.setDesiredState(desiredStates[2], false);
		leftRearSwerveModule.setDesiredState(desiredStates[3], false);
	}
}