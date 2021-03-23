// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {

    public static final class DriveConstants {

        // Drivetrain Motor IDs
        // There are the CANBus IDs of the SparkMax controllers
        public static final int kLeftFrontDriveMotorID = 1 ;
        public static final int kLeftFrontSteerMotorID = 2 ;
        public static final int kRightFrontDriveMotorID = 3 ;
        public static final int kRightFrontSteerMotorID = 4 ;
        public static final int kRightRearDriveMotorID = 5 ;
        public static final int kRightRearSteerMotorID = 6 ;
        public static final int kLeftRearDriveMotorID = 7 ;
        public static final int kLeftRearSteerMotorID = 8 ;

        // Drivetrain Encoder IDs
        // These are the CANBus IDs of the CTRE CANCoders
        public static final int kLeftFrontSteerEncoderID = 10 ;
        public static final int kRightFrontSteerEncoderID = 11 ;
        public static final int kRightRearSteerEncoderID = 12 ;
        public static final int kLeftRearSteerEncoderID = 13 ;

        // These constants define the location of the wheels from the center of the robot.
        // These coordinates are determined by the right hand rule.
        // Index finger points in the forward X direction, Thumb points up in the positive Z direction,
        // Middle finger points left in the positive Y direction.
        public static final double kLeftFrontWheel_X = 11.75 * 0.0254; // meters
        public static final double kLeftFrontWheel_Y = 11.75 * 0.0254; // meters
        public static final double kRightFrontWheel_X = 11.75 * 0.0254; // meters
        public static final double kRightFrontWheel_Y = -11.75 * 0.0254; // meters
        public static final double kRightRearWheel_X = -11.75 * 0.0254; // meters
        public static final double kRightRearWheel_Y = - 11.75 * 0.0254; // meters
        public static final double kLeftRearWheel_X = -11.75 * 0.0254; // meters
        public static final double kLeftRearWheel_Y = 11.75 * 0.0254; // meters

        public static final double kWheelRadius = 2.0 * 0.0254; // meters

        // Drive motor gear ratio.
        //              | Driving Gear | Driven Gear |
        // First Stage  |     14       |     50      |
        // Second Stage |     28       |     16      |
        // Third Stage  |     15       |     60      |
        //
        // Overall Gear Ratio = 0.1225 
        // One rotation of the motor gives 0.1225 rotations of the wheel. 
        public static final double kDriveGearRatio = (14.0 / 50.0) * (28.0 / 16.0 ) * (15.0 / 60.0 ) ; 

    }
    public static final class ShooterConstants {

        public static final int SHOOTER_MOTOR_ID = 52;
        
        public static final double SHOOTER_FIRE_RPM = 1000;
        public static final double SHOOTER_MAX_RPM = 5700;
        public static final double SHOOTER_DEFAULT_RPM = 0.0; 

    }

}
