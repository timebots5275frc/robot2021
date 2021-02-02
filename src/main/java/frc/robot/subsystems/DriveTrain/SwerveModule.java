package frc.robot.subsystems.DriveTrain;


import com.revrobotics.* ;
import com.ctre.phoenix.sensors.*;

public class SwerveModule {

    private CANSparkMax m_driveMotor ;
    private CANSparkMax m_steerMotor ;
    private CANEncoder m_driveMotorEncoder ;
    private CANEncoder m_steerMotorEncoder ;

    private CANCoder m_steerEncoder ;


    public SwerveModule(int driveMotorID, int steerMotorID, int steerEncoderId ){
        
        m_driveMotor = new CANSparkMax(driveMotorID, CANSparkMaxLowLevel.MotorType.kBrushless) ;
        m_steerMotor = new CANSparkMax(steerMotorID, CANSparkMaxLowLevel.MotorType.kBrushless) ;

        m_steerEncoder = new CANCoder(steerEncoderId) ;

        m_driveMotorEncoder = m_driveMotor.getEncoder() ;
        m_steerMotorEncoder = m_steerMotor.getEncoder() ;

    }
    
}
