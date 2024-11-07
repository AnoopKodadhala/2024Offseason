package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType; 

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;
    private final RelativeEncoder  driveEncoder;
    private final RelativeEncoder  turningEncoder;
    private final CANcoder absoluteEncoder;

    private final PIDController turningPidController;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;
    private int encId;

    public SwerveModule(
        int driveMotorId, 
        int turningMotorId, 
        int absoluteEncoderId, 
        double absoluteEncoderOffset, 
        boolean absoluteEncoderReversed) {
            
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;


        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);
        absoluteEncoder = new CANcoder(absoluteEncoderId);

        driveMotor.setSmartCurrentLimit(40);
        turningMotor.setSmartCurrentLimit(30);

        driveMotor.burnFlash();
        turningMotor.burnFlash();
        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();


        encId = absoluteEncoderId;

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }
    
    public double getDrivePosition() { 
       return (driveEncoder.getPosition() * ModuleConstants.kDriveMotorGearRatio) * Math.PI * ModuleConstants.kWheelDiameterMeters;
    }

    public double getTurningPosition() {
        return (getAbsoluteEncoderRad());
    }

    public double getDriveVelocity() {
        return ((driveEncoder.getPosition() / ModuleConstants.kEncoderCPR) * ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getAbsolutePosition().getValue() * (2*Math.PI);        
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {

        SmartDashboard.putNumber("Swerve[" + encId + "] state", getAbsoluteEncoderRad());
        SmartDashboard.putNumber("degrees_Swerve[" + encId + "] state", Units.radiansToDegrees(getAbsoluteEncoderRad()));
        SmartDashboard.putNumber("Module[" + encId + "]", state.angle.getDegrees());
        SmartDashboard.putNumber("swerve_[" + encId + "] Velocity", driveMotor.getEncoder().getVelocity());
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);


                        
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));

    }

  


    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    public void brake(boolean doBrake){
        if(doBrake){
            driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        }
        else{
            driveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        }
        
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(), new Rotation2d(turningEncoder.getPosition()));
  }

}