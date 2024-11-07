package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class NewSwerveJoystick extends Command {

  private final SwerveSubsystem swerveSubsystem;

  private final Supplier<Double> xSpdFunctionField, ySpdFunctionField, turningSpdJoystick;
  private final Supplier<Boolean> lockSourceAngle;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  Translation2d linearVelocity; // if it doesnt work check this
  double thetaSpeed = 0;
  double rotationRate = 0;
  private PIDController autoRotateController = new PIDController(0.04, 0, 0.00);
    



  public NewSwerveJoystick(SwerveSubsystem swerveSubsystem, 
    Supplier<Double> xSpdFunctionField, 
    Supplier<Double> ySpdFunctionField, 

    Supplier<Double> turningSpdJoystick,

    Supplier<Boolean> lockSourceAngle) 
    {
      this.swerveSubsystem = swerveSubsystem;
      this.xSpdFunctionField = xSpdFunctionField;
      this.ySpdFunctionField = ySpdFunctionField;
      this.turningSpdJoystick = turningSpdJoystick;
      this.lockSourceAngle = lockSourceAngle;


      this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
      this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
      this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
      autoRotateController.enableContinuousInput(0, 360);
      addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {

      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        autoRotateController.setSetpoint(45);
      }
      else {
        autoRotateController.setSetpoint(360-45);
      }      
      
    }



    @Override
    public void execute() {
      
      // turning
      if (lockSourceAngle.get()) { //turn to source
        thetaSpeed = autoRotateController.calculate(swerveSubsystem.getHeading());    
        thetaSpeed = MathUtil.applyDeadband(thetaSpeed, 0.1);
        thetaSpeed = Math.copySign(thetaSpeed * thetaSpeed, thetaSpeed);
        thetaSpeed = turningLimiter.calculate(thetaSpeed);
        thetaSpeed = Math.abs(thetaSpeed) > SwerveConstants.kDeadband ? thetaSpeed : 0.0;
        thetaSpeed = Math.copySign(thetaSpeed * thetaSpeed, thetaSpeed);
      }
      else { // turning generic
        double thetaSpeed = turningSpdJoystick.get();
        thetaSpeed = (1 / (1 - SwerveConstants.kDeadband)) * (thetaSpeed + ( -Math.signum(thetaSpeed) * SwerveConstants.kDeadband));
        thetaSpeed = turningLimiter.calculate(thetaSpeed);
        thetaSpeed = Math.abs(thetaSpeed) > SwerveConstants.kDeadband ? thetaSpeed : 0.0;
        thetaSpeed = Math.copySign(thetaSpeed * thetaSpeed, thetaSpeed);
      }

      double xSpeed = xSpdFunctionField.get() / 2; 
      xSpeed = (1 / (1 - SwerveConstants.kDeadband)) * (xSpeed + ( -Math.signum(xSpeed) * SwerveConstants.kDeadband));
      double ySpeed = ySpdFunctionField.get() / 2;
      ySpeed = (1 / (1 - SwerveConstants.kDeadband)) * (ySpeed + ( -Math.signum(ySpeed) * SwerveConstants.kDeadband));

      xSpeed = xLimiter.calculate(xSpeed);
      ySpeed = yLimiter.calculate(ySpeed);
      
      double linearMagnitude = Math.pow(MathUtil.applyDeadband(Math.hypot(xSpeed, ySpeed), SwerveConstants.kDeadband),2);
      Rotation2d linearDirection = new Rotation2d(xSpeed, ySpeed);
      
      Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
      .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
      .getTranslation();
      ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      linearVelocity.getX() * SwerveConstants.kMaxSpeed, 
      linearVelocity.getY() * SwerveConstants.kMaxSpeed,
      thetaSpeed * SwerveConstants.kMaxAngularSpeed, 
      swerveSubsystem.getRotation2d());

      ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
      SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(discreteSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.kMaxSpeed);

      swerveSubsystem.setModuleStates(moduleStates);
    }
    



  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}