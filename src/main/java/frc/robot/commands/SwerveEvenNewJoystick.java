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

public class SwerveEvenNewJoystick extends Command {

  private final SwerveSubsystem swerveSubsystem;

  private final Supplier<Double> xSpdFunctionField, ySpdFunctionField, xSpdFunctionRobot, ySpdFunctionRobot, turningSpdFunctionLeft, turningSpdFunctionRight;
  private final Supplier<Boolean> lockSourceAngle;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  Translation2d linearVelocity = null; // if it doesnt work check this
  double thetaSpeed = 0;
  double rotationRate = 0;
  private PIDController autoRotateController = new PIDController(3, 0, 0.5); //TODO: TUNEEEEEEEEE
    
  public SwerveEvenNewJoystick(SwerveSubsystem swerveSubsystem, 
    Supplier<Double> xSpdFunctionField, 
    Supplier<Double> ySpdFunctionField, 

    Supplier<Double> xSpdFunctionRobot,
    Supplier<Double> ySpdFunctionRobot,

    Supplier<Double> turningSpdFunctionLeft,
    Supplier<Double> turningSpdFunctionRight,
    Supplier<Boolean> lockSourceAngle) 
    {
      this.swerveSubsystem = swerveSubsystem;

      this.xSpdFunctionField = xSpdFunctionField;
      this.ySpdFunctionField = ySpdFunctionField;

      this.xSpdFunctionRobot = xSpdFunctionRobot;
      this.ySpdFunctionRobot = ySpdFunctionRobot;

      this.turningSpdFunctionLeft = turningSpdFunctionLeft;
      this.turningSpdFunctionRight = turningSpdFunctionRight;
      this.lockSourceAngle = lockSourceAngle;



      this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
      this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
      this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
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
        double thetaSpeed = turningSpdFunctionLeft.get() - turningSpdFunctionRight.get();
        thetaSpeed = turningLimiter.calculate(thetaSpeed);
        thetaSpeed = Math.abs(thetaSpeed) > SwerveConstants.kDeadband ? thetaSpeed : 0.0;
        thetaSpeed = Math.copySign(thetaSpeed * thetaSpeed, thetaSpeed);
      }



      if (Math.abs(xSpdFunctionField.get()) >= 0.05  || Math.abs(ySpdFunctionField.get()) >= 0.05) {
        double xSpeed = xSpdFunctionField.get() / swerveSubsystem.getSpeedMultiplier(); 
        /* Scale to deadband, chage per controller type (Logetech is 20%????) */
        xSpeed = (1 / (1 - SwerveConstants.kDeadband)) * (xSpeed + ( -Math.signum(xSpeed) * SwerveConstants.kDeadband));
        double ySpeed = ySpdFunctionField.get()/ swerveSubsystem.getSpeedMultiplier();
        ySpeed = (1 / (1 - SwerveConstants.kDeadband)) * (ySpeed + ( -Math.signum(ySpeed) * SwerveConstants.kDeadband));

        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);
        
        double linearMagnitude = Math.pow(MathUtil.applyDeadband(Math.hypot(xSpeed, ySpeed), SwerveConstants.kDeadband),2);
        Rotation2d linearDirection = new Rotation2d(xSpeed, ySpeed);
        
        Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
         .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();
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
      else if (Math.abs(xSpdFunctionRobot.get()) >= 0.05  || Math.abs(ySpdFunctionRobot.get()) >= 0.05) 
      {
        double xSpeed = xSpdFunctionRobot.get() / swerveSubsystem.getSpeedMultiplier(); 
        xSpeed = (1 / (1 - SwerveConstants.kDeadband)) * (xSpeed + ( -Math.signum(xSpeed) * SwerveConstants.kDeadband));
        double ySpeed = ySpdFunctionRobot.get() / swerveSubsystem.getSpeedMultiplier();
        ySpeed = (1 / (1 - SwerveConstants.kDeadband)) * (ySpeed + ( -Math.signum(ySpeed) * SwerveConstants.kDeadband));

        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);
        thetaSpeed = turningLimiter.calculate(thetaSpeed);

        double linearMagnitude = Math.pow(MathUtil.applyDeadband(Math.hypot(xSpeed, ySpeed), SwerveConstants.kDeadband),2);
        Rotation2d linearDirection = new Rotation2d(xSpeed, ySpeed);


        Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
          .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
          .getTranslation();

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
          linearVelocity.getX() * SwerveConstants.kMaxSpeed, 
          linearVelocity.getY() * SwerveConstants.kMaxSpeed, 
          thetaSpeed * SwerveConstants.kMaxAngularSpeed);
  
          ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
          SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(discreteSpeeds);
          SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.kMaxSpeed);
  
          swerveSubsystem.setModuleStates(moduleStates);
      }
      else {
        double xSpeed = 0;
        xSpeed = (1 / (1 - SwerveConstants.kDeadband)) * (xSpeed + ( -Math.signum(xSpeed) * SwerveConstants.kDeadband));
        double ySpeed = 0;

        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);

        double linearMagnitude = Math.pow(MathUtil.applyDeadband(Math.hypot(xSpeed, ySpeed), SwerveConstants.kDeadband),2);
        //linearMagnitude = linearMagnitude * linearMagnitude;
        Rotation2d linearDirection = new Rotation2d(xSpeed, ySpeed);


        Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
          .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
          .getTranslation();

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          linearVelocity.getX() * SwerveConstants.kMaxSpeed, 
          linearVelocity.getY() * SwerveConstants.kMaxSpeed,
          thetaSpeed * SwerveConstants.kMaxAngularSpeed, 
          swerveSubsystem.getRotation2d()
        );

        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.kMaxSpeed);

        swerveSubsystem.setModuleStates(moduleStates);
      }
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