package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.NewSwerveJoystick;
import frc.robot.commands.SwerveEvenNewJoystick;
import frc.robot.commands.TeleopCommands.SwerveNewJoystick;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  public SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private XboxController driverController = new XboxController(OIConstants.driverControllerPort);
  
  //private CommandXboxController operatorController = new CommandXboxController(OIConstants.operatorControllerPort);

  public RobotContainer() {
    ShuffleboardTab baseTab = Shuffleboard.getTab("Base Tab");

    

    namedCommands();
    defaultCommands();
    configureButtonBindings();
  }

  /* Sets default commands for each subsystem */
  private void defaultCommands() {
    

    // swerveSubsystem.setDefaultCommand(new SwerveEvenNewJoystick(
    //   swerveSubsystem,
    //   // Left Joystick Field Oriented
    //   () -> (-driverController.getRawAxis(OIConstants.leftStickY)) - (driverController.getRightTriggerAxis() * driverController.getLeftY()),
    //   () -> driverController.getRawAxis(OIConstants.leftStickX) - (driverController.getRightTriggerAxis() * driverController.getLeftX()),

    //   //Right Joystick For Robot Centic
    //   () -> -driverController.getRawAxis(OIConstants.rightStickY),
    //   () -> driverController.getRawAxis(OIConstants.rightStickX),

    //   // Triggers for turning
    //   () -> driverController.getRawAxis(OIConstants.rightTrigger),
    //   () -> driverController.getRawAxis(OIConstants.leftTrigger),

    //   // Auto Turn
    //   () -> driverController.getRawButton(OIConstants.rightBumper)
    // ));

    swerveSubsystem.setDefaultCommand(new NewSwerveJoystick(
      swerveSubsystem,
      // Left Joystick Field Oriented
      () -> (-MathUtil.clamp(-driverController.getLeftY(),-0.5,0.5)) 
        - (MathUtil.applyDeadband(driverController.getRightTriggerAxis(), 0.1) 
          *  MathUtil.applyDeadband(MathUtil.clamp(-driverController.getLeftY(),-0.5,0.5),0.1))
        + (driverController.getLeftTriggerAxis() *  MathUtil.clamp(-driverController.getLeftY(),-0.5,0.5)),

      () -> MathUtil.clamp(driverController.getRightX(),-0.5,0.5) 
        - (MathUtil.applyDeadband(driverController.getRightTriggerAxis(), 0.1) 
          *  MathUtil.applyDeadband(MathUtil.clamp(-driverController.getRightX(),-0.5,0.5),0.1))
        + (driverController.getLeftTriggerAxis() *  MathUtil.clamp(-driverController.getRightX(),-0.5,0.5)),

        
      //right joystick turning
      () -> MathUtil.applyDeadband(MathUtil.clamp(-driverController.getRightX(),-0.5,0.5),0.1)
        - (MathUtil.applyDeadband(driverController.getRightTriggerAxis(),0.1) *  MathUtil.applyDeadband(MathUtil.clamp(driverController.getRightX(),-0.5,0.5),0.1))
        + (MathUtil.applyDeadband(driverController.getLeftTriggerAxis(),0.1) *  MathUtil.applyDeadband(MathUtil.clamp(driverController.getRightX(),-0.5,0.5),0.1)),

      // Auto Turn
      () -> driverController.getRightBumper()
    ));

  }

  /* Registers Named Commands used in Paths */
  private void namedCommands() {}

  /* Create button Bindings*/
  private void configureButtonBindings() {


    /* Maybe adda cmd when it finshes to 0 it? */

    /* Robot oriented Dpad */
    new POVButton(driverController, 0)
    .whileTrue(new InstantCommand(() -> swerveSubsystem.dPadDrive(0.2,0)));
    new POVButton(driverController, 90)
    .whileTrue(new InstantCommand(() -> swerveSubsystem.dPadDrive(0,0.2)));
    new POVButton(driverController, 180)
    .whileTrue(new InstantCommand(() -> swerveSubsystem.dPadDrive(-0.2,0)));
    new POVButton(driverController, 270)
    .whileTrue(new InstantCommand(() -> swerveSubsystem.dPadDrive(0,-0.2)));

    new JoystickButton(driverController, Constants.OIConstants.backButton)
    .onTrue(new InstantCommand(() -> swerveSubsystem.resetGyro()));

    /* A */
    new JoystickButton(driverController, Constants.OIConstants.aButton)
    .onTrue(new InstantCommand(() -> swerveSubsystem.setSpeedMultiplier(4)));
    new JoystickButton(driverController, Constants.OIConstants.bButton)
    .onTrue(new InstantCommand(() -> swerveSubsystem.setSpeedMultiplier(2)));
    new JoystickButton(driverController, Constants.OIConstants.xButton)
    .onTrue(new InstantCommand(() -> swerveSubsystem.setSpeedMultiplier(1.5)));


  } 

  public Command getAutonomousCommand() {
    /* Run no Auto */
    return new InstantCommand();
  }


}
