package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class LEDStatusCommand extends Command {
  LEDSubsystem ledSubsystem;
  Boolean allianceRed = true;
  private enum colorStatus {
    BLINK_RED,
    RAINBOW,
    RED_ALLIANCE,
    BLUE_ALLIANCE
  };


  public LEDStatusCommand(LEDSubsystem ledSubsystem) {
    addRequirements(ledSubsystem);
    this.ledSubsystem = ledSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() || allianceRed != (alliance.get() == DriverStation.Alliance.Red)) {
      allianceRed = (alliance.get() == DriverStation.Alliance.Red);
    }
    if (DriverStation.isDSAttached() == false) {/* Blink red */}
    else if (DriverStation.isAutonomousEnabled()) {/* rainbow? */}
    else if (DriverStation.isEStopped()) {/* Blink red Rapidly(?) */}
    
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ledSubsystem.setLEDRGB(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled(){
    return true;
  }
}
