package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralDispenserSubsystem;

public class CollectCoralCmd extends Command {
  CoralDispenserSubsystem coralDispenserSubsystem;

  public CollectCoralCmd(CoralDispenserSubsystem coralDispenserSubsystem) {
    addRequirements(coralDispenserSubsystem);
    this.coralDispenserSubsystem = coralDispenserSubsystem;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    coralDispenserSubsystem.runMotors(.5, -.5);
  }

  @Override
  public void end(boolean interrupted) {
    coralDispenserSubsystem.runMotors(0, 0);
  }

  @Override
  public boolean isFinished() {
    return coralDispenserSubsystem.hascoral;
  }
}
