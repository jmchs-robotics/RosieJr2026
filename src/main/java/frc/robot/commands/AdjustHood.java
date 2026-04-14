package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Hood;

public class AdjustHood extends Command {

  private final Hood hood;

  public AdjustHood(Hood hood) {

    this.hood = hood;
    addRequirements(hood);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    hood.setPosition(Rotations.of(-1.5));
  }

  @Override
  public void end(boolean interrupted) {
    hood.setSpeed(0);
  }
}
