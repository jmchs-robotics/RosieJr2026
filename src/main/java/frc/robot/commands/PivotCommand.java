package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Pivot;

public class PivotCommand extends Command {

  private final Pivot m_pivot;

  public PivotCommand(Pivot pivot) {

    m_pivot = pivot;
    addRequirements(m_pivot);
  }

  @Override
  public void execute() {
    m_pivot.setPosition(Units.Rotations.of(0.3));
    ;
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_pivot.stopMotor();
  }
}
