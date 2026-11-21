package frc.robot.commands;

// TODO Change all Aglae things to correct thing

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
    m_pivot.setposition(0.1);
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
