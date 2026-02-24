package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeUp extends Command {

  private final Intake m_IntakeUp;

  public IntakeUp(Intake IntakeUp) {

    m_IntakeUp = IntakeUp;
    addRequirements(m_IntakeUp);
  }

  @Override
  public void initialize() {
    m_IntakeUp.setPosition(Rotations.of(0));
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return (m_IntakeUp.getPosition() < 0.5);
  }

  @Override
  public void end(boolean interrupted) {
    m_IntakeUp.setSlapDownMotor(0);
  }
}
