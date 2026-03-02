package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class SlapDown extends Command {

  private final Intake m_slapDown;

  public SlapDown(Intake slapDown) {

    m_slapDown = slapDown;
    addRequirements(m_slapDown);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_slapDown.setSlapDownMotor(0.15);
  }

  @Override
  public void end(boolean interrupted) {
    m_slapDown.setPosition(Rotations.of(60));
  }
}
