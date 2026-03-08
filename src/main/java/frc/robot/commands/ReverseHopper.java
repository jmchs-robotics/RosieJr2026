package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Hopper;

public class ReverseHopper extends Command {

  private final Hopper m_Hopper;

  public ReverseHopper(Hopper Hopper) {
    m_Hopper = Hopper;
    addRequirements(m_Hopper);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_Hopper.setMotor(0.50);
  }

  @Override
  public void end(boolean interrupted) {
    m_Hopper.setMotor(0);
  }
}
