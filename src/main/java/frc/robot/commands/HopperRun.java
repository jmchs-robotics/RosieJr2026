package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Hopper;

public class HopperRun extends Command {

  private final Hopper m_Hopper;

  public HopperRun(Hopper Hopper) {
    m_Hopper = Hopper;
    addRequirements(m_Hopper);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_Hopper.setMotors(-0.7);
  }

  @Override
  public void end(boolean interrupted) {
    m_Hopper.setMotors(0);
  }
}
