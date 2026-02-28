package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class ClimbUp extends Command {

  private final Climb m_climb;
  double motorSpeed = 0;

  public ClimbUp(Climb climb) {
    m_climb = climb;
    addRequirements(m_climb);
  }
  // this for motor speed
  @Override
  public void execute() {
    // TODO replace this with a real value
    motorSpeed = 3.0;
    m_climb.setMotor(motorSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_climb.setMotor(0.0);
  }
}
