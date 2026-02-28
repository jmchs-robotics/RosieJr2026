package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class ClimbDown extends Command {
    private final Climb m_climb;
  private double motorSpeed = 0;

  public ClimbDown(Climb climb) {
    m_climb = climb;
    addRequirements(m_climb);
  }
  // this for motor speed
  @Override
  public void execute() {
    // TODO replace this with a real value
    motorSpeed = -0.3;
    m_climb.setMotor(motorSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_climb.setMotor(0.0);
  }


    
}
