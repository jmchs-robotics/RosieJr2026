package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class PassingCommand extends Command {

  private final Shooter m_shooter;
  double motorSpeed = 50;

  public PassingCommand(Shooter shooter) {
    m_shooter = shooter;
    addRequirements(m_shooter);
  }
  // this for motor speed
  @Override
  public void execute() {

    m_shooter.setMotor(motorSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.setMotor(0.0);
  }
}
