package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterRun extends Command {

  private final Shooter m_shooter;
  double speed = 0;

  public ShooterRun(Shooter shooter) {
    m_shooter = shooter;
    addRequirements(m_shooter);
  }
  // this for motor speed
  @Override
  public void execute() {
    speed = m_shooter.calculateSpeed();
    m_shooter.setMotor(speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.setMotor(0.0);
  }
}
