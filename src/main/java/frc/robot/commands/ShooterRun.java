package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shoot;

public class ShooterRun extends Command {

  private final Shoot m_shooter;
  double motorSpeed = 0;

  public ShooterRun(Shoot shooter) {
    m_shooter = shooter;
    addRequirements(m_shooter);
  }
  // this for motor speed
  @Override
  public void execute() {
    motorSpeed = m_shooter.calculateSpeed();
    m_shooter.setMotor(motorSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.setMotor(0.0);
  }
}
