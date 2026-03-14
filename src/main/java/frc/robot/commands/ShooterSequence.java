package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Hopper;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterSequence extends Command {

  private final Shooter m_shooter;
  private final Hopper m_hopper;
  double motorSpeed = 0;
  private final Timer m_timer = new Timer();

  public ShooterSequence(Shooter shooter, Hopper hopper) {
    m_shooter = shooter;
    m_hopper = hopper;
    addRequirements(m_shooter, m_hopper);
  }
  // this for motor speed
  @Override
  public void execute() {
    if (m_timer.get() < .25) {
      motorSpeed = m_shooter.calculateSpeed();
      m_shooter.setMotor(motorSpeed);
      m_hopper.setMotor(0.7);
    } else {
      motorSpeed = m_shooter.calculateSpeed();
      m_shooter.setMotor(motorSpeed);
      m_hopper.setMotor(-0.7);
    }
  }

  @Override
  public void initialize() {
    m_timer.start();
    m_timer.reset();
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.setMotor(0.0);
    m_hopper.setMotor(0.0);
    m_timer.stop();
  }
}
