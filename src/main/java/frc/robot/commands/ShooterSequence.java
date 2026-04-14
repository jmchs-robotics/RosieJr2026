package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Hopper;
import frc.robot.subsystems.shooter.Shoot;

public class ShooterSequence extends Command {

  private final Shoot m_shoot;
  private final Hopper m_hopper;
  double motorSpeed = 0;
  private final Timer m_timer = new Timer();

  public ShooterSequence(Shoot shoot, Hopper hopper) {
    m_shoot = shoot;
    m_hopper = hopper;
    addRequirements(m_shoot, m_hopper);
  }
  // this for motor speed
  @Override
  public void execute() {
    if (m_timer.get() < .25) {
      motorSpeed = m_shoot.calculateSpeed();
      m_shoot.setMotor(motorSpeed);
      // m_hopper.setMotors(-0.3);
    } else {
      motorSpeed = m_shoot.calculateSpeed();
      m_shoot.setMotor(motorSpeed);
      m_hopper.setMotors(0.8);
    }
  }

  @Override
  public void initialize() {
    m_timer.start();
    m_timer.reset();
  }

  @Override
  public void end(boolean interrupted) {
    m_shoot.setMotor(0.0);
    m_hopper.setMotors(0.0);
    m_timer.stop();
  }
}
