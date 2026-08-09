package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Hopper;
import frc.robot.subsystems.shooter.Shooter;

public class PassToKid extends Command {

  private final Shooter m_shooter;
  private final Hopper m_hopper;
  private final Timer m_timer = new Timer();

  public PassToKid(Shooter shooter, Hopper hopper) {
    m_shooter = shooter;
    m_hopper = hopper;
    addRequirements(m_shooter, m_hopper);
  }
  // this for motor speed
  @Override
  public void execute() {
    if (m_timer.get() < .25) {
      m_shooter.setMotor(-0.3);
      // m_hopper.setMotors(-0.3);
    } else {
      m_shooter.setMotor(-0.3);
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
    m_shooter.setMotor(0.0);
    m_hopper.setMotors(0.0);
    m_timer.stop();
  }
}
