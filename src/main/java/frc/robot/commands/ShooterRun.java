package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterRun extends Command {

  private final Shooter m_Shooter;

  public ShooterRun(Shooter Shooter) {
    m_Shooter = Shooter;
    addRequirements(m_Shooter);
  }
  // this for motor speed
  @Override
  public void execute() {
    m_Shooter.setMotor(0.9);
  }

  @Override
  public void end(boolean interrupted) {
    m_Shooter.setMotor(0.0);
  }
}
