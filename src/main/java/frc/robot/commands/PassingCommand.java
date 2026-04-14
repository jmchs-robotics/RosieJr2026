package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shoot;

public class PassingCommand extends Command {

  private final Shoot m_shoot;
  double motorSpeed = 380;

  public PassingCommand(Shoot shoot) {
    m_shoot = shoot;
    addRequirements(m_shoot);
  }
  // this for motor speed
  @Override
  public void execute() {

    m_shoot.setMotor(motorSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_shoot.setMotor(0.0);
  }
}
