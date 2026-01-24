package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;

  public Shooter(ShooterIO io) {

    this.io = io;
  }

  public void setMotor(double speed) {
    io.setOpenLoop(speed);
  }
}
