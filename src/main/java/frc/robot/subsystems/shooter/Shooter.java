package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;

  private final ShooterIOInputsAutoLogged inputs;

  public Shooter(ShooterIO io, Drive drive) {
    this.io = io;
    inputs = new ShooterIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void setMotor(double speed) {
    io.setOpenLoop(speed);
    Logger.processInputs("Shooter", inputs);
  }

  public double calculateSpeed() {
    double shooterToHub = Constants.hubPose.minus(null)
    double speed = 0.65;
    return speed;
  }
}
