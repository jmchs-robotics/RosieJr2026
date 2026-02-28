package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private final ClimbIO io;

  private final ClimbIOInputsAutoLogged inputs;

  public void climb(ClimbIO io) {
    this.io = io;
    inputs = new ClimbIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void setMotor(double speed) {
    io.setOpenLoop(speed);
    Logger.processInputs("Climb", inputs);
  }
}
