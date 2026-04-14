package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class Hood extends SubsystemBase {

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs;

  public Hood(HoodIO io) {

    this.io = io;
    inputs = new HoodIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void setSpeed(double speed) {
    io.setOpenLoop(speed);
  }

  public void setPosition(Angle angle) {
    io.setHoodPosition(angle);
  }

  @AutoLogOutput(key = "hood/position")
  public double getCurrentPosition() {
    return io.getHoodPosition();
  }
}
