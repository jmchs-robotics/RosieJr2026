package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {

  private final HopperIO io;

  private final Alert hopperAlert = new Alert("hopper disconnected", AlertType.kError);

  private final HopperIOInputsAutoLogged inputs;

  public Hopper(HopperIO io) {
    this.io = io;
    inputs = new HopperIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("hopper", inputs);

    hopperAlert.set(!inputs.hopperIsConnected);
  }

  public void setMotor(double speed) {
    io.setOpenLoop(speed);
  }
}
