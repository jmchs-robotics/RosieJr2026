package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

// kraken 60 and neo vortex
// addie put this in to check
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private final IntakeIO io;

  private final Alert intakeWheelsAlert = new Alert("intake wheels disconnected", AlertType.kError);
  private final Alert slapDownAlert = new Alert("slap down moter disconnected", AlertType.kError);

  private final IntakeIOInputsAutoLogged inputs;

  public Intake(IntakeIO io) {
    this.io = io;
    inputs = new IntakeIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("intake", inputs);

    intakeWheelsAlert.set(!inputs.intakeIsConnected);
    slapDownAlert.set(!inputs.slapDownIsConnected);
  }

  public void setWheelMotor(double speed) {
    io.setOpenLoopWheels(speed);
  }

  public void setSlapDownMotor(double speed) {
    io.setOpenLoopSlapDown(speed);
  }

  public void setPosition(Angle angle) {
    io.setPosition(angle);
  }

  public double getPosition() {
    return io.getPosition();
  }
}
