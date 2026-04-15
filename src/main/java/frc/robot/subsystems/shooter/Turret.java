package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {

  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  public Turret(TurretIO io, Drive drive) {

    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("turret", inputs);
  }

  public void setOpenLoop(double speed) {
    io.setOpenLoop(speed);
  }

  public void setTurretAngle(double degrees) {

    double currentAngle = io.getPosition().getDegrees();

    double distance = degrees - currentAngle;

    if (degrees < -135) {
      degrees += 270;
    } else if (degrees > 135) {
      degrees -= 270;
    }

    io.setPosition(Degrees.of(degrees * (200 / 7)));
  }
}
