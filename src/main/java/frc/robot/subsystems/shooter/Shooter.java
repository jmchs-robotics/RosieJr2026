package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final Drive drive;
  private static boolean isFlipped = false;

  private final Alert shooterAlert = new Alert("shooter disconnected", AlertType.kError);
  private final Alert followAlert = new Alert("shooter follower disconnected", AlertType.kError);

  private final ShooterIOInputsAutoLogged inputs;

  public Shooter(ShooterIO io, Drive drive) {

    this.io = io;
    this.drive = drive;
    inputs = new ShooterIOInputsAutoLogged();
    isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

    shooterAlert.set(!inputs.shooterIsConnected);
    followAlert.set(!inputs.followerIsConnected);
  }

  public void setMotor(double speed) {
    io.setVelocity(speed);
  }

  @AutoLogOutput
  public double calculateSpeed() {
    double shooterToHub;
    if (!isFlipped) {
      shooterToHub =
          Units.metersToFeet(
              Constants.blueHub
                  .getTranslation()
                  .getDistance(
                      drive.getPose().getTranslation())); // distance between shooter and hub
    } else {
      shooterToHub =
          Units.metersToFeet(
              Constants.redHub
                  .getTranslation()
                  .getDistance(
                      drive.getPose().getTranslation())); // distance between shooter and hub
    }

    double velocity = (21 + 15 * ((shooterToHub - 4.1) / 13.2)) * 1.047 * 5.75 * 2;

    // return velocity;
    return 0.6;
  }
}
