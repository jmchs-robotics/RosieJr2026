package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final Drive drive;

  private final ShooterIOInputsAutoLogged inputs;

  public Shooter(ShooterIO io, Drive drive) {
    this.io = io;
    this.drive = drive;
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
    double shooterToHub =
        Constants.hubPose.getDistance(
            drive.getPose().getTranslation()); // distance between shooter and hub

    // Aiden's magic equation
    double velocity =
        (shooterToHub * Math.sin(ShooterConstants.shooterTheta)
                - (2
                    * Math.cos(ShooterConstants.shooterTheta)
                    * (ShooterConstants.ShooterToHubdeltaZ + ShooterConstants.arcToHub)))
            / (Math.sin(ShooterConstants.shooterTheta)
                * Math.cos(ShooterConstants.shooterTheta)
                * Math.sqrt((2 * ShooterConstants.arcToHub / ShooterConstants.gravity)));

    double motorSpeed = 0.65;
    return motorSpeed;
  }
}
