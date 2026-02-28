package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class Shoot extends SubsystemBase {

  private final ShootIO io;
  private final Drive drive;

  private final ShooterIOInputsAutoLogged inputs;

  public Shoot(ShootIO io, Drive drive) {
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
        (shooterToHub * Math.sin(ShootConstants.shooterTheta)
                - (2
                    * Math.cos(ShootConstants.shooterTheta)
                    * (ShootConstants.ShooterToHubdeltaZ + ShootConstants.arcToHub)))
            / (Math.sin(ShootConstants.shooterTheta)
                * Math.cos(ShootConstants.shooterTheta)
                * Math.sqrt((2 * ShootConstants.arcToHub / ShootConstants.gravity)));

    double motorSpeed =
        (2.42 * Math.pow(10, -3) * (Math.pow(velocity, 2))) + (-0.127 * velocity) + 2.27;
    return motorSpeed;
  }
}
