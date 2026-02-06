package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;

public class DriveToPose extends Command {

  private static final LoggedTunableNumber thetakP = new LoggedTunableNumber("DriveToPose/ThetakP");
  private static final LoggedTunableNumber thetakD = new LoggedTunableNumber("DriveToPose/ThetakD");

  private static boolean isFlipped = false;

  private final CommandXboxController driveController;

  static {
    thetakP.initDefault(5.0);
    thetakD.initDefault(0.5);
  }

  private final Drive drive;

  private final PIDController thetaController =
      new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);

  public DriveToPose(Drive drive, CommandXboxController driveController) {
    this.drive = drive;
    this.driveController = driveController;
    if (drive != null) {
      addRequirements(drive);
    }

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    thetaController.reset();
  }

  @Override
  public void execute() {
    thetaController.setP(thetakP.get());
    thetaController.setD(thetakD.get());

    double thetaVelocity;
    if (!isFlipped) {
      thetaVelocity =
          thetaController.calculate(
              drive.getRotation().getRadians(), Constants.hubPose.getRotation().getRadians());
    } else {
      thetaVelocity =
          thetaController.calculate(
              drive.getRotation().getRadians(),
              Constants.hubPose
                  .rotateAround(
                      new Translation2d(Units.inchesToMeters(325.61), Units.inchesToMeters(158.84)),
                      new Rotation2d(180))
                  .getRotation()
                  .getRadians());
    }

    if (drive != null) {
      drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              driveController.getLeftX(),
              driveController.getLeftY(),
              thetaVelocity,
              drive.getRotation()));
    }
  }
}
