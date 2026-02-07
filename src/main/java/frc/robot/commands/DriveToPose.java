package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;

public class DriveToPose extends Command {

  private static final LoggedTunableNumber thetakP = new LoggedTunableNumber("DriveToPose/ThetakP");
  private static final LoggedTunableNumber thetakD = new LoggedTunableNumber("DriveToPose/ThetakD");

  private static boolean isFlipped = false;
  private static final Translation2d hubPose =
      new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));

  private final CommandXboxController driveController;

  static {
    thetakP.initDefault(10);
    thetakD.initDefault(0.8);
  }

  private final Drive drive;

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(thetakP.get(), 0.0, thetakD.get(), new Constraints(8.0, 20.0));

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
    thetaController.reset(drive.getRotation().getRadians());
  }

  @Override
  public void execute() {
    thetaController.setP(thetakP.get());
    thetaController.setD(thetakD.get());
    Pose2d currentPose = drive.getPose();

    Rotation2d currentToHubAngle = hubPose.minus(currentPose.getTranslation()).getAngle();

    double thetaVelocity;
    if (!isFlipped) {
      thetaVelocity =
          thetaController.calculate(
              drive.getRotation().getRadians(), currentToHubAngle.getRadians());
    } else {
      thetaVelocity =
          thetaController.calculate(
              drive.getRotation().getRadians(),
              currentToHubAngle.minus(Rotation2d.kPi).getRadians());
    }

    if (drive != null) {
      drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              -driveController.getLeftY() * drive.getMaxLinearSpeedMetersPerSec(),
              -driveController.getLeftX() * drive.getMaxLinearSpeedMetersPerSec(),
              thetaVelocity, // * drive.getMaxAngularSpeedRadPerSec(),
              drive.getRotation()));
    }
  }
}
