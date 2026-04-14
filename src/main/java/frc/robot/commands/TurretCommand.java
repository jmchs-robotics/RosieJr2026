package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.Turret;

public class TurretCommand extends Command {

  private final Turret turret;

  public TurretCommand(Turret turret) {

    this.turret = turret;
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    turret.zeroTurret();
  }

  @Override
  public void execute() {
    Pose2d currentPose = turret.getPose();
    Rotation2d currentToHubAngle;
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      currentToHubAngle =
          Constants.redHub.getTranslation().minus(currentPose.getTranslation()).getAngle();
    } else {
      currentToHubAngle =
          Constants.blueHub.getTranslation().minus(currentPose.getTranslation()).getAngle();
    }
    turret.setTarget(currentToHubAngle, turret.getVelocity());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
