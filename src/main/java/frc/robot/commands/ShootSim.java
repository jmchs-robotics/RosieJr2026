package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

public class ShootSim extends Command {

  private final SwerveDriveSimulation driveSim;
  private final Shooter shooter;

  private static boolean isFlipped = false;

  public ShootSim(SwerveDriveSimulation driveSim, Shooter shooter) {

    this.driveSim = driveSim;
    this.shooter = shooter;

    isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  @Override
  public void execute() {

    double velocity = shooter.calculateSpeed() / 1.047;

    SimulatedArena.getInstance()
        .addGamePieceProjectile(
            new RebuiltFuelOnFly(
                driveSim.getSimulatedDriveTrainPose().getTranslation(),
                new Translation2d(),
                driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                driveSim.getSimulatedDriveTrainPose().getRotation(),
                Inches.of(22),
                FeetPerSecond.of(velocity),
                Degrees.of(70)));
  }
}
