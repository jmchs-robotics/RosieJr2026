package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;

public class Turret extends SubsystemBase {

  private final TurretIO io;
  private final Drive drive;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private Rotation2d target = new Rotation2d();
  private double lastTarget = 0;
  private double goalVelocity = 0;
  private double turretOffset = 0;
  private static final double minAngle = Units.degreesToRadians(-210.0);
  private static final double maxAngle = Units.degreesToRadians(210.0);

  private static final double acceptableAngleError = Units.degreesToRadians(1);
  private static final double acceptableVelocityError = 0.5;

  private boolean turretZeroed = false;
  @AutoLogOutput private boolean atGoal = false;

  private TrapezoidProfile profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
  private State setPoint = new State();

  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("Turret/MaxVelocity");
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Turret/MaxAcceleration", 9999999);
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Turret/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Turret/kD");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Turret/kA");

  static {
    maxVelocity.initDefault(0);
    maxAcceleration.initDefault(0);
    kP.initDefault(0);
    kD.initDefault(0);
    kA.initDefault(0);
  }

  public Turret(TurretIO io, Drive drive) {

    this.io = io;
    this.drive = drive;
  }

  @Override
  public void periodic() {
    Logger.processInputs("Turret", inputs);
    io.updateInputs(inputs);

    if (maxVelocity.hasChanged(hashCode())) {
      profile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
    }

    if (DriverStation.isDisabled()) {
      setPoint = new State(io.getTurretPosition(), 0.0);
      lastTarget = getTurretPosition();
    }

    if (turretZeroed) {
      Rotation2d robotAngle = drive.getRotation();
      double robotAngularVelocity = drive.getFieldVelocity().omegaRadiansPerSecond;

      Rotation2d robotRelativeGoalAngle = target.minus(robotAngle);
      double robotRelativeGoalVelocity = goalVelocity - robotAngularVelocity;

      boolean hasBestAngle = false;
      double bestAngle = 0;

      for (int i = -2; i < 3; i++) {
        double potentialSetpoint = robotRelativeGoalAngle.getRadians() + Math.PI * 2.0 * i;
        if (potentialSetpoint < minAngle || potentialSetpoint > maxAngle) {
          continue;
        } else {
          if (!hasBestAngle) {
            bestAngle = potentialSetpoint;
            hasBestAngle = true;
          }
          if (Math.abs(lastTarget - potentialSetpoint) < Math.abs(lastTarget - bestAngle)) {
            bestAngle = potentialSetpoint;
          }
        }
      }

      lastTarget = bestAngle;

      State goalState =
          new State(MathUtil.clamp(bestAngle, minAngle, maxAngle), robotRelativeGoalVelocity);
      setPoint = profile.calculate(Constants.loopPeriodSecs, setPoint, goalState);

      atGoal =
          (Math.abs(bestAngle - setPoint.position) <= acceptableAngleError)
              && (Math.abs(robotRelativeGoalVelocity - setPoint.velocity)
                  <= acceptableVelocityError);
    }
  }

  public void setTarget(Rotation2d angle, double velocity) {

    target = angle;
    goalVelocity = velocity;
  }

  public State getSetPoint() {
    return setPoint;
  }

  public void zeroTurret() {
    turretOffset = -io.getTurretPosition();
    turretZeroed = true;
  }

  public double getTurretPosition() {
    return io.getTurretPosition() + turretOffset;
  }

  public boolean getAtGoal() {
    return atGoal;
  }
}
