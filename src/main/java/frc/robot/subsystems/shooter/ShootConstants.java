package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class ShootConstants {

  /** current angle of the shooter in degrees */
  public static final double shooterTheta = 70.0;
  /** in inches/sec ^2 */
  public static final double gravity = Units.metersToInches(9.81);
  /** distance between top of arc and hub as estimated by Aiden White in inches */
  public static final double arcToHub = 36;
  /** height of hub in inches - height of shooter on robot in inches */
  public static final double ShooterToHubdeltaZ = 72 - 21.75;

  public static final Translation2d turretToRobot = new Translation2d();
}
