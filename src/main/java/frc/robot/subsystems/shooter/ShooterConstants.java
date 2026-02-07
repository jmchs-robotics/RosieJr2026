package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;

public class ShooterConstants {

  public static final double shooterTheta = 70.0; // current angle of the shooter in degrees
  public static final double gravity = Units.metersToInches(9.81); // in inches/sec ^2
  // distance between top of arc and hub as estimated by Aiden White in inches
  public static final double arcToHub = 3;
  // height of hub in inches - height of shooter on robot in inches
  public static final double ShooterToHubdeltaZ = 72 - 21.75;
}
