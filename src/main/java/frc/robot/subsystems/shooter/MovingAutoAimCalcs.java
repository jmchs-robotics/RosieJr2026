package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

// calcs short for calculations

public class MovingAutoAimCalcs {

  public static final boolean tuning = true;
  public static final int iterations = 100;
  public static final double fudge = 5.67;

  public static final double flywheelRadius = Units.inchesToMeters(2);

  public static InterpolatingDoubleTreeMap flightTimeMap;

  static {
    flightTimeMap.put(1.41567635288239, 1.2);
    flightTimeMap.put(2.00764727190795, 1.2);
    flightTimeMap.put(2.81173626842406, 1.0 + 16.0 / 60.0);
    flightTimeMap.put(3.4956144644965, 1.0 + 18.0 / 60.0);
    flightTimeMap.put(4.25871165123726, 1.0 + 18.0 / 60.0);
    flightTimeMap.put(4.84412724500999, 1.0 + 23.0 / 60.0);
    flightTimeMap.put(5.37548577935445, 1.4);
    flightTimeMap.put(6.22187555861643, 1.0 + 26.0 / 60.0);
  }

  private MovingAutoAimCalcs() {}

  public static double rpmToExitVelocity(double rpm) {
    return (rpm * 2.0 * Math.PI / 60.0) * flywheelRadius;
  }

  public static double[] rotate(double x, double y, Rotation2d rotation) {
    return new double[] {
      rotation.getCos() * x - rotation.getSin() * y, rotation.getSin() * x + rotation.getCos() * y
    };
  }

  public static double distance(Translation2d translation0, Translation2d translation1) {
    return translation0.minus(translation1).getNorm();
  }

  public static Translation2d getOffset(
      double velocityX,
      double velocityY,
      Translation2d currentTranslation,
      Translation2d targetTranslation) {

    double time0 = 0;
    Translation2d translation0 = targetTranslation;

    double time1 = flightTimeMap.get(distance(translation0, currentTranslation));
    Translation2d translation1 =
        translation0.plus(new Translation2d(-velocityX * time1, -velocityY * time1));

    int finalIterations = 0;
    double finalFlightTime = 0;

    for (int i = 0; i < iterations; i++) {
      if (Math.abs(time1 - time0) < 1e-5) {
        finalIterations = i;
        break;
      }

      double flightTime0 = flightTimeMap.get(distance(translation0, currentTranslation));
      double flightTime1 = flightTimeMap.get(distance(translation1, currentTranslation));

      double newFlightTime =
          (time0 * (flightTime1 - time1) - time1 * (flightTime0 - time0))
              / (flightTime1 - time1 - flightTime0 + time0);

      time0 = time1;
      translation0 = translation1;

      time1 = newFlightTime;
      translation1 =
          targetTranslation.plus(
              new Translation2d(-velocityX * newFlightTime, -velocityY * newFlightTime));

      finalFlightTime = newFlightTime;
      finalIterations = i;
    }

    Logger.recordOutput("AutoAimCalcs/Final Iterations", finalIterations);
    Logger.recordOutput("AutoAimCalcs/Final Flight Time", finalFlightTime);

    // calcs short for calculations

    return translation1.minus(targetTranslation);
  }
}
