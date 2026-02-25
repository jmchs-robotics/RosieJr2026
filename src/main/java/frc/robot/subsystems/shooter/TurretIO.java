package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

    @AutoLog
    public static class TurretIOInputs {

        public double turretCurrentAmps = 0;
        public double turretAppliedVolts = 0;
        public double turretVelocityRotPerSec = 0;

    }

    public default void updateInputs(TurretIOInputs inputs) {}

    public default double getTurretPosition() {return 0;}
    
}
