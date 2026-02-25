package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

    private TurretIO io;

    private Rotation2d target = new Rotation2d();
    private double goalVelocity = 0;
    private double turretOffset = 0;

    public Turret(TurretIO io) {

        this.io = io;

    }

    @Override
    public void periodic() {}

    public void setTarget(Rotation2d angle, double velocity) {

        target = angle;
        goalVelocity = velocity;

    }

    public void zeroTurret() {
        turretOffset = -io.getTurretPosition();
    }

    public double getTurretPosition() {
        return io.getTurretPosition() + turretOffset;
    }
    
}
