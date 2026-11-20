package frc.robot.commands;

//TODO fix this

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterCommand extends Command {

    private final Shooter m_shooter;

    public ShooterCommand(Shooter shooterWheels) {

        m_shooter = shooterWheels;
        addRequirements(m_shooter);

    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_shooter.setWheelMotors(1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stopWheelMotors();
    }
}