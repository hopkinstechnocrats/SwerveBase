package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.drive.DriveSubsystem;

public class LevelAutoRoutine extends CommandBase {
    DriveSubsystem m_robotDrive;
    public LevelAutoRoutine(DriveSubsystem m_robotDrive) {
        this.m_robotDrive = m_robotDrive;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        new RunCommand(() -> m_robotDrive.drive(0, 2, 0, 0), m_robotDrive).withInterrupt(m_robotDrive.isUp());
        new RunCommand(() -> m_robotDrive.level(), m_robotDrive);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}

}
