package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private Command teleopCommand;

    public Robot() {
        Logger.addDataReceiver(new NT4Publisher());

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter());
        }

        if (isSimulation() && Constants.isReplay) {
            Logger.setReplaySource(new WPILOGReader("replay.wpilog"));
        }

        Logger.start();

        RobotContainer robotContainer = new RobotContainer();

        autonomousCommand = robotContainer.getAutonomousCommand();
        teleopCommand = robotContainer.getTeleopCommand();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        } else {
            System.out.println("Autonomous command not configured.");
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopInit() {
        if (teleopCommand != null) {
            teleopCommand.schedule();
        } else {
            System.out.println("Teleop command not configured.");
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {
        if (teleopCommand != null) {
            teleopCommand.cancel();
        }
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}