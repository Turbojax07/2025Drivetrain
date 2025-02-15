package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drivetrain.*;
import frc.robot.Subsystems.Drivetrain.Commands.*;
import frc.robot.Subsystems.Gyro.*;

public class RobotContainer {
    private CommandXboxController controller = new CommandXboxController(0);

    public RobotContainer() {
        if (RobotBase.isReal()) {
            new Gyro(new GyroIOPigeon2());
            new Drivetrain(new ModuleIOSparkMax(0), new ModuleIOSparkMax(1), new ModuleIOSparkMax(2), new ModuleIOSparkMax(3));
        } else {
            new Drivetrain(new ModuleIOSim(0), new ModuleIOSim(1), new ModuleIOSim(2), new ModuleIOSim(3));
        }

        configureBindings();
    }

    private void configureBindings() {}

    public Command getAutonomousCommand() {
        return null;
    }

    public Command getTeleopCommand() {
        return new SwerveDrive(controller::getLeftX, controller::getLeftY, controller::getRightX);
    }
}