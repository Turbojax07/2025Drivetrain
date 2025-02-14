package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drivetrain.Drivetrain;
import frc.robot.Subsystems.Drivetrain.ModuleIOSim;
import frc.robot.Subsystems.Drivetrain.ModuleIOTalonFX;
import frc.robot.Subsystems.Drivetrain.Commands.SwerveDrive;
import frc.robot.Subsystems.Gyro.Gyro;
import frc.robot.Subsystems.Gyro.GyroIOBNO085;

public class RobotContainer {
    private CommandXboxController controller = new CommandXboxController(0);

    public RobotContainer() {
        if (RobotBase.isReal()) {
            new Gyro(new GyroIOBNO085());
            new Drivetrain(new ModuleIOTalonFX(0), new ModuleIOTalonFX(1), new ModuleIOTalonFX(2), new ModuleIOTalonFX(3));
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