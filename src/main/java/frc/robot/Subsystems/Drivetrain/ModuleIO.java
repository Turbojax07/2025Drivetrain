package frc.robot.Subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    public class ModuleIOInputs {
        SwerveModulePosition modulePosition;
        SwerveModuleState moduleState;

        Rotation2d steerAbsAngle;

        Rotation2d steerAngle;
        AngularVelocity steerVelocity;
        AngularAcceleration steerAcceleration;

        Distance driveDistance;
        LinearVelocity driveVelocity;
        LinearAcceleration driveAcceleration;

        Voltage driveVoltage;
        Voltage steerVoltage;
        
        Current driveCurrent;
        Current steerCurrent;

        Temperature driveTemperature;
        Temperature steerTemperature;
    }

    public void updateInputs();

    public void setState(SwerveModuleState state);
    public void resetPosition(SwerveModulePosition position);

    public SwerveModuleState getState();
    public SwerveModulePosition getPosition();

    public Rotation2d getAbsoluteAngle();

    public Rotation2d getAngle();
    public AngularVelocity getSteerVelocity();
    public AngularAcceleration getSteerAcceleration();

    public Distance getDistance();
    public LinearVelocity getDriveVelocity();
    public LinearAcceleration getDriveAcceleration();
    
    public Voltage getDriveVoltage();
    public Voltage getSteerVoltage();
    
    public Current getDriveCurrent();
    public Current getSteerCurrent();

    public Temperature getDriveTemperature();
    public Temperature getSteerTemperature();
}