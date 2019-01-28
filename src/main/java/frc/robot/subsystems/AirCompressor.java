package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AirCompressor.*;
import edu.wpi.first.wpilibj.Compressor;

/**
 *
 */
public class AirCompressor extends Subsystem {
	Compressor compressor;

	public AirCompressor() {
		compressor = new Compressor(RobotMap.AIR_COMPRESSOR);
		compressor.setClosedLoopControl(true);
	}

	public void start() {
		Robot.airCompressor.start();
		
	}

	public void stop() {
		Robot.airCompressor.stop();
	}

	public boolean pressureLow() {
		return compressor.getPressureSwitchValue();
	}
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	@Override
	public void initDefaultCommand() {

		setDefaultCommand(new StartCompressor());
	}

	public boolean isRunning() {
		return (!pressureLow());
	}

	public void updateStatus() {
		SmartDashboard.putBoolean("Compressor Running", isRunning());
		SmartDashboard.putBoolean("Pressure Low", pressureLow());
	}
}
