package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import frc.robot.AutoChoosers;
import frc.robot.SD;

/**
 *
 */
public class PowerPanel extends Subsystem {
	PowerDistributionPanel powerPanel;

	public PowerPanel() {
		powerPanel = new PowerDistributionPanel(RobotMap.POWER_DISTRIBUTION_PANEL);
	}

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	public double getTotalCurrent() {
		return Robot.pdp.getTotalCurrent();
	}

	public double getTotalEnergy() {
		return Robot.pdp.getTotalEnergy();
	}

	public double getTotalPower() {
		return Robot.pdp.getTotalPower();
	}

	public double getVoltage() {
		return Robot.pdp.getVoltage();
	}

	public double getChannelCurrent(int channel) {
		return Robot.pdp.getChannelCurrent(channel);
	}

	public void resetTotalEnergy() {
		Robot.pdp.resetTotalEnergy();
	}

	public void updateStatus() {
		if (AutoChoosers.debugChooser.getSelected() == 5) {
			SD.putN1("Energy Used", getTotalEnergy());
			SD.putN1("Amps Total", getTotalCurrent());
			SD.putN1("Channel 0", getChannelCurrent(0));
			SD.putN1("Channel 1", getChannelCurrent(1));
			SD.putN1("Channel 2", getChannelCurrent(2));
			SD.putN1("Channel 3", getChannelCurrent(3));
		}
	}
}
