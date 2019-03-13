package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.SD;
import frc.robot.AutoChoosers;
import frc.robot.Pref;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

public class GamePieceHandler extends Subsystem {
	public static TalonSRX cargoMotor;
	public DoubleSolenoid hatchCoverGripper;
	public Solenoid hatchCoverExtend;
	// public DoubleSolenoid hatchCoverExtend;
	// public DoubleSolenoid hatchCoverSecondExtend;
	public Solenoid hatchCoverPusher;
	private int gripperCounter;
	private boolean stopCargoIntake;
	public static AnalogTrigger leftPusherSwitch;
	public static AnalogTrigger rightPusherSwitch;

	public GamePieceHandler() {

		cargoMotor = new TalonSRX(RobotMap.CARGO_MOTOR);
		cargoMotor.configFactoryDefault();
		cargoMotor.setNeutralMode(NeutralMode.Brake);
		cargoMotor.configVoltageCompSaturation(12, 0);
		cargoMotor.enableVoltageCompensation(true);

		leftPusherSwitch = new AnalogTrigger(RobotMap.LEFT_PUSHER_SWITCH);
		leftPusherSwitch.setAveraged(true);
		leftPusherSwitch.setLimitsVoltage(1.0, 5.0);

		rightPusherSwitch = new AnalogTrigger(RobotMap.RIGHT_PUSHER_SWITCH);
		rightPusherSwitch.setAveraged(true);
		rightPusherSwitch.setLimitsVoltage(1.0, 5.0);

		hatchCoverGripper = new DoubleSolenoid(1, 0);
		hatchCoverGripper.set(DoubleSolenoid.Value.kForward);

		// hatchCoverExtend = new DoubleSolenoid(2, 3);
		// hatchCoverExtend.set(DoubleSolenoid.Value.kReverse);

		hatchCoverExtend = new Solenoid(2);
		hatchCoverExtend.set(false);

		// hatchCoverSecondExtend = new DoubleSolenoid(4, 5);
		// hatchCoverSecondExtend.set(DoubleSolenoid.Value.kReverse);

		hatchCoverPusher = new Solenoid(6);
		hatchCoverPusher.set(false);

	}

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand() {

	}

	public double getDriverSlider() {
		double temp = (1 - Robot.m_oi.driverController.getThrottle()) / 2;
		return temp;
	}

	public void stopCargoMotor() {
		cargoMotor.set(ControlMode.PercentOutput, 0);
	}

	public void pickUpCargo(double speed) {
		cargoMotor.set(ControlMode.PercentOutput, speed);
	}

	public void deliverCargo(double speed) {
		stopCargoIntake = false;
		cargoMotor.set(ControlMode.PercentOutput, -speed);
	}

	public void gripHatchPanel() {
		hatchCoverGripper.set(DoubleSolenoid.Value.kForward);
		gripperCounter = 0;
	}

	public void releaseHatchPanel() {
		hatchCoverGripper.set(DoubleSolenoid.Value.kReverse);
		gripperCounter = 0;
	}

	private void gripHatchPanelOff() {
		hatchCoverGripper.set(DoubleSolenoid.Value.kOff);
	}

	public void extendHatchPanel() {
		hatchCoverExtend.set(true);
	}

	public void retractHatchPanel() {
		hatchCoverExtend.set(false);
	}

	public void pushHatchPanel() {
		hatchCoverPusher.set(true);
	}

	public void retractPusher() {
		hatchCoverPusher.set(false);
	}

	public boolean getHatchDetected() {
		return leftPusherSwitch.getInWindow() || rightPusherSwitch.getInWindow();
	}

	public void updateStatus() {
		if (cargoMotor.getOutputCurrent() > Pref.getPref("CargoIntakeAmpsLimit"))
			stopCargoIntake = true;
		if (stopCargoIntake)
			stopCargoMotor();
		if (hatchCoverGripper.get() != DoubleSolenoid.Value.kOff)
			gripperCounter++;
		if (gripperCounter > 2) {
			gripHatchPanelOff();
		}
		SD.putN1("CargoMotorAmps", cargoMotor.getOutputCurrent());
		SD.putN1("CargoMotorPct", cargoMotor.getMotorOutputPercent());
		SD.putN2("Sldr", getDriverSlider());
		if (AutoChoosers.debugChooser.getSelected() == 4) {
			SD.putN1("CargoMotorVolts", cargoMotor.getBusVoltage());

		}
	}
}
