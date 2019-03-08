package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.SD;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AutoChoosers;
import frc.robot.Constants;
import frc.robot.commands.Elevator.*;
import frc.robot.commands.Cargo.*;
import frc.robot.Robot;

public class Elevator extends Subsystem {
	public TalonSRX elevatorMotor = null;
	// public double holdPositionEncoderCounts;
	public double holdPositionInches;

	// public static DigitalInput elevatorSwitch;
	public static AnalogTrigger elevatorSwitch;
	public boolean brakeState;

	public boolean elevatorTooHigh;
	public boolean elevatorTooLow;
	public boolean moveIsUp;
	public boolean moveIsDown;
	public boolean elevatorOnSwitch;

	public double elevatorTargetPosition;
	private boolean switchWasSeen;
	// public boolean elevatorMotionDown;
	public double lastHoldPositionInches;
	private int elevatorHiCurrent;
	public int testGPButton;
	public boolean elevatorMoveInProgress;
	private int switchCounter;

	public Elevator() {
		elevatorMotor = new TalonSRX(RobotMap.ELEVATOR_MOTOR);
		elevatorMotor.configFactoryDefault();
		elevatorMotor.setInverted(true);
		elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		elevatorMotor.setSensorPhase(true);
		elevatorMotor.setNeutralMode(NeutralMode.Brake);
		elevatorMotor.configVoltageCompSaturation(12, 0);
		elevatorMotor.enableVoltageCompensation(true);
		elevatorSwitch = new AnalogTrigger(RobotMap.ELEVATOR_TRAVEL_SWITCH);
		elevatorSwitch.setAveraged(true);
		elevatorSwitch.setLimitsVoltage(1.0, 5.0);

		// elevatorSwitch = new DigitalInput(RobotMap.ELEVATOR_TRAVEL_SWITCH);
		// Put methods for controlling this subsystem
		// here. Call these from Commands.
	}

	public void initDefaultCommand() {
		// setDefaultCommand(new RunElevatorFromGamepad());
		setDefaultCommand(new HoldElevatorPositionMotionMagic());
	}

	public int getElevatorEncoderPosition() {
		return elevatorMotor.getSelectedSensorPosition(0);
	}

	public double getElevatorPositionInches() {
		return elevatorMotor.getSelectedSensorPosition(0) / Constants.ELEVATOR_ENCODER_COUNTS_PER_INCH;
	}

	public int getElevatorEncoderSpeedCountsPer100mS() {
		return elevatorMotor.getSelectedSensorVelocity(0);
	}

	public double getElevatorSpeedInchesPerSecond() {
		return getElevatorEncoderSpeedCountsPer100mS() / Constants.ELEVATOR_IN_PER_SEC_TO_ENC_CTS_PER_100MS;
	}

	public void magicMotionElevator(double distance, double speedIPS) {
		// elevator motor 775 Pro with 70:1 gear reduction and a 4096 count encoder
		//
		// Motor data 18000 rpm = 300 rps = 30 revs/100ms
		//
		//
		// measured rate at 100% was 1500
		//
		// Use measured rate not theoretical so 100% Kf would be 1023/1500 =.7
		//
		//
		//
		// For error of 1 inch, to add another 2% of motor
		// output. p-gain .02 x 1023 / (341) = .06
		//
		// start P-gain = .06
		//

		elevatorTargetPosition = distance;

		/*
		 * set acceleration and cruise velocity - see documentation accel is in units of
		 * enc cts per 100ms per second so to accelerate in 1/2 second, use velocity x 2
		 */

		int cruiseVelocity = (int) (speedIPS * Constants.ELEVATOR_IN_PER_SEC_TO_ENC_CTS_PER_100MS);

		int acceleration = cruiseVelocity * 2;

		elevatorMotor.configMotionCruiseVelocity(cruiseVelocity, 0);
		elevatorMotor.configMotionAcceleration(acceleration, 0);
		elevatorMotor.set(ControlMode.MotionMagic, distance * Constants.ELEVATOR_ENCODER_COUNTS_PER_INCH);
	}

	public boolean inPosition() {
		return Math.abs(elevatorTargetPosition - getElevatorPositionInches()) < Constants.ELEVATOR_IN_POSITION_BAND;
	}

	public void runElevatorMotor(double speed) {
		// if (elevatorTooLow && speed < 0)
		// speed = 0;
		// if (elevatorTooHigh && speed > 0)
		// speed = 0;
		SmartDashboard.putNumber("speed", speed);
		elevatorMotor.set(ControlMode.PercentOutput, speed);
	}

	public void resetElevatorPosition() {
		elevatorMotor.setSelectedSensorPosition(0, 0, 0);
		holdPositionInches = getElevatorPositionInches();
	}

	public void updateStatus() {

		elevatorOnSwitch = !elevatorSwitch.getInWindow();
		// check for elevator unable to reach position for 250 * 20ms = 5 sec
		if (elevatorMotor.getOutputCurrent() > 6)
			elevatorHiCurrent++;
		else
			elevatorHiCurrent = 0;
		if (elevatorHiCurrent > 250)
			holdPositionInches = getElevatorPositionInches();

		elevatorTooLow = getElevatorPositionInches() <= Constants.ELEVATOR_MIN_HEIGHT;
		elevatorTooHigh = getElevatorPositionInches() >= Constants.ELEVATOR_MAX_HEIGHT;
		if (elevatorOnSwitch)
			switchCounter++;
		else
			switchCounter = 0;

		if (switchCounter > 10 && !switchWasSeen) {
			resetElevatorPosition();
			holdPositionInches = getElevatorPositionInches();
			switchWasSeen = true;
		}
		if (switchWasSeen)
			switchWasSeen = elevatorOnSwitch;
		SD.putN2("Elevator Inches", getElevatorPositionInches());
		SmartDashboard.putBoolean("Elevator Too Low", elevatorTooLow);
		SmartDashboard.putBoolean("Elevator Too High", elevatorTooHigh);
		SD.putN1("Elevator Target", elevatorTargetPosition);
		SD.putN1("Elevator Speed IPS", getElevatorSpeedInchesPerSecond());
		SD.putN1("Elev Talon Temp", elevatorMotor.getTemperature());
		SmartDashboard.putBoolean("Elevator Switch", elevatorOnSwitch);

		if (AutoChoosers.debugChooser.getSelected() == 3) {
			SD.putN1("Elevator Amps", elevatorMotor.getOutputCurrent());
			SmartDashboard.putBoolean("Elev In Pos", inPosition());
			SmartDashboard.putNumber("Elevator Encoder", elevatorMotor.getSelectedSensorPosition(0));
			SmartDashboard.putNumber("Elevator EncCtsPer100ms", elevatorMotor.getSelectedSensorVelocity(0));
			SD.putN1("Elevator Hold", holdPositionInches);
			SD.putN1("Elevator Last Hold", lastHoldPositionInches);
			SD.putN1("Elevator Pct V", elevatorMotor.getMotorOutputPercent());
			SD.putN1("ElI", elevatorMotor.getIntegralAccumulator(0));
			SmartDashboard.putBoolean("Switch Was Seen", switchWasSeen);
			SmartDashboard.putNumber("ELHIAMPS", elevatorHiCurrent);

		}

	}
}
