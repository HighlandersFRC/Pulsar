package org.usfirst.frc4499.robot.subsystems;
import java.util.ArrayList;

import org.usfirst.frc4499.robot.Constants;
import org.usfirst.frc4499.robot.Robot;
import org.usfirst.frc4499.robot.RobotMap;
import org.usfirst.frc4499.robot.commands.ElevatorSetPositionMM;
import org.usfirst.frc4499.robot.commands.presets.CargoHigh;
import org.usfirst.frc4499.robot.commands.presets.CargoLow;
import org.usfirst.frc4499.robot.commands.presets.CargoMid;
import org.usfirst.frc4499.robot.commands.presets.HatchHigh;
import org.usfirst.frc4499.robot.commands.presets.HatchLow;
import org.usfirst.frc4499.robot.commands.presets.HatchMid;
import org.usfirst.frc4499.robot.commands.presets.SetPositionElevatorSlider;
import org.usfirst.frc4499.robot.commands.presets.StowElevator;
import org.usfirst.frc4499.utility.ControlLoopable;
import org.usfirst.frc4499.utility.Loop;
import org.usfirst.frc4499.utility.MPTalonPIDController;
import org.usfirst.frc4499.utility.PIDParams;
import org.usfirst.frc4499.utility.TalonSRXEncoder;
import org.usfirst.frc4499.utility.TalonSRXFactory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

public class Elevator extends Subsystem implements ControlLoopable
{
	private static Elevator instance;

	public static enum ElevatorControlMode { MOTION_PROFILE, JOYSTICK_PID, JOYSTICK_MANUAL, MANUAL, MOTION_MAGIC};
	public static enum PlaceMode { HATCH, CARGO };

	// One revolution of the 30T Drive 1.880" PD pulley = Pi * PD inches = 36/24 revs due to pulleys * 34/24 revs due to gears * 36/12 revs due mag encoder gear on ball shifter * 4096 ticks
	public static final double ENCODER_TICKS_TO_INCHES = (1);


	private double periodMs;
	private double lastControlLoopUpdatePeriod = 0.0;
	private double lastControlLoopUpdateTimestamp = 0.0;
	// Defined speeds
	public static final double CLIMB_SPEED = -1.0;
	public static final double TEST_SPEED_UP = 0.3;
	public static final double TEST_SPEED_DOWN = -0.3;
	public static final double AUTO_ZERO_SPEED = -0.3;
	public static final double JOYSTICK_INCHES_PER_MS_HI = 35;
	public static final double JOYSTICK_INCHES_PER_MS_LO = 40;

	// Defined positions
	public static final double ZERO_POSITION_AUTON_FORWARD_INCHES = 8.0;
	public static final double ZERO_POSITION_INCHES = -0.25;
	public static final double NEAR_ZERO_POSITION_INCHES = 0.0;
	public static final double MIN_POSITION_INCHES = 0;
	public static final double MAX_POSITION_INCHES = 6000;
	public static final double AFTER_INTAKE_POSITION_INCHES = 4.0;


	// Motion profile max velocities and accel times
	public static final double MP_MAX_VELOCITY_INCHES_PER_SEC =  60;
	public static final double MP_T1 = 300;  // Fast = 300
	public static final double MP_T2 = 150;  // Fast = 150

	// Motor controllers
	private ArrayList<TalonSRXEncoder> motorControllers = new ArrayList<TalonSRXEncoder>();

	public TalonSRXEncoder motor1;
	private TalonSRX motor2;

	// PID controller and params
	private MPTalonPIDController mpController;

	public static int PID_SLOT = 0;
	public static int MM_SLOT = 1;
	public static int MP_SLOT = 2;

	private PIDParams mpPIDParams = new PIDParams(0.2, 0.0, 0.0, 0.0, 0.005, 0.0);
	private PIDParams pidPIDParamsHiGear = new PIDParams(0.075, 0.0, 0.0, 0.0, 0.0, 0.0);
	public static final double KF_UP = 0.05;//0.01;
	public static final double KF_DOWN = 0;// 0.0;
	public static final double P_Value = 1.5;// 2;
	public static final double I_Value = 0.00100;// 0.00030;
	public static final double D_Value = 100;// 200;
	public static final double F_Value = 0.50;	// 1023 / 1360 max speed (ticks/100ms)
	public static final double maxGravityComp = 0.00;
	public static final double RampRate = 0;// 0.0;
	public static final int A_value = 1500;
	public static final int CV_value = 3000;




	private PIDParams elevatorPIDParams = new PIDParams(P_Value, I_Value, D_Value, KF_DOWN);	// KF gets updated later
	public static final double PID_ERROR_INCHES = 10;
	LimitSwitchSource limitSwitchSource;

	// Pneumatics
	private Solenoid speedShift;

	//DPad Buttons
	static final int DPAD_UP = 0;
	static final int DPAD_RIGHT = 90;
	static final int DPAD_DOWN = 180;
	static final int DPAD_LEFT = 270;

	// Misc
	public static final double AUTO_ZERO_MOTOR_CURRENT = 4.0;
	private boolean isFinished;
	private ElevatorControlMode elevatorControlMode = ElevatorControlMode.MOTION_MAGIC;
	public PlaceMode placeMode = PlaceMode.HATCH;
	public double targetPositionInchesPID = 0;
	public double targetPositionInchesMM = 0;
	private boolean firstMpPoint;
	private double joystickInchesPerMs = JOYSTICK_INCHES_PER_MS_LO;
	private double p = 0;


	public Elevator() {
		try {
			motor1 = TalonSRXFactory.createTalonEncoder(RobotMap.ELEVATOR_MOTOR1_ID, (ENCODER_TICKS_TO_INCHES), false, FeedbackDevice.QuadEncoder);
			motor2 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.ELEVATOR_MOTOR2_ID, RobotMap.ELEVATOR_MOTOR1_ID);


			motor1.setInverted(false);
			motor2.setInverted(false);

//	        if (motor1.isSensorPresent(CANTalon.FeedbackDevice.QuadEncoder) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
//	            Driver.reportError("Could not detect elevator motor 1 encoder encoder!", false);
//	        }

			motor1.configNominalOutputForward(0, TalonSRXEncoder.TIMEOUT_MS);
			motor1.configNominalOutputReverse(0, TalonSRXEncoder.TIMEOUT_MS);
			motor1.configPeakOutputForward(1, TalonSRXEncoder.TIMEOUT_MS);
			motor1.configPeakOutputReverse(-1, TalonSRXEncoder.TIMEOUT_MS);

			motor1.selectProfileSlot(MM_SLOT, 0);
			motor1.config_kF(MM_SLOT, F_Value, TalonSRXEncoder.TIMEOUT_MS);
			motor1.config_kP(MM_SLOT, P_Value, TalonSRXEncoder.TIMEOUT_MS);
			motor1.config_kI(MM_SLOT, I_Value, TalonSRXEncoder.TIMEOUT_MS);
			motor1.config_kD(MM_SLOT, D_Value, TalonSRXEncoder.TIMEOUT_MS);
			motor1.setSensorPhase(true);

			motor1.configForwardLimitSwitchSource(limitSwitchSource, LimitSwitchNormal.NormallyOpen, 0);
			motor1.configReverseLimitSwitchSource(limitSwitchSource, LimitSwitchNormal.NormallyOpen, 0);
			motor1.setNeutralMode(NeutralMode.Brake);
    		motor2.setNeutralMode(NeutralMode.Brake);
			motor1.enableCurrentLimit(true);
			motorControllers.add(motor1);
			//motor1.setSelectedSensorPosition(0, , );


		}
		catch (Exception e) {
			System.err.println("An error occurred in the Elevator constructor");
		}
	}

	@Override
	public void initDefaultCommand() {
	}

	public void resetZeroPosition(double position) {
		mpController.resetZeroPosition(position);
	}

	public void resetEncoder(){
		motor1.setPosition(0);
		System.err.println("here");
		//targetPositionInchesMM = 0;
		//targetPositionInchesPID = 0;
	}

	private synchronized void setElevatorControlMode(ElevatorControlMode controlMode) {
		this.elevatorControlMode = controlMode;
	}

	private synchronized ElevatorControlMode getElevatorControlMode() {
		return this.elevatorControlMode;
	}

	public void setSpeed(double speed) {
		motor1.set(ControlMode.PercentOutput, speed);
		setElevatorControlMode(ElevatorControlMode.MANUAL);
	}

	public void setSpeedJoystick(double speed) {
		motor1.set(ControlMode.PercentOutput, speed);
		setElevatorControlMode(ElevatorControlMode.JOYSTICK_MANUAL);
	}
	public void setPositionMM(double targetPositionInches){
		motor1.set(ControlMode.MotionMagic, targetPositionInches);
		//System.err.println(motor1.getControlMode());
		motor1.selectProfileSlot(MM_SLOT, 0);
		setElevatorControlMode(ElevatorControlMode.MOTION_MAGIC);
		updatePositionMM(targetPositionInches);
		setFinished(false);
	}

	public double calcGravityCompensationAtCurrentPosition() {
		int ticks = motor1.getSelectedSensorPosition();
		double degreesFromDown = (ticks+920)*(360.0/(4096*3));
		double compensation = maxGravityComp * Math.sin(degreesFromDown*Math.PI/180);
		//System.err.println("comp(" + degreesFromDown + "^) = " + compensation);
		return compensation;
	}
	public void updatePositionMM(double targetPositionInches){
		targetPositionInchesMM = limitPosition(targetPositionInches);
		//double startPositionInches = motor1.getPositionWorld();
		double compensation = calcGravityCompensationAtCurrentPosition();
		//System.err.println("compensation = " + compensation);
		// motor1.set(ControlMode.MotionMagic, targetPositionInches);
		motor1.set(ControlMode.MotionMagic, targetPositionInches, DemandType.ArbitraryFeedForward, compensation);
		//System.err.println(motor1.getControlMode());
		motor1.configMotionCruiseVelocity(CV_value, TalonSRXEncoder.TIMEOUT_MS);
		motor1.configMotionAcceleration(A_value, TalonSRXEncoder.TIMEOUT_MS);


	}

	public void setPositionPID(double targetPositionInches) {
		motor1.set(ControlMode.Position, targetPositionInches);
		mpController.setPIDSlot(PID_SLOT);	//TODO: verify that motor's selectProfileSlot() should be called AFTER its control mode is set
		updatePositionPID(targetPositionInches);
		setElevatorControlMode(ElevatorControlMode.JOYSTICK_PID);
		setFinished(false);
	}

	public void updatePositionPID(double targetPositionInches) {
		targetPositionInchesPID = limitPosition(targetPositionInches);
		if (limitPosition(motor1.getPositionWorld()) == MIN_POSITION_INCHES){
			resetEncoder();
		}
		double startPositionInches = motor1.getPositionWorld();
		//mpController.setTarget(targetPositionInchesPID, targetPositionInchesPID > startPositionInches ? KF_UP : KF_DOWN);
		motor1.set(ControlMode.Position, targetPositionInches);
		motor1.configClosedloopRamp(0);
		//motor1.configPeakCurrentLimit(5);
		motor1.configContinuousCurrentLimit(2);
		motor1.config_kP(0, P_Value, TalonSRXEncoder.TIMEOUT_MS);
		motor1.config_kI(0, I_Value, TalonSRXEncoder.TIMEOUT_MS);
		motor1.config_kD(0, D_Value, TalonSRXEncoder.TIMEOUT_MS);
		motor1.config_kF(0, targetPositionInchesPID > startPositionInches ? KF_UP : KF_DOWN, TalonSRXEncoder.TIMEOUT_MS);
		//System.err.println(motor1.getControlMode());
		//System.err.print(motor1.getClosedLoopError());
	}

	public void setPositionMP(double targetPositionInches) {
		double startPositionInches = motor1.getPositionWorld();
		mpController.setMPTarget(startPositionInches, limitPosition(targetPositionInches), MP_MAX_VELOCITY_INCHES_PER_SEC, MP_T1, MP_T2);
		setFinished(false);
		firstMpPoint = true;
		setElevatorControlMode(ElevatorControlMode.MOTION_PROFILE);
 	}

	private double limitPosition(double targetPosition) {
		if (targetPosition < MIN_POSITION_INCHES) {
			return MIN_POSITION_INCHES;
		}
		if (targetPosition > MAX_POSITION_INCHES) {
			return MAX_POSITION_INCHES;
		}

		return targetPosition;
	}
	@Override
	public void setPeriodMs(long periodMs) {
		mpController = new MPTalonPIDController(periodMs, motorControllers);
		mpController.setPID(mpPIDParams, MP_SLOT);
		mpController.setPID(elevatorPIDParams, PID_SLOT);
		mpController.setPIDSlot(PID_SLOT);
		this.periodMs = periodMs;
	}

	private int lastDPadAngle = -1;
	public void dPadButtons(){
		int dPadAngle = Robot.oi.getOperatorController().getDpadAngle();
		if (dPadAngle == DPAD_UP && lastDPadAngle == -1){
			new CargoHigh().start();
		}			
		if (dPadAngle == DPAD_RIGHT && lastDPadAngle == -1){
			new CargoMid().start();
		}
		if (dPadAngle == DPAD_DOWN && lastDPadAngle == -1){
			new CargoLow().start();
		}

		if (dPadAngle == DPAD_LEFT && lastDPadAngle == -1){
			new StowElevator().start();
		}
		SmartDashboard.putNumber("DPad Angle", dPadAngle);
		lastDPadAngle = dPadAngle;
	}

	public synchronized void controlLoopUpdate() {
		// Measure *actual* update period
		double currentTimestamp = Timer.getFPGATimestamp();
		if (lastControlLoopUpdateTimestamp > 0.001)	{	// ie, if this is NOT the first time
			lastControlLoopUpdatePeriod = currentTimestamp - lastControlLoopUpdateTimestamp;
		}
		lastControlLoopUpdateTimestamp = currentTimestamp;

		dPadButtons();

		if (motor1.getSensorCollection().isRevLimitSwitchClosed()){
			resetEncoder();
		}

		// Do the update
		if (elevatorControlMode == ElevatorControlMode.JOYSTICK_MANUAL) {
			controlManualWithJoystick();
			//System.err.println(motor1.getControlMode());
		}
		else if (!isFinished) {
			if (elevatorControlMode == ElevatorControlMode.MOTION_PROFILE) {
				isFinished = mpController.controlLoopUpdate(getPositionInches());

			}
			if (elevatorControlMode == ElevatorControlMode.JOYSTICK_PID){
					controlPidWithJoystick();
					//System.err.println(motor1.getControlMode());
			}
			if (elevatorControlMode == ElevatorControlMode.MOTION_MAGIC){
				controlMMWithJoystick();
				//System.err.println(motor1.getControlMode());
			}

			/*else if (elevatorControlMode == ElevatorControlMode.MP_PATH_VELOCITY) {
				isFinished = mpPathVelocityController.controlLoopUpdate(getGyroAngleDeg());
			}
			else if (elevatorControlMode == ElevatorControlMode.ADAPTIVE_PURSUIT) {
				updatePose();
				isFinished = adaptivePursuitController.controlLoopUpdate(currentPose);
			}*/
		}
	}







	private void controlPidWithJoystick() {
		double joystickPosition = -Robot.oi.getOperatorController().getLeftYAxis();
		double deltaPosition = joystickPosition * joystickInchesPerMs;
		targetPositionInchesPID = targetPositionInchesPID + deltaPosition;
		updatePositionPID(targetPositionInchesPID);
	}
	private void controlMMWithJoystick() {
		double joystickPosition = -Robot.oi.getOperatorController().getLeftYAxis();
		double deltaPosition = joystickPosition * joystickInchesPerMs;
		targetPositionInchesMM = targetPositionInchesMM + deltaPosition;
		updatePositionMM(targetPositionInchesMM);
		//Robot.slider.targetPositionInchesPID = targetPositionInchesPID - (deltaPosition/3);
		//Robot.slider.updatePositionPID(Robot.slider.targetPositionInchesPID);


	}

	private void ControlWithJoystickhold(){
		double holdPosition = motor1.getPositionWorld();
		updatePositionPID(holdPosition);

	}

	private void controlManualWithJoystick() {
		double joyStickSpeed = -Robot.oi.getOperatorController().getLeftYAxis();
		setSpeedJoystick(joyStickSpeed);
	}
	/*
	public void setShiftState(ElevatorSpeedShiftState state) {

			joystickInchesPerMs = JOYSTICK_INCHES_PER_MS_HI;
			speedShift.set(true);
			mpController.setPID(pidPIDParamsHiGear, PID_SLOT);
		}
		else if(state == ElevatorSpeedShiftState.LO) {
			joystickInchesPerMs = JOYSTICK_INCHES_PER_MS_LO;
			speedShift.set(false);
			mpController.setPID(pidPIDParamsLoGear, PID_SLOT);
		}
	}

	public ElevatorSpeedShiftState getShiftState() {
		return shiftState;
	}
*/
	public double getPositionInches() {
		return motor1.getPositionWorld();
	}

//	public double getAverageMotorCurrent() {
//		return (Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_1_CAN_ID) + Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_2_CAN_ID) + Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_3_CAN_ID)) / 3;
//	}

	public double getAverageMotorCurrent() {
		return (motor1.getOutputCurrent() + motor2.getOutputCurrent()) / 2;
	}

	public synchronized boolean isFinished() {
		return isFinished;
	}

	public synchronized void setFinished(boolean isFinished) {
		this.isFinished = isFinished;
	}

	public double getPeriodMs() {
		return periodMs;
	}

	public void updateStatus(Robot.OperationMode operationMode) {
		//System.err.println("the encoder is right after this");
			try {

				SmartDashboard.putNumber("Elevator Ticks", motor1.getPositionWorld());
				//SmartDashboard.putNumber("Elevator Motor 1 Amps", motor1.getOutputCurrent());
				//SmartDashboard.putNumber("Elevator Motor 2 Amps", motor2.getOutputCurrent());
				//SmartDashboard.putNumber("sensor vel", motor1.getSelectedSensorVelocity());
				//SmartDashboard.putNumber("Elevator Average Amps", getAverageMotorCurrent());
				SmartDashboard.putNumber("Elevator Error", motor1.getClosedLoopError());
				SmartDashboard.putNumber("Elevator Amps", getAverageMotorCurrent());
				SmartDashboard.putNumber("Elevator Target MM", targetPositionInchesMM);
				//SmartDashboard.putNumber("elevator output", motor1.getMotorOutputPercent());
				//SmartDashboard.putNumber("Elevator Motor 1 Amps PDP", Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_1_CAN_ID));
				//SmartDashboard.putNumber("Elevator Motor 2 Amps PDP", Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_2_CAN_ID));
				//SmartDashboard.putNumber("Elevator Motor 3 Amps PDP", Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_3_CAN_ID));
				//SmartDashboard.putNumber("Elevator Target PID Position", targetPositionInchesPID);
			}
			catch (Exception e) {
				System.err.println("Elevator update status error" +e.getMessage());
			}

	}

	public static Elevator getInstance() {
		if(instance == null) {
			instance = new Elevator();
		}
		return instance;
	}
}
