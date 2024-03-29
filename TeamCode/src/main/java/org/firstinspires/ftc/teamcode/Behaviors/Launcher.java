package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import FTCEngine.Core.Auto.AutoBehavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Math.Mathf;

public class Launcher extends AutoBehavior<Launcher.Job>
{
	/**
	 * NOTE: Do not configure the electronics in the constructor, do them in the awake method
	 *
	 * @param opMode
	 */
	public Launcher(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);

		flywheel = hardwareMap.dcMotor.get("flywheel");
		lift = hardwareMap.dcMotor.get("lift");
		locker = hardwareMap.servo.get("trigger");

		flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		lift.setDirection(DcMotorSimple.Direction.REVERSE);

		touchUpper = hardwareMap.touchSensor.get("sensorTop");
		touchLower = hardwareMap.touchSensor.get("sensorBottom");

		opMode.input.registerButton(Input.Source.CONTROLLER_2, Input.Button.LEFT_BUMPER);

		opMode.input.registerButton(Input.Source.CONTROLLER_2, Input.Button.DPAD_UP);
		opMode.input.registerButton(Input.Source.CONTROLLER_2, Input.Button.DPAD_DOWN);

		opMode.input.registerButton(Input.Source.CONTROLLER_2, Input.Button.DPAD_RIGHT);
		opMode.input.registerButton(Input.Source.CONTROLLER_2, Input.Button.DPAD_LEFT);

		apply();
	}

	private DcMotor flywheel;
	private DcMotor lift;

	private TouchSensor touchUpper;
	private TouchSensor touchLower;

	private Servo locker;

	public static final float HIGH_POWER = 0.7975f; //Power for high goal
	public static final float SHOT_POWER = 0.6725f; //Power for power shots
	public static final float UNSTUCK_POWER = -0.45f; //Power for power shots

	private float flywheelPower = HIGH_POWER;

	private boolean primed = false;
	private float liftPower = -1f;

	@Override
	public void update()
	{
		super.update();

		if (!opMode.hasSequence())
		{
			if (opMode.input.getButtonDown(Input.Source.CONTROLLER_2, Input.Button.LEFT_BUMPER)) primed = !primed;
			float direction = -opMode.input.getVector(Input.Source.CONTROLLER_2, Input.Button.RIGHT_JOYSTICK).x;

			if (direction > -0.3f) direction = 0f;
			boolean rest = Mathf.almostEquals(direction, 0f);

			if (direction > 0f && !rest) liftPower = direction;
			else liftPower = rest && liftPower >= 0f ? 0f : -1f;

			final float POWER_CHANGE_RATE = 0.0025f;

			if (opMode.input.getButtonDown(Input.Source.CONTROLLER_2, Input.Button.DPAD_UP)) flywheelPower += POWER_CHANGE_RATE;
			if (opMode.input.getButtonDown(Input.Source.CONTROLLER_2, Input.Button.DPAD_DOWN)) flywheelPower -= POWER_CHANGE_RATE;

			if (opMode.input.getButtonDown(Input.Source.CONTROLLER_2, Input.Button.DPAD_RIGHT)) flywheelPower = HIGH_POWER;
			if (opMode.input.getButtonDown(Input.Source.CONTROLLER_2, Input.Button.DPAD_LEFT)) flywheelPower = SHOT_POWER;
			if (opMode.input.getTrigger(Input.Source.CONTROLLER_2, Input.Button.LEFT_TRIGGER) > 0.5f) flywheelPower = UNSTUCK_POWER;
		}

		opMode.debug.addData("Flywheel Power", flywheelPower);
		apply();
	}

	private void apply()
	{
		flywheel.setPower(primed ? flywheelPower : 0d);

		if (liftPower < 0f)
		{
			locker.setPosition(0.8d); //Released
			lift.setPower(touchLower.isPressed() ? 0f : liftPower);
		}
		else
		{
			locker.setPosition(1d); //Locked
			lift.setPower(touchUpper.isPressed() ? 0f : liftPower);
		}
	}

	@Override
	protected void updateJob()
	{
		Launcher.Job job = getCurrentJob();

		if (job instanceof Launcher.Prime)
		{
			Launcher.Prime prime = (Launcher.Prime)job;
			if (prime.power >= 0f) flywheelPower = prime.power;

			primed = prime.primed;
			prime.finishJob();
		}

		if (job instanceof Lift)
		{
			Lift lift = (Lift)job;

			liftPower = lift.direction;
			lift.finishJob();
		}
	}

	static abstract class Job extends FTCEngine.Core.Auto.Job
	{
	}

	public static class Prime extends Launcher.Job
	{
		public Prime(float power, boolean primed)
		{
			this.power = power;
			this.primed = primed;
		}

		public Prime(boolean primed)
		{
			power = -1f; //Do not change power;
			this.primed = primed;
		}

		public final float power;
		public final boolean primed;
	}

	public static class Lift extends Launcher.Job
	{
		public Lift(int direction)
		{
			this.direction = direction;
		}

		public final int direction;
	}
}
