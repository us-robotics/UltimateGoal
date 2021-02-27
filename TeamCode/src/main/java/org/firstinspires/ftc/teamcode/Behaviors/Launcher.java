package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import FTCEngine.Core.Auto.AutoBehavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;

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

		flywheel = hardwareMap.dcMotor.get("flywheel"); //Rename launcher to flywheel in the config file
		trigger = hardwareMap.servo.get("trigger");

		flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		opMode.input.registerButton(Input.Source.CONTROLLER_2, Input.Button.LEFT_BUMPER);
		opMode.input.registerButton(Input.Source.CONTROLLER_2, Input.Button.RIGHT_BUMPER);

		opMode.input.registerButton(Input.Source.CONTROLLER_2, Input.Button.DPAD_UP);
		opMode.input.registerButton(Input.Source.CONTROLLER_2, Input.Button.DPAD_DOWN);

		opMode.input.registerButton(Input.Source.CONTROLLER_2, Input.Button.DPAD_RIGHT);
		opMode.input.registerButton(Input.Source.CONTROLLER_2, Input.Button.DPAD_LEFT);

		apply();
	}

	private DcMotor flywheel;
	private Servo trigger;

	public static final float HIGH_POWER = 0.80f; //Power for high goal
	public static final float SHOT_POWER = 0.7275f; //Power for power shots

	private float power = HIGH_POWER;

	private boolean primed;
	private boolean hit;

	@Override
	public void update()
	{
		super.update();

		if (!opMode.hasSequence())
		{
			if (opMode.input.getButtonDown(Input.Source.CONTROLLER_2, Input.Button.LEFT_BUMPER)) primed = !primed;
			hit = opMode.input.getButton(Input.Source.CONTROLLER_2, Input.Button.RIGHT_BUMPER);

			final float POWER_CHANGE_RATE = 0.0025f;

			if (opMode.input.getButtonDown(Input.Source.CONTROLLER_2, Input.Button.DPAD_UP)) power += POWER_CHANGE_RATE;
			if (opMode.input.getButtonDown(Input.Source.CONTROLLER_2, Input.Button.DPAD_DOWN)) power -= POWER_CHANGE_RATE;

			if (opMode.input.getButtonDown(Input.Source.CONTROLLER_2, Input.Button.DPAD_RIGHT)) power = HIGH_POWER;
			if (opMode.input.getButtonDown(Input.Source.CONTROLLER_2, Input.Button.DPAD_LEFT)) power = SHOT_POWER;
		}

		opMode.debug.addData("Launcher Power", power);
		apply();
	}

	private void apply()
	{
		flywheel.setPower(primed ? power : 0d);
		trigger.setPosition(hit ? 0d : 0.52d);
	}

	@Override
	protected void updateJob()
	{
		Launcher.Job job = getCurrentJob();

		if (job instanceof Launcher.Prime)
		{
			Launcher.Prime prime = (Launcher.Prime)job;

			power = prime.power;
			primed = prime.primed;

			prime.finishJob();
		}

		if (job instanceof Launch)
		{
			Launch launch = (Launch)job;

			hit = launch.launch;
			launch.finishJob();
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

		public final float power;
		public final boolean primed;
	}

	public static class Launch extends Launcher.Job
	{
		public Launch(boolean launch)
		{
			this.launch = launch;
		}

		public final boolean launch;
	}
}
