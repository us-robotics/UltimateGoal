package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import FTCEngine.Core.Auto.JobSequence;
import FTCEngine.Core.Behavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Math.Vector2;

public class TeleOpSequences extends Behavior
{
	/**
	 * NOTE: Do not configure the electronics in the constructor, do them in the awake method!
	 *
	 * @param opMode
	 */
	public TeleOpSequences(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);

		opMode.input.registerButton(Input.Source.CONTROLLER_1, Input.Button.RIGHT_BUMPER);
		opMode.input.registerButton(Input.Source.CONTROLLER_1, Input.Button.LEFT_BUMPER);

		opMode.input.registerButton(Input.Source.CONTROLLER_1, Input.Button.DPAD_LEFT);
		opMode.input.registerButton(Input.Source.CONTROLLER_1, Input.Button.DPAD_RIGHT);
		opMode.input.registerButton(Input.Source.CONTROLLER_1, Input.Button.DPAD_UP);
	}

	@Override
	public void update()
	{
		super.update();

		if (opMode.input.getButtonDown(Input.Source.CONTROLLER_1, Input.Button.RIGHT_BUMPER)) opMode.assignSequence(null);

		if (!opMode.hasSequence())
		{
			if (opMode.input.getButtonDown(Input.Source.CONTROLLER_1, Input.Button.LEFT_BUMPER)) opMode.assignSequence(new PowerShotsSequence(opMode));

			//NOTE: test these out, they should now work
			if (opMode.input.getButtonDown(Input.Source.CONTROLLER_1, Input.Button.DPAD_LEFT)) opMode.assignSequence(new RotateSequence(opMode, -90f));
			if (opMode.input.getButtonDown(Input.Source.CONTROLLER_1, Input.Button.DPAD_RIGHT)) opMode.assignSequence(new RotateSequence(opMode, 90f));
			if (opMode.input.getButtonDown(Input.Source.CONTROLLER_1, Input.Button.DPAD_UP)) opMode.assignSequence(new RotateSequence(opMode, 180f));
		}
	}

	private static class PowerShotsSequence extends JobSequence
	{
		public PowerShotsSequence(OpModeBase opMode)
		{
			super(opMode);
		}

		@Override
		protected void queueJobs()
		{
			Drivetrain drivetrain = opMode.getBehavior(Drivetrain.class);
			Launcher launcher = opMode.getBehavior(Launcher.class);

			execute(launcher, new Launcher.Prime(Launcher.SHOT_POWER, true));
			execute(drivetrain, new Drivetrain.Move(new Vector2(-15f, 0f), 0.55f));

			wait(0.75f);

			//Launch ring 1
			execute(launcher, new Launcher.Launch(true));
			wait(1f);

			execute(launcher, new Launcher.Launch(false));
			execute(drivetrain, new Drivetrain.Move(new Vector2(-9.25f, 0f), 0.55f));

			wait(0.75f);

			//Launch ring 2
			execute(launcher, new Launcher.Launch(true));
			wait(1f);

			execute(launcher, new Launcher.Launch(false));
			execute(launcher, new Launcher.Prime(Launcher.SHOT_POWER - 0.01f, true));
			execute(drivetrain, new Drivetrain.Move(new Vector2(-9.25f, 0f), 0.55f));
		}
	}

	private static class RotateSequence extends JobSequence
	{
		public RotateSequence(OpModeBase opMode, float angle)
		{
			super(opMode);
			this.angle = angle;
		}

		private final float angle;

		@Override
		protected void queueJobs()
		{
			Drivetrain drivetrain = opMode.getBehavior(Drivetrain.class);
			execute(drivetrain, new Drivetrain.Rotate(angle));
		}
	}
}
