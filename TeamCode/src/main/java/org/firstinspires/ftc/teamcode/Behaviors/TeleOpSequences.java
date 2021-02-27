package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Iteration4.CommonSequence;

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

		opMode.input.registerButton(Input.Source.CONTROLLER_1, Input.Button.X);
		opMode.input.registerButton(Input.Source.CONTROLLER_1, Input.Button.LEFT_BUMPER);
		opMode.input.registerButton(Input.Source.CONTROLLER_1, Input.Button.RIGHT_BUMPER);

		opMode.input.registerButton(Input.Source.CONTROLLER_1, Input.Button.DPAD_LEFT);
		opMode.input.registerButton(Input.Source.CONTROLLER_1, Input.Button.DPAD_RIGHT);
		opMode.input.registerButton(Input.Source.CONTROLLER_1, Input.Button.DPAD_UP);
	}

	@Override
	public void update()
	{
		super.update();

		if (opMode.input.getButtonDown(Input.Source.CONTROLLER_1, Input.Button.X)) opMode.assignSequence(null);

		if (!opMode.hasSequence())
		{
			if (opMode.input.getButtonDown(Input.Source.CONTROLLER_1, Input.Button.LEFT_BUMPER)) opMode.assignSequence(new PowerShotsSequence(opMode));
			if (opMode.input.getButtonDown(Input.Source.CONTROLLER_1, Input.Button.RIGHT_BUMPER)) opMode.assignSequence(new LaunchAlignmentSequence(opMode));

			if (opMode.input.getButtonDown(Input.Source.CONTROLLER_1, Input.Button.DPAD_LEFT)) opMode.assignSequence(new RotateSequence(opMode, 90f));
			if (opMode.input.getButtonDown(Input.Source.CONTROLLER_1, Input.Button.DPAD_RIGHT)) opMode.assignSequence(new RotateSequence(opMode, -90f));
			if (opMode.input.getButtonDown(Input.Source.CONTROLLER_1, Input.Button.DPAD_UP)) opMode.assignSequence(new RotateSequence(opMode, 180f));
		}
	}

	private static class PowerShotsSequence extends CommonSequence
	{
		public PowerShotsSequence(OpModeBase opMode)
		{
			super(opMode);
		}

		@Override
		protected void queueJobs()
		{

//			Drivetrain drivetrain = opMode.getBehavior(Drivetrain.class);
//			execute(drivetrain, new Drivetrain.Obstacle(56f));

			powerShots();
		}
	}

	private static class LaunchAlignmentSequence extends JobSequence
	{
		public LaunchAlignmentSequence(OpModeBase opMode)
		{
			super(opMode);
		}

		@Override
		protected void queueJobs()
		{
			Drivetrain drivetrain = opMode.getBehavior(Drivetrain.class);

			execute(drivetrain, new Drivetrain.Obstacle(4f));
			execute(drivetrain, new Drivetrain.Drive(Vector2.right, 0.8f));

			wait(0.3f);

			execute(drivetrain, new Drivetrain.Reset());
			execute(drivetrain, new Drivetrain.Drive(Vector2.zero));

			execute(drivetrain, new Drivetrain.Line());
			execute(drivetrain, new Drivetrain.Drive(Vector2.right, 0.8f));

			wait(0.3f);

			execute(drivetrain, new Drivetrain.Reset());
			execute(drivetrain, new Drivetrain.Drive(Vector2.zero));
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
