package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Autos.DrivetrainAuto;
import org.firstinspires.ftc.teamcode.Autos.IntakeAuto;
import org.firstinspires.ftc.teamcode.Autos.LauncherAuto;

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
			DrivetrainAuto drivetrain = opMode.getBehavior(DrivetrainAuto.class);
			LauncherAuto launcher = opMode.getBehavior(LauncherAuto.class);

			execute(launcher, new LauncherAuto.Prime(Launcher.SHOT_POWER - 0.01f, true));
			execute(drivetrain, new DrivetrainAuto.Move(new Vector2(-24f, 0f), 0.55f));

			wait(0.75f);

			//Launch ring 1
			execute(launcher, new LauncherAuto.Hit(true));
			wait(1f);

			execute(launcher, new LauncherAuto.Hit(false));
			execute(drivetrain, new DrivetrainAuto.Move(new Vector2(-10f, 0f), 0.55f));

			wait(0.75f);

			//Launch ring 2
			execute(launcher, new LauncherAuto.Hit(true));
			wait(1f);

			execute(launcher, new LauncherAuto.Hit(false));
			execute(drivetrain, new DrivetrainAuto.Move(new Vector2(-10f, 0f), 0.55f));
		}
	}

	private static class RotateSequence extends JobSequence
	{
		public RotateSequence(OpModeBase opMode, float angle)
		{
			super(opMode);
			this.angle = angle;
		}

		private final float angle; //Doesnt work right now because it is assigned after the constructor

		@Override
		protected void queueJobs()
		{
			DrivetrainAuto drivetrain = opMode.getBehavior(DrivetrainAuto.class);
			execute(drivetrain, new DrivetrainAuto.Rotate(angle));
		}
	}
}
