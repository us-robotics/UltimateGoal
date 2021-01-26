package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Autos.DrivetrainAuto;
import org.firstinspires.ftc.teamcode.Autos.LauncherAuto;
import org.firstinspires.ftc.teamcode.Autos.WobbleGrabberAuto;

import FTCEngine.Core.Auto.JobSequence;
import FTCEngine.Core.Behavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Math.Vector2;

public class PowerShots extends Behavior
{
	/**
	 * NOTE: Do not configure the electronics in the constructor, do them in the awake method!
	 *
	 * @param opMode
	 */
	public PowerShots(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);
		opMode.input.registerButton(Input.Source.CONTROLLER_1, Input.Button.X);
	}

	@Override
	public void update()
	{
		super.update();

		if (opMode.input.getButtonDown(Input.Source.CONTROLLER_1, Input.Button.X))
		{
			if (opMode.hasSequence()) opMode.assignSequence(null);
			else opMode.assignSequence(new PowerShotsSequence(opMode));
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

			execute(launcher, new LauncherAuto.Prime(true));
			execute(drivetrain, new DrivetrainAuto.Move(new Vector2(24f, 0f)));

			//Launch ring 1
			execute(launcher, new LauncherAuto.Hit(true));
			wait(0.75f);

			execute(launcher, new LauncherAuto.Hit(false));
			execute(drivetrain, new DrivetrainAuto.Move(new Vector2(6f, 0f)));

			//Launch ring 2
			execute(launcher, new LauncherAuto.Hit(true));
			wait(0.75f);

			execute(launcher, new LauncherAuto.Hit(false));
			execute(drivetrain, new DrivetrainAuto.Move(new Vector2(6f, 0f)));

			//Launch ring 3
			execute(launcher, new LauncherAuto.Hit(true));
			wait(0.75f);

			execute(launcher, new LauncherAuto.Hit(false));
			execute(launcher, new LauncherAuto.Prime(false));
		}
	}
}
