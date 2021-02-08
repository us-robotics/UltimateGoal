package org.firstinspires.ftc.teamcode.Iteration4;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Behaviors.CameraVision;
import org.firstinspires.ftc.teamcode.Behaviors.ColorSensors;
import org.firstinspires.ftc.teamcode.Behaviors.DistanceSensors;
import org.firstinspires.ftc.teamcode.Behaviors.Drivetrain;
import org.firstinspires.ftc.teamcode.Behaviors.InertialMeasurementUnit;
import org.firstinspires.ftc.teamcode.Behaviors.Launcher;

import java.util.List;

import FTCEngine.Core.Auto.ConfigOption;
import FTCEngine.Core.Auto.JobSequence;
import FTCEngine.Core.Behavior;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Math.Vector2;

@Autonomous(name = "Ultimate Goal AutoColorSensorsTest")
public class AutoColorSensorsTest extends OpModeBase
{
	@Override
	protected void addBehaviors(List<Behavior> behaviorList)
	{
		behaviorList.add(new Drivetrain(this));
		behaviorList.add(new InertialMeasurementUnit(this));
		behaviorList.add(new ColorSensors(this));
		behaviorList.add(new DistanceSensors(this));
		behaviorList.add(new Launcher(this));
	}

	@Override
	protected void appendConfigOptions(List<ConfigOption> options)
	{

	}

	@Override
	public void init()
	{
		super.init();
		assignSequence(new Sequence(this));
	}

	private static class Sequence extends JobSequence
	{
		public Sequence(OpModeBase opMode)
		{
			super(opMode);
		}

		@Override
		protected void queueJobs()
		{
			Drivetrain drivetrain = opMode.getBehavior(Drivetrain.class);
			Launcher launcher = opMode.getBehavior(Launcher.class);

			execute(launcher, new Launcher.Prime(Launcher.HIGH_POWER, true));

			execute(launcher, new Launcher.Launch(false));
			execute(drivetrain, new Drivetrain.Trace(56f));
			execute(drivetrain, new Drivetrain.Move(new Vector2(0f, 4f)));

			execute(launcher, new Launcher.Launch(true));
			wait(0.75f);

			execute(launcher, new Launcher.Launch(false));
			execute(drivetrain, new Drivetrain.Trace(72f));
			execute(drivetrain, new Drivetrain.Move(new Vector2(0f, 4f)));

			execute(launcher, new Launcher.Launch(true));
			wait(0.75f);

			execute(launcher, new Launcher.Launch(false));
			execute(drivetrain, new Drivetrain.Trace(88f));
			execute(drivetrain, new Drivetrain.Move(new Vector2(0f, 4f)));

			execute(launcher, new Launcher.Launch(true));
			wait(0.75f);

			execute(launcher, new Launcher.Prime(Launcher.HIGH_POWER, false));
		}
	}
}
