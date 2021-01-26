package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autos.DrivetrainAuto;
import org.firstinspires.ftc.teamcode.Autos.IntakeAuto;
import org.firstinspires.ftc.teamcode.Autos.LauncherAuto;
import org.firstinspires.ftc.teamcode.Autos.WobbleGrabberAuto;
import org.firstinspires.ftc.teamcode.Behaviors.InertialMeasurementUnit;
import org.firstinspires.ftc.teamcode.Behaviors.Drivetrain;
import org.firstinspires.ftc.teamcode.Behaviors.Intake;
import org.firstinspires.ftc.teamcode.Behaviors.Launcher;
import org.firstinspires.ftc.teamcode.Behaviors.WobbleGrabber;

import java.util.List;

import FTCEngine.Core.Auto.ConfigOption;
import FTCEngine.Core.Auto.JobSequence;
import FTCEngine.Core.Behavior;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Math.Vector2;

@Autonomous(name = "MecanumI4 Auto")
public class MecanumI4Auto extends OpModeBase
{
	@Override
	public void addBehaviors(List<Behavior> behaviorList)
	{
		behaviorList.add(new Drivetrain(this));
		behaviorList.add(new DrivetrainAuto(this));
		behaviorList.add(new WobbleGrabber(this));
		behaviorList.add(new WobbleGrabberAuto(this));
		behaviorList.add(new Launcher(this));
		behaviorList.add(new LauncherAuto(this));
		behaviorList.add(new Intake(this));
		behaviorList.add(new IntakeAuto(this));
		behaviorList.add(new InertialMeasurementUnit(this));
	}

	@Override
	protected void appendConfigOptions(List<ConfigOption> options)
	{

	}

	@Override
	public void init()
	{
		super.init();
		assignSequence(new AutoSequence(this));
	}

	private static class AutoSequence extends JobSequence
	{
		public AutoSequence(OpModeBase opMode)
		{
			super(opMode);
		}

		@Override
		protected void queueJobs()
		{
			DrivetrainAuto drivetrain = opMode.getBehavior(DrivetrainAuto.class);
			WobbleGrabberAuto wobbleGrabber = opMode.getBehavior(WobbleGrabberAuto.class);
			LauncherAuto launcher = opMode.getBehavior(LauncherAuto.class);
			IntakeAuto intake = opMode.getBehavior(IntakeAuto.class);

			grabWobble(10f, 9f);

			execute(drivetrain, new DrivetrainAuto.Move(new Vector2(0f, 80f)));
			execute(wobbleGrabber, new WobbleGrabberAuto.Move(0));

			execute(drivetrain, new DrivetrainAuto.Move(new Vector2(20f, 0f)));
			execute(drivetrain, new DrivetrainAuto.Rotate(180f));

			execute(wobbleGrabber, new WobbleGrabberAuto.Move(1));
			wait(0.4f);

			//Hit wall to Reset Y
			execute(drivetrain, new DrivetrainAuto.Move(new Vector2(0f, -36f)));
			execute(drivetrain, new DrivetrainAuto.Drive(new Vector2(0f, -1f), 0.5f));
			execute(wobbleGrabber, new WobbleGrabberAuto.Move(0));
			wait(0.7f);

			execute(drivetrain, new DrivetrainAuto.Drive(Vector2.zero));
			execute(wobbleGrabber, new WobbleGrabberAuto.Move(-1, true));

			execute(drivetrain, new DrivetrainAuto.Move(new Vector2(-10f, 0f)));
			execute(wobbleGrabber, new WobbleGrabberAuto.Move(-1));

			//Forward to launch position
			execute(launcher, new LauncherAuto.Prime(Launcher.HIGH_POWER, true));
			execute(drivetrain, new DrivetrainAuto.Move(new Vector2(0f, 62f)));

			wait(0.2f);

			execute(drivetrain, new DrivetrainAuto.Move(new Vector2(24f, 0f)));
			execute(drivetrain, new DrivetrainAuto.Drive(new Vector2(1f, 0f), 0.55f));

			wait(0.7f);
			execute(drivetrain, new DrivetrainAuto.Drive(Vector2.zero));

			//Launch ring 1
			execute(launcher, new LauncherAuto.Hit(true));
			wait(0.75f);

			execute(launcher, new LauncherAuto.Hit(false));
			wait(0.75f);

			//Launch ring 2
			execute(launcher, new LauncherAuto.Hit(true));
			wait(0.75f);

			execute(launcher, new LauncherAuto.Hit(false));
			execute(launcher, new LauncherAuto.Prime(Launcher.HIGH_POWER - 0.005f, true));

			execute(intake, new IntakeAuto.Run(-0.45f));
			wait(0.75f);
			execute(intake, new IntakeAuto.Run(0f));

			//Launch ring 3
			execute(launcher, new LauncherAuto.Hit(true));
			wait(0.75f);

			execute(launcher, new LauncherAuto.Hit(false));
			execute(launcher, new LauncherAuto.Prime(0f, false));

			//Rotate back to starting rotation
			execute(drivetrain, new DrivetrainAuto.Move(new Vector2(-16f, 0f)));
			execute(drivetrain, new DrivetrainAuto.Move(new Vector2(0f, 48f)));
			execute(drivetrain, new DrivetrainAuto.Rotate(180f));

			//Hit wall reset Y
			execute(drivetrain, new DrivetrainAuto.Drive(new Vector2(0f, -1f), 0.5f));

			wait(0.75f);
			execute(drivetrain, new DrivetrainAuto.Drive(Vector2.zero));
			execute(drivetrain, new DrivetrainAuto.Move(new Vector2(6f, 0f)));
			execute(drivetrain, new DrivetrainAuto.Move(new Vector2(0f, 12f)));

			grabWobble(4f, 10f);

			execute(drivetrain, new DrivetrainAuto.Move(new Vector2(-12f, 0f)));
			execute(drivetrain, new DrivetrainAuto.Move(new Vector2(0f, 84f)));

			//Drop second wobble
			execute(wobbleGrabber, new WobbleGrabberAuto.Move(-1));
			wait(0.45f);

			execute(wobbleGrabber, new WobbleGrabberAuto.Move(0, true));
			wait(0.5f);

			execute(drivetrain, new DrivetrainAuto.Move(new Vector2(-12f, 0f)));
			execute(wobbleGrabber, new WobbleGrabberAuto.Move(-1));
			execute(drivetrain, new DrivetrainAuto.Move(new Vector2(0f, -24f)));
		}

		private void grabWobble(float leftDistance, float rightDistance)
		{
			DrivetrainAuto drivetrain = opMode.getBehavior(DrivetrainAuto.class);
			WobbleGrabberAuto wobbleGrabber = opMode.getBehavior(WobbleGrabberAuto.class);

			execute(wobbleGrabber, new WobbleGrabberAuto.Move(-1));
			execute(drivetrain, new DrivetrainAuto.Move(new Vector2(-leftDistance, 0f), 0.75f));

			execute(wobbleGrabber, new WobbleGrabberAuto.Reset());
			execute(wobbleGrabber, new WobbleGrabberAuto.Move(1, true));

			wait(0.34f);
			execute(wobbleGrabber, new WobbleGrabberAuto.Move(0, true));
			execute(drivetrain, new DrivetrainAuto.Move(new Vector2(rightDistance, 0f), 0.65f));

			execute(wobbleGrabber, new WobbleGrabberAuto.Move(0));
			wait(0.3f);

			execute(drivetrain, new DrivetrainAuto.Move(new Vector2(-4f, 0f), 0.75f));
			execute(wobbleGrabber, new WobbleGrabberAuto.Move(1));
		}
	}
}