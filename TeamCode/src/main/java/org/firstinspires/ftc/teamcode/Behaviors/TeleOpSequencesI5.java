package org.firstinspires.ftc.teamcode.Behaviors;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Iteration5.CommonSequence;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import FTCEngine.Core.Behavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;

public class TeleOpSequencesI5 extends Behavior
{
	/**
	 * NOTE: Do not configure the electronics in the constructor, do them in the awake method!
	 *
	 * @param opMode
	 */
	public TeleOpSequencesI5(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);

		opMode.input.registerButton(Input.Source.CONTROLLER_1, Input.Button.X);
		opMode.input.registerButton(Input.Source.CONTROLLER_1, Input.Button.Y);

		opMode.input.registerButton(Input.Source.CONTROLLER_1, Input.Button.LEFT_BUMPER);
		opMode.input.registerButton(Input.Source.CONTROLLER_1, Input.Button.RIGHT_BUMPER);
	}

	@Override
	public void update()
	{
		super.update();

		if (opMode.input.getButtonDown(Input.Source.CONTROLLER_1, Input.Button.X)) opMode.assignSequence(null);

		if (!opMode.hasSequence())
		{
			if (opMode.input.getButtonDown(Input.Source.CONTROLLER_1, Input.Button.Y)) opMode.assignSequence(new PowerShotsSequence(opMode));

			if (opMode.input.getButtonDown(Input.Source.CONTROLLER_1, Input.Button.LEFT_BUMPER)) opMode.assignSequence(new LaunchSequence(opMode, false));
			if (opMode.input.getButtonDown(Input.Source.CONTROLLER_1, Input.Button.RIGHT_BUMPER)) opMode.assignSequence(new LaunchSequence(opMode, true));
		}
	}

	private static class LaunchSequence extends CommonSequence
	{
		public LaunchSequence(OpModeBase opMode, boolean reset)
		{
			super(opMode);
			this.reset = reset;
		}

		private final boolean reset;

		@Override
		protected void queueJobs()
		{
			DrivetrainI5 drivetrain = opMode.getBehavior(DrivetrainI5.class);
			Launcher launcher = opMode.getBehavior(Launcher.class);
			SampleMecanumDrive drive = drivetrain.getDrive();

			execute(launcher, new Launcher.Prime(true));
			Pose2d launchPoint = new Pose2d(0, 0, Math.toRadians(180d)); //TODO

			if (reset) drive.setPoseEstimate(launchPoint);
			else
			{
				TrajectoryBuilder builder = drive.trajectoryBuilder(drive.getPoseEstimate());
				Trajectory path = builder.splineTo(launchPoint.vec(), launchPoint.getHeading()).build();

				execute(drivetrain, new DrivetrainI5.Follow(path));
			}

			execute(launcher, new Launcher.Lift(1));

			wait(1f);

			execute(launcher, new Launcher.Lift(-1));
			execute(launcher, new Launcher.Prime(false));
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
			DrivetrainI5 drivetrain = opMode.getBehavior(DrivetrainI5.class);
			powerShots(drivetrain.getDrive().getPoseEstimate());
		}
	}
}
