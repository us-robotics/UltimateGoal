package org.firstinspires.ftc.teamcode.Behaviors;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

		opMode.input.registerButton(Input.Source.CONTROLLER_1, Input.Button.B);
		opMode.input.registerButton(Input.Source.CONTROLLER_1, Input.Button.X);
		opMode.input.registerButton(Input.Source.CONTROLLER_2, Input.Button.Y);

		opMode.input.registerButton(Input.Source.CONTROLLER_1, Input.Button.LEFT_BUMPER);
		opMode.input.registerButton(Input.Source.CONTROLLER_1, Input.Button.RIGHT_BUMPER);
	}

	@Override
	public void update()
	{
		super.update();

		if (opMode.input.getButtonDown(Input.Source.CONTROLLER_1, Input.Button.B)) opMode.assignSequence(null);

		if (!opMode.hasSequence())
		{
			if (opMode.input.getButtonDown(Input.Source.CONTROLLER_1, Input.Button.X)) opMode.assignSequence(new PowerShotsSequence(opMode));
			if (opMode.input.getButtonDown(Input.Source.CONTROLLER_2, Input.Button.Y)) opMode.assignSequence(new WobbleDropSequence(opMode));

			if (opMode.input.getButtonDown(Input.Source.CONTROLLER_1, Input.Button.LEFT_BUMPER)) opMode.assignSequence(new LaunchSequence(opMode, true));
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

			if (reset) drive.setPoseEstimate(LAUNCH_POINT);
			else
			{
				TrajectoryBuilder builder = drive.trajectoryBuilder(drive.getPoseEstimate());
				Trajectory path = builder.lineToLinearHeading(LAUNCH_POINT).build();

				execute(drivetrain, new DrivetrainI5.Follow(path));
			}

			launchRings();
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

	private static class WobbleDropSequence extends CommonSequence
	{
		public WobbleDropSequence(OpModeBase opMode)
		{
			super(opMode);
		}

		@Override
		protected void queueJobs()
		{
			DrivetrainI5 drivetrain = opMode.getBehavior(DrivetrainI5.class);
			WobbleGrabberI5 grabber = opMode.getBehavior(WobbleGrabberI5.class);
			SampleMecanumDrive drive = drivetrain.getDrive();

			Pose2d start = drive.getPoseEstimate();

			Trajectory driveToDrop = drive.trajectoryBuilder(start)
					.splineToSplineHeading(new Pose2d(-63f, start.getY(), Math.toRadians(-90f)), Math.toRadians(180f)).build();

			buffer(grabber, new WobbleGrabberI5.Move(WobbleGrabberI5.Mode.HIGH));
			buffer(drivetrain, new DrivetrainI5.Follow(driveToDrop));

			execute();

			execute(grabber, new WobbleGrabberI5.Move(WobbleGrabberI5.Mode.DROP));
			execute(grabber, new WobbleGrabberI5.Grab(false));

			wait(0.3f);
			execute(grabber, new WobbleGrabberI5.Grab(true));

			Trajectory moveAway = drive.trajectoryBuilder(driveToDrop.end())
					.splineTo(driveToDrop.end().vec().plus(new Vector2d(20f, -20f)), Math.toRadians(0f)).build();


			buffer(grabber, new WobbleGrabberI5.Move(WobbleGrabberI5.Mode.FOLD));
			buffer(drivetrain, new DrivetrainI5.Follow(moveAway));

			execute();
		}
	}
}
