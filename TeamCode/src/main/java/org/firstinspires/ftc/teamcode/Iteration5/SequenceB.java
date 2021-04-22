
package org.firstinspires.ftc.teamcode.Iteration5;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Behaviors.DrivetrainI5;
import org.firstinspires.ftc.teamcode.Behaviors.Launcher;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import FTCEngine.Core.OpModeBase;

public class SequenceB extends CommonSequence
{
	public SequenceB(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	protected void queueJobs()
	{
		DrivetrainI5 drivetrain = opMode.getBehavior(DrivetrainI5.class);
		Launcher launcher = opMode.getBehavior(Launcher.class);
		SampleMecanumDrive drive = drivetrain.getDrive();

		Pose2d position = new Pose2d(-63d, 53d, 0d);
		Vector2d center = new Vector2d(36d, 36d);

		drive.setPoseEstimate(position);

		execute(launcher, new Launcher.Lift(-1));
		execute(launcher, new Launcher.Prime(Launcher.SHOT_POWER, true));

		position = dropWobbles(position, center, new Vector2d(-41d, 30d));
		position = powerShots(position);

		Trajectory park = drive.trajectoryBuilder(position, true)
				.splineTo(new Vector2d(12d, 32d), Math.toRadians(150d)).build();

		execute(drivetrain, new DrivetrainI5.Follow(park));
	}
}
