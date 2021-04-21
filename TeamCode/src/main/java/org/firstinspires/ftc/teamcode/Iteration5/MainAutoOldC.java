package org.firstinspires.ftc.teamcode.Iteration5;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MainAutoOldC")
public class MainAutoOldC extends MainAutoOld
{
	@Override
	public Vector2d getTargetCenter()
	{
		return new Vector2d(60d, 60d);
	}

	@Override
	public Pose2d getEndPose()
	{
		return new Pose2d(12d, 36d, Math.toRadians(180d));
	}
}
