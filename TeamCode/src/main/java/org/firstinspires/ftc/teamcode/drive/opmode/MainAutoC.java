package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MainAutoC")
public class MainAutoC extends MainAuto
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
