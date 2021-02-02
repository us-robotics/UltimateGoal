package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.ftccommon.configuration.EditLegacyServoControllerActivity;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;

import FTCEngine.Core.*;

public class CameraVision extends Behavior
{
	/**
	 * NOTE: Do not configure the electronics in the constructor, do them in the awake method!
	 *
	 * @param opMode
	 */
	public CameraVision(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);

		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		WebcamName webcamName = hardwareMap.get(WebcamName.class, "frontCamera");

		camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
		camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
		{
			@Override
			public void onOpened()
			{
				camera.startStreaming(ResolutionX, ResolutionY, OpenCvCameraRotation.UPRIGHT);
				camera.setPipeline(pipeline = new CameraPipeline());
			}
		});
	}

	private OpenCvCamera camera;
	private CameraPipeline pipeline;

	private static final int ResolutionX = 320;
	private static final int ResolutionY = 240;

	@Override
	public void update()
	{
		super.update();

		if (pipeline != null)
		{
			float saturationUpper = (float)pipeline.getMeanUpper().val[0];
			float saturationLower = (float)pipeline.getMeanLower().val[0];

			final float Threshold = 100f;
			Position position;

			if (saturationLower < Threshold) position = Position.A;
			else position = saturationUpper > Threshold ? Position.C : Position.B;

			opMode.debug.addData("Position", position);
		}
	}

	public void closeCamera()
	{
		camera.stopStreaming();
	}

	public enum Position
	{
		A, B, C
	}

	private static class CameraPipeline extends OpenCvPipeline
	{
		private static final Scalar Purple = new Scalar(255f, 0f, 255f);
		private static final Point RegionCenter = new Point(ResolutionX * 0.755f, ResolutionY / 3f);

		private static final float RegionWidth = ResolutionX / 7f;
		private static final float RegionHeight = ResolutionY / 7f;

		private static final Rect RegionUpper = new Rect
				(
						new Point(RegionCenter.x - RegionWidth / 2f, RegionCenter.y - RegionHeight / 2f),
						new Point(RegionCenter.x + RegionWidth / 2f, RegionCenter.y)
				);

		private static final Rect RegionLower = new Rect
				(
						new Point(RegionCenter.x - RegionWidth / 2f, RegionCenter.y),
						new Point(RegionCenter.x + RegionWidth / 2f, RegionCenter.y + RegionHeight / 2f)
				);

		private final Mat hsv = new Mat();
		private final Mat saturation = new Mat();

		private Mat submatUpper;
		private Mat submatLower;

		private volatile Scalar meanUpper;
		private volatile Scalar meanLower;

		@Override
		public Mat processFrame(Mat input)
		{
			Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
			Core.extractChannel(hsv, saturation, 1);

			if (submatUpper == null) submatUpper = saturation.submat(RegionUpper);
			if (submatLower == null) submatLower = saturation.submat(RegionLower);

			meanUpper = Core.mean(submatUpper);
			meanLower = Core.mean(submatLower);

			Imgproc.rectangle(saturation, RegionUpper, Purple, 1);
			Imgproc.rectangle(saturation, RegionLower, Purple, 1);

			return saturation;
		}

		public Scalar getMeanUpper()
		{
			return meanUpper;
		}

		public Scalar getMeanLower()
		{
			return meanLower;
		}
	}
}
