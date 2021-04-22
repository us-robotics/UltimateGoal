package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import FTCEngine.Core.Behavior;
import FTCEngine.Core.OpModeBase;

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
			Scalar saturationUpper = pipeline.getMeanUpper();
			Scalar saturationLower = pipeline.getMeanLower();

			if (saturationLower != null && saturationUpper != null)
			{
				final float Threshold = 100f;
				Position position;

				if (saturationLower.val[0] < Threshold) position = Position.A;
				else position = saturationUpper.val[0] > Threshold ? Position.C : Position.B;

				opMode.debug.addData("Position", position);
			}
		}
	}

	public boolean available()
	{
		return pipeline != null && pipeline.getMeanUpper() != null && pipeline.getMeanLower() != null;
	}

	public Position getPosition()
	{
		if (!available()) throw new IllegalStateException("Attempting to access position while vision is unavailable!");

		double saturationUpper = pipeline.getMeanUpper().val[0];
		double saturationLower = pipeline.getMeanLower().val[0];

		final float Threshold = 100f;

		if (saturationLower < Threshold) return Position.A;
		return saturationUpper > Threshold ? Position.C : Position.B;
	}

	public void closeCamera()
	{
		camera.stopStreaming();
		camera.closeCameraDeviceAsync(() ->
		                              {
		                              });

		pipeline = null;
	}

	public enum Position
	{
		A, B, C;
	}

	private static class CameraPipeline extends OpenCvPipeline
	{
		private static final Scalar GRAY = new Scalar(127f, 127f, 127f);
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

			Imgproc.rectangle(saturation, RegionUpper, GRAY, 1);
			Imgproc.rectangle(saturation, RegionLower, GRAY, 1);

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
