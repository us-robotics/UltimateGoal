package org.firstinspires.ftc.teamcode.Behaviors;

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
		camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

		camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
		{
			@Override
			public void onOpened()
			{
				pipeline = new CameraPipeline();

				camera.startStreaming(ResolutionX, ResolutionY, OpenCvCameraRotation.SIDEWAYS_RIGHT);
				camera.setPipeline(pipeline);
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

		if (pipeline == null) opMode.debug.addData("No Pipeline", 0);
		else
		{
			Scalar mean = pipeline.getMean();

			if (mean == null) opMode.debug.addData("No Mean", 1);
			else opMode.debug.addData("Mean", mean);
		}
	}

	public void closeCamera()
	{
		camera.stopStreaming();
	}

	private static class CameraPipeline extends OpenCvPipeline
	{
		public CameraPipeline()
		{
		}

		private Scalar mean;

		static final Scalar Blue = new Scalar(0f, 0f, 255f);

		static final Point RegionCenter = new Point(ResolutionX / 4f, ResolutionY * 0f);
		static final int RegionSize = 50;

		static final Rect RegionRect = new Rect
				(
						new Point(RegionCenter.x - RegionSize / 2f, RegionCenter.y - RegionSize / 2f),
						new Point(RegionCenter.x + RegionSize / 2f, RegionCenter.y + RegionSize / 2f)
				);

		Mat hsv = new Mat();
		Mat saturation = new Mat();
		Mat submat = saturation.submat(RegionRect);

		@Override
		public Mat processFrame(Mat input)
		{
			mean = Core.mean(input);

			Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
			Core.extractChannel(hsv, saturation, 1);

			Imgproc.rectangle(saturation, RegionRect, Blue, 1);
			mean = Core.mean(submat);

			return saturation;
		}

		public Scalar getMean()
		{
			return mean;
		}
	}
}
