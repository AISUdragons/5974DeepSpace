package org.usfirst.frc5974.DeepSpace.subsystems;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;

/*import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionRunner;*/
import edu.wpi.first.vision.VisionThread;
import org.opencv.core.Rect;
import org.usfirst.frc5974.grip.GripPipeline;
//import java.util.Set;

public class Camera{

	private static final int IMG_WIDTH = 240;
	private static final int IMG_HEIGHT = 180;
	private static final int fps = 20;
	private VisionThread visionThread;
	double centerX = 0.0;
	private final Object imgLock = new Object();
    //UsbCamera camera = new UsbCamera(String name, String path)*/
    
    //Camera Stuff
    public void cameraInit(){
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
		camera.setFPS(fps);

		new Thread(() -> {
			//Creates a UsbCamera on the default port and streams output on MjpegServer [1]
			//equivalent to "".startAutomaticCapture(0), which is equivalent to "".startAutomaticCapture("USB Camera 0", 0)

			//Creates an image input (sink) that takes video from the primary feed (UsbCamera camera)
			//equivalent to "".getVideo(camera)
			CvSink cvSink = CameraServer.getInstance().getVideo();

			//Creates an image stream (source) MjpegServer [2] with the name "Blur"
			CvSource outputStream = CameraServer.getInstance().putVideo("Blur", IMG_WIDTH, IMG_HEIGHT);
			
			Mat source = new Mat(); //unreleated to CvSource
			Mat output = new Mat();

			while (!Thread.interrupted()) {
				//Applies the 'Imgproc.COLOR_BGR2GRAY' filter to a frame from 'cvSink' and puts the result on 'outputStream'
				cvSink.grabFrame(source);
				Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
				outputStream.putFrame(output);
			}
		}).start();

		visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
			if (!pipeline.findBlobsOutput().empty()) {
				Rect r = Imgproc.boundingRect(pipeline.findBlobsOutput());
				synchronized (imgLock) {
					centerX = r.x + (r.width / 2);
				}
			}
		});
		visionThread.start();

		/*NetworkTableInstance inst = NetworkTableInstance.getDefault();
		NetworkTable table = inst.getTable("GRIP/myContours Report");
		double[] defaultValue = new double[0];
		while(true) {
			double[] areas = table.getEntry("area").getDoubleArray(defaultValue);
			System.out.print("areas: ");
			for (double area : areas) {
				System.out.print(area + " ");
			}
			System.out.println();
			Timer.delay(1);
		}*/
    }
}