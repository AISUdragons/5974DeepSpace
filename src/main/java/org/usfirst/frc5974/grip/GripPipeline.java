import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.HashMap;

import edu.wpi.first.wpilibj.vision.VisionPipeline;

import org.opencv.core.*;
import org.opencv.core.Core.*;
import org.opencv.features2d.FeatureDetector;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;
import org.opencv.objdetect.*;

/**
* GripPipeline class.
*
* <p>An OpenCV pipeline generated by GRIP.
*
* @author GRIP
*/
public class GripPipeline implements VisionPipeline {

	//Outputs
	private Mat cvResizeOutput = new Mat();
	private Mat blurOutput = new Mat();
	private Mat hsvThresholdOutput = new Mat();
	private Mat maskOutput = new Mat();
	private MatOfKeyPoint findBlobsOutput = new MatOfKeyPoint();

	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}

	/**
	 * This is the primary method that runs the entire pipeline and updates the outputs.
	 */
	@Override	public void process(Mat source0) {
		// Step CV_resize0:
		Mat cvResizeSrc = source0;
		Size cvResizeDsize = new Size(0, 0);
		double cvResizeFx = 0.5;
		double cvResizeFy = 0.5;
		int cvResizeInterpolation = Imgproc.INTER_LINEAR;
		cvResize(cvResizeSrc, cvResizeDsize, cvResizeFx, cvResizeFy, cvResizeInterpolation, cvResizeOutput);

		// Step Blur0:
		Mat blurInput = cvResizeOutput;
		BlurType blurType = BlurType.get("Box Blur");
		double blurRadius = 20.754716981132077;
		blur(blurInput, blurType, blurRadius, blurOutput);

		// Step HSV_Threshold0:
		Mat hsvThresholdInput = blurOutput;
		double[] hsvThresholdHue = {0.0, 47.135325131810184};
		double[] hsvThresholdSaturation = {0.0, 252.7592267135325};
		double[] hsvThresholdValue = {187.28813559322035, 255.0};
		hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

		// Step Mask0:
		Mat maskInput = cvResizeOutput;
		Mat maskMask = hsvThresholdOutput;
		mask(maskInput, maskMask, maskOutput);

		// Step Find_Blobs0:
		Mat findBlobsInput = maskOutput;
		double findBlobsMinArea = 10.0;
		double[] findBlobsCircularity = {0.0, 1.0};
		boolean findBlobsDarkBlobs = false;
		findBlobs(findBlobsInput, findBlobsMinArea, findBlobsCircularity, findBlobsDarkBlobs, findBlobsOutput);

	}

	/**
	 * This method is a generated getter for the output of a CV_resize.
	 * @return Mat output from CV_resize.
	 */
	public Mat cvResizeOutput() {
		return cvResizeOutput;
	}

	/**
	 * This method is a generated getter for the output of a Blur.
	 * @return Mat output from Blur.
	 */
	public Mat blurOutput() {
		return blurOutput;
	}

	/**
	 * This method is a generated getter for the output of a HSV_Threshold.
	 * @return Mat output from HSV_Threshold.
	 */
	public Mat hsvThresholdOutput() {
		return hsvThresholdOutput;
	}

	/**
	 * This method is a generated getter for the output of a Mask.
	 * @return Mat output from Mask.
	 */
	public Mat maskOutput() {
		return maskOutput;
	}

	/**
	 * This method is a generated getter for the output of a Find_Blobs.
	 * @return MatOfKeyPoint output from Find_Blobs.
	 */
	public MatOfKeyPoint findBlobsOutput() {
		return findBlobsOutput;
	}


	/**
	 * Resizes an image.
	 * @param src The image to resize.
	 * @param dSize size to set the image.
	 * @param fx scale factor along X axis.
	 * @param fy scale factor along Y axis.
	 * @param interpolation type of interpolation to use.
	 * @param dst output image.
	 */
	private void cvResize(Mat src, Size dSize, double fx, double fy, int interpolation,
		Mat dst) {
		if (dSize==null) {
			dSize = new Size(0,0);
		}
		Imgproc.resize(src, dst, dSize, fx, fy, interpolation);
	}

	/**
	 * An indication of which type of filter to use for a blur.
	 * Choices are BOX, GAUSSIAN, MEDIAN, and BILATERAL
	 */
	enum BlurType{
		BOX("Box Blur"), GAUSSIAN("Gaussian Blur"), MEDIAN("Median Filter"),
			BILATERAL("Bilateral Filter");

		private final String label;

		BlurType(String label) {
			this.label = label;
		}

		public static BlurType get(String type) {
			if (BILATERAL.label.equals(type)) {
				return BILATERAL;
			}
			else if (GAUSSIAN.label.equals(type)) {
			return GAUSSIAN;
			}
			else if (MEDIAN.label.equals(type)) {
				return MEDIAN;
			}
			else {
				return BOX;
			}
		}

		@Override
		public String toString() {
			return this.label;
		}
	}

	/**
	 * Softens an image using one of several filters.
	 * @param input The image on which to perform the blur.
	 * @param type The blurType to perform.
	 * @param doubleRadius The radius for the blur.
	 * @param output The image in which to store the output.
	 */
	private void blur(Mat input, BlurType type, double doubleRadius,
		Mat output) {
		int radius = (int)(doubleRadius + 0.5);
		int kernelSize;
		switch(type){
			case BOX:
				kernelSize = 2 * radius + 1;
				Imgproc.blur(input, output, new Size(kernelSize, kernelSize));
				break;
			case GAUSSIAN:
				kernelSize = 6 * radius + 1;
				Imgproc.GaussianBlur(input,output, new Size(kernelSize, kernelSize), radius);
				break;
			case MEDIAN:
				kernelSize = 2 * radius + 1;
				Imgproc.medianBlur(input, output, kernelSize);
				break;
			case BILATERAL:
				Imgproc.bilateralFilter(input, output, -1, radius, radius);
				break;
		}
	}

	/**
	 * Segment an image based on hue, saturation, and value ranges.
	 *
	 * @param input The image on which to perform the HSL threshold.
	 * @param hue The min and max hue
	 * @param sat The min and max saturation
	 * @param val The min and max value
	 * @param output The image in which to store the output.
	 */
	private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
	    Mat out) {
		Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
		Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
			new Scalar(hue[1], sat[1], val[1]), out);
	}

	/**
	 * Filter out an area of an image using a binary mask.
	 * @param input The image on which the mask filters.
	 * @param mask The binary image that is used to filter.
	 * @param output The image in which to store the output.
	 */
	private void mask(Mat input, Mat mask, Mat output) {
		mask.convertTo(mask, CvType.CV_8UC1);
		Core.bitwise_xor(output, output, output);
		input.copyTo(output, mask);
	}

	/**
	 * Detects groups of pixels in an image.
	 * @param input The image on which to perform the find blobs.
	 * @param minArea The minimum size of a blob that will be found
	 * @param circularity The minimum and maximum circularity of blobs that will be found
	 * @param darkBlobs The boolean that determines if light or dark blobs are found.
	 * @param blobList The output where the MatOfKeyPoint is stored.
	 */
	private void findBlobs(Mat input, double minArea, double[] circularity,
		Boolean darkBlobs, MatOfKeyPoint blobList) {
		FeatureDetector blobDet = FeatureDetector.create(FeatureDetector.SIMPLEBLOB);
		try {
			File tempFile = File.createTempFile("config", ".xml");

			StringBuilder config = new StringBuilder();

			config.append("<?xml version=\"1.0\"?>\n");
			config.append("<opencv_storage>\n");
			config.append("<thresholdStep>10.</thresholdStep>\n");
			config.append("<minThreshold>50.</minThreshold>\n");
			config.append("<maxThreshold>220.</maxThreshold>\n");
			config.append("<minRepeatability>2</minRepeatability>\n");
			config.append("<minDistBetweenBlobs>10.</minDistBetweenBlobs>\n");
			config.append("<filterByColor>1</filterByColor>\n");
			config.append("<blobColor>");
			config.append((darkBlobs ? 0 : 255));
			config.append("</blobColor>\n");
			config.append("<filterByArea>1</filterByArea>\n");
			config.append("<minArea>");
			config.append(minArea);
			config.append("</minArea>\n");
			config.append("<maxArea>");
			config.append(Integer.MAX_VALUE);
			config.append("</maxArea>\n");
			config.append("<filterByCircularity>1</filterByCircularity>\n");
			config.append("<minCircularity>");
			config.append(circularity[0]);
			config.append("</minCircularity>\n");
			config.append("<maxCircularity>");
			config.append(circularity[1]);
			config.append("</maxCircularity>\n");
			config.append("<filterByInertia>1</filterByInertia>\n");
			config.append("<minInertiaRatio>0.1</minInertiaRatio>\n");
			config.append("<maxInertiaRatio>" + Integer.MAX_VALUE + "</maxInertiaRatio>\n");
			config.append("<filterByConvexity>1</filterByConvexity>\n");
			config.append("<minConvexity>0.95</minConvexity>\n");
			config.append("<maxConvexity>" + Integer.MAX_VALUE + "</maxConvexity>\n");
			config.append("</opencv_storage>\n");
			FileWriter writer;
			writer = new FileWriter(tempFile, false);
			writer.write(config.toString());
			writer.close();
			blobDet.read(tempFile.getPath());
		} catch (IOException e) {
			e.printStackTrace();
		}

		blobDet.detect(input, blobList);
	}




}

