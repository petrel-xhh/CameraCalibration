package test;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.junit.Test;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

public class OpenCVCameraCalibrationTest {

	// @Test
	public static void main(String[] args) {
		calibrate();
		//test2();
	}
	
	public static void calibrate() {
		try
		{
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		String dirPath = "C:\\Users\\Neo\\Documents\\calibrationImages";
		float cbSize = 50;
		Scalar red = new Scalar(0, 0, 255);
		Size patternSize = new Size(10, 9);
		File dir = new File(dirPath);
		String[] names = dir.list();
		List<Mat> objectPoints = new ArrayList<Mat>();
		List<Mat> imagePoints = new ArrayList<Mat>();
		Size imageSize = null;
		Mat cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
		//cameraMatrix.put(0, 0, 1);
		//cameraMatrix.put(1, 1, 1);
		Mat distCoeffs = new Mat();
		List<Mat> rvecs = new ArrayList<Mat>();
		List<Mat> tvecs = new ArrayList<Mat>();
		ImageViewer viewer  = new ImageViewer(null);
		MatOfPoint3f objPoint = new MatOfPoint3f();
		for(int i = 0; i < 9; i++) {
			for(int j = 0; j < 10; j++) {
				float y = i * cbSize;
				float x = j * cbSize;
				objPoint.push_back(new MatOfPoint3f(new Point3(x, y, 0.0f)));
			}
		}
		for(String name : names) {
			String imgPath = dirPath + File.separator + name;
			//System.out.println(imgPath);
			Mat img = Imgcodecs.imread(imgPath);
			Mat dst = new Mat();
			Size dsize = new Size(img.cols() / 5, img.rows() / 5);
			imageSize = dsize;
			Imgproc.resize(img, dst, dsize);
			// 提取角点
			Mat gray = new Mat();
			Imgproc.cvtColor(dst, gray, Imgproc.COLOR_BGR2GRAY);
			MatOfPoint2f corners = new MatOfPoint2f();
			boolean found = Calib3d.findChessboardCorners(gray, patternSize, corners);
			System.out.println(name + "\t" + found);
			if(!found) {
				continue;
			}
			imagePoints.add(corners);
			objectPoints.add(objPoint);
			for(int i = 0; i < corners.rows(); i++) {
				for(int j = 0; j < corners.cols(); j++) {
					double[] values = corners.get(i, j);
					//System.out.println(values);
					Imgproc.circle(dst, new Point(values[0], values[1]), 4, red);
					int index = i * corners.cols() + j;
					double[] objCoord = objPoint.get(index, 0);
					String text = "(" + (int)objCoord[0] + "," + (int)objCoord[1] + ")";
					Imgproc.putText(dst, text, new Point(values[0], values[1]), Imgproc.FONT_HERSHEY_PLAIN, 0.8, red, 1, Imgproc.FONT_HERSHEY_SCRIPT_SIMPLEX);
				}
			}
			viewer.setImage(dst);
			viewer.imshow();
			/*
			for(int i = 0; i < corners.rows(); i++) {
				for(int j = 0; j < corners.cols(); j++) {
					//double[] values = corners.get(i, j);
					//System.out.println(values);
					int index = i * corners.cols() + j;
					objPoint.put(index, 0, i * cbSize, -j * cbSize, 0);
					//Point point = new Point();
					//point.set(values);
					//Imgproc.circle(dst, point, 4, red);
				}
			}
			*/
			//viewer.setImage(dst);
			//viewer.imshow();
			//objectPoints.add(objPoint);
			//imagePoints.add(imgPoint);
		}
		double error = Calib3d.calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);
		System.out.println(cameraMatrix.dump());
		System.out.println(distCoeffs.dump());
		System.out.println(error);
		for(Mat rvec : rvecs) {

			System.out.println(rvec.dump());
		}
		}catch (Exception e) {
			// TODO: handle exception
			e.printStackTrace();
		}
	}
	
	public static void test1() {
		// 打开图像
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		String path = "C:\\Users\\Neo\\Documents\\calibrationImages\\03.JPG";
		Mat img = Imgcodecs.imread(path);
		Mat dst = new Mat();
		Imgproc.resize(img, dst, new Size(img.cols() / 5, img.rows() / 5));
		System.out.println(dst.size());
		//ImageViewer viewer = new ImageViewer(dst);
		//viewer.imshow();
		//找出内角点
		Mat gray = new Mat();
		Imgproc.cvtColor(dst, gray, Imgproc.COLOR_BGR2GRAY);
		Size patternSize = new Size(10, 9);
		MatOfPoint2f corners = new MatOfPoint2f();
		boolean found = Calib3d.findChessboardCorners(gray, patternSize, corners);
		System.out.println(found);
		Scalar red = new Scalar(0, 0, 255);
		for(int i = 0; i < corners.cols(); i++) {
			for(int j = 0; j < corners.rows(); j++) {
				double[] corner = corners.get(j, i);
				Point point = new Point();
				point.set(corner);
				Imgproc.circle(dst, point, 4, red);
			}
		}
		ImageViewer viewer = new ImageViewer(dst);
		viewer.imshow();
		
	}
	public static void test2() {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		String dirPath = "C:\\Users\\Neo\\Documents\\calibrationImages";
		Size patternSize = new Size(10, 9);
		String path = "C:\\Users\\Neo\\Documents\\calibrationImages\\03.JPG";
		Mat img = Imgcodecs.imread(path);
		Size size = new Size(img.rows() / 5, img.cols() / 5);	//问题在这里
		Mat dst = new Mat();
		Imgproc.resize(img, dst, size);
		MatOfPoint2f corners = new MatOfPoint2f();
		///boolean found = findCorners(dst, patternSize, corners);
		Mat gray = new Mat();
		Imgproc.cvtColor(dst, gray, Imgproc.COLOR_BGR2GRAY);
		boolean found = Calib3d.findChessboardCorners(gray, patternSize, corners);
		System.out.println(path + "\t" + found);
	}
	public static boolean findCorners(Mat img, Size patternSize, MatOfPoint2f corners) {
		// cvtColor to gray
		Mat gray = new Mat();
		Imgproc.cvtColor(img, gray, Imgproc.COLOR_BGR2GRAY);
		return Calib3d.findChessboardCorners(gray, patternSize, corners);
	}
}


/*
 * 1.载入图片
 * 2.得到角点的图像坐标与世界坐标
 * 3.求解H
 * 4.求解B、A
 * 5.求解E
 */
