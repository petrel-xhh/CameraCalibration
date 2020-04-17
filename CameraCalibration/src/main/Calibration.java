package main;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

public class Calibration {

	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}
	public static void main(String[] args) {
		Calibration calib = new Calibration();
		double cbSize = 50;
		String dirPath = "C:\\Users\\Neo\\Documents\\calibrationImages";
		Size patternSize = new Size(10, 9);
		System.out.println(patternSize.height);
		MatOfPoint3f objPoints = new MatOfPoint3f();
		calib.calcObjectCoords(patternSize, cbSize, objPoints);
		List<Mat> Hs = new ArrayList<Mat>();
		for(String name : new File(dirPath).list()) {
			String path = dirPath + File.separator + name;
			Mat img = Imgcodecs.imread(path);
			Size size = new Size(img.cols() / 5, img.rows() / 5);
			Mat dst = new Mat();
			Imgproc.resize(img, dst, size);
			MatOfPoint2f corners = new MatOfPoint2f();
			boolean found = calib.findCorners(dst, patternSize, corners);
			System.out.println(name + "\t" + found);
			if(found) {
				Mat tmpObjPoints = new Mat();
				objPoints.copyTo(tmpObjPoints);
				Mat homography = new Mat();
				calib.calcHomograpy(tmpObjPoints, corners, homography);
				Hs.add(homography);
			}
		}
		Mat cameraMatrix = new Mat();
		calib.calcCameraMatrix(Hs, cameraMatrix);
		System.out.println(cameraMatrix.dump());
		List<Mat> Es = new ArrayList<Mat>();
		calib.calcExternalMatrix(cameraMatrix, Hs, Es);
		for(Mat E : Es) {
			System.out.println(E.dump());
		}
	}
	
	public boolean findCorners(Mat img, Size patternSize, MatOfPoint2f corners) {
		// cvtColor to gray
		Mat gray = new Mat();
		Imgproc.cvtColor(img, gray, Imgproc.COLOR_BGR2GRAY);
		return Calib3d.findChessboardCorners(gray, patternSize, corners);
	}
	
	public void calcObjectCoords(Size patternSize, double cbSize, MatOfPoint3f objPoints) {
		for(int i = 0; i < (int)patternSize.height; i++) {
			for(int j = 0; j < (int)patternSize.width; j++) {
				double x = j * cbSize;
				double y = i * cbSize;
				objPoints.push_back(new MatOfPoint3f(new Point3(x, y, 0)));
			}
		}
	}
	
	public void calcHomograpy(Mat objPoints, Mat imgPoints, Mat homography) {
		if(objPoints.rows() != imgPoints.rows()) {
			System.err.println("number of points in objPoints must equal to imgPoints");
			System.exit(-1);
		}
		//¹¹Ôì¾ØÕóL
		Mat L = new Mat();
		for(int i = 0; i < imgPoints.rows(); i++) {
			Mat objPoint = objPoints.row(i);
			Mat imgPoint = imgPoints.row(i);
			double[] objValues = objPoint.get(0, 0);
			double[] imgValues = imgPoint.get(0, 0);
			double u = imgValues[0], v = imgValues[1];
			double x = objValues[0], y = objValues[1], z = objValues[2];
			Mat row1 = new MatOfDouble(x, y, 1, 0, 0, 0, -x * u, -y * u, -u).t();
			Mat row2 = new MatOfDouble(0, 0, 0, x, y, 1, -v * x, -v * y, -v).t();
			//row1.put(0, 0, x, y, 1, 0, 0, 0, -x * u, -y * u, -u);
			L.push_back(row1);
			L.push_back(row2);
		}
		Mat u = new Mat();
		Mat s = new Mat();
		Mat vt = new Mat();
		Core.SVDecomp(L, u, s, vt);
		vt.t().col(8).t().reshape(1, 3).copyTo(homography);
		//System.out.println(homography.dump());
	}
	
	public void calcVij(Mat H, int _i, int _j, Mat vij) {
		int i = _i - 1;
		int j = _j - 1;
		System.out.println(H.dump());
		System.out.println(H.row(0).get(0, 0).length);
		double[] h1 = H.row(0).get(0, 0);
		double[] h2 = H.row(1).get(0, 0);
		double[] h3 = H.row(2).get(0, 0);
		double  h1i = H.get(0, i)[0],
				h1j = H.get(0, j)[0],
				h2i = H.get(1, i)[0],
				h2j = H.get(1, j)[0],
				h3i = H.get(2, i)[0],
				h3j = H.get(2, j)[0];
		double v1 = h1i * h1j,
				v2 = h1i * h2j + h2i * h1j,
				v3 = h2i * h2j,
				v4 = h1i * h3j + h3i * h1j,
				v5 = h2i * h3j + h3i * h2j,
				v6 = h3i * h3j;
		new MatOfDouble(v1, v2, v3, v4, v5, v6).copyTo(vij);
	}
	
	public void calcCameraMatrix(List<Mat> Hs, Mat cameraMatrix) {
		Mat V = new Mat();
		for(Mat H : Hs) {
			Mat v11 = new Mat();
			Mat v12 = new Mat();
			Mat v22 = new Mat();
			calcVij(H, 1, 1, v11);
			calcVij(H, 1, 2, v12);
			calcVij(H, 2, 2, v22);
			Mat row1 = v12.t();
			Mat row2 = new Mat();
			Core.subtract(v11.t(), v22.t(), row2);
			//System.out.println(v11.t().dump() + "\n" + v22.t().dump());
			//System.out.println(row2.dump());
			V.push_back(row1);
			V.push_back(row2);
		}
		//System.out.println("V.rows(): " + V.rows());
		Mat vt = new Mat();
		Core.SVDecomp(V, new Mat(), new Mat(), vt);
		//System.out.println(vt.dump());
		Mat B = vt.row(5).t();
		double B11 = B.get(0, 0)[0];
		double B12 = B.get(1, 0)[0];
		double B22 = B.get(2, 0)[0];
		double B13 = B.get(3, 0)[0];
		double B23 = B.get(4, 0)[0];
		double B33 = B.get(5, 0)[0];
		double v0 = (B12 * B13 - B11 * B23) / (B11 * B22 - B12 * B12);
		double lambda = B33 - (B13 * B13 + v0 * (B12 * B13 - B11 * B23)) / B11;
		double alpha = Math.sqrt(lambda / B11);
		double beta = Math.sqrt(lambda * B11 / (B11 * B22 - B12 * B12));
		double gama = -B12 * alpha * alpha * beta / lambda;
		double u0 = gama * v0 / beta - B13 * alpha * alpha / lambda;
		Mat A = new Mat(3, 3, CvType.CV_64FC1);
		A.put(0, 0, alpha);
		A.put(0, 1, gama);
		A.put(0, 2, u0);
		A.put(1, 0, 0);
		A.put(1, 1, beta);
		A.put(1, 2, v0);
		A.put(2, 0, 0);
		A.put(2, 1, 0);
		A.put(2, 2, 1);
		A.copyTo(cameraMatrix);
		//System.out.println(B.dump());
	}
	
	public void calcExternalMatrix(Mat cameraMatrix, List<Mat> Hs, List<Mat> Es) {
		Mat A = new Mat();
		cameraMatrix.copyTo(A);
		Mat A_inv = new Mat();
		Core.invert(A, A_inv);
		for(Mat H : Hs) {
			Mat h1 = H.col(0);
			Mat h2 = H.col(1);
			Mat h3 = H.col(2);
			System.out.println(h1.dump());
			Mat dst = new Mat();
			Core.gemm(A_inv, h1, 1, new Mat(), 0, dst);
			System.out.println(dst.dump());
			double lambda = 1.0 / Core.norm(dst);
			System.out.println(lambda);
			Mat r1 = new Mat();
			Mat r2 = new Mat();
			Mat r3 = new Mat();
			Mat t = new Mat();
			Core.gemm(A_inv, h1, 1, new Mat(), 0, r1);
			Core.gemm(A_inv, h2, 1, new Mat(), 0, r2);
			Core.gemm(A_inv, h3, 1, new Mat(), 0, t);
			Core.multiply(r1, r2, r3);
			r1 = r1.mul(r1, lambda);
			r2 = r2.mul(r2, lambda);
			Mat E = new Mat();
			
			Core.multiply(A_inv, H, E);
			E = E.mul(E, lambda);
			Es.add(E);
		}

	}
}
