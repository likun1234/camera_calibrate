#include <opencv2/opencv.hpp>
#include <iostream>
#include <ctype.h>
#include <dirent.h>
using namespace std;
using namespace cv;



static bool readStringList( const string& filename, vector<string>& l )
{
	l.resize(0);
	FileStorage fs(filename, FileStorage::READ);
	if( !fs.isOpened() )
		return false;
	FileNode n = fs.getFirstTopLevelNode();
	if( n.type() != FileNode::SEQ )
		return false;
	FileNodeIterator it = n.begin(), it_end = n.end();
	for( ; it != it_end; ++it )
		l.push_back((string)*it);
	return true;
}

bool GetFileNames(const string path, vector<string>& filename)//返回文件中的名称
{
	DIR *pDir;
	struct dirent* ptr;
	if(!(pDir = opendir(path.c_str())))
		return false;
	while((ptr = readdir(pDir))!=0) {
		if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
			filename.push_back(path + "/" + ptr->d_name);
	}
	closedir(pDir);
	return true;
}





static void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners)
{
	corners.resize(0);
	for (int i = 0; i < boardSize.height; i++)        //height和width位置不能颠倒
		for (int j = 0; j < boardSize.width; j++)
		{
			corners.push_back(Point3f(j*squareSize, i*squareSize, 0));
		}
}

bool calibrate(Mat& intrMat, Mat& distCoeffs, vector<vector<Point2f>>& imagePoints,
		vector<vector<Point3f>>& ObjectPoints, Size& imageSize,const int cameraId , 
		vector<string> imageList)
{
	int w = 6;
	int h = 9;
	double rms = 0;

	Size boardSize;
	boardSize.width = w;
	boardSize.height = h;
	vector<Point2f> pointBuf;
	float squareSize = 1.f;
	vector<Mat> rvecs, tvecs;
	bool ok = false;

	int nImages = (int)imageList.size() / 2;

	namedWindow("View", 1);
	for (int i = 0; i<nImages ; i++)
	{
		Mat view;
		Mat viewGray(view.size(),CV_8UC1);
		string path1 = "/home/lk/project/opencv_practice/practice/calibrate/image/";
		view = imread(path1+imageList[i*2+cameraId]);
		imageSize = view.size();
		cvtColor(view, viewGray, COLOR_BGR2GRAY);
		bool found = findChessboardCorners(view, boardSize, pointBuf,
				CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

		if (found)
		{
			cornerSubPix(viewGray, pointBuf, Size(11, 11),
					Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(view, boardSize, Mat(pointBuf), found);
			bitwise_not(view, view);
			imagePoints.push_back(pointBuf);
			cout << '.';
		}
		imshow("View", view);
		waitKey(50);
	}
	//calculate chessboardCorners
	calcChessboardCorners(boardSize, squareSize, ObjectPoints[0]);
	ObjectPoints.resize(imagePoints.size(), ObjectPoints[0]);

	rms = calibrateCamera(ObjectPoints, imagePoints, imageSize, intrMat, distCoeffs,
			rvecs, tvecs);
	ok = checkRange(intrMat) && checkRange(distCoeffs);

	if (ok)
	{
		cout << "done with RMS error=" << rms << endl;
		return true;
	}
	else
		return false;
}


int main(int argc, char** argv)
{
	//initialize some parameters
	bool okcalib = false;
	Mat intrMatFirst, intrMatSec, distCoeffsFirst, distCoffesSec;
	Mat R, T, E, F, RFirst, RSec, PFirst, PSec, Q;
	vector<vector<Point2f>> imagePointsFirst, imagePointsSec;
	vector<vector<Point3f>> ObjectPoints(1);
	Rect validRoi[2];
	Size imageSize;
	int cameraIdFirst = 0, cameraIdSec = 1;
	double rms = 0;

	//get pictures and calibrate
	vector<string> imageList;
	//string filename = "/home/lk/project/opencv_practice/practice/calibrate/build/stereo_calib.xml";
	string filename = "stereo_calib.xml";
	//string path = argv[1];
	bool okread = readStringList(filename, imageList);
	//cout<<imageList.size()<<endl;
	for (int i=0;i<imageList.size();i++)
	{
		cout<<imageList[i]<<endl;
	}
    //return 0;
	//bool okread = GetFileNames(path,imageList);
	if (!okread || imageList.empty())
	{
		//cout << "can not open " << path << " or the string list is empty" << endl;
		return false;
	}
	if (imageList.size() % 2 != 0)
	{
		cout << "Error: the image list contains odd (non-even) number of elements\n";
		return false;
	}

	//calibrate
	cout << "calibrate left camera..." << endl;
	okcalib = calibrate(intrMatFirst, distCoeffsFirst, imagePointsFirst, ObjectPoints,
			imageSize, cameraIdFirst, imageList);
	FileStorage fs("instrinstic.xml",FileStorage::WRITE);
	fs<<"camera_matrix_L"<<intrMatFirst;
    fs<<"distortion_coefficient_L"<<distCoeffsFirst;
    cout<<"cali 1 over"<<endl;
	if (!okcalib)
	{
		cout << "fail to calibrate left camera" << endl;
		return -1;
	}
	else
	{
		cout << "calibrate the right camera..." << endl;
	}

	okcalib = calibrate(intrMatSec, distCoffesSec, imagePointsSec, ObjectPoints,
			imageSize, cameraIdSec, imageList);
	fs<<"camera_matrix_R"<<intrMatSec;
    fs<<"distortion_coefficient_R"<<distCoffesSec;
    fs.release();
	if (!okcalib)
	{
		cout << "fail to calibrate the right camera" << endl;
		return -1;
	}
	destroyAllWindows();

	//estimate position and orientation
	cout << "estimate position and orientation of the second camera" << endl
		<< "relative to the first camera..." << endl;
	rms = stereoCalibrate(ObjectPoints, imagePointsFirst, imagePointsSec,
			intrMatFirst, distCoeffsFirst, intrMatSec, distCoffesSec,
			imageSize, R, T, E, F, CV_CALIB_FIX_INTRINSIC,
			TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 1e-6));
	cout << "done with RMS error=" << rms << endl;

	FileStorage fs2("canshu.xml",FileStorage::WRITE);
	fs2<<"R"<<R;
	fs2<<"T"<<T;
	fs2<<"E"<<E;
	fs2<<"F"<<F;
	//stereo rectify
	cout << "stereo rectify..." << endl;
	stereoRectify(intrMatFirst, distCoeffsFirst, intrMatSec, distCoffesSec, imageSize, R, T, RFirst,
			RSec, PFirst, PSec, Q, 0, 1, imageSize, &validRoi[0], &validRoi[1]);
    
	fs2<<"RF"<<F;
	fs2<<"RS"<<F;
	fs2<<"PF"<<F;
	fs2<<"PS"<<F;
	fs2<<"Q"<<Q;
	fs2.release();
	//read pictures for 3d-reconstruction
	namedWindow("canvas", 1);
	cout << "read the picture for 3d-reconstruction...";
	Mat canvas(imageSize.height, imageSize.width * 2, CV_8UC3), viewLeft, viewRight;
	Mat canLeft = canvas(Rect(0, 0, imageSize.width, imageSize.height));
	Mat canRight = canvas(Rect(imageSize.width, 0, imageSize.width, imageSize.height));
	string path1 = "/home/lk/project/opencv_practice/practice/calibrate/image/";
	viewLeft = imread(path1+imageList[cameraIdFirst], 1);
	viewRight = imread(path1+imageList[cameraIdSec], 1);
	viewLeft.copyTo(canLeft);
	viewRight.copyTo(canRight);
	cout << "done" << endl;
	imshow("canvas", canvas);
	waitKey(0);

	//stereoRectify
	Mat rmapFirst[2], rmapSec[2], rviewFirst, rviewSec;
	initUndistortRectifyMap(intrMatFirst, distCoeffsFirst, RFirst, PFirst,
			imageSize, CV_16SC2, rmapFirst[0], rmapFirst[1]);
	initUndistortRectifyMap(intrMatSec, distCoffesSec, RSec, PSec,
			imageSize, CV_16SC2, rmapSec[0], rmapSec[1]);
	remap(viewLeft, rviewFirst, rmapFirst[0], rmapFirst[1], INTER_LINEAR);
	remap(viewRight, rviewSec, rmapSec[0], rmapSec[1], INTER_LINEAR);
	rviewFirst.copyTo(canLeft);
	rviewSec.copyTo(canRight);

	rectangle(canLeft, validRoi[0], Scalar(255, 0, 0), 3, 8);
	rectangle(canRight, validRoi[1], Scalar(255, 0, 0), 3, 8);
	for (int j = 0; j <= canvas.rows; j += 16)
		line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
	cout << "stereo rectify done" << endl;
	imshow("canvas", canvas);
	waitKey(0);
}
