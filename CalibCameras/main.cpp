/* jiaqiong */

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <fstream>

/* for windows */
#   include<direct.h>
#   include<io.h>
#	include<sys/stat.h>
#	include<sys/types.h>
#	include<ShlObj.h>

typedef std::vector<std::string> chess_pic_names;

int scan_files(std::string const& dirname, chess_pic_names & filenames)
{
	WIN32_FIND_DATAA data;
	HANDLE hf = FindFirstFileA((dirname+"/*").c_str(), &data);
	int file_count = 0;
	do
	{
		if (!std::strcmp(data.cFileName, "."))
			continue;
		if (!std::strcmp(data.cFileName, ".."))
			continue;
		filenames.push_back(data.cFileName);
		file_count += 1;
	} while (FindNextFileA(hf, &data) != 0);
	FindClose(hf);
	return file_count;
}

std::string sanitize_path(std::string const& path)
{
	if (path.empty())
		return "";
	std::string result = path;
	std::replace(result.begin(), result.end(), '\\', '/');
	for (std::size_t i = 0; i < result.size();)
	{
		if (result[i] == '/' && result[i + 1] == '/')
			result.erase(i, 1);
		else
			i += 1;
	}

	if (result.size() > 1 && result[result.size() - 1] == '/')
		result.erase(result.end() - 1);

	return result;
}

std::string join_path(std::string const& path1, std::string const& path2)
{
	std::string p2 = sanitize_path(path2);
	if (!p2.empty() && p2[0] == '/')
		return sanitize_path(path1) + p2;
	return sanitize_path(path1) + '/' + p2;
}


int main()
{
	/*-------------------------------*/
	std::string pathname = sanitize_path("../chess_pic_data");
	chess_pic_names  chesspic_vector;//preserve chess picture names
	std::string destpath = "../result_dir";//destination directory to conserve result files
	if (::_mkdir(destpath.c_str()) < 0)
	{
		std::cout << "warning: mkdir failed, save files in current directory" << std::endl;
		destpath.clear();
	}
	std::ofstream fout(join_path(destpath, "calib_result.txt"));//conserve calibration result
	scan_files(pathname, chesspic_vector);//find chess picture names in the pathname directory
	std::sort(chesspic_vector.begin(), chesspic_vector.end(), std::less<std::string>());

	/*-------------------------------*/
	std::cout << "starting to extract corner points..." << std::endl;
	cv::Size board_size = cv::Size(4, 6);
	cv::Size image_size;
	std::vector<std::vector<cv::Point2f>> images_points_list;
	for (size_t i = 0; i < chesspic_vector.size(); i++)
	{
		cv::Mat image = cv::imread(join_path(pathname, chesspic_vector.at(i)));
		if (i == 0)
		{
			image_size.width = image.cols; image_size.height = image.rows;
		}
		std::cout << "image " << i << " : width->" << image.cols << " height->" << image.rows << std::endl;
		std::vector<cv::Point2f> image_points;//corner points on every image
		if (cv::findChessboardCorners(image, board_size, image_points) == 0)
		{
			std::cout << "error: can't find corner points on the chess board!" << std::endl;
			std::exit(-1);
		}else
		{
			cv::Mat image_gray;
			cv::cvtColor(image, image_gray, CV_RGB2GRAY);
			cv::find4QuadCornerSubpix(image_gray, image_points, cv::Size(5, 5));//extract subpix information to refine
			images_points_list.push_back(image_points);
			/* display the chess board picture with corner points information */
			drawChessboardCorners(image_gray, board_size, image_points, true); 
			cv::imshow("Camera Calibration", image_gray);
			cv::waitKey(1000);
		}
	}

	/*-------------------------------*/
	std::cout << "starting to calibrate camera..." << std::endl;
	cv::Size square_size = cv::Size(10, 10);//every chess square size measured
	std::vector<std::vector<cv::Point3f>> object_points;// conserve world points
	cv::Mat intrinsicMat = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
	cv::Mat distCoeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0));// k1, k2, p1, p2, k3
	std::vector<cv::Mat> tvecsMat; std::vector<cv::Mat> rvecsMat;
	for (std::size_t i = 0; i < chesspic_vector.size(); i++)
	{
		std::vector<cv::Point3f> image_point3f;
		for (std::size_t j = 0; j < board_size.height; j++)
		{
			for (std::size_t k = 0; k < board_size.width; k++)
			{
				cv::Point3f temp;
				temp.x = j*square_size.width;
				temp.y = k*square_size.height;
				temp.z = 0;
				image_point3f.push_back(temp);
			}
		}
		object_points.push_back(image_point3f);
	}
	cv::calibrateCamera(object_points, images_points_list, image_size, intrinsicMat, distCoeffs, rvecsMat, tvecsMat, 0);

	/*-------------------------------*/
	std::cout << "starting to evaluate the calibration result..." << std::endl;
	double total_err = 0.0;
	double err = 0.0;
	fout << "calibration error of each image: " << std::endl;
	for (int i = 0; i < object_points.size();i++)
	{
		std::vector<cv::Point3f> tempPoint3fSet = object_points.at(i);
		std::vector<cv::Point2f> new_image_points;
		cv::projectPoints(tempPoint3fSet, rvecsMat[i], tvecsMat[i], intrinsicMat, distCoeffs, new_image_points);
		std::vector<cv::Point2f> temp_points = images_points_list.at(i);
		cv::Mat temp_points_mat = cv::Mat(1, temp_points.size(), CV_32FC2);
		cv::Mat new_image_points_mat = cv::Mat(1, new_image_points.size(), CV_32FC2);
		for (std::size_t j = 0; j < temp_points.size(); j++)
		{
			temp_points_mat.at<cv::Vec2f>(0, j) = cv::Vec2f(temp_points.at(j).x, temp_points.at(j).y);
			new_image_points_mat.at<cv::Vec2f>(0, j) = cv::Vec2f(new_image_points.at(j).x, new_image_points.at(j).y);
		}
		err = cv::norm(new_image_points_mat, temp_points_mat, cv::NORM_L2);
		total_err += err /= temp_points.size();
		fout << "image " << i << ": " << err << " pixels" << std::endl;
	}

	/*-------------------------------*/
	std::cout << "starting to save the calibration result..." << std::endl;
	fout << "average error of total images: " << total_err / chesspic_vector.size() << " pixels" << std::endl;
	fout << "intrinsic camera mat: " << std::endl;
	fout << intrinsicMat << std::endl << std::endl;
	fout << "distort coefficients: " << std::endl;
	fout << distCoeffs << std::endl << std::endl;
	cv::Mat rot_mat = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
	for (std::size_t i = 0; i < chesspic_vector.size(); i++)
	{
		fout << "image " << i << " t vec: " << std::endl;
		fout << tvecsMat.at(i) << std::endl;
		cv::Rodrigues(rvecsMat[i], rot_mat);//convert r vec to rotation matrix
		fout << "image " << i << " r mat: " << std::endl;
		fout << rot_mat << std::endl;
	}
	fout << std::endl;
	fout.close();

	/*-------------------------------*/
	std::cout << "calibrate images and save..." << std::endl;
	cv::Mat mapx = cv::Mat(image_size, CV_32FC1);
	cv::Mat mapy = cv::Mat(image_size, CV_32FC1);
	cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
	std::string outfilename;
	std::stringstream ss;
	for (std::size_t i = 0; i < chesspic_vector.size(); i++)
	{
		std::cout << "undistort image " << i << "..." << std::endl;
		cv::initUndistortRectifyMap(intrinsicMat, distCoeffs, R, intrinsicMat, image_size, CV_32FC1, mapx, mapy);
		outfilename.clear(); 
		ss.clear();
		ss << i;
		ss >> outfilename;
		outfilename += ".bmp";
		std::string outfilepath = join_path(destpath, outfilename);
		cv::Mat sourceImage = cv::imread(join_path(pathname, chesspic_vector.at(i)));
		cv::Mat destImage = sourceImage.clone();
		cv::remap(sourceImage, destImage, mapx, mapy, cv::INTER_LINEAR);
		cv::imwrite(outfilepath, destImage);
	}
	std::cout << "save finished !" << std::endl;
	/*-------------------------------*/

	return 1;
}