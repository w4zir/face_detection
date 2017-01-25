#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32MultiArray.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/contrib/contrib.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Subscriber face_data_sub_;
	image_transport::Publisher image_pub_;
	cv::Mat img;
	std::vector<int> face_coord;
	std::vector<std::string> face_n_image;
	std::vector<int> face_ids;
	std::vector<std::string> image_names;
	std::string folder = "/home/khan/phd_ws/face_ws/src/face_detection/images/*";
public:
	ImageConverter()
: it_(nh_)
{
		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);

		face_data_sub_ = nh_.subscribe("/faceCoord", 1, &ImageConverter::getFaceCoord, this);

		image_pub_ = it_.advertise("/overlay_image/output_video", 1);

		// read all files in folder
		cv::glob(folder, image_names);

		// Random shuffle
		std::random_shuffle(image_names.begin(), image_names.end());

		//	std::cout<< "input image rows:" << img.rows <<"\t cols:" << img.cols << std::endl;
		cv::namedWindow(OPENCV_WINDOW);
}

	~ImageConverter()
	{
		cv::destroyWindow(OPENCV_WINDOW);
	}

	cv::Mat getUserImage(int user_id)
	{
		std::string image_name;

		std::vector<int>::iterator it;
		it = std::find(face_ids.begin(), face_ids.end(), user_id);
		if(it != face_ids.end())
		{
			auto pos = std::distance(face_ids.begin(), it);
			image_name = face_n_image[pos];

		}else
		{
			image_name = getRandomImageName();
			face_ids.push_back(user_id);
			face_n_image.push_back(image_name);
		}

		// load image and return
		img = cv::imread(image_name,CV_LOAD_IMAGE_COLOR);
		if (img.empty()) {
			std::cout << "Error reading file." << std::endl;
			exit(0);
		}
		return(img);
	}

	std::string getRandomImageName()
	{
		int file_count = image_names.size();
		int rand_no = std::rand()%file_count;
		return (image_names[rand_no]);
	}


	void getFaceCoord(const std_msgs::Int32MultiArray::ConstPtr& msg)
	{
		if (msg->data.size() < 10)
		{
			// do nothing if no face detected
		} else
		{
			face_coord.clear();
			int vec_size = (int)msg->data.size();
			for (int i=6; i<vec_size; i+=6)
			{
				face_coord.push_back(msg->data[i]);
				face_coord.push_back(msg->data[i+1]);
				face_coord.push_back(msg->data[i+2]);
				face_coord.push_back(msg->data[i+3]);
				face_coord.push_back(msg->data[i-2]);
			}

		}
	}
	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		// check if faces detected
		int vec_size = (int)face_coord.size();
		std::cout << "No of faces detected:" << (int)vec_size/4 << std::endl;
		if (vec_size != 0)
		{
			for (int i=0; i<vec_size; i+=5)
			{
				cv::Mat tmp_img = getUserImage(face_coord[i+4]);
				cv::resize(tmp_img,tmp_img,cv::Size(face_coord[i+2],face_coord[i+3]));
				tmp_img.copyTo(cv_ptr->image(cv::Rect(face_coord[i] ,face_coord[i+1] , face_coord[i+2], face_coord[i+3])));
			}
		}

		// Update GUI Window
		cv::imshow(OPENCV_WINDOW, cv_ptr->image);
		cv::waitKey(3);

		// Output modified video stream
		image_pub_.publish(cv_ptr->toImageMsg());
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_overlay");
	ImageConverter ic;
	ros::spin();
	return 0;
}
