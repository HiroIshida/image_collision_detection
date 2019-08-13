#include "image_dif.hpp"
int main()
{ 
  //auto predicate = gen_hsi_filter(0, 1, 0.3, 1.0, 0.5, 0.8);
  auto predicate = gen_hsi_filter(0, 1, 0.3, 1.0, 0.5, 0.8);
  cv::Mat img = cv::imread("/home/h-ishida/catkin_ws/src/contact_localozation/image_collision_detection/src/test.png", CV_LOAD_IMAGE_COLOR);
  auto vec = compute_filtered_center(img, predicate);
  std::cout<< vec[0] << std::endl;
  std::cout<< vec[1] << std::endl;
  

  /*
  cv::namedWindow("Image", CV_WINDOW_AUTOSIZE);
  cv::imshow("Image", img1_filtered);
  cv::waitKey();
  */
}
