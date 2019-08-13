#include "image_dif.hpp"
int main()
{ 
  //auto predicate = gen_hsi_filter(0, 1, 0.3, 1.0, 0.5, 0.8);
  auto predicate = gen_hsi_filter(0, 1, 0.3, 1.0, 0.5, 0.8);
  cv::Mat img1 = cv::imread("/home/h-ishida/catkin_ws/src/pr2_send_me_a_flower/particle_filter/image/pre.png", CV_LOAD_IMAGE_COLOR);
  cv::Mat img2 = cv::imread("/home/h-ishida/catkin_ws/src/pr2_send_me_a_flower/particle_filter/image/post_without_move.png", CV_LOAD_IMAGE_COLOR);
  cv::Mat img1_filtered = convert_bf(img1, predicate);
  cv::Mat img2_filtered = convert_bf(img2, predicate);
  int cost = compute_cost(img1_filtered, img2_filtered);
  std::cout<<cost<<std::endl;

  cv::namedWindow("Image", CV_WINDOW_AUTOSIZE);
  cv::imshow("Image", img1_filtered);
  cv::waitKey();
}
