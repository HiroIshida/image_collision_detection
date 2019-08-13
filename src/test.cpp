#include "image_dif.hpp"
int main()
{ 
  //auto predicate = gen_hsi_filter(0, 1, 0.3, 1.0, 0.5, 0.8);
  auto predicate = gen_hsi_filter(0, 1, 0.3, 1.0, 0.5, 0.8);
  cv::Mat img = cv::imread("/home/h-ishida/catkin_ws/src/contact_localozation/image_collision_detection/src/test.png", CV_LOAD_IMAGE_COLOR);
  auto roi = determine_ROI(img, predicate, 100);
  auto img_new = img(roi);

  //cv::circle(img, cv::Point(round(vec[1]), round(vec[0])), 10, cv::Scalar(0, 0, 200), 3, 4);
  //cv::circle(img, cv::Point(200, 200), 100, cv::Scalar(0, 0, 200), 3, 4);
  cv::namedWindow("Image", CV_WINDOW_AUTOSIZE);
  cv::imshow("Image", img_new);
  cv::waitKey();
}
