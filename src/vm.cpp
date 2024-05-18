#include "lane_follow/vm.hpp"
void Sub::mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    //gettimeofday(&start, NULL);
	if (frame.empty()) { std::cerr << "frame empty!" << std::endl; return; }
	ROI = frame(cv::Rect(0, 270, 640, 90));
	cv::cvtColor(ROI, gray, cv::COLOR_BGR2GRAY);
	meangray = gray + (cv::Scalar(100) - mean(gray));
	cv::threshold(meangray, bin, 120, 255, cv::THRESH_BINARY);
	//라인 160
	//레인 불 켬 120, 130
	cnt = cv::connectedComponentsWithStats(bin, labels, stats, centroids);
	cv::cvtColor(bin, color, cv::COLOR_GRAY2BGR);
	centermin = abs(centerq.back() - centroids.at<double>(1, 0));
	centerymin = abs(centeryq.back() - centroids.at<double>(1, 1));

	centermin2 = abs(centerq2.back() - centroids.at<double>(1, 0));//오른쪽 라인 최소값 알고리즘
	centerymin2 = abs(centeryq2.back() - centroids.at<double>(1, 1));

	for (int i = 1; i < cnt; i++) {
		p = stats.ptr<int>(i);
		c = centroids.ptr<double>(i);
		if (p[4] > 100) {
			cv::rectangle(color, cv::Rect(p[0], p[1], p[2], p[3]), cv::Scalar(255, 0, 0), 2);
			cv::circle(color, cv::Point(c[0], c[1]), 2, cv::Scalar(255, 0, 0), 2);
			if ((centermin >= abs(centerq.back() - c[0])) || (centerymin >= abs(centeryq.back() - c[1]))) {//왼쪽 라인과 검출된 객체의 차이 중 가장 가까운 객체 검출
				if ((abs(centerq.back() - c[0]) < 150) && (abs(centeryq.back() - c[1]) < 50)) {//이전의 라인과 현재 검출된 객체의 무게중심이 특정값 이하일때만 동작
					cntmin = i;//이전 라인과 현재 검출된 객체가 가장 가까울때의 인덱스 저장
					centermin = abs(centerq.back() - c[0]);//최소값 알고리즘
					centerymin = abs(centeryq.back() - c[1]);//최소값 알고리즘
					nearx = centroids.at<double>(cntmin, 0);//이전 라인무게중심과 가장 가까운 무게중심을 가지는 객체의 무게중심 x값 저장
					neary = centroids.at<double>(cntmin, 1);//이전 라인무게중심과 가장 가까운 무게중심을 가지는 객체의 무게중심 y값 저장
					cx = stats.at<int>(cntmin, 0);
					cy = stats.at<int>(cntmin, 1);
					cw = stats.at<int>(cntmin, 2);
					ch = stats.at<int>(cntmin, 3);//이전 라인의 무게중심과 가장 가까운 객체의 박스 좌표 정보 저장
				}
			}
			if ((centermin2 >= abs(centerq2.back() - c[0])) || (centerymin2 >= abs(centeryq2.back() - c[1]))) {//오른쪽 라인과 검출된 객체의 차이 중 가장 가까운 객체 검출
				if ((abs(centerq2.back() - c[0]) < 150) && (abs(centeryq2.back() - c[1]) < 50)) {//이전의 라인과 현재 검출된 객체의 무게중심이 특정값 이하일때만 동작
					cntmin2 = i;//이전 라인과 현재 검출된 객체가 가장 가까울때의 인덱스 저장
					centermin2 = abs(centerq2.back() - c[0]);//최소값 알고리즘
					centerymin2 = abs(centeryq2.back() - c[1]);//최소값 알고리즘
					nearx2 = centroids.at<double>(cntmin2, 0);//이전 라인무게중심과 가장 가까운 무게중심을 가지는 객체의 무게중심 x값 저장
					neary2 = centroids.at<double>(cntmin2, 1);//이전 라인무게중심과 가장 가까운 무게중심을 가지는 객체의 무게중심 y값 저장
					cx2 = stats.at<int>(cntmin2, 0);
					cy2 = stats.at<int>(cntmin2, 1);
					cw2 = stats.at<int>(cntmin2, 2);
					ch2 = stats.at<int>(cntmin2, 3);//이전 라인의 무게중심과 가장 가까운 객체의 박스 좌표 정보 저장
				}
			}
		}
	}
	//1
	centerq.push(nearx);//검출된 왼쪽라인의 무게중심 x값 푸쉬
	centeryq.push(neary);//검출된 왼쪽라인의 무게중심 y값 푸쉬
	if (abs(centerq.back() - centerq.front()) > 200) {//이전의 라인과 현재 검출된 라인의 무게중심 x좌표 차이가 특정 값 이상이면
		centerq.push(centerq.front());
		centerq.pop();
		centerq.push(centerq.back());
		centerq.pop();
		cx = 0; cy = 0; cw = 0; ch = 0;
		//이전의 라인 무게중심값 저장
	}
	if (abs(centeryq.back() - centeryq.front()) > 60) {//이전의 라인과 현재 검출된 라인의 무게중심 y좌표 차이가 특정 값 이상이면
		centeryq.push(centeryq.front());
		centeryq.pop();
		centeryq.push(centeryq.back());
		centeryq.pop();
		//이전의 라인 무게중심값 저장
	}
	centerqbacksave = centerq.back();//현재 검출된 왼쪽 라인의 좌표를 변수에 저장
	centerysave = centeryq.back();//현재 검출된 왼쪽 라인의 좌표를 변수에 저장


	cv::circle(color, cv::Point(centerqbacksave, centerysave), 2, cv::Scalar(0, 0, 255), 2);
	cv::rectangle(color, cv::Rect(cx, cy, cw, ch), cv::Scalar(0, 0, 255), 2);
		
	centerq2.push(nearx2);//검출된 오른쪽라인의 무게중심 x값 푸쉬
	centeryq2.push(neary2);//검출된 오른쪽라인의 무게중심 y값 푸쉬
	if (abs(centerq2.back() - centerq2.front()) > 200) {//이전의 라인과 현재 검출된 라인의 무게중심 x좌표 차이가 특정 값 이상이면
		centerq2.push(centerq2.front());
		centerq2.pop();
		centerq2.push(centerq2.back());
		centerq2.pop();
		cx2 = 0; cy2 = 0; cw2 = 0; ch2 = 0;
	}
	if (abs(centeryq2.back() - centeryq2.front()) > 60) {//이전의 라인과 현재 검출된 라인의 무게중심 y좌표 차이가 특정 값 이상이면
		centeryq2.push(centeryq2.front());
		centeryq2.pop();
		centeryq2.push(centeryq2.back());
		centeryq2.pop();
	}
	centerqbacksave2 = centerq2.back();//현재 검출된 오른쪽 라인의 좌표를 변수에 저장
	centerysave2 = centeryq2.back();//현재 검출된 오른쪽 라인의 좌표를 변수에 저장
	cv::circle(color, cv::Point(centerqbacksave2, centerysave2), 2, cv::Scalar(0, 0, 255), 2);//검출된 오른쪽 라인의 무게중심에 빨간색 원 그림
	cv::rectangle(color, cv::Rect(cx2, cy2, cw2, ch2), cv::Scalar(0, 0, 255), 2);//검출된 오른쪽 라인에 빨간색 사각형 그림
	cv::line(color, cv::Point(centerq.back(), centeryq.back()), cv::Point(centerq2.back(), centeryq2.back()), cv::Scalar(255, 0, 255), 2);//왼쪽과 오른쪽 라인의 무게중심을 잇는 선 그림
	cv::line(color, cv::Point(320, 0), cv::Point(320, 90), cv::Scalar(0, 255, 255), 2);//영상의x좌표 중심에 수직축 그림
	if (centerysave2 >= centerysave)
		cv::circle(color, cv::Point((centerqbacksave2 - centerqbacksave) / 2 + centerqbacksave, (centerysave2 - centerysave) / 2 + centerysave), 2, cv::Scalar(255, 255, 255), 2);
	else cv::circle(color, cv::Point((centerqbacksave2 - centerqbacksave) / 2 + centerqbacksave, (centerysave - centerysave2) / 2 + centerysave2), 2, cv::Scalar(255, 255, 255), 2);
	//왼쪽 라인과 오른쪽 라인의 무게중심의 중간에 원 그림
	
	err = cameracentroidsx - ((centerqbacksave2 - centerqbacksave) / 2 + centerqbacksave);//왼쪽 라인의 무게중심과 오른쪽 라인의 무게중심의 중간값을 영상의 중간좌표에 뺀 값을 에러에 저장
	lvel = 200 - gain * err;//왼쪽 바퀴 속도
	rvel = -(200 + gain * err);//오른쪽 바퀴 속도
	
	//1
	centerq.pop();//이전 왼쪽 라인 x좌표값 제거
	centeryq.pop();//이전 왼쪽 라인 y좌표값 제거
	//2
	centerq2.pop();//이전 오른쪽 라인 x좌표값 제거
	centeryq2.pop();//이전 오른쪽 라인 x좌표값 제거

	writer1 << frame;
	writer2 << color;
    cv::imshow("frame", frame);
    cv::imshow("color", color);
    cv::waitKey(1);
    RCLCPP_INFO(this->get_logger(), "Received Image : %s,%d,%d", msg->format.c_str(),frame.rows,frame.cols);
}
void Sub::publish_msg()
{
    intmsg.data = err;
	RCLCPP_INFO(this->get_logger(), "Publish: %d", intmsg.data);
	pub_->publish(intmsg);
}
Sub::Sub() : Node("camsub_wsl")
{
    centerq.push(320);
    centeryq.push(45);
	centerq2.push(450);
	centeryq2.push(45);
	writer1.open("output1.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30, cv::Size(640, 360));
	writer2.open("output2.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30, cv::Size(640, 90));
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile, std::bind(&Sub::mysub_callback, this, _1));
    pub_ = this->create_publisher<std_msgs::msg::Int32>("err", qos_profile);
    timer_ = this->create_wall_timer(50ms, std::bind(&Sub::publish_msg, this));
}
  
