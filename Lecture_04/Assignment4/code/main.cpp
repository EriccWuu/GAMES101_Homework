#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 5) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f lerp(cv::Point2f a, cv::Point2f b, double t)
{
    return (1-t)*a + t*b;
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, double t) 
{
    // TODO: Implement de Casteljau's algorithm
    auto count = control_points.size();
    std::vector<cv::Point2f> P;
    if(count == 1)
    {
        return control_points[0];
    }
    else if(count > 1)
    {
        for(int i = 0 ; i < count - 1 ; i++)
        {
            P.push_back(lerp(control_points[i], control_points[i+1], t));
        }
    }
    else
    {
        std::cout<<"Error."<<std::endl;
    }
    
    return recursive_bezier(P, t);
    //return cv::Point2f();
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for(double t = 0.0; t <= 1.0; t += 0.001)
    {
        auto point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;

        //cv::Point2f a(point.x-1,point.y), b(point.x+1, point.y), c(point.x, point.y+1), d(a.x+1, point.y-1);
        cv::Point2f a(point.x-1,point.y), b(point.x+1, point.y), c(point.x, point.y-1), d(a.x+1, point.y+1);
        double k1 = 1 - (pow(point.x - a.x,2.0) + pow(point.y - a.y,2.0))/2.0,
               k2 = 1 - (pow(point.x - b.x,2.0) + pow(point.y - b.y,2.0))/2.0,
               k3 = 1 - (pow(point.x - c.x,2.0) + pow(point.y - c.y,2.0))/2.0,
               k4 = 1 - (pow(point.x - d.x,2.0) + pow(point.y - d.y,2.0))/2.0;
        window.at<cv::Vec3b>(a.y, a.x)[1] = window.at<cv::Vec3b>(a.y, a.x)[1]?window.at<cv::Vec3b>(a.y, a.x)[1] : 255*k1;
        window.at<cv::Vec3b>(b.y, b.x)[1] = window.at<cv::Vec3b>(b.y, b.x)[1]?window.at<cv::Vec3b>(b.y, b.x)[1] : 255*k2;
        window.at<cv::Vec3b>(c.y, c.x)[1] = window.at<cv::Vec3b>(c.y, c.x)[1]?window.at<cv::Vec3b>(c.y, c.x)[1] : 255*k3;
        window.at<cv::Vec3b>(d.y, d.x)[1] = window.at<cv::Vec3b>(d.y, d.x)[1]?window.at<cv::Vec3b>(d.y, d.x)[1] : 255*k4;
    }

}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 3) 
        {
            // naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve_3P.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
