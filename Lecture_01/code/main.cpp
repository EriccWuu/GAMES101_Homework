#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    rotation_angle = 2*MY_PI * rotation_angle / 360; // angel to degree
    model << cos(rotation_angle), -sin(rotation_angle), 0, 0,
             sin(rotation_angle), cos(rotation_angle), 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    Eigen::Matrix4f m_persp2ortho, m_ortho, i, j;
    float height, width;
    height = -2*zNear*tan(eye_fov/2); // camera is in O, so zNear and zFar < 0
    width = height*aspect_ratio;

    // perspective to Orthographic
    m_persp2ortho << zNear, 0, 0, 0,
                     0, zNear, 0, 0,
                     0, 0, zNear + zFar, -zNear * zFar,
                     0, 0, 1, 0;
    
    // canonicalize into canonical space coordinate ---> [-1,1]^3
    i << 2/width, 0, 0, 0,
         0, 2/height, 0, 0,
         0, 0, 2/(zNear - zFar), 0,
         0, 0, 0, 1;
    
    j << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
    
    // orthographic
    m_ortho = i*j;

    projection << m_ortho * m_persp2ortho;

    return projection;
}

// Rotate along any Vector
Eigen::Matrix4f get_rotation(Vector3f axis, float rotation_angle)
{
    Eigen::Matrix4f m_rota;
    Eigen::Matrix3f N, m, I;

    axis = axis/axis.norm();
    I << 1.0f, 0.0f, 0.0f, 0.0f,1.0f, 0.0f, 0.0f, 0.0f, 1.0f;
    N << 0, -axis(2), axis(1), axis(2), 0, -axis(0), -axis(1), axis(0), 0;
    rotation_angle = 2.0f * MY_PI * rotation_angle / 360.0f;

    m = I*cos(rotation_angle) + (1-cos(rotation_angle))*axis*axis.transpose() + N*sin(rotation_angle);
    m_rota.fill(0);
    m_rota.block<3, 3>(0, 0) = m;
    m_rota(3,3) = 1.0f;

    return m_rota;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";
    Eigen::Vector3f axis(0,0,1);

    // get_rotation(axis , 45);

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 10};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, -0.1, -50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
