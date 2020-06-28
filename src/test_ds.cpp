
#include "ds.hpp"

int main(){
    typedef DoubleSphereCamera::Vec2 Vec2;
    typedef DoubleSphereCamera::Vec4 Vec4;
    typedef DoubleSphereCamera::VecN VecN;

    VecN cam_param_vector;
    cam_param_vector << 484.9926595731991, 482.9666824294857, 641.834683493474, 450.7731511073807, -0.17184553814057157, 0.6302589803408714;
    std::cout << "Camera parameter vector: " << cam_param_vector << std::endl;
    DoubleSphereCamera cam(cam_param_vector);

    double x, y, z;
    x = 0;
    y = 0;
    z = 5;

    Vec4 p3d(x, y, z, 0);
    Vec2 proj_res;
    Vec4 p_normalized = p3d.normalized();
    cam.project(p3d, proj_res);
    std::cout << "project result: (" << proj_res[0] << ", " << proj_res[1] << ")" << std::endl;

    double u, v;
    u = 100;
    v = 0;

    Vec2 p2d(u, v);
    Vec4 uproj_res;
    cam.unproject(p2d, uproj_res);
    std::cout << "unproject result: (" << uproj_res[0] << ", " << uproj_res[1]
              << ", "<< uproj_res[2] << ", " << uproj_res[3] << ")" << std::endl;

    double xmin, ymin, xmax, ymax;
    xmin=377, xmax=489, ymin=527, ymax=635;

    // Eigen::MatrixXcd box_p2d(4, 2);
    // Eigen::MatrixXcd box_uproj_res(4, 4);
    // // std::vector<Vec2> box_p2d;
    // // std::vector<Vec4> box_uproj_res;

    // box_p2d << xmin, ymin,
    //            xmax, ymin,
    //            xmin, ymax,
    //            xmax, ymax;
    // for (int i = 0; i< 4; i++){
    //     cam.unproject(box_p2d[i], box_uproj_res[i]);
    //     std::cout << "unproject result: (" << box_uproj_res[i][0] << ", " << box_uproj_res[i][1]
    //             << ", "<< box_uproj_res[i][2] << ", " << box_uproj_res[i][3] << ")" << std::endl;
    // }
    // box_p2d.push_back(p2d(xmin, ymin));
    // box_p2d.push_back(p2d(xmax, ymin));
    // box_p2d.push_back(p2d(xmin, ymax));
    // box_p2d.push_back(p2d(xmax, ymax));
    // for (int i = 0; i< 4; i++){
    //     cam.unproject(box_p2d[i], uproj_res);
    //     std::cout << "unproject result: (" << uproj_res[0] << ", " << uproj_res[1]
    //             << ", "<< uproj_res[2] << ", " << uproj_res[3] << ")" << std::endl;
    // }
    Vec4 uproj_center;
    Eigen::Matrix<double, 2, 1> p2d0, p2d1, p2d2, p2d3;
    Eigen::Matrix<double, 4, 1> p3d0, p3d1, p3d2, p3d3;
    p2d0 << xmin, ymin;
    p2d1 << xmax, ymin;
    p2d2 << xmin, ymax;
    p2d3 << xmax, ymax;
    uproj_center << 0, 0, 0, 0;

    std::cout << "p2d0: (" << p2d0[0] << ", " << p2d0[1] << ")" << std::endl;
    cam.unproject(p2d0, p3d0);
    uproj_center += p3d0;
    std::cout << "p3d0: (" << p3d0[0] << ", " << p3d0[1] << ", " << p3d0[2] << ", " << p3d0[3] << ")" << std::endl;
    std::cout << "uproj_center: (" << uproj_center[0] << ", " << uproj_center[1] << ", " << uproj_center[2] << ", " << uproj_center[3] << ")" << std::endl;
    cam.unproject(p2d1, p3d1);
    uproj_center += p3d1;
    std::cout << "p3d1: (" << p3d1[0] << ", " << p3d1[1] << ", " << p3d1[2] << ", " << p3d1[3] << ")" << std::endl;
    std::cout << "uproj_center: (" << uproj_center[0] << ", " << uproj_center[1] << ", " << uproj_center[2] << ", " << uproj_center[3] << ")" << std::endl;
    cam.unproject(p2d2, p3d2);
    uproj_center += p3d2;
    std::cout << "p3d2: (" << p3d2[0] << ", " << p3d2[1] << ", " << p3d2[2] << ", " << p3d2[3] << ")" << std::endl;
    std::cout << "uproj_center: (" << uproj_center[0] << ", " << uproj_center[1] << ", " << uproj_center[2] << ", " << uproj_center[3] << ")" << std::endl;
    cam.unproject(p2d3, p3d3);
    uproj_center += p3d3;
    std::cout << "p3d3: (" << p3d3[0] << ", " << p3d3[1] << ", " << p3d3[2] << ", " << p3d3[3] << ")" << std::endl;
    std::cout << "uproj_center: (" << uproj_center[0] << ", " << uproj_center[1] << ", " << uproj_center[2] << ", " << uproj_center[3] << ")" << std::endl;
    uproj_center *= 0.25;
    double max_pair_unit_dist = 0;
    std::cout << "uproj_center: (" << uproj_center[0] << ", " << uproj_center[1] << ", " << uproj_center[2] << ", " << uproj_center[3] << ")" << std::endl;
    std::cout << "uproj_center norm: " << uproj_center.norm() << std::endl;
    // pair unit distance: (0, 1),  (0, 2),  (0, 3),  (1, 2),  (1, 3),  (2, 3)
    // corresponding to the 4 edges and two diagnoal lines of the box quadrilateral
    // that is: top edge, left edge, backslash diagonal, slash diagonal, right edge, bottom edge
    std::vector<double> pair_unit_dist_list;
    pair_unit_dist_list.push_back((p3d0 - p3d1).norm());
    pair_unit_dist_list.push_back((p3d0 - p3d2).norm());
    pair_unit_dist_list.push_back((p3d0 - p3d3).norm());
    pair_unit_dist_list.push_back((p3d1 - p3d2).norm());
    pair_unit_dist_list.push_back((p3d1 - p3d3).norm());
    pair_unit_dist_list.push_back((p3d2 - p3d3).norm());
    std::sort(pair_unit_dist_list.begin(), pair_unit_dist_list.end());

    max_pair_unit_dist = pair_unit_dist_list[pair_unit_dist_list.size()-1];
    std::cout << "max_pair_unit_dist: " << max_pair_unit_dist << std::endl;

    return 0;
}