#include "vi_init/initial_alignment.h"
#include "glog/logging.h"
//参考崔的笔记5.4.1
//目的是利用IMU预积分所得的第k和第k+1帧之间的旋转矩阵与视觉所解的两帧之间的旋转阵残差，以bias作为变量来优化求解
//这个方法与传感器（相机或雷达）无关，构建的约束为相对旋转残差与Bias的约束关系。
void solveGyroscopeBias(
    const std::deque<ImageFrame> &all_image_frame, Vector3d* Bgs){
  Matrix3d A;
  Vector3d b;
  Vector3d delta_bg;
  A.setZero();
  b.setZero();
  
  for(int i = 0; i < all_image_frame.size()-2; ++i){
    const ImageFrame& frame_i = all_image_frame[i];
    const ImageFrame& frame_j = all_image_frame[i+1];
    MatrixXd tmp_A(3, 3);
    tmp_A.setZero();
    VectorXd tmp_b(3);
    tmp_b.setZero();
    //从相机求得的i,j帧之间的相对旋转
    Eigen::Quaterniond q_ij(frame_i.R.transpose() * frame_j.R);
    //旋转对于Bias的雅克比
    tmp_A = frame_j.pre_integration->jacobian.template block<3, 3>(
      IntegrationBase::O_R, IntegrationBase::O_BG);
    //残差，只用四元数的虚部进行计算
    tmp_b = 2 * (frame_j.pre_integration->delta_q.inverse() * q_ij).vec();
    A += tmp_A.transpose() * tmp_A;
    b += tmp_A.transpose() * tmp_b;
  }
  //Cholesky分解求解Bias
  delta_bg = A.ldlt().solve(b);
  // ROS_WARN_STREAM("gyroscope bias initial calibration " << delta_bg.transpose());

  for (int i = 0; i <= all_image_frame.size(); i++)
    Bgs[i] += delta_bg;

  //利用求解得到的Bias重新预积分
  // for(int i = 0; i < all_image_frame.size()-2; ++i){
  //   frame_i = all_image_frame.begin()+i;
  //   frame_j = frame_i+1;
  //   // frame_j = next(frame_i);
  //   frame_j.pre_integration->repropagate(Vector3d::Zero(), Bgs[0]);
  // }
}

//建立坐标系
MatrixXd TangentBasis(Vector3d &g0){
  Vector3d b, c;
  Vector3d a = g0.normalized();
  Vector3d tmp(0, 0, 1);
  if(a == tmp)
      tmp << 1, 0, 0;
  b = (tmp - a * (a.transpose() * tmp)).normalized();
  c = a.cross(b);
  MatrixXd bc(3, 2);
  bc.block<3, 1>(0, 0) = b;
  bc.block<3, 1>(0, 1) = c;
  return bc;
}
//由于速度和尺度求解过程所解得的g没有强约束，因此通过限制其大小的尺度，建立局部坐标系，对重力进行精细调整
/* void RefineGravity(const std::deque<ImageFrame> &all_image_frame,
    Vector3d &g, VectorXd &x){
  Vector3d g0 = g.normalized() * 9.7964;//TODO
  Vector3d lx, ly;
  //VectorXd x;
  int all_frame_count = all_image_frame.size();
  int n_state = all_frame_count * 3 + 2 + 1;

  MatrixXd A{n_state, n_state};
  A.setZero();
  VectorXd b{n_state};
  b.setZero();

  for(int k = 0; k < 4; k++){
    MatrixXd lxly(3, 2);
    lxly = TangentBasis(g0);
    for(int i = 0; i < all_image_frame.size()-2; ++i){
      const ImageFrame& frame_i = all_image_frame[i];
      const ImageFrame& frame_j = all_image_frame[i+1];

      MatrixXd tmp_A(6, 9);
      tmp_A.setZero();
      VectorXd tmp_b(6);
      tmp_b.setZero();

      double dt = frame_j.pre_integration->sum_dt;


      tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
      tmp_A.block<3, 2>(0, 6) = frame_i.R.transpose() * dt * dt / 2 * Matrix3d::Identity() * lxly;
      tmp_A.block<3, 1>(0, 8) = frame_i.R.transpose() * (frame_j.T - frame_i.T) / 100.0;     
      tmp_b.block<3, 1>(0, 0) = frame_j.pre_integration->delta_p + frame_i.R.transpose() * frame_j.R * TIC[0] - TIC[0] - frame_i.R.transpose() * dt * dt / 2 * g0;

      tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
      tmp_A.block<3, 3>(3, 3) = frame_i.R.transpose() * frame_j.R;
      tmp_A.block<3, 2>(3, 6) = frame_i.R.transpose() * dt * Matrix3d::Identity() * lxly;
      tmp_b.block<3, 1>(3, 0) = frame_j.pre_integration->delta_v - frame_i.R.transpose() * dt * Matrix3d::Identity() * g0;


      Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
      //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
      //MatrixXd cov_inv = cov.inverse();
      cov_inv.setIdentity();

      MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
      VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

      A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
      b.segment<6>(i * 3) += r_b.head<6>();

      A.bottomRightCorner<3, 3>() += r_A.bottomRightCorner<3, 3>();
      b.tail<3>() += r_b.tail<3>();

      A.block<6, 3>(i * 3, n_state - 3) += r_A.topRightCorner<6, 3>();
      A.block<3, 6>(n_state - 3, i * 3) += r_A.bottomLeftCorner<3, 6>();
    }
    A = A * 1000.0;
    b = b * 1000.0;
    x = A.ldlt().solve(b);
    VectorXd dg = x.segment<2>(n_state - 3);
    g0 = (g0 + lxly * dg).normalized() * 9.7964;
  }   
  g = g0;
} */


void RefineGravity(const std::deque<ImageFrame> &all_image_frame,
    Vector3d &g, VectorXd &x){
  Vector3d g0 = g.normalized() * 9.7964;//TODO
  Vector3d lx, ly;
  
  for(int k = 0; k < 4; k++){
    MatrixXd lxly(3, 2);
    lxly = TangentBasis(g0);
    const int frame_num = all_image_frame.size();
    MatrixXd A{(frame_num - 1) * 3, frame_num * 3 + 2};//v_k^k,...,g^c0
    A.setZero();
    VectorXd b{(frame_num - 1) * 3};
    b.setZero();
    for(int i = 0; i < all_image_frame.size()-2; ++i){
      const ImageFrame& frame_i = all_image_frame[i];
      const ImageFrame& frame_j = all_image_frame[i+1];

      MatrixXd tmp_A(3, 8);
      tmp_A.setZero();
      VectorXd tmp_b(3);
      tmp_b.setZero();

      double dt = frame_j.pre_integration->sum_dt;
      tmp_A.block<3, 3>(0, 0) = -Matrix3d::Identity();
      tmp_A.block<3, 3>(0, 3) = frame_i.R.transpose() * frame_j.R;
      tmp_A.block<3, 2>(0, 6) = frame_i.R.transpose() * dt * Matrix3d::Identity() * lxly;
      tmp_b.block<3, 1>(0, 0) = frame_j.pre_integration->delta_v
          - frame_i.R.transpose() * dt * Matrix3d::Identity() * g0;
      
      A.block<3, 6>(i * 3, 0) += tmp_A.topLeftCorner<3, 6>();
      A.block<3, 2>(i * 3, frame_num * 3) += tmp_A.topRightCorner<3, 2>();
      b.segment<3>(i * 3) += tmp_b;
    }

    MatrixXd ATA = A.transpose() * A;
    VectorXd ATb = A.transpose() * b;
    ATA = ATA * 1000.0;
    ATb = ATb * 1000.0;
    
    x = ATA.ldlt().solve(ATb);
    VectorXd dg = x.segment<2>(frame_num * 3);
    g0 = (g0 + lxly * dg).normalized() * 9.7964;
  }
  g = g0;
}

//利用预积分所得的位移变化（非传统意义）和速度变化值和经过帧的理想位置之间的差异来构建残差项。建立误差方程JtJx=Jb，通过Cholesky分解来求解理想参数。
bool LinearAlignment(const std::deque<ImageFrame> &all_image_frame, 
    Vector3d &g, Vector3d &T_I_C, VectorXd &x, bool fix_scale){
  /* int all_frame_count = all_image_frame.size();
  LOG(INFO)<<"all_frame_count: "<<all_frame_count;
  int n_state = all_frame_count * 3 + 3 + 3 + 1 ;//V+G+T_I_C+S

  MatrixXd A{n_state, n_state};
  A.setZero();
  VectorXd b{n_state};
  b.setZero();
  
  for(int i = 0; i < all_image_frame.size()-2; ++i){
    const ImageFrame& frame_i = all_image_frame[i];
    const ImageFrame& frame_j = all_image_frame[i+1];
    
    // MatrixXd tmp_A(6, 10);
    MatrixXd tmp_A(6, 13);
    tmp_A.setZero();
    VectorXd tmp_b(6);
    tmp_b.setZero();

    double dt = frame_j.pre_integration->sum_dt;

    tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
    tmp_A.block<3, 3>(0, 6) = frame_i.R.transpose() * dt * dt / 2 * Matrix3d::Identity();
    tmp_A.block<3, 3>(0, 9) = Matrix3d::Identity() - frame_i.R.transpose() * frame_j.R;
    tmp_A.block<3, 1>(0, 12) = frame_i.R.transpose() * (frame_j.T - frame_i.T) / 100.0;

    tmp_b.block<3, 1>(0, 0) = frame_j.pre_integration->delta_p; 
    // + frame_i.R.transpose() * frame_j.R * TIC[0] - TIC[0];
    //cout << "delta_p   " << frame_j.pre_integration->delta_p.transpose() << endl;
    tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
    tmp_A.block<3, 3>(3, 3) = frame_i.R.transpose() * frame_j.R;
    tmp_A.block<3, 3>(3, 6) = frame_i.R.transpose() * dt * Matrix3d::Identity();
    tmp_b.block<3, 1>(3, 0) = frame_j.pre_integration->delta_v;
    //cout << "delta_v   " << frame_j.pre_integration->delta_v.transpose() << endl;

    Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
    //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
    //MatrixXd cov_inv = cov.inverse();
    cov_inv.setIdentity();

    MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
    VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;
 
    A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
    b.segment<6>(i * 3) += r_b.head<6>();

    A.bottomRightCorner<4, 4>() += r_A.bottomRightCorner<4, 4>();
    b.tail<4>() += r_b.tail<4>();

    A.block<6, 4>(i * 3, n_state - 4) += r_A.topRightCorner<6, 4>();
    A.block<4, 6>(n_state - 4, i * 3) += r_A.bottomLeftCorner<4, 6>();
  }
  A = A * 1000.0;
  b = b * 1000.0;
  x = A.ldlt().solve(b);
  double s = x(n_state - 1) / 100.0;
  LOG(INFO)<<"Estimated scale: "<< s;
  g = x.segment<3>(n_state - 7);
  T_I_C = x.segment<3>(n_state - 4);
  LOG(INFO)<<" Result g " << g.norm() << " " << g.transpose();
  LOG(INFO)<<" Result T_I_C " << T_I_C.norm() << " " << T_I_C.transpose();
  LOG(INFO)<<" Result x " << x.norm() << " " << x.transpose();
  if(fabs(g.norm() - G.norm()) > 1.0 || s < 0){
    return false; 
  }

  RefineGravity(all_image_frame, g, x);
  s = (x.tail<1>())(0) / 100.0;
  (x.tail<1>())(0) = s;
  ROS_DEBUG_STREAM(" refine     " << g.norm() << " " << g.transpose());
  if(s < 0.0 )
    return false;   
  else
    return true; */


  //The first step, solving gravity and velocities using \beta of pre-integration.
  {
    // int all_frame_count = all_image_frame.size();
    // int n_state = all_frame_count * 3 + 3;

    // MatrixXd A{n_state, n_state};
    // A.setZero();
    // VectorXd b{n_state};
    // b.setZero();
    const int frame_num = all_image_frame.size();
    MatrixXd A{(frame_num - 1) * 3, frame_num * 3 + 3};//v_k^k,...,g^c0
    A.setZero();
    VectorXd b{(frame_num - 1) * 3};
    b.setZero();
   
    for(int i = 0; i < all_image_frame.size()-2; ++i){
      const ImageFrame& frame_i = all_image_frame[i];
      const ImageFrame& frame_j = all_image_frame[i+1];

      MatrixXd tmp_A(3, 9);
      tmp_A.setZero();
      VectorXd tmp_b(3);
      tmp_b.setZero();

      double dt = frame_j.pre_integration->sum_dt;

      // tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
      // tmp_A.block<3, 3>(0, 6) = frame_i.R.transpose() * dt * dt / 2 * Matrix3d::Identity();
      // tmp_A.block<3, 1>(0, 9) = frame_i.R.transpose() * (frame_j.T - frame_i.T) / 100.0;  //为啥除以100?
      // tmp_b.block<3, 1>(0, 0) = frame_j.pre_integration->delta_p + frame_i.R.transpose() * frame_j.R * TIC[0] - TIC[0];
      //cout << "delta_p   " << frame_j.pre_integration->delta_p.transpose() << endl;
      tmp_A.block<3, 3>(0, 0) = -Matrix3d::Identity();
      tmp_A.block<3, 3>(0, 3) = frame_i.R.transpose() * frame_j.R;
      tmp_A.block<3, 3>(0, 6) = frame_i.R.transpose() * dt * Matrix3d::Identity();
      tmp_b.block<3, 1>(0, 0) = frame_j.pre_integration->delta_v;
      //cout << "delta_v   " << frame_j.pre_integration->delta_v.transpose() << endl;

      // Matrix<double, 3, 3> cov_inv = Matrix<double, 3, 3>::Zero();
      //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
      //MatrixXd cov_inv = cov.inverse();
      // cov_inv.setIdentity();

      // MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A; //9*9
      // VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b; //9*1

      // A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();//v_k,v_{k+1}
      // b.segment<6>(i * 3) += r_b.head<6>();

      // A.bottomRightCorner<3, 3>() += r_A.bottomRightCorner<3, 3>();//g^{c0}
      // b.tail<3>() += r_b.tail<3>();

      // A.block<6, 3>(i * 3, n_state - 3) += r_A.topRightCorner<6, 3>();
      // A.block<3, 6>(n_state - 3, i * 3) += r_A.bottomLeftCorner<3, 6>();

      A.block<3, 6>(i * 3, 0) += tmp_A.topLeftCorner<3, 6>();
      A.block<3, 3>(i * 3, frame_num * 3) += tmp_A.topRightCorner<3, 3>();
      b.segment<3>(i * 3) += tmp_b;
    }
    MatrixXd ATA = A.transpose() * A;
    VectorXd ATb = A.transpose() * b;
    ATA = ATA * 1000.0;
    ATb = ATb * 1000.0;
    
    x = ATA.ldlt().solve(ATb);
    g = x.segment<3>(frame_num * 3);
    // ROS_WARN_STREAM(" result g " << g.norm() << " " << g.transpose());
  }
  LOG(INFO)<<"Before refinement: "<<g.norm()<<" "<<g.transpose();
  RefineGravity(all_image_frame, g, x);
  LOG(INFO)<<"After refinement: "<<g.norm()<<" "<<g.transpose();
    
  // Solving G succeed, then solve the extrinsic translation and scale.
  if(fabs(g.norm() - 9.7964) < 0.5){
    const int frame_num = all_image_frame.size();; 
    
    if(fix_scale){
      MatrixXd A{(frame_num-1) * 3, 3};
      A.setZero();
      VectorXd b{(frame_num-1) * 3};
      b.setZero();

      map<double, ImageFrame>::iterator frame_i;
      map<double, ImageFrame>::iterator frame_j;
      for(int i = 0; i < all_image_frame.size()-2; ++i){
        const ImageFrame& frame_i = all_image_frame[i];
        const ImageFrame& frame_j = all_image_frame[i+1];

        MatrixXd tmp_A(3, 3);
        tmp_A.setZero();
        VectorXd tmp_b(3);
        tmp_b.setZero();

        double dt = frame_j.pre_integration->sum_dt;
        
        tmp_A.block<3, 3>(0, 0) = 
          Matrix3d::Identity() - frame_i.R.transpose()*frame_j.R;
        // tmp_A.block<3, 1>(0, 3) = 
        //   frame_i.R.transpose() * (frame_j.T - frame_i.T);

        tmp_b.block<3, 1>(0, 0) = 
          frame_j.pre_integration->delta_p 
          + dt*Matrix3d::Identity() * x.segment<3>(i * 3)
          - (frame_i.R.transpose() * dt * dt / 2) * g
          - frame_i.R.transpose() * (frame_j.T - frame_i.T);

        A.block<3, 3>(i * 3, 0) += tmp_A;
        b.segment<3>(i * 3) += tmp_b;
      }
      MatrixXd ATA = A.transpose() * A;
      VectorXd ATb = A.transpose() * b;
      // ATA = ATA * 1000.0;
      // ATb = ATb * 1000.0;
      
      VectorXd translation;
      translation = ATA.ldlt().solve(ATb);
      T_I_C = translation.segment<3>(0);
      return true;
    }else{
      MatrixXd A{(frame_num-1) * 3, 4};
      A.setZero();
      VectorXd b{(frame_num-1) * 3};
      b.setZero();

      map<double, ImageFrame>::iterator frame_i;
      map<double, ImageFrame>::iterator frame_j;
      for(int i = 0; i < all_image_frame.size()-2; ++i){
        const ImageFrame& frame_i = all_image_frame[i];
        const ImageFrame& frame_j = all_image_frame[i+1];

        MatrixXd tmp_A(3, 4);
        tmp_A.setZero();
        VectorXd tmp_b(3);
        tmp_b.setZero();

        double dt = frame_j.pre_integration->sum_dt;
        
        tmp_A.block<3, 3>(0, 0) = 
          Matrix3d::Identity() - frame_i.R.transpose()*frame_j.R;
        tmp_A.block<3, 1>(0, 3) = 
          frame_i.R.transpose() * (frame_j.T - frame_i.T);

        tmp_b.block<3, 1>(0, 0) = 
          frame_j.pre_integration->delta_p 
          + dt*Matrix3d::Identity() * x.segment<3>(i * 3)
          - (frame_i.R.transpose() * dt * dt / 2) * g;

        A.block<3, 4>(i * 3, 0) += tmp_A;
        b.segment<3>(i * 3) += tmp_b;
      }
      MatrixXd ATA = A.transpose() * A;
      VectorXd ATb = A.transpose() * b;
      // ATA = ATA * 1000.0;
      // ATb = ATb * 1000.0;
      
      VectorXd t_s;
      t_s = ATA.ldlt().solve(ATb);
      x = t_s;
      double s = (t_s.tail<1>())(0);
      T_I_C = t_s.segment<3>(0);
      if(s > 0){
        LOG(INFO)<<"Estimated scale " << s;
        return true;
      }else{
        return false;
      }
    }
  }else{
    return false;
  }
}


bool LinearAlignmentWithScale(const std::deque<ImageFrame> &all_image_frame, 
    Vector3d &g, Vector3d &T_I_C, VectorXd &x, double scale_guess){

  //The first step, solving gravity and velocities using \beta of pre-integration.
  {
    const int frame_num = all_image_frame.size();
    MatrixXd A{(frame_num - 1) * 3, frame_num * 3 + 3};//v_k^k,...,g^c0
    A.setZero();
    VectorXd b{(frame_num - 1) * 3};
    b.setZero();
   
    for(int i = 0; i < all_image_frame.size()-2; ++i){
      const ImageFrame& frame_i = all_image_frame[i];
      const ImageFrame& frame_j = all_image_frame[i+1];

      MatrixXd tmp_A(3, 9);
      tmp_A.setZero();
      VectorXd tmp_b(3);
      tmp_b.setZero();

      double dt = frame_j.pre_integration->sum_dt;

      tmp_A.block<3, 3>(0, 0) = -Matrix3d::Identity();
      tmp_A.block<3, 3>(0, 3) = frame_i.R.transpose() * frame_j.R;
      tmp_A.block<3, 3>(0, 6) = frame_i.R.transpose() * dt * Matrix3d::Identity();
      tmp_b.block<3, 1>(0, 0) = frame_j.pre_integration->delta_v;
      //cout << "delta_v   " << frame_j.pre_integration->delta_v.transpose() << endl;

      A.block<3, 6>(i * 3, 0) += tmp_A.topLeftCorner<3, 6>();
      A.block<3, 3>(i * 3, frame_num * 3) += tmp_A.topRightCorner<3, 3>();
      b.segment<3>(i * 3) += tmp_b;
    }
    MatrixXd ATA = A.transpose() * A;
    VectorXd ATb = A.transpose() * b;
    ATA = ATA * 1000.0;
    ATb = ATb * 1000.0;
    
    x = ATA.ldlt().solve(ATb);
    g = x.segment<3>(frame_num * 3);
    // ROS_WARN_STREAM(" result g " << g.norm() << " " << g.transpose());
  }
    
  // Solving G succeed, then solve the extrinsic translation and scale.
  const double G = 9.7964;
  if(fabs(g.norm() - G) < 1.0){
    const int frame_num = all_image_frame.size();; 

    MatrixXd A{(frame_num-1) * 3, 3};
    A.setZero();
    VectorXd b{(frame_num-1) * 3};
    b.setZero();

    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    for(int i = 0; i < all_image_frame.size()-2; ++i){
      const ImageFrame& frame_i = all_image_frame[i];
      const ImageFrame& frame_j = all_image_frame[i+1];

      MatrixXd tmp_A(3, 3);
      tmp_A.setZero();
      VectorXd tmp_b(3);
      tmp_b.setZero();

      double dt = frame_j.pre_integration->sum_dt;
      
      tmp_A.block<3, 3>(0, 0) = 
        Matrix3d::Identity() - frame_i.R.transpose()*frame_j.R;
      // tmp_A.block<3, 1>(0, 3) = 
      //   frame_i.R.transpose() * (frame_j.T - frame_i.T);

      tmp_b.block<3, 1>(0, 0) = 
        frame_j.pre_integration->delta_p 
        + dt*Matrix3d::Identity() * x.segment<3>(i * 3)
        - (frame_i.R.transpose() * dt * dt / 2) * g
        - scale_guess * frame_i.R.transpose() * (frame_j.T - frame_i.T);

      A.block<3, 3>(i * 3, 0) += tmp_A;
      b.segment<3>(i * 3) += tmp_b;
    }
    MatrixXd ATA = A.transpose() * A;
    VectorXd ATb = A.transpose() * b;
    ATA = ATA * 1000.0;
    ATb = ATb * 1000.0;
    
    VectorXd translation;
    translation = ATA.ldlt().solve(ATb);
    T_I_C = translation.segment<3>(0);
    return true;
  }else{
    return false;
  }
}

bool VisualIMUAlignment(const std::deque<ImageFrame> &all_image_frame, 
    Vector3d* Bgs, Vector3d &g, Vector3d &T_I_C, VectorXd &x, bool fix_scale){
  // solveGyroscopeBias(all_image_frame, Bgs);
  if(LinearAlignment(all_image_frame, g, T_I_C, x, fix_scale))
    return true;
  else 
    return false;
}


bool VisualIMUAlignmentWithScale(const std::deque<ImageFrame> &all_image_frame, 
                    Vector3d* Bgs, Vector3d &g, Vector3d &T_I_C, VectorXd &x,
                    double scale_guess){
  if(LinearAlignmentWithScale(all_image_frame, g, T_I_C, x, scale_guess))
    return true;
  else 
    return false;
}