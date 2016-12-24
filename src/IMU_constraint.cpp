 // Eigen                               
#include <Eigen/Core>                  
#include <Eigen/Geometry>              

#include "Thirdparty/g2o_EXTERNAL/ceres/autodiff.h"

#include "IMUErrorModel.h"
#include "IMU_constraint.h"

IMUProcessor::IMUProcessor()
  :speed_bias_1(Eigen::Matrix<double, 9,1>::Zero()), bStatesInitialized(false),reinit_data(true),
   is_measurement_good(false),imuSize(0), pred_speed_bias_2(Eigen::Matrix<double, 9,1>::Zero())
    
{
  time_pair[0]=-1;
  time_pair[1]=-1;
  
  
}
void IMUProcessor::GrapIMU(const sensor_msgs::ImuConstPtr& msgIMU)
{
    Eigen::Matrix<double, 7,1> transMat;
    transMat[0] = msgIMU->header.stamp.toSec();
    transMat[1] = msgIMU->linear_acceleration.x;
    transMat[2] = msgIMU->linear_acceleration.y;
    transMat[3] = msgIMU->linear_acceleration.z;
    transMat[4] = msgIMU->angular_velocity.x;
    transMat[5] = msgIMU->angular_velocity.y;
    transMat[6] = msgIMU->angular_velocity.z;
    measureSub.push_back(transMat);

    //std::cout<<"数目："<<measurement.size()<<std::endl;
}
void IMUProcessor::initStates(const double timestamp)      
{
  time_pair[0] = timestamp;
  assert(!bStatesInitialized);   
  bool is_meas_good = getObservation(timestamp);
  //assert(!is_meas_good);         
  //resetStates(Ts1tow, sb1);      
                                         
}
bool IMUProcessor::getObservation(double tk)
{
  std::vector<Eigen::Matrix<double, 7,1> > measureTemp; 
  measureRaw.assign(measureSub.begin()+imuSize, measureSub.end());
  imuSize = measureSub.size();
  

  if(!bStatesInitialized) {
    bStatesInitialized=true; 
    unsigned int i=0;
    while(i<measureRaw.size())
    {
      double dt=measureRaw[i][0]-time_pair[0];
      if(dt<0)
      {
        measureTemp.push_back(measureRaw[i]); // 
      }  

      i++;
    }
    measurement_.assign(measureTemp.begin(), measureTemp.end());
  }else{
    unsigned int i=0;
    while(i<measureRaw.size())
    {
      double dt1=measureRaw[i][0]-time_pair[0];
      double dt2=measureRaw[i][0]-time_pair[1];
      if(dt1>0 && dt2<0)
      {
        measureTemp.push_back(measureRaw[i]); // 
      }

      i++;
    }
    measurement_.front()=measurement_.back(); 
    measurement_.resize(1);                  
    measurement_.insert(measurement_.end(), measureTemp.begin(),measureTemp.end()); 
  }

  is_measurement_good = true;
  return is_measurement_good;
} 
void IMUProcessor::resetStates(const Sophus::SE3d &Ts1tow, const Eigen::Matrix<double, 9,1> & sb1)
{
  T_s1_to_w=Ts1tow;              
  speed_bias_1=sb1;              
}

template<typename Scalar>
void strapdown_local_quat_bias(const Eigen::Matrix<Scalar,3,1>& rs0, const Eigen::Matrix<Scalar,3,1> & vs0, 
                               const Eigen::Quaternion<Scalar>& qs0_2_s,const Eigen::Matrix<Scalar,3,1> & a, 
                               const Eigen::Matrix<Scalar,3,1>& w, Scalar dt,
                               const Eigen::Matrix<Scalar, 6,1>& gomegas0,Eigen::Matrix<Scalar,3,1>* rs0_new, 
                               Eigen::Matrix<Scalar,3,1>* vs0_new, Eigen::Quaternion<Scalar>* qs0_2_s_new)
{
  Eigen::Matrix<Scalar,3,1> wie2s0=gomegas0.template tail<3>();
  //Update attitude
  // method (1) second order integration
  Eigen::Quaternion<Scalar> qe=rvec2quat(wie2s0*dt);
  (*qs0_2_s_new)=qs0_2_s*qe;
  Eigen::Quaternion<Scalar> qb=rvec2quat(-w*dt);
  (*qs0_2_s_new)=qb*(*qs0_2_s_new);

  Eigen::Matrix<Scalar,3,1> vel_inc1=(qs0_2_s.conjugate()._transformVector(a*dt)+(*qs0_2_s_new).conjugate()._transformVector(a*dt))*Scalar(0.5);   

  Eigen::Matrix<Scalar,3,1> vel_inc2=(gomegas0.template head<3>()-Scalar(2)*wie2s0.cross(vs0))*dt;

  (*vs0_new)=vs0+vel_inc1+vel_inc2;
  //Update_pos
  (*rs0_new)=rs0+((*vs0_new)+vs0)*dt*Scalar(0.5);
  
}
/*void strapdown_local_quat_bias(const Eigen::Matrix<double,3,1>& rs0, const Eigen::Matrix<double,3,1> & vs0, 
                               const Eigen::Quaternion<double>& qs0_2_s,const Eigen::Matrix<double,3,1> & a, 
                               const Eigen::Matrix<double,3,1>& w, double dt,
                               const Eigen::Matrix<double, 6,1>& gomegas0,Eigen::Matrix<double,3,1>* rs0_new, 
                               Eigen::Matrix<double,3,1>* vs0_new, Eigen::Quaternion<double>* qs0_2_s_new)
{
  Eigen::Matrix<double,3,1> wie2s0=gomegas0.template tail<3>();
  //Update attitude
  // method (1) second order integration
  Eigen::Quaternion<double> qe=rvec2quat(wie2s0*dt);
  (*qs0_2_s_new)=qs0_2_s*qe;
  Eigen::Quaternion<double> qb=rvec2quat(-w*dt);
  (*qs0_2_s_new)=qb*(*qs0_2_s_new);

  Eigen::Matrix<double,3,1> vel_inc1=(qs0_2_s.conjugate()._transformVector(a*dt)+(*qs0_2_s_new).conjugate()._transformVector(a*dt))*double(0.5);   

  Eigen::Matrix<double,3,1> vel_inc2=(gomegas0.template head<3>()-double(2)*wie2s0.cross(vs0))*dt;

  (*vs0_new)=vs0+vel_inc1+vel_inc2;
  //Update_pos
  (*rs0_new)=rs0+((*vs0_new)+vs0)*dt*double(0.5);
  
}
*/
template<typename Scalar>
void sys_local_dcm_bias(const Eigen::Matrix<Scalar, 3,1> & rs0, const Eigen::Matrix<Scalar, 3,1> & vs0,const Eigen::Quaternion<Scalar>& qs0_2_s,
                        const Eigen::Matrix<Scalar, 3,1> & acc, const Eigen::Matrix<Scalar, 3,1> & gyro, Scalar dt,
                        const Eigen::Matrix<Scalar, 3,1> & q_n_a,const Eigen::Matrix<Scalar, 3,1> & q_n_w, const Eigen::Matrix<Scalar, 3,1> & q_n_ba,
                        const Eigen::Matrix<Scalar, 3,1>& q_n_bw, Eigen::Matrix<Scalar, 15, 15>* P)
{
  const int navStates=9;
  Eigen::Matrix<Scalar, navStates, 6> Nnav=Eigen::Matrix<Scalar, navStates, 6>::Zero();

  Eigen::Matrix<Scalar, 3,3> Cs02s=qs0_2_s.toRotationMatrix();

  Nnav.template block<3,3>(3,0)=Cs02s.transpose(); //velocity
  Nnav.template block<3,3>(6,3)=-Cs02s.transpose(); //attitude
  Eigen::Matrix<Scalar, navStates, navStates> Anav=Eigen::Matrix<Scalar, navStates, navStates>::Zero();
  Anav.template block<3,3>(0,3).template setIdentity(); // rs in s0
  Anav.template block<3,3>(3,6)=skew3d(Cs02s.transpose()*acc);
  Eigen::Matrix<Scalar, 6,6> Rimu=Eigen::Matrix<Scalar, 6,6>::Zero();

  Rimu.template topLeftCorner<3,3>()=q_n_a.asDiagonal();
  Rimu.template bottomRightCorner<3,3>()=q_n_w.asDiagonal();
  Eigen::Matrix<Scalar,6,6> Qimu_d=Eigen::Matrix<Scalar, 6,6>::Zero();
  Qimu_d.template topLeftCorner<3,3>()=q_n_ba.asDiagonal()*dt;
  Qimu_d.template bottomRightCorner<3,3>()=q_n_bw.asDiagonal()*dt;

  Eigen::Matrix<Scalar, navStates,navStates> Anav_d=Eigen::Matrix<Scalar, navStates,navStates>::Identity()+dt*Anav;  //Use 1st order taylor series to di
  Eigen::Matrix<Scalar, navStates, navStates> Qnav= (Nnav*Rimu).eval()*Nnav.transpose();
  Eigen::Matrix<Scalar, navStates, navStates> Qnav_d=dt*Scalar(0.5)*(Anav_d*Qnav+Qnav*Anav_d.transpose());      //Use trapezoidal rule to discretize Rim

  Eigen::Matrix<Scalar, 15,15> STM=Eigen::Matrix<Scalar, 15,15>::Zero();

  STM.template topLeftCorner<navStates,navStates>()=Anav_d;
  STM.template block<navStates, 6>(0,navStates)=Nnav*dt;
  STM.template block<6,6>(navStates, navStates).setIdentity();

  Eigen::Matrix<Scalar, 15,15> Qd=Eigen::Matrix<Scalar, 15,15>::Zero();
                                                                                           
  Qd.template topLeftCorner<navStates,navStates>()=Qnav_d;
  Qd.template block<6,6>(navStates, navStates)=Qimu_d;
  Qd.template block<navStates,6>(0,navStates)=Nnav*Qimu_d*dt*Scalar(0.5);
  Qd.template block<6,navStates>(navStates,0)=Qd.template block<navStates,6>(0,navStates).template transpose();

  (*P)=STM*(*P)*STM.transpose()+Qd;// covariance of the navigation states and imu error terms
}

template<typename Scalar>
void predictStates(const Sophus::SE3Group<Scalar> &T_sk_to_w, const Eigen::Matrix<Scalar, 9,1>& speed_bias_k,                             
                   const Scalar * time_pair,const std::vector<Eigen::Matrix<Scalar, 7,1> >& measurements, 
                   const Eigen::Matrix<Scalar, 6,1> & gwomegaw,const Eigen::Matrix<Scalar, 12, 1>& q_n_aw_babw,
                   Sophus::SE3Group<Scalar>* pred_T_skp1_to_w, Eigen::Matrix<Scalar, 3,1>* pred_speed_kp1,
                   Eigen::Matrix<Scalar, 15,15> *P, const Eigen::Matrix<Scalar, 27,1> shape_matrices= Eigen::Matrix<Scalar, 27,1>::Zero())

{
  bool predict_cov=(P!=NULL);
  int every_n_reading=2;// update covariance every n IMU readings,

  // the eventual covariance has little to do with this param as long as it remains small
  Eigen::Matrix<Scalar, 3,1> r_new, r_old(T_sk_to_w.translation()), v_new, v_old(speed_bias_k.template head<3>());                         
  Eigen::Quaternion<Scalar> q_new, q_old(T_sk_to_w.unit_quaternion().conjugate());
  //assert(dt>Scalar(0)&&dt<=Scalar(0.01+1e-8));
  Scalar dt=measurements[1][0]-time_pair[0];
  Scalar covupt_time(time_pair[0]);//the time to which the covariance is updated. N.B. the initial covariance is updated to $t_k$ 
  IMUErrorModel<Scalar> iem(speed_bias_k.template block<6,1>(3,0));
  iem.estimate(measurements[0].template block<3,1>(1,0), measurements[0].template block<3,1>(4,0));//estimte accumulation  angle velocity
  const Eigen::Matrix<Scalar, 3,1> qna=q_n_aw_babw.template head<3>(), qnw=q_n_aw_babw.template segment<3>(3),
                                   qnba=q_n_aw_babw.template segment<3>(6),qnbw=q_n_aw_babw.template tail<3>();
  if(predict_cov)
  {
    sys_local_dcm_bias(r_old, v_old, q_old, iem.a_est, iem.w_est,
                       measurements[1][0]-covupt_time,qna, qnw, qnba, qnbw,P);
    //for more precise covariance update, we can use average estimated accel and angular rate over n(every_n_reading) IMU readings
    covupt_time=measurements[1][0];
  }
                   
  strapdown_local_quat_bias( r_old, v_old, q_old, iem.a_est, iem.w_est,
                             dt, gwomegaw, &r_new, &v_new, &q_new);
  r_old=r_new;                                                                                            
  v_old=v_new;
  q_old=q_new;
  int unsigned i=1;
  for (; i<measurements.size()-1;++i){
    dt=measurements[i+1][0]-measurements[i][0];
    iem.estimate(measurements[i].template block<3,1>(1,0), measurements[i].template block<3,1>(4,0));
    strapdown_local_quat_bias( r_old, v_old, q_old, iem.a_est,
                               iem.w_est, dt, gwomegaw, &r_new, &v_new, &q_new);
    if(predict_cov&&(i%every_n_reading==0))                                                    
    {
      sys_local_dcm_bias(r_old, v_old, q_old, iem.a_est,
                         iem.w_est, measurements[i+1][0]-covupt_time,qna, qnw, qnba, qnbw,P);
      covupt_time=measurements[i+1][0];
    }
    
    r_old=r_new;
    v_old=v_new;
    q_old=q_new;
  }
  
  dt=time_pair[1]-measurements[i][0];//the last measurement                                           
  //assert(dt>=Scalar(0)&& dt<Scalar(0.01));
  iem.estimate(measurements[i].template block<3,1>(1,0), measurements[i].template block<3,1>(4,0));
  strapdown_local_quat_bias( r_old, v_old, q_old, iem.a_est, iem.w_est,
                             dt, gwomegaw, &r_new, &v_new, &q_new);
 
  if(predict_cov)
  {    
    sys_local_dcm_bias(r_old, v_old, q_old, iem.a_est, iem.w_est,
                       time_pair[1]-covupt_time,qna, qnw, qnba, qnbw,P);
    covupt_time=time_pair[1];
  }    

  pred_T_skp1_to_w->setQuaternion(q_new.conjugate());
  pred_T_skp1_to_w->translation()=r_new;
  (*pred_speed_kp1)=v_new;
  
}
/*
void predictStates(const Sophus::SE3Group<double> &T_sk_to_w, const Eigen::Matrix<double, 9,1>& speed_bias_k,                             
                   const double * time_pair,const std::vector<Eigen::Matrix<double, 7,1> >& measurements, 
                   const Eigen::Matrix<double, 6,1> & gwomegaw,const Eigen::Matrix<double, 12, 1>& q_n_aw_babw,
                   Sophus::SE3Group<double>* pred_T_skp1_to_w, Eigen::Matrix<double, 3,1>* pred_speed_kp1,
                   Eigen::Matrix<double, 15,15> *P, const Eigen::Matrix<double, 27,1> shape_matrices= Eigen::Matrix<double, 27,1>::Zero())

{
  bool predict_cov=(P!=NULL);
  int every_n_reading=2;// update covariance every n IMU readings,

  // the eventual covariance has little to do with this param as long as it remains small
  Eigen::Matrix<double, 3,1> r_new, r_old(T_sk_to_w.translation()), v_new, v_old(speed_bias_k.template head<3>());                         
  Eigen::Quaternion<double> q_new, q_old(T_sk_to_w.unit_quaternion().conjugate());
  //assert(dt>Scalar(0)&&dt<=Scalar(0.01+1e-8));
  double dt=measurements[1][0]-time_pair[0];
  double covupt_time(time_pair[0]);//the time to which the covariance is updated. N.B. the initial covariance is updated to $t_k$ 
  IMUErrorModel iem(speed_bias_k.template block<6,1>(3,0));
  iem.estimate(measurements[0].template block<3,1>(1,0), measurements[0].template block<3,1>(4,0));//estimte accumulation  angle velocity
  const Eigen::Matrix<double, 3,1> qna=q_n_aw_babw.template head<3>(), qnw=q_n_aw_babw.template segment<3>(3),
                                   qnba=q_n_aw_babw.template segment<3>(6),qnbw=q_n_aw_babw.template tail<3>();
  if(predict_cov)
  {
    sys_local_dcm_bias(r_old, v_old, q_old, iem.a_est, iem.w_est,
                       measurements[1][0]-covupt_time,qna, qnw, qnba, qnbw,P);
    //for more precise covariance update, we can use average estimated accel and angular rate over n(every_n_reading) IMU readings
    covupt_time=measurements[1][0];
  }
                   
  strapdown_local_quat_bias( r_old, v_old, q_old, iem.a_est, iem.w_est,
                             dt, gwomegaw, &r_new, &v_new, &q_new);
  r_old=r_new;                                                                                            
  v_old=v_new;
  q_old=q_new;
  int unsigned i=1;
  for (; i<measurements.size()-1;++i){
    dt=measurements[i+1][0]-measurements[i][0];
    iem.estimate(measurements[i].template block<3,1>(1,0), measurements[i].template block<3,1>(4,0));
    strapdown_local_quat_bias( r_old, v_old, q_old, iem.a_est,
                               iem.w_est, dt, gwomegaw, &r_new, &v_new, &q_new);
    if(predict_cov&&(i%every_n_reading==0))                                                    
    {
      sys_local_dcm_bias(r_old, v_old, q_old, iem.a_est,
                         iem.w_est, measurements[i+1][0]-covupt_time,qna, qnw, qnba, qnbw,P);
      covupt_time=measurements[i+1][0];
    }
    
    r_old=r_new;
    v_old=v_new;
    q_old=q_new;
  }
  
  dt=time_pair[1]-measurements[i][0];//the last measurement                                           
  //assert(dt>=Scalar(0)&& dt<Scalar(0.01));
  iem.estimate(measurements[i].template block<3,1>(1,0), measurements[i].template block<3,1>(4,0));
  strapdown_local_quat_bias( r_old, v_old, q_old, iem.a_est, iem.w_est,
                             dt, gwomegaw, &r_new, &v_new, &q_new);
 
  if(predict_cov)
  {    
    sys_local_dcm_bias(r_old, v_old, q_old, iem.a_est, iem.w_est,
                       time_pair[1]-covupt_time,qna, qnw, qnba, qnbw,P);
    covupt_time=time_pair[1];
  }    

  pred_T_skp1_to_w->setQuaternion(q_new.conjugate());
  pred_T_skp1_to_w->translation()=r_new;
  (*pred_speed_kp1)=v_new;
  
}
*/
Eigen::Quaternion<double> rvec2quat(const Eigen::Matrix<double,3,1>& rvec)               
{                                                                                                            
  //normalize                                                                                              
  double rot_ang=rvec.norm();    //assume always positive                                                  
  if (rot_ang<double(1e-18))                                                                               
    return Eigen::Quaternion<double>(double(1), double(0),double(0),double(0));                          
  else{                                                                                                    
    double f= sin(rot_ang*double(0.5))/rot_ang;                                                          
    return Eigen::Quaternion<double>(cos(rot_ang*double(0.5)), rvec[0] * f, rvec[1] * f, rvec[2] * f);   
  }                                                                                                        

}  

Sophus::SE3d IMUProcessor::propagate(const double time_frame)                                          
{         
  Eigen::Matrix<double, 15,15>* holder=NULL;  
  time_pair[1] = time_frame;
  //std::cout<<"调试:"<<time_pair[1]-time_pair[0]<<std::endl;
  bool is_meas_good = getObservation(time_frame);                                                
  //assert(is_meas_good); 
  Eigen::Vector3d tempVs0inw;                                                                        
  //std::cout<<"调试:"<<measurement_[0][0]<<std::endl;
  predictStates(T_s1_to_w, speed_bias_1, time_pair,                                       
                measurement_, imu_.gwomegaw, imu_.q_n_aw_babw,                          
                &pred_T_s2_to_w, &tempVs0inw,holder);                                    
  pred_speed_bias_2.head<3>()=tempVs0inw;                                                            
  pred_speed_bias_2.tail<6>()=speed_bias_1.tail<6>();     //biases do not change in propagation      
  
  Sophus::SE3d pred_Tr_delta=pred_T_s2_to_w*imu_.T_imu_from_cam;                                     

  pred_Tr_delta=pred_Tr_delta.inverse()*(T_s1_to_w*imu_.T_imu_from_cam);                             
  pred_Tr_delta.translation().setZero();  // remove the translation part   
  T_s1_to_w=pred_T_s2_to_w;                                                                          
  speed_bias_1=pred_speed_bias_2; 
  time_pair[0] = time_pair[1];  
  
  return pred_Tr_delta;                                                                              
}                                                                                                        
bool G2oVertexSpeedBias::read(std::istream& is) {        
  Eigen::Matrix<double, 9,1> est;                             
          for (int i=0; i<9; i++)                              
                    is >> est[i];                                    
              setEstimate(est);                                    
                  return true;                                         
}                                                        
bool G2oVertexSpeedBias::write(std::ostream& os) const { 
      for (int i=0; i<9; i++)                              
                os << estimate()[i] << " ";                      
          return os.good();                                    
}                                                        

bool G2oEdgeIMUConstraint::write(std::ostream& os) const 
{                           
  os <<g2o_IMU->id() << std::endl;                             
  for (unsigned int i=0; i<measurement().size(); i++){    
    for(int j=0; j<7; ++j)                              
      os  << measurement()[i][j] << " ";              
    os<<std::endl;                                           
  }                                                       
  for (int i=0; i<15; i++){                               
    for (int j=i; j<15; j++){                           
      os << information()(i,j)<<" ";                  
    }                                                   
    os<<std::endl;                                           
  }                                                       
  return os.good();                                       
}                                                           

bool G2oEdgeIMUConstraint::read(std::istream& is) 
{         
  int paramId;                                            
  is >> paramId;                                          
  setParameterId(0, paramId);                             
  int num_measurements;                                   
  is >>num_measurements;                                  
  _measurement.resize(num_measurements);                  
  for (int i=0; i<num_measurements; ++i){                 
    for(int j=0; j<7; ++j)                              
      is  >> _measurement[i][j];                      
  }                                                       
  for (int i=0; i<15; i++)                                
    for (int j=i; j<15; j++) {                          
      is >> information()(i,j);                       
      if (i!=j)                                       
      
        information()(j,i)=information()(i,j);      
  }                                                   
  return true;                                            

}                                                           

template <typename T>                                                                                                     
bool G2oEdgeIMUConstraint::operator ()( const  T* pTw2ck, const T* epsilonk, const T* pXsbk, const T* pTw2ckp1, const T* pXsbkp1, T* error) const  
{                                                                                                                         
  const Eigen::Map<const Sophus::SE3Group<T> > se3Tw2ck(pTw2ck); //qxyzw txyz                                           
  typename Eigen::Matrix<T, 6, 1, Eigen::ColMajor>::ConstMapType epsk(epsilonk);                                        
  typename Eigen::Matrix<T, 9, 1, Eigen::ColMajor>::ConstMapType Xsbk(pXsbk);                                           
  Eigen::Matrix<T,9,1> cast_Xsbk=Xsbk;                                                                                  
  const Eigen::Map<const Sophus::SE3Group<T> > se3Tw2ckp1(pTw2ckp1); //qxyzw txyz                                       
  typename Eigen::Matrix<T, 9, 1, Eigen::ColMajor>::ConstMapType Xsbkp1(pXsbkp1);                                       
  const G2oIMUParameters * params_imu                                                                                   
    = static_cast<const G2oIMUParameters *>(parameter(0));                                                        
  Eigen::Map<Eigen::Matrix<T,9,1> > value(error);//delta eps and vel                                                    

  Sophus::SE3Group<T> perturbed_Tw2ck=Sophus::SE3Group<T>::exp(epsk)*se3Tw2ck;                                                  
  Sophus::SE3Group<T>  T_s1_to_w=(params_imu->T_imu_from_cam.cast<T>()*perturbed_Tw2ck).inverse();                      

  std::vector<Eigen::Matrix<T,7,1> > cast_measurement(_measurement.size());                                             
  for(unsigned int jack=0; jack<_measurement.size(); ++jack)                                                            
    cast_measurement[jack]=_measurement[jack].cast<T>();                                                              
  T cast_time_frames[2]={T(time_frames[0]), T(time_frames[1])};                                                         
  Eigen::Matrix<T, 6, 1> cast_gwomegaw=params_imu->gwomegaw.cast<T>();                                                  
  Eigen::Matrix<T,12,1> cast_q=params_imu->q_n_aw_babw.cast<T>();                                                       
  Sophus::SE3Group<T>  pred_T_s2_to_w;                                                                                  
  Eigen::Matrix<T,3,1> pred_speed_2;                                                                                           

  Eigen::Matrix<T, 15,15> *holder=NULL;                                                                                 
  predictStates(T_s1_to_w, cast_Xsbk, cast_time_frames,                                                                 
      cast_measurement, cast_gwomegaw, cast_q,                                                                
      &pred_T_s2_to_w, &pred_speed_2, holder);                                                                
  Sophus::SE3Group<T> predTckp12w=  pred_T_s2_to_w*params_imu->T_imu_from_cam.cast<T>();                                        
  value.template head<6>()=Sophus::SE3Group<T>::log(predTckp12w*se3Tw2ckp1);                                                    
  value.template tail<3>()=pred_speed_2-Xsbkp1.template head<3>();  
   
  return true;                                                                                                          
}                                                                                                                         


/*bool G2oEdgeIMUConstraint::operator ()( const  double* pTw2ck, const double* epsilonk, const double* pXsbk, const double* pTw2ckp1, const double* pXsbkp1, double* error) const   
//must use const because AutoDiff::Differentiate's first argument is const *this                                           
{                                                                                                                          
  //given IMU measurements, and states at time 1,i.e., t(k), predict states at time 2, i.e., t(k+1)                      
  const Eigen::Map<const Sophus::SE3Group<double> > se3Tw2ck(pTw2ck); //qxyzw txyz      
  typename Eigen::Matrix<double, 6, 1, Eigen::ColMajor>::ConstMapType epsk(epsilonk);   
  typename Eigen::Matrix<double, 9, 1, Eigen::ColMajor>::ConstMapType Xsbk(pXsbk);      
  Eigen::Matrix<double,9,1> cast_Xsbk=Xsbk;                                             
  const Eigen::Map<const Sophus::SE3Group<double> > se3Tw2ckp1(pTw2ckp1); //qxyzw txyz  
  typename Eigen::Matrix<double, 9, 1, Eigen::ColMajor>::ConstMapType Xsbkp1(pXsbkp1);  
  const G2oIMUParameters * params_imu                                              
             = static_cast<const G2oIMUParameters *>(parameter(0));                   
  Eigen::Map<Eigen::Matrix<double,9,1> > value(error);//delta eps and vel               
                                                                                    
  Sophus::SE3Group<double> perturbed_Tw2ck=Sophus::SE3Group<double>::exp(epsk)*se3Tw2ck;                            
  Sophus::SE3Group<double>  T_s1_to_w=(params_imu->T_imu_from_cam.cast<double>()*perturbed_Tw2ck).inverse();
                                                                                                 
  std::vector<Eigen::Matrix<double,7,1> > cast_measurement(_measurement.size());                       
  for(unsigned int jack=0; jack<_measurement.size(); ++jack)                                      
      cast_measurement[jack]=_measurement[jack].cast<double>();                                        
  double cast_time_frames[2]={double(time_frames[0]), double(time_frames[1])};                                   
  Eigen::Matrix<double, 6, 1> cast_gwomegaw=params_imu->gwomegaw.cast<double>();                            
  Eigen::Matrix<double,12,1> cast_q=params_imu->q_n_aw_babw.cast<double>();                                 
  Sophus::SE3Group<double>  pred_T_s2_to_w;                                                            
  Eigen::Matrix<double,3,1> pred_speed_2;                                                                     
  Eigen::Matrix<double, 15,15> *holder=NULL;                                                           
  predictStates(T_s1_to_w, cast_Xsbk, cast_time_frames,                                           
                cast_measurement, cast_gwomegaw, cast_q,                                          
                &pred_T_s2_to_w, &pred_speed_2, holder);                                          
  Sophus::SE3Group<double> predTckp12w=  pred_T_s2_to_w*params_imu->T_imu_from_cam.cast<double>();                  
  value.template head<6>()=Sophus::SE3Group<double>::log(predTckp12w*se3Tw2ckp1);                              
  value.template tail<3>()=pred_speed_2-Xsbkp1.template head<3>();                                
  return true;                                                                                    
                                                                                                
}
*/

void G2oEdgeIMUConstraint::computeError()          
{                         
  //first two nodes     
  const G2oVertexSE3 * T_c1_from_world                             
        = static_cast<const G2oVertexSE3*>(_vertices[0]);        
  const G2oVertexSpeedBias * speed_bias_1                          
        = static_cast<const G2oVertexSpeedBias*>(_vertices[1]);  
  //second two nodes                                               
  const G2oVertexSE3 * T_c2_from_world                                                               
            = static_cast<const G2oVertexSE3*>(_vertices[2]);                                          
  const G2oVertexSpeedBias * speed_bias_2                                                            
            = static_cast<const G2oVertexSpeedBias*>(_vertices[3]);                                    
                                                                                                     
  Eigen::Matrix<double, 9,1 > error_posvel= Eigen::Matrix<double, 9,1 >::Zero();                     
  Eigen::Matrix<double, 6, 1> zero_delta= Eigen::Matrix<double, 6, 1>::Zero();                                     
  (*this)(T_c1_from_world->estimate().data(), zero_delta.data(), speed_bias_1->estimate().data(),    
              T_c2_from_world->estimate().data(), speed_bias_2->estimate().data(), error_posvel.data() );
  _error.head<9>()=error_posvel;                                                                     
  _error.tail<6>()=(speed_bias_1->estimate()).tail<6>()-(speed_bias_2->estimate()).tail<6>();        
}
/*
struct LogDeltaSE3Vee                                                                                
{                                                                                                    
  bool operator()(const double* predTskp12w, const double* pDeltaxpsi, const double* pTw2skp1, double* value) const    
  {                                                                                                
    Eigen::Map<const Sophus::SE3Group<double> > se3Tskp12wConst(predTskp12w); //qxyzw txyz            
    Sophus::SE3Group<double> se3Tskp12w=se3Tskp12wConst;                                              
    typename Eigen::Matrix<double, 6, 1, Eigen::ColMajor>::ConstMapType deltaxpsi(pDeltaxpsi);        
    se3Tskp12w.translation() =se3Tskp12wConst.translation() - deltaxpsi.head<3>();      
    Eigen::Matrix<double,3,1> rvec= deltaxpsi.tail<3>();                                            
    se3Tskp12w.setQuaternion(quaternionFromSmallAngle(rvec)*se3Tskp12wConst.unit_quaternion());  

    const Eigen::Map<const Sophus::SE3Group<double> > se3Tw2skp1(pTw2skp1); //qxyzw txyz              
    Eigen::Map<Eigen::Matrix<double,6,1> > tang(value);                                               

    tang= (se3Tskp12w*se3Tw2skp1).log();                                                         
    return true;                                                                                 

  }                                                                                                

};*/

struct LogDeltaSE3Vee                                                                                  
{                                                                                                      
  template<typename T>                                                                               
    bool operator()(const T* predTskp12w, const T* pDeltaxpsi, const T* pTw2skp1, T* value) const      
    {                                                                                                  
      Eigen::Map<const Sophus::SE3Group<T> > se3Tskp12wConst(predTskp12w); //qxyzw txyz              

      Sophus::SE3Group<T> se3Tskp12w=se3Tskp12wConst;                                                
      typename Eigen::Matrix<T, 6, 1, Eigen::ColMajor>::ConstMapType deltaxpsi(pDeltaxpsi);          
      se3Tskp12w.translation() =se3Tskp12wConst.translation() - deltaxpsi.template head<3>();        
      Eigen::Matrix<T,3,1> rvec= deltaxpsi.template tail<3>();                                              
      se3Tskp12w.setQuaternion(quaternionFromSmallAngle(rvec)*se3Tskp12wConst.unit_quaternion());    

      const Eigen::Map<const Sophus::SE3Group<T> > se3Tw2skp1(pTw2skp1); //qxyzw txyz                
      Eigen::Map<Eigen::Matrix<T,6,1> > tang(value);                                                 
      tang= (se3Tskp12w*se3Tw2skp1).log();                                                           
      return true;                                                                                   
    }                                                                                                  
};                                                                                                     


void G2oEdgeIMUConstraint::calcAndSetInformation(const G2oIMUParameters & params_imu)                                                          
{                                                                                                                     
    
  const G2oVertexSE3 * T_c1_from_world = static_cast<const G2oVertexSE3*>(_vertices[0]);                                                         
  const G2oVertexSpeedBias * speed_bias_1 = static_cast<const G2oVertexSpeedBias*>(_vertices[1]);                                                   
  const G2oVertexSE3 * T_c2_from_world = static_cast<const G2oVertexSE3*>(_vertices[2]);                                                         
                                                                                                                                    
  Sophus::SE3d T_s1_to_w=(params_imu.T_imu_from_cam*T_c1_from_world->estimate()).inverse();                                 
  Sophus::SE3d pred_T_s2_to_w;                                                                                              
  Eigen::Vector3d pred_speed_2;                                                                                            

  Eigen::Matrix<double, 15,15> P= Eigen::Matrix<double, 15,15>::Identity()*1e-10; // covariance of first states are 0 in this case

  predictStates(T_s1_to_w, speed_bias_1->estimate(), time_frames,                                                   

      _measurement, params_imu.gwomegaw, params_imu.q_n_aw_babw,                                          

      &pred_T_s2_to_w, &pred_speed_2, &P);                                                                

  Eigen::Matrix<double,15,15> J_pred = Eigen::Matrix<double,15,15>::Zero();     

  const int kGlobalSize=7, kLocalSize=6, num_outputs=6;                                             
  Eigen::Matrix<double, kLocalSize, 1> zero_delta= Eigen::Matrix<double, kLocalSize, 1>::Zero();                  
  double value[num_outputs];                                                                        
  Eigen::Matrix<double, num_outputs, kLocalSize, Eigen::RowMajor> dTinv_de_AD;                             
  const double *parameters[3] = { pred_T_s2_to_w.data(), zero_delta.data(),                         
                                  (params_imu.T_imu_from_cam*T_c2_from_world->estimate()).data()};  
  double *jacobians[3] = {NULL, dTinv_de_AD.data(), NULL };                                         
  LogDeltaSE3Vee deltaTvee;                                                                         
  ceres::internal::AutoDiff<LogDeltaSE3Vee, double, kGlobalSize, kLocalSize, kGlobalSize>           
          ::Differentiate(deltaTvee, parameters, num_outputs, value, jacobians);                    
  J_pred.block<6,3>(0,0)= dTinv_de_AD.block<6,3>(0,0);                                              
  J_pred.block<6,3>(0,6)= dTinv_de_AD.block<6,3>(0,3);                                              
  J_pred.block<3,3>(6,3)=Eigen::Matrix3d::Identity();//delta velocity                                      
  J_pred.block<3,3>(9,9)=Eigen::Matrix3d::Identity();//b_a                                                 
  J_pred.block<3,3>(12,12)=Eigen::Matrix3d::Identity();//b_g                                               
  _information=(J_pred*P*J_pred.transpose()).inverse();                                             

}

void G2oEdgeIMUConstraint::linearizeOplus()                                                                              
{                                                                                                                        
  G2oVertexSE3 * vpose_1 = static_cast<G2oVertexSE3 *>(_vertices[0]);                                                  
  Sophus::SE3d T_1w =  (vpose_1->first_estimate==NULL? vpose_1->estimate(): *(vpose_1->first_estimate)); //T world to 1 frame  
  G2oVertexSpeedBias* vsb_1 = static_cast<G2oVertexSpeedBias*>(_vertices[1]);                                          

  Eigen::Matrix<double, 9,1> sb_1 = (vsb_1->first_estimate==NULL? vsb_1->estimate(): *(vsb_1->first_estimate));               
  G2oVertexSE3 * vpose_2 = static_cast<G2oVertexSE3 *>(_vertices[2]);                                                  
  
  Sophus::SE3d T_2w =(vpose_2->first_estimate==NULL? vpose_2->estimate(): *(vpose_2->first_estimate));                         
  G2oVertexSpeedBias* vsb_2 = static_cast<G2oVertexSpeedBias*>(_vertices[3]);                                          
  Eigen::Matrix<double, 9,1> sb_2 = (vsb_2->first_estimate==NULL? vsb_2->estimate(): *(vsb_2->first_estimate));               
  const G2oIMUParameters * params_imu = static_cast<const G2oIMUParameters *>(parameter(0));                                                       

  _jacobianOplus[0]=Eigen::Matrix<double,15, 6>::Zero(); //15x6                                                   
  _jacobianOplus[1]=Eigen::Matrix<double,15, 9>::Zero(); //15x9                                                   
  const int kGlobalSize=7, kLocalSize=6, num_outputs=9, sbDim=9;                                           
  Eigen::Matrix<double, kLocalSize, 1> zero_delta= Eigen::Matrix<double, kLocalSize, 1>::Zero();                         
                                                                                                           
  typedef ceres::internal::AutoDiff<G2oEdgeIMUConstraint, double, kGlobalSize,                             
                  kLocalSize, sbDim, kGlobalSize, sbDim > IMUConstraintAutoDiff;                                   
  Eigen::Matrix<double, num_outputs, kLocalSize, Eigen::RowMajor> dError_dTw2ck;                                  
  Eigen::Matrix<double, num_outputs, sbDim, Eigen::RowMajor> dError_dsb;                                          
  const double *parameters[] = { T_1w.data(), zero_delta.data(), sb_1.data(), T_2w.data(), sb_2.data()};   
  double value[num_outputs];                                                                               
  double *jacobians[] = { NULL, dError_dTw2ck.data(), dError_dsb.data(), NULL, NULL };                     
  bool diffState = IMUConstraintAutoDiff::Differentiate(*this, parameters, num_outputs, value, jacobians); 
  if (diffState) {                                                                                         
        _jacobianOplus[0].topLeftCorner<9,6>() = dError_dTw2ck;                                              
            _jacobianOplus[1].topLeftCorner<9,9>() = dError_dsb;                                                 
  } else {                                                                                                 
        assert(0 && "Error while AD differentiating");                                                       
  }                                                                                                        

  _jacobianOplus[1].bottomRightCorner<6,6>().setIdentity();                     
                                                                                
  Sophus::SE3d Ts1tow=(params_imu->T_imu_from_cam*T_1w).inverse();                      
  Sophus::SE3d pred_T_s2_to_w;                                                          
  Eigen::Vector3d pred_speed_2;                                                        
  Eigen::Matrix<double, 15,15> *holder=NULL;                                    
  predictStates(Ts1tow, sb_1, time_frames,                                      
                _measurement, params_imu->gwomegaw, params_imu->q_n_aw_babw,    
                &pred_T_s2_to_w, &pred_speed_2, holder);                        

  Sophus::SE3d predTckp12w=  pred_T_s2_to_w*params_imu->T_imu_from_cam;                 
  Eigen::Matrix<double, 6,1> error_fe= Sophus::SE3d::log(predTckp12w*T_2w);             
  _jacobianOplus[2]=Eigen::Matrix<double, 15, 6>::Zero(); //15x6                       
  _jacobianOplus[2].topLeftCorner<6,6>()= third(predTckp12w,  error_fe);        
                                                                                
  _jacobianOplus[3]=Eigen::Matrix<double, 15, 9>::Zero(); //15x9                       
  _jacobianOplus[3].bottomRightCorner<9,9>()= -Eigen::Matrix<double, 9,9>::Identity(); 

}
