#ifndef IMU_CONSTRAINT_H 
#define IMU_CONSTRAINT_H 
#include <string>
#include <sensor_msgs/Imu.h>

// OpenCV                              
#include <opencv2/core/core.hpp>       
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/core/eigen.hpp>      

#include "Thirdparty/Sophus/sophus/se3.hpp"
#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include "Thirdparty/g2o/g2o/core/parameter.h"

#include "eigen_utils.h"
#include "anchored_points.h"

#include "IMUErrorModel.cpp" //template class 

class G2oIMUParameters : public g2o::Parameter
{                                                                                                                                              
public:
  G2oIMUParameters()
  {
    q_n_aw_babw.setZero();
    gwomegaw.setZero();
    gwomegaw[3]=9.81;
  }

  G2oIMUParameters(const Eigen::Matrix<double, 3,1> & q_noise_acc,const Eigen::Matrix<double, 3,1> &q_noise_gyr,
                   const Eigen::Matrix<double, 3,1> & q_noise_accbias,const Eigen::Matrix<double, 3,1> &q_noise_gyrbias,
                   const Sophus::SE3Group<double> & T_c_to_s, const Eigen::Matrix<double, 6,1> &gomegaw)
    : T_imu_from_cam(T_c_to_s), gwomegaw(gomegaw)
  {
    q_n_aw_babw.head<3>()=q_noise_acc;
    q_n_aw_babw.segment<3>(3)=q_noise_gyr;
    q_n_aw_babw.segment<3>(6)=q_noise_accbias;

    q_n_aw_babw.tail<3>()=q_noise_gyrbias;

  }
  G2oIMUParameters(const G2oIMUParameters& other)
    : q_n_aw_babw(other.q_n_aw_babw), T_imu_from_cam(other.T_imu_from_cam),gwomegaw(other.gwomegaw)
  {

  }
  G2oIMUParameters & operator= (const G2oIMUParameters& other)
  {
    if(this==&other)
      return *this;
    q_n_aw_babw=other.q_n_aw_babw;
    T_imu_from_cam=other.T_imu_from_cam;
    gwomegaw=other.gwomegaw;

    return *this;
  }

  virtual bool                                        
  read                       (std::istream& is)       
  {                                                   
    for(int i=0; i<12;++i)                          
      is >> q_n_aw_babw[i];                       
      return true;                                    
  }                                                   
                                                        
  virtual bool                                        
  write                      (std::ostream& os) const 
  {                                                   
    for(int i=0; i<12;++i)                          
    os << q_n_aw_babw[i]<<" ";                  
    return true;                                    
  }                                                   

  Eigen::Matrix<double,12,1> q_n_aw_babw; //squared noise density of VRW, ARW, noise of random walk of accelerometer bias and gyro bias
  Sophus::SE3Group<double> T_imu_from_cam; // the transformation from camera to IMU sensor frame
  Eigen::Matrix<double, 6,1> gwomegaw;// gravity in local world frame in m/s^2, and earth rotation rate in world frame in rad/sec
  double sampling_interval; // 1/frequency, unit sec

};

class G2oVertexSpeedBias : public g2o::BaseVertex<9, Eigen::Matrix<double,9,1> >   
{                                                                                  

public:                                                                            
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW                                                
  G2oVertexSpeedBias               ():first_estimate(NULL){}                     

  ~G2oVertexSpeedBias               (){                                          
    if(first_estimate)                                                         
      delete first_estimate;                                                 
    first_estimate=NULL;                                                       
  }                                                                              
  virtual bool                                                                   
  read                       (std::istream& is);                                 

  virtual bool                                                                   
  write                      (std::ostream& os) const;                           

  virtual void                                                         
  oplusImpl                  (const double * update_p)                 
  {                                                                    
    Eigen::Map<const Eigen::Matrix<double, 9,1> > update(update_p);  
    setEstimate(update+estimate());                                  
  }                                                                    
                                                                       
  virtual void                                                         
  setToOriginImpl            () {                                      
    _estimate.setZero();                                             

  }                                                                    
  void setFirstEstimate(const Eigen::Matrix<double,9,1>& fe){          
    first_estimate=new Eigen::Matrix<double,9,1>(fe);                
  }                                                                    
  Eigen::Matrix<double,9,1>* first_estimate;                           
};
class G2oEdgeIMUConstraint : public  g2o::BaseMultiEdge<15, std::vector<Eigen::Matrix<double, 7,1> > >  
{                                                                                                       
  //IMU measurements are stored in a std::vector<Matrix<double, 7,1> > structure, each entry: timestamp in seconds,
  //acceleration measurements in m/s^2, gyro measurements in radian/sec                                            
  //the measurements goes from $t(p^k-1)$ which is the closest epoch less than or equal to $t(k)$, up to           
  // $t(p^(k+1)-1)$ which is the closest epoch less than or equal to $t(k+1)$.                                     
public:                                                                                                                           
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW                                                                                               
    
    G2oEdgeIMUConstraint()                                                                                                        
    {                                                                                                                             
        g2o_IMU=0;                                                                                                                
        resizeParameters(1);                                                                                                      
        installParameter(g2o_IMU, 0);                                                                                             
    }                                                                                                                             

    virtual bool                                                                                                                  
    read                       (std::istream& is);                                                                                
    virtual bool                                                                                                                  
    write                      (std::ostream& os) const;                                                                          
    
    void computeError               ();                                                                                                
    //bool operator ()( const  double* pTw2ck, const double* epsilonk, const double* pXsbk, const double* pTw2ckp1, const double* pXsbkp1, double* error) const;  
    template<typename T>                                                                         
    bool operator ()( const  T* pTw2ck, const T* epsilonk, const T* pXsbk, const T* pTw2ckp1, const T* pXsbkp1, T* error) const;

    void linearizeOplus        ();                                                                                                
    void calcAndSetInformation(const G2oIMUParameters &);                                                                         
    void SetFrameEpoch(const double timestamp, const int index){                                                                  
        time_frames[index]=timestamp;                                                                                             
    }                                                                                                                             
    G2oIMUParameters * g2o_IMU;                                                                                                   
    double time_frames[2]; //timestamps for the two frames connected by this multi-edge, $t(k)$ and $t(k+1)$                      
};

class IMUProcessor
{
public:
  IMUProcessor();
 
  Sophus::SE3d propagate(const double time_frame);                                                              
     
  //void printStateAndCov(std::ofstream &output, double time)const;
      
  // read each measurement from the IMU file and do free inertial integration
  //void freeInertial(std::string output_file, double finish_time);
   
  void GrapIMU(const sensor_msgs::ImuConstPtr& msgIMU);
  void initStates(const double timestamp);
  void resetStates(const Sophus::SE3d &Ts1tow, const Eigen::Matrix<double, 9,1> & sb1);
  bool getObservation(double tk);
                           
  //template<typename Scalar>
  /*void predictStates(const Sophus::SE3Group<double> &T_sk_to_w, const Eigen::Matrix<double, 9,1>& speed_bias_k,                             
                     const double * time_pair,const std::vector<Eigen::Matrix<double, 7,1> >& measurements, 
                     const Eigen::Matrix<double, 6,1> & gwomegaw,const Eigen::Matrix<double, 12, 1>& q_n_aw_babw,
                     Sophus::SE3Group<double>* pred_T_skp1_to_w, Eigen::Matrix<double, 3,1>* pred_speed_kp1);
  void strapdown_local_quat_bias(const Eigen::Matrix<double,3,1>& rs0, const Eigen::Matrix<double,3,1> & vs0, 
                                 const Eigen::Quaternion<double>& qs0_2_s,const Eigen::Matrix<double,3,1> & a, 
                                 const Eigen::Matrix<double,3,1>& w, double dt,
                                 const Eigen::Matrix<double, 6,1>& gomegas0,Eigen::Matrix<double,3,1>* rs0_new, 
                                 Eigen::Matrix<double,3,1>* vs0_new, Eigen::Quaternion<double>* qs0_2_s_new);
  */
  //IMU readings from t(p(k)-1) to t(p(k+1)-1)
  const std::vector<Eigen::Matrix<double, 7,1> > & getMeasurements() const
  {
    return measurement_;
  }
  const Eigen::Matrix<double, 9,1>& getSpeedbias() const
  {
    return speed_bias_1;
  }
  Sophus::SE3d T_s1_to_w;               //IMU pose at t(k) which timestamps image indexed k              
  G2oIMUParameters imu_;                                                                                 
  Eigen::Matrix<double, 9,1> speed_bias_1; //IMU speed and acc bias and gyro biases at t(k)              
  
  bool bStatesInitialized;                                                                               
  bool reinit_data;
private:
  bool is_measurement_good;
  int imuSize;
  double time_pair[2];              // timestamps of the last and current images, k and k+1              
  Sophus::SE3d pred_T_s2_to_w;          //predicted IMU pose at t(k+1) of image indexed k+1              
  Eigen::Matrix<double, 9,1> pred_speed_bias_2; //predicted IMU speed in world frame at t(k+1) and biases
  std::vector<Eigen::Matrix<double, 7,1> > measureSub;
  std::vector<Eigen::Matrix<double, 7,1> > measureRaw;
  std::vector<Eigen::Matrix<double, 7,1> > measurement_;
  Eigen::Matrix<double, 15, 15> P_; //covariance of error states as described in sys_local_dcm_bias()    
  bool bPredictCov; //predict covariance or not                                                          
  //std::ofstream imu_traj_stream; //log IMU states and covariances                                        

};

#endif
