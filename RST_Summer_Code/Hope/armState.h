#ifndef _ARM_STATE_
#define _ARM_STATE_

#include <math.h>
#include <float.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <robina_kin/robina_kin.h>

/**
 * @class armState
 */
class armState
{
   public:
    armState();
    ~armState();

   typedef enum
   {
     IN_OPEN_SET,
     IN_CLOSED_SET,
     IN_NO_SET
   } state_status;


    int key;
    int parent;
    state_status status;
    Eigen::VectorXd joints;
    Eigen::Vector3d xyz;
    Eigen::Vector3d rpy;
    double costF;
    double costG;
    double costH;

    static double MAX_COST;

    /** Same for all states */
    static Eigen::Transform< double, 3, Eigen::Affine > TWBase_;
    static Eigen::Transform< double, 3, Eigen::Affine > Tee_;

   /** Functions */
   void initialize( Eigen::VectorXd _joints, int _key );
   void calculate_pose();
   static void set_Transforms( const Eigen::Transform< double, 3, Eigen::Affine > &_TWBase, const Eigen::Transform< double, 3, Eigen::Affine > &_Tee );

  private:
  Eigen::Transform< double, 3, Eigen::Affine > get_T();
  Eigen::Vector3d get_rpy( const Eigen::Transform<double, 3, Eigen::Affine> & _T );

};


/**--------------------------------------*/

/**
 * @function get_T
 */
inline Eigen::Transform< double, 3, Eigen::Affine > armState::get_T()
{
  Eigen::Transform< double, 3, Eigen::Affine > _T;
  robinaLeftArm_fk( joints, armState::TWBase_, armState::Tee_, _T );
  return _T;
}

/**
 * @function get_rpy
 */
inline Eigen::Vector3d armState::get_rpy( const Eigen::Transform<double, 3, Eigen::Affine> & TT )
{
   Eigen::Vector3d _rpy; 
   _rpy(0) = atan2( TT(2,1), TT(2,2) );
   _rpy(1) = atan2( -TT(2,0), sqrt( TT(2,1)*TT(2,1) + TT(2,2)*TT(2,2) )  );
   _rpy(2) = atan2( TT(1,0), TT(0,0) ); 
   return _rpy;
}

/**
 * @function setTransforms
 * @brief Set the static variables
 */
inline void armState::set_Transforms( const Eigen::Transform< double, 3, Eigen::Affine > &_TWBase, const Eigen::Transform< double, 3, Eigen::Affine > &_Tee )
{
  armState::TWBase_ = _TWBase;
  armState::Tee_ = _Tee;
}

#endif /** _ARM_STATE_ */

