#include <Eigen/Geometry>

inline float wrapToPi(float angle){
  float ret=angle;
  while(ret>M_PI){
    ret-=2*M_PI;
  }

  while(ret<=-M_PI){
    ret+=2*M_PI;
  }

  return ret;
}

inline float angDiff(float thetaD, float theta){
  float alpha=0;
  Eigen::Vector2d nD,n;

  nD<<cos(thetaD), sin(thetaD);
  n<<cos(theta), sin(theta);

  float alphaAbs = acos(nD.transpose()*n);

  Eigen::Vector3f n3,nD3;

  n3<<n(0),n(1),0;
  nD3<<nD(0),nD(1),0;

  Eigen::Vector3f nC3;

  nC3=n3.cross(nD3);

  if(nC3(2)>0){
    alpha=alphaAbs;
  }
  else{
    alpha=-alphaAbs;
  }

  return alpha;

}

inline Eigen::MatrixXf matrixPower(Eigen::MatrixXf& A, int exp){

	Eigen::MatrixXf result = Eigen::MatrixXf::Identity(A.rows(),A.cols());

	for (int i=0; i<exp;++i)
        	result *= A;

	return result;
}

inline float sign(float x){
	if(x>0) return +1;
	if(x<0) return -1;
	return -1;
}
