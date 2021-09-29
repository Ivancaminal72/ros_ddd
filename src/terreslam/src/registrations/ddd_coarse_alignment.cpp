/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-08-27 10:13:32
 */

#include "terreslam/registrations/ddd_coarse_alignment.h"

namespace terreslam
{
	const float RAD2DEG = 180.0/M_PI;
	const float LARGE_NUMBER = FLT_MAX;

	// calculates squared error from two point mapping; assumes rotation around Origin.
	float sqErr_6Dof(cv::Point3f p1, cv::Point3f p2, const cv::Mat& RTr) {

		cv::Mat p2_est = RTr(cv::Rect( 0, 0, 4, 3 )) * cv::Mat(cv::Matx41f(p1.x, p1.y, p1.z, 1));
		cv::Mat dp = p2_est-cv::Mat(p2);
		float sq_er = dp.dot(dp); // squared distance

		//cout<<dp<<endl;
		return sq_er;
	}

	// calculate RMSE for point-to-point metrics
	float RMSE_6Dof(const std::vector<cv::Point3f>& src, const std::vector<cv::Point3f>& dst,
					const cv::Mat& RTr, const bool* inliers, const cv::Point3f center) {

		const bool all_inliers = (inliers==NULL); // handy when we run QUADRTATIC will all inliers
		unsigned int n = src.size();
		assert(n>0 && n==dst.size());

		double RMSE = 0.0;
		int ninliers = 0;
		for (unsigned int i=0; i<n; ++i) {
			if (all_inliers || inliers[i]) {
				RMSE += sqErr_6Dof(src[i]-center, dst[i]-center, RTr);
				ninliers++;
			}
		}

		//cout<<"RMSE = "<<RMSE<<endl;
		if (ninliers>0)
			return sqrt(RMSE/ninliers);
		else
			return LARGE_NUMBER;
	}

	// Sets inliers and returns their count
	int setInliers6Dof(const std::vector<cv::Point3f>& src, const std::vector <cv::Point3f>& dst,
					bool* inliers,
					const cv::Mat& RTr,
					const float max_er,
					const cv::Point3f center) {

		// set inliers
		unsigned int ninliers = 0;
		unsigned int n = src.size();
		assert(n>0 && n==dst.size());

		float max_sqErr = max_er*max_er; // comparing squared values

		if (inliers==NULL) {

			// just get the number of inliers (e.g. after QUADRATIC fit only)
			for (unsigned int i=0; i<n; ++i) {
				
				float sqErr = sqErr_6Dof(src[i]-center, dst[i]-center, RTr);
				if ( sqErr < max_sqErr)
					ninliers++;
			}
		} else {

				// get the number of inliers and set them (e.g. for RANSAC)
				for (unsigned int i=0; i<n; ++i) {
					
					float sqErr = sqErr_6Dof(src[i]-center, dst[i]-center, RTr);
					if ( sqErr < max_sqErr) {
						inliers[i] = 1;
						ninliers++;
					} else {
						inliers[i] = 0;
					}
				}
		}

		return ninliers;
	}

	// fits 3DOF (rotation and translation in 2D) with least squares.
	float fit6DofQUADRATIC(const std::vector<cv::Point3f>& src_, const std::vector<cv::Point3f>& dst_,
					float* param, cv::Mat& param_RTr, const bool* inliers, const cv::Point3f center) {

		const bool debug = false;                   // print more debug info
		const bool all_inliers = (inliers==NULL);   // handy when we run QUADRTATIC will all inliers
		assert(dst_.size() == src_.size());
		int N = src_.size();

		// collect inliers
		std::vector<cv::Point3f> src, dst;
		int ninliers;
		if (all_inliers) {
			ninliers = N;
			src = src_; // copy constructor
			dst = dst_;
		} else {
			ninliers = 0;
			for (int i=0; i<N; ++i){
				if (inliers[i]) {
					ninliers++;
					src.push_back(src_[i]);
					dst.push_back(dst_[i]);
				}
			}
		}
		if (ninliers<2) {
			param[0] = 0.0f; // default return when there is not enough points
			param[1] = 0.0f;
			param[2] = 0.0f;
			param[3] = 0.0f;
			param[4] = 0.0f;
			param[5] = 0.0f;
			return LARGE_NUMBER;
		}

		/* Algorithm: Least-Square Rigid Motion Using SVD by Olga Sorkine
		* http://igl.ethz.ch/projects/ARAP/svd_rot.pdf
		*
		* Subtract centroids, calculate SVD(cov),
		* R = V[1, det(VU')]'U', T = mean_q-R*mean_p
		*/

		// Calculate data centroids
		cv::Scalar centroid_src = cv::mean(src);
		cv::Scalar centroid_dst = cv::mean(dst);
		cv::Point3f center_src(centroid_src[0], centroid_src[1], centroid_src[2]);
		cv::Point3f center_dst(centroid_dst[0], centroid_dst[1], centroid_dst[2]);
		if (debug) 
			cout<<"Centers: "<<center_src<<", "<<center_dst<<endl;

		// subtract centroids from data
		for (int i=0; i<ninliers; ++i) {
			src[i] -= center_src;
			dst[i] -= center_dst;
		}

		// compute a covariance matrix
		float Cxx = 0.0, Cxy = 0.0, Cxz = 0.0;
		float Cyx = 0.0, Cyy = 0.0, Cyz = 0.0;
		float Czx = 0.0, Czy = 0.0, Czz = 0.0;
		for (int i=0; i<ninliers; ++i) {
			Cxx += src[i].x*dst[i].x;
			Cxy += src[i].x*dst[i].y;
			Cxz += src[i].x*dst[i].z;
			Cyx += src[i].y*dst[i].x;
			Cyy += src[i].y*dst[i].y;
			Cyz += src[i].y*dst[i].z;
			Czx += src[i].z*dst[i].x;
			Czy += src[i].z*dst[i].y;
			Czz += src[i].z*dst[i].z;
		}
		cv::Mat Mcov = (cv::Mat_<float>(3, 3)<<Cxx, Cxy, Cxz, Cyx, Cyy, Cyz, Czx, Czy, Czz);
		Mcov /= (ninliers-1);
		if (debug) 
			cout<<"Covariance-like Matrix "<<Mcov<<endl;

		// SVD of covariance
		cv::SVD svd;
		svd = cv::SVD(Mcov, cv::SVD::FULL_UV);
		if (debug) {
			cout<<"U = "<<svd.u<<endl;
			cout<<"W = "<<svd.w<<endl;
			cout<<"V transposed = "<<svd.vt<<endl;
		}

		// rotation (V*Ut)
		cv::Mat V = svd.vt.t();
		cv::Mat Ut = svd.u.t();
		float det_VUt = cv::determinant(V*Ut);
		cv::Mat W = (cv::Mat_<float>(3, 3)<<1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, det_VUt);
		float rot[6];
		cv::Mat R_est(3, 3, CV_32F, rot);
		R_est = V*W*Ut;
		if (debug) 
			cout<<"Rotation matrix: "<<R_est<<endl;


		float pitch = atan2(rot[7], rot[8]);
   	float yaw = asin(-rot[6]);
   	float roll = atan2(rot[3], rot[0]);

		// translation (mean_dst - R*mean_src)
		cv::Mat center_srcRot = R_est * cv::Mat(center_src);
		cv::Mat T_est_mat = cv::Mat(center_dst) - center_srcRot;
		cv::Point3f T_est(T_est_mat);

		// Final estimate msg
		if (debug){
			cout<<"Estimate roll = "<<roll*RAD2DEG<<"deg. "<<endl;
			cout<<"Estimate pitch = "<<pitch*RAD2DEG<<"deg. "<<endl;
			cout<<"Estimate yaw = "<<yaw*RAD2DEG<<"deg. "<<endl;
			cout<<"Estimate Translation = " << T_est<<"m. "<<endl;
		}

		param[0] = roll;
		param[1] = pitch;
		param[2] = yaw;
		param[3] = T_est.x;
		param[4] = T_est.y;
		param[5] = T_est.z;

		param_RTr = (cv::Mat_<float>(4, 4)<<rot[0], rot[1], rot[2], T_est.x, 
																				rot[3], rot[4], rot[5], T_est.y, 
																				rot[6], rot[7], rot[8], T_est.z,
																				0     , 0     , 0     , 1      );

		// calculate RMSE
		float RMSE = RMSE_6Dof(src_, dst_, param_RTr, inliers, center);
		return RMSE;
	} // fit6DofQUADRATIC()

	// RANSAC fit in 3DOF: 1D rot and 2D translation (maximizes the number of inliers)
	// NOTE: no data normalization is currently performed
	float fit6DofRANSAC(const std::vector<cv::Point3f>& src, const std::vector<cv::Point3f>& dst,
					float* best_param, cv::Mat& best_param_RTr,  bool* inliers,
					const cv::Point3f center ,
					const float inlierMaxEr,
					const int niter,
					const bool debug) {

		const int ITERATION_TO_SETTLE = 2;		// iterations to settle inliers and param
		const float INLIERS_RATIO_OK = 0.95f;	// stopping criterion

		// size of data std::vector
		unsigned int N = src.size();
		assert(N==dst.size());

		// unrealistic case
		if(N<2) {
			best_param[0] = 0.0f; // ?
			best_param[1] = 0.0f;
			best_param[2] = 0.0f;
			best_param[3] = 0.0f;
			best_param[4] = 0.0f;
			best_param[5] = 0.0f;
			return LARGE_NUMBER;
		}

		unsigned int ninliers;         			// current number of inliers
		unsigned int best_ninliers = 0;			// number of inliers
		float best_rmse = LARGE_NUMBER;			// error
		float cur_rmse;                			// current distance error
		float param[6];                			// roll, pitch, yaw, Tx, Ty, Tz
		std::vector <cv::Point3f> src_2pt(2), dst_2pt(2);// min set of 2 points (1 correspondence generates 3 equations)
		srand(time(NULL));

		// iterations
		for (int iter = 0; iter<niter; iter++) {
			if(debug) cout<<"iteration "<<iter<<": ";
			
			// 1. Select a random set of 2 points (not obligatory inliers but valid)
			int i1, i2;
			i1 = rand() % N; // [0, N[
			i2 = i1;
			while (i2==i1) {
				i2 = rand() % N;
			}
			src_2pt[0] = src[i1]; // corresponding points
			src_2pt[1] = src[i2];
			dst_2pt[0] = dst[i1];
			dst_2pt[1] = dst[i2];
			bool two_inliers[] = {true, true};
			cv::Mat param_RTr;

			// 2. Quadratic fit for 2 points
			cur_rmse = fit6DofQUADRATIC(src_2pt, dst_2pt, param, param_RTr, two_inliers, center);

			// 3. Recalculate to settle params and inliers using a larger set
			for (int iter2=0; iter2<ITERATION_TO_SETTLE; iter2++) {
				ninliers = setInliers6Dof(src, dst, inliers, param_RTr, inlierMaxEr, center);   // changes inliers
				cur_rmse = fit6DofQUADRATIC(src, dst, param, param_RTr, inliers, center);              // changes cur_param
			}

			// potential ill-condition or large error
			if (ninliers<2) {
				if(debug) cout<<" !!! less than 2 inliers "<<endl;
				continue;
			} else {
				if(debug) cout<<" "<<ninliers<<" inliers;";
			}

			if(debug) cout<<" recalculate: RMSE = "<<cur_rmse<<endl;


			// 4. found a better solution?
			if (ninliers > best_ninliers) {
				best_ninliers = ninliers;
				best_param[0] = param[0];
				best_param[1] = param[1];
				best_param[2] = param[2];
				best_param[3] = param[3];
				best_param[4] = param[4];
				best_param[5] = param[5];
				best_param_RTr = param_RTr.clone();
				best_rmse = cur_rmse;

				if(debug){
					cout<<" --- Solution improved: "<<endl;
					cout<< best_param[0]<<", "<<best_param[1]<<", "<<param[2]<<endl;
					cout<< best_param[3]<<", "<<best_param[4]<<", "<<param[5]<<endl;
				}
																									

				// exit condition
				float inlier_ratio = (float)best_ninliers/N;
				if (inlier_ratio > INLIERS_RATIO_OK) {
					if(debug) cout<<"Breaking early after "<< iter+1<<" iterations; inlier ratio = "<<inlier_ratio<<endl;
					break;
				}
			} else if(ninliers == best_ninliers && best_rmse > cur_rmse) {
				best_ninliers = ninliers;
				best_param[0] = param[0];
				best_param[1] = param[1];
				best_param[2] = param[2];
				best_param[3] = param[3];
				best_param[4] = param[4];
				best_param[5] = param[5];
				best_param_RTr = param_RTr.clone();
				best_rmse = cur_rmse;

				if(debug){
					cout<<" --- Solution improved: "<<endl;
					cout<< best_param[0]<<", "<<best_param[1]<<", "<<param[2]<<endl;
					cout<< best_param[3]<<", "<<best_param[4]<<", "<<param[5]<<endl;
				}
			}
		} // iterations

		// 5. recreate inliers for the best parameters
		ninliers = setInliers6Dof(src, dst, inliers, best_param_RTr, inlierMaxEr, center);

		if(debug){
			cout<<"Best iteration: "<<ninliers<<" inliers; recalculate: RMSE = "<<best_rmse<<endl;
			cout<<" --- Final solution: "<<endl;
			cout<< best_param[0]<<", "<<best_param[1]<<", "<<param[2]<<endl;
			cout<< best_param[3]<<", "<<best_param[4]<<", "<<param[5]<<endl;
		}

		return best_rmse;
	} // fit6DofRANSAC()
}