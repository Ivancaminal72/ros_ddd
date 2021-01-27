/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-01-26 20:38:10
 *    Last Modified: 2021-01-27 10:12:58
 */

/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2020-12-17 17:21
#
# Filename:		map.cpp
#
# Description: 
#
************************************************/

#include "terreslam/types/ulysses_map.h"
#include <pcl/filters/voxel_grid.h>

namespace ulysses
{
	CameraIntrinsic camera_intrinsic;

//	using namespace std;
//	FeatureAssociation::FeatureAssociation(Map *map, Scan* scan_cur, Scan* scan_ref)
//	{
//		for(const_iterLandmark it=map->beginLandmark();it!=map->endLandmark();it++)
//		{
//			Landmark* lm=it->second;
//			for(iterObserv it_obs=lm->beginObserv();it_obs!=lm->endObserv();it_obs++)
//			{
//				double time=it_obs->first;
//				std::vector<std::string> ids=it_obs->second;
//				if(fabs(time-scan_cur->time())<1e-4)
//				{
////					cout<<endl<<lm->ID()<<endl;
////					cout<<scan_cur->time()<<endl;
//					for(int i=0;i<ids.size();i++)
//					{
//						Feature *ft=scan_cur->findFeature(ids[i]);
//						if(ft!=0) {ft->ptrLandmark()=lm;}
//					}
//				}
//				if(fabs(time-scan_ref->time())<1e-4)
//				{
////					cout<<endl<<lm->ID()<<endl;
////					cout<<scan_ref->time()<<endl;
//					for(int i=0;i<ids.size();i++)
//					{
//						Feature *ft=scan_ref->findFeature(ids[i]);
//						if(ft!=0) {ft->ptrLandmark()=lm;}
//					}
//				}
//			}
//		}
//		for(iterFeature it=scan_cur->beginFeature();it!=scan_cur->endFeature();it++)
//		{
//			for(iterFeature it2=scan_ref->beginFeature();it2!=scan_ref->endFeature();it2++)
//			{
//				Feature *ft_cur=it->second;
//				Feature *ft_ref=it2->second;
//				if(ft_cur->ptrLandmark()==0 || ft_ref->ptrLandmark()==0) continue;
//				if(ft_cur->ptrLandmark()==ft_ref->ptrLandmark())
//				{
////					cout<<endl;
////					cout<<ft_cur<<endl;
////					cout<<ft_ref<<endl;
////					cout<<"insert: "<<ft_cur->ptrLandmark()->ID()<<endl;
//					insert(ft_cur,ft_ref,ft_cur->ptrLandmark());
//				}
//			}
//		}
//
//	}

//	void FeatureAssociation::insert(Feature *cur, Feature *ref, Landmark *lm)
//	{
//		indices.insert(std::pair<std::string,int>(cur->ID(),features_cur.size()));
//		features_cur.push_back(cur);
//		features_ref.push_back(ref);
//		landmarks.push_back(lm);
//	}

	void FeatureAssociation::insert(Feature *cur, Feature *ref, Scan *scan)
	{
		indices.insert(std::pair<std::string,int>(cur->ID(),features_cur.size()));
		features_cur.push_back(cur);
		features_ref.push_back(ref);

		Landmark* lm;
		if(ref->ptrLandmark()==0) 
		{
			lm=new Landmark(ref,scan->ref()->Tcg());
			lm->pushObserv(scan->ref()->time(),ref->ID());
			lm->pushObserv(scan->time(),cur->ID());
		}
		else 
		{
			lm=ref->ptrLandmark();
			lm->pushObserv(scan->time(),cur->ID());
		}
		cur->ptrLandmark()=lm;
		landmarks.push_back(lm);
	}

	void FeatureAssociation::insert(Feature *cur, Landmark *lm, Scan *scan, Map *map)
	{
		indices.insert(std::pair<std::string,int>(cur->ID(),features_cur.size()));
		features_cur.push_back(cur);
		features_ref.push_back(cur);

		if(cur->ptrLandmark()==0)
		{
			lm->pushObserv(scan->time(),cur->ID());
//			landmarks.push_back(lm);
		}
		else if(cur->ptrLandmark()==lm)
		{
//			landmarks.push_back(lm);
		}
		else // cur->ptrLandmark()!=lm or 0
		{
			// merge cur->ptrLandmark() to lm;
			lm->merge(cur->ptrLandmark());
			map->eraseLandmark(cur->ptrLandmark()->ID());
			delete cur->ptrLandmark();
			cur->ptrLandmark()=lm;
//			landmarks.push_back(lm);
		}
		landmarks.push_back(lm);
	}

//	void FeatureAssociation::insert(Feature *cur, Feature *ref)
//	{
//		indices.insert(std::pair<std::string,int>(cur->ID(),features_cur.size()));
//		features_cur.push_back(cur);
//		features_ref.push_back(ref);
//	}

	int FeatureAssociation::findFeature(std::string id) 
	{
		std::map<std::string,int>::iterator it=indices.find(id);
		if(it==indices.end()) return -1;
		else return it->second;
//		else return features_ref[it->second]->ID();
	}

	bool FeatureAssociation::computePsi(Transform Tcr)
	{
		Psi_pi.setZero();
		Eigen::Matrix<double,4,6> J_pi;
		Eigen::Matrix<double,6,6> J_L;

		for(int i=0;i<features_cur.size();i++)
		{
			if(features_cur[i]->Type()==PLANE)
			{
				Eigen::Vector3d nr=features_ref[i]->getDirection();
				Eigen::Matrix4d info=features_cur[i]->plane()->info;
				J_pi.setZero();
				J_pi.block<1,3>(3,0)=nr.transpose()*Tcr.R.transpose();
				J_pi.block<3,3>(0,3)=Transform::skew_sym(Tcr.R*nr);
				J_pi.block<1,3>(3,3)=-Tcr.t.transpose()*J_pi.block<3,3>(0,3);
				Psi_pi+=J_pi.transpose()*info*J_pi;
			}
			else if(features_cur[i]->Type()==LINE)
			{
				Eigen::Vector3d vr=features_ref[i]->getDirection();
				Eigen::Vector3d ur=features_cur[i]->getDistance();
				Matrix6d info=features_cur[i]->line()->info;
				J_L.setZero();
				J_L.block<3,3>(0,0)=Transform::skew_sym(Tcr.R*vr);
				J_L.block<3,3>(3,3)=J_L.block<3,3>(0,0);
				J_L.block<3,3>(0,3)=Transform::skew_sym(Tcr.R*ur)
					+Transform::skew_sym(Tcr.t)*Transform::skew_sym(Tcr.R*vr);
				Psi_pi+=J_L.transpose()*info*J_L;
			}
		}
	}
//
//	std::string FeatureAssociation::findLandmark(std::string id) 
//	{
////			std::ofstream fp; fp.open("find.txt",std::ios::app);
//		std::map<std::string,int>::iterator it=indices.find(id);
////			fp<<std::endl<<id<<std::endl;
////			fp<<it->first<<" "<<it->second<<std::endl;
//		if(it==indices.end()) return std::string("");
//		else return landmarks[it->second]->ID();
//	}
//
//	void FeatureAssociation::evalulatePR(FeatureAssociation *truth, double &precision, double &recall)
//	{
//		double true_pos=0.0, pos=double(this->size()), neg=double(truth->size());
//		for(int i=0;i<size();i++)
//		{
//			if(truth->findFeature(features_cur[i]->ID())==features_ref[i]->ID())
//			{
//				true_pos+=1.0;
//			}
//		}
//		precision=true_pos/pos;
//		recall=true_pos/neg;
//	}
//
//	using namespace std;
//	void FeatureAssociation::evalulatePR(double ref_time, LandmarkAssociation *truth, double &precision, double &recall)
//	{
//		double true_pos=0.0, pos=double(this->size()), neg=0;//double(truth->size());
//		for(int i=0;i<size();i++)
//		{
//			std::string lm_id=truth->findLandmark(features_cur[i]->ID());
//			if(lm_id.empty()) continue;
//			else 
//			{
//				Landmark* lm=truth->getLandmark(lm_id);
//				std::vector<std::string> obs=lm->Observ(ref_time);
//				if(!obs.empty())
//				{
//					neg+=obs.size();
//					for(int j=0;j<obs.size();j++)
//					{
//						if(obs[j]==features_ref[i]->ID())
//						{
//							true_pos+=1.0;
//							break;
//						}
//					}
//				}
//				
////				for(int t=0;t<lm->sizeObserv();t++)
////				{
////					if(fabs(lm->observTime(t)-ref_time)<1e-6)
////					{
////						neg+=1.0;
////						if(lm->observID(t)==features_ref[i]->ID())
////						{
////							true_pos+=1.0;
////							break;
////						}
////					}
////				}
//			}
//		}
////		cout<<"true_pos="<<true_pos<<endl;
////		cout<<"pos="<<pos<<endl;
////		cout<<"neg="<<neg<<endl;
//		precision=true_pos/pos;
//		recall=true_pos/neg;
//	}
//
////	using namespace std;
//	void FeatureAssociation::evalulatePR(double cur_time, double ref_time, Map *map, double &precision, double &recall)
//	{
//		double true_pos=0.0, pos=double(this->size()), neg=0;//double(truth->size());
//		for(int i=0;i<size();i++)
//		{
//			std::string id_cur=features_cur[i]->ID();
//			std::string id_ref=features_ref[i]->ID();
////			cout<<"pair --- "<<id_cur<<" "<<id_ref<<endl;
//			for(const_iterLandmark it=map->beginLandmark();it!=map->endLandmark();it++)
//			{
//				Landmark* lm=it->second;
//				std::vector<std::string> obs_cur=lm->Observ(cur_time);
////				if(!obs_cur.empty()) neg+=1.0;
//				bool flag=false;
//				for(int i_cur=0;i_cur<obs_cur.size();i_cur++)
//				{
////					cout<<obs_cur[i_cur]<<" ";
//					if(obs_cur[i_cur]==id_cur)
//					{
////						flag=true;
//						std::vector<std::string> obs_ref=lm->Observ(ref_time);
//						if(!obs_ref.empty()) flag=true;
//						for(int i_ref=0;i_ref<obs_ref.size();i_ref++)
//						{
////							cout<<obs_ref[i_ref]<<" ";
//							if(obs_ref[i_ref]==id_ref)
//							{
//								true_pos+=1.0;
//							}
//						}
//					}
////					cout<<endl;
//				}
//				if(flag) neg+=1.0;
//			}
//		}
////		cout<<"true_pos="<<true_pos<<endl;
////		cout<<"pos="<<pos<<endl;
////		cout<<"neg="<<neg<<endl;
//		precision=true_pos/pos;
//		recall=true_pos/neg;
//	}

	void FeatureAssociation::print(std::ostream &os) const 
	{
		for(int i=0;i<features_cur.size();i++)
		{
			os<<features_cur[i]->ID()<<" \t"<<features_ref[i]->ID()<<" \t"<<landmarks[i]->ID()<<std::endl;
		}
	}


//	void FeatureAssociation::load(const std::string &folder, Scan *scan, Map *map)
//	{
//		std::string filename=folder+"/scans/"+std::to_string(scan->time())+"_association.txt";
////		std::cout<<filename<<std::endl;
//		std::ifstream fp;
//		fp.open(filename,std::ios::in);
//		while(!fp.eof())
//		{
//			std::string s; getline(fp,s);
//			if(!s.empty())
//			{
//				std::stringstream ss; ss<<s;
//				std::string cur,ref,lm;
//				ss>>cur>>ref>>lm;
////				std::cout<<"load "<<cur<<", "<<ref<<", "<<lm<<std::endl;
////				std::cout<<scan->findFeature(cur)<<std::endl;
////				std::cout<<scan->ref()->findFeature(ref)<<std::endl;
////				std::cout<<map->findLandmark(lm)<<std::endl;
//				insert(scan->findFeature(cur),scan->ref()->findFeature(ref),map->findLandmark(lm));
//			}
//		}
//		fp.close();
//	}

	void FeatureAssociation::vis(Scan *s, boost::shared_ptr<pcl::visualization::PCLVisualizer> v)
	{
		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};
		v->removeAllPointClouds();
		v->removeAllCoordinateSystems();
		s->vis(v);
		s->ref()->vis(v);
		for(int i=0;i<features_cur.size();i++)
		{
//			if (!v->updatePointCloud (s->points(), "scan"))
//				v->addPointCloud (s->points(), "scan");
			features_cur[i]->vis(v,red[i%14],grn[i%14],blu[i%14],std::to_string(i)+"cur");
			features_ref[i]->vis(v,red[i%14],grn[i%14],blu[i%14],std::to_string(i)+"ref");
		}
		v->spin();
	}

//	void LandmarkAssociation::insert(Feature *cur, Landmark *lm)
//	{
//		indices_feature.insert(std::pair<std::string,int>(cur->ID(),features_cur.size()));
//		// fill the indices_map in Map::addScan();
//		// indices_map.insert(std::pair<std::string,int>(lm->ID(),features_cur.size()));
//		features_cur.push_back(cur);
//		landmarks.push_back(lm);
//		if(cur->getDirection().dot(lm->getDirection())<0)
//		{
//			if(cur->Type()==PLANE)
//			{
//				Plane* p=cur->plane();
//				p->n=-p->n;
//				p->d=-p->d;
//			}
//			else if(cur->Type()==LINE)
//			{
//				Line* l=cur->line();
//				l->v=-l->v;
//				l->u=-l->u;
//			}
//		}
//	}
//
//	void LandmarkAssociation::fillIndicesMap()
//	{
//		for(int i=0;i<landmarks.size();i++)
//		{
//			indices_map.insert(std::pair<std::string,int>(landmarks[i]->ID(),i));
//		}
//	}
//
//	std::string LandmarkAssociation::findLandmark(std::string id_feature) 
//	{
//		std::map<std::string,int>::iterator it=indices_feature.find(id_feature);
//		if(it==indices_feature.end()) return std::string("");
//		else return landmarks[it->second]->ID();
//	}
//
//	std::string LandmarkAssociation::findFeature(std::string id_landmark) 
//	{
////		std::cout<<"id_landmark="<<id_landmark<<std::endl;
////		for(std::map<std::string,int>::iterator it=indices_map.begin();it!=indices_map.end();it++)
////		{
////			std::cout<<it->first<<" "<<it->second<<std::endl;
////		}
//		std::map<std::string,int>::iterator it=indices_map.find(id_landmark);
//		if(it==indices_map.end()) return std::string("");
//		else return features_cur[it->second]->ID();
//	}
//
//	void LandmarkAssociation::evalulatePR(LandmarkAssociation *truth, double &precision, double &recall)
//	{
//		double true_pos=0.0, pos=double(this->size()), neg=double(truth->size());
//		for(int i=0;i<size();i++)
//		{
//			if(truth->findLandmark(features_cur[i]->ID())==landmarks[i]->ID())
//			{
//				true_pos+=1.0;
//			}
//		}
//		precision=true_pos/pos;
//		recall=true_pos/neg;
//	}
//
//	void LandmarkAssociation::evalulatePR(double time, Map *map, double &precision, double &recall)
//	{
//		double true_pos=0.0, pos=double(this->size()), neg=0;//double(truth->size());
//		for(int i=0;i<size();i++)
//		{
//			std::string id_ft=features_cur[i]->ID();
//			std::string id_lm=landmarks[i]->ID();
////			cout<<"pair --- "<<id_cur<<" "<<id_ref<<endl;
//			for(const_iterLandmark it=map->beginLandmark();it!=map->endLandmark();it++)
//			{
//				Landmark* lm=it->second;
//				if(lm->ID()!=id_lm) continue;
//				std::vector<std::string> obs_cur=lm->Observ(time);
//				if(!obs_cur.empty()) neg+=1.0;
//				for(int i_cur=0;i_cur<obs_cur.size();i_cur++)
//				{
//					if(obs_cur[i_cur]==id_ft)
//					{
//						true_pos+=1.0;
//					}
//				}
//			}
//		}
////		cout<<"true_pos="<<true_pos<<endl;
////		cout<<"pos="<<pos<<endl;
////		cout<<"neg="<<neg<<endl;
//		precision=true_pos/pos;
//		recall=true_pos/neg;
//	}
//	void LandmarkAssociation::evalulatePR(double ref_time, LandmarkAssociation *truth, double &precision, double &recall)
//	{
//		double true_pos=0.0, pos=double(this->size()), neg=double(truth->size());
//		for(int i=0;i<size();i++)
//		{
//			std::string lm_id=truth->findLandmark(features_cur[i]->ID());
//			if(lm_id.empty()) continue;
//			else 
//			{
//				Landmark* lm=truth->getLandmark(lm_id);
//				for(int t=0;t<lm->sizeObserv();t++)
//				{
//					if(fabs(lm->observTime(t)-ref_time)<1e-6)
//					{
//						true_pos+=1.0;
//						break;
//					}
//				}
//			}
//		}
//		precision=true_pos/pos;
//		recall=true_pos/neg;
//	}

//	void LandmarkAssociation::print(std::ostream &os) const 
//	{
//		for(int i=0;i<features_cur.size();i++)
//		{
//			os<<features_cur[i]->ID()<<" \t"<<landmarks[i]->ID()<<std::endl;
//		}
//	}
//
//	void LandmarkAssociation::load(const std::string &folder, Scan *scan, Map *map)
//	{
//		std::string filename=folder+"/scans/"+std::to_string(scan->time())+"_association.txt";
////		std::cout<<filename<<std::endl;
//		std::ifstream fp;
//		fp.open(filename,std::ios::in);
//		while(!fp.eof())
//		{
//			std::string s; getline(fp,s);
//			if(!s.empty())
//			{
//				std::stringstream ss; ss<<s;
//				std::string cur,ref,lm;
//				ss>>cur>>ref>>lm;
////				std::cout<<"load "<<cur<<", "<<ref<<", "<<lm<<std::endl;
////				std::cout<<scan->findFeature(cur)<<std::endl;
////				std::cout<<scan->ref()->findFeature(ref)<<std::endl;
////				std::cout<<map->findLandmark(lm)<<std::endl;
//				insert(scan->findFeature(cur),map->findLandmark(lm));
//			}
//		}
//		fp.close();
//	}
//
//	void LandmarkAssociation::vis(Scan *s, boost::shared_ptr<pcl::visualization::PCLVisualizer> v)
//	{
//		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
//		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
//		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};
//		v->removeAllPointClouds();
//		v->removeAllCoordinateSystems();
//		s->vis(v);
////		s->ref()->vis(v);
//		for(int i=0;i<features_cur.size();i++)
//		{
////			if (!v->updatePointCloud (s->points(), "scan"))
////				v->addPointCloud (s->points(), "scan");
//			features_cur[i]->vis(v,red[i%14],grn[i%14],blu[i%14],std::to_string(i)+"cur");
////			features_ref[i]->vis(v,red[i%14],grn[i%14],blu[i%14],std::to_string(i)+"ref");
//		}
//		v->spin();
//	}

	Map::~Map()
	{
		for(iterLandmark it=landmarks.begin();it!=landmarks.end();it++) {delete it->second;}
//		for(std::map<double,Scan*>::iterator it=scans.begin();it!=scans.end();it++) {delete it->second;}
	}

	Transform Map::camera(double time) const 
	{
		const_iterCamera it=cameras.find(time);
		if(it!=cameras.end()) return it->second;
		else return Transform();
	}

	void Map::updateCamera(double time, const Transform &T)
	{
		iterCamera it=cameras.find(time);
		if(it!=cameras.end()) { it->second=T; }
	}

	Scan* Map::scan(double time) const 
	{
		const_iterScan it=scans.find(time);
		if(it!=scans.end()) return it->second;
		else return 0;
	}

	void Map::addScan(Scan *scan)
	{
		scans.insert(std::pair<double,Scan*>(scan->time(),scan));
		addCamera(scan->Tcg(),scan->time());
		FeatureAssociation *fa=scan->association_map();
		for(iterFeature it=scan->beginFeature();it!=scan->endFeature();it++)
		{
			std::string id=it->first;
			Feature *ft=it->second;
			Landmark *lm;
			if(fa==0) // first frame;
			{
				lm=new Landmark(ft,scan->Tcg());
				lm->pushObserv(scan->time(),id);
			}
			else
			{
				int idx=fa->findFeature(id);
				if(idx<0) // new features;
				{
					lm=new Landmark(ft,scan->Tcg());
					lm->pushObserv(scan->time(),id);
				}
				else // associated with features in ref;
				{
					lm=fa->getLandmark(idx);
				}
			}
			addLandmark(lm);
		}
//		if(firstFr)
//		{
//			for(iterFeature it=scan->beginFeature();it!=scan->endFeature();it++)
//			{ addLandmark(new Landmark(it->second,scan->Tcg()),scan->time(),it->second->ID()); }
//		}
//		else 
//		{
//			addAssociation(scan->association(),scan->time());
//		}
	}

//	void Map::addScan(Scan *scan)
//	{
//		scans.insert(std::pair<double,Scan*>(scan->time(),scan));
//		scans.insert(std::pair<double,Scan*>(scan->ref()->time(),scan->ref()));
//		cameras.insert(std::pair<double,Transform>(scan->time(),scan->Tcg()));
//		cameras.insert(std::pair<double,Transform>(scan->ref()->time(),scan->ref()->Tcg()));
//		FeatureAssociation* fa=scan->association();
//		for(int i=0;i<fa->size();i++)
//		{
//			Landmark* lm=fa->getLandmark(i);
//
//			std::string observ=fa->getFeature(i)->ID();
//			double time=scan->time();
//			lm->pushObserv(time,observ);
//
//			observ=fa->getFeatureRef(i)->ID();
//			time=scan->ref()->time();
//			lm->pushObserv(time,observ);
//
//			lm->setID(landmarks.size());
//			landmarks.insert(std::pair<std::string,Landmark*>(lm->ID(),lm));
//		}
//	}

	void Map::addCamera(Transform Tcg, double timestamp)
	{
		cameras.insert(std::pair<double,Transform>(timestamp,Tcg));
	}

	void Map::addLandmark(Landmark *lm) //, const double &time, const std::string &observ)
	{
		iterLandmark it=landmarks.find(lm->ID());
		if(it==landmarks.end()) 
		{ 
			lm->setID(landmarks.size());
			landmarks.insert(std::pair<std::string,Landmark*>(lm->ID(),lm));
		}
	}

//	void Map::addAssociation(FeatureAssociation *fa, double time)
//	{
//		for(int i=0;i<fa->size();i++)
//		{
//			addLandmark(fa->getLandmark(i),time,fa->getFeature(i)->ID());
//		}
//	}

//	void Map::addAssociation(LandmarkAssociation *la, double time)
//	{
//		for(int i=0;i<la->size();i++)
//		{
//			addLandmark(la->getLandmark(i),time,la->getFeature(i)->ID());
//		}
//		la->fillIndicesMap();
//	}

//	void Map::addAssociation(Scan *scan_cur,Scan *scan_ref)
//	{
//		scans.insert(std::pair<double,Scan*>(scan_cur->time(),scan_cur));
//		scans.insert(std::pair<double,Scan*>(scan_ref->time(),scan_ref));
//		addCamera(scan_cur->Tcg(),scan_cur->time());
//		addCamera(scan_ref->Tcg(),scan_ref->time());
//		FeatureAssociation *fa=scan_cur->association();
//		for(int i=0;i<fa->size();i++)
//		{
//			Landmark* lm=fa->getLandmark(i);
//			Feature* ft_cur=fa->getFeature(i);
//			Feature* ft_ref=fa->getFeatureRef(i);
//			Landmark* landmark=new Landmark(lm);
//			landmark->pushObserv(scan_cur->time(),ft_cur->ID());
//			landmark->pushObserv(scan_ref->time(),ft_ref->ID());
//			landmarks.insert(std::pair<std::string,Landmark*>(landmark->ID(),landmark));
//		}
//	}

	Landmark* Map::findLandmark(std::string id) 
	{
		iterLandmark it=landmarks.find(id);
		if(it!=landmarks.end()) return it->second;
		else return 0;
	}

	iterLandmark Map::eraseLandmark(std::string id)
	{
		iterLandmark it=landmarks.find(id);
		if(it!=landmarks.end())
		{
			it=landmarks.erase(it);
		}
		return it;
	}

	void Map::printCameras(std::ostream &os) const 
	{
		os.precision(6); os<<std::fixed;
		for(const_iterCamera it=cameras.begin();it!=cameras.end();it++)
		{
			os<<it->first<<" "<<it->second<<std::endl;
		}
	}

	void Map::printLandmarks(std::ostream &os) const 
	{
		for(const_iterLandmark it=landmarks.begin();
			it!=landmarks.end();it++)
		{
			os<<it->second<<std::endl;
		}
	}
	
	void Map::save(std::string folder) const 
	{
		std::string traj=folder+"/traj.txt";
		std::string map=folder+"/map.txt";
		saveTraj(traj);
		saveMap(map);
//		std::string traj_slam=folder+"/traj_slam.txt";
//		saveTrajSLAM(traj_slam);
//		std::string traj_vo=folder+"/traj_vo.txt";
//		saveTrajVO(traj_vo);
	}

	void Map::load(std::string folder)
	{
		std::string traj=folder+"/traj.txt";
		std::string map=folder+"/map.txt";
		loadTraj(traj);
		loadMap(map);
	}

//	void Map::saveTimes(const std::string &folder) const 
//	{
//		std::string filename=folder+"/timestamp.txt";
//		std::ofstream fp;
//		fp.open(filename,std::ios::out);
//		fp.precision(6); fp<<std::fixed;
//		for(const_iterCamera it=cameras.begin();it!=cameras.end();it++)
//		{
//			fp<<it->first<<std::endl;
//		}
//		fp.close();
//	}

	using namespace std;
	void Map::vis(boost::shared_ptr<pcl::visualization::PCLVisualizer> v, bool fuse)
	{
		v->removeAllPointClouds();
		v->removeAllCoordinateSystems();

		for(std::map<double,Transform>::iterator it=cameras.begin();it!=cameras.end();it++)
		{
			Transform Tgc=it->second.inv();
			Tgc.vis(v,it->first);
		}

//		iterScan it_scan=scans.end();
//		it_scan--;
//		it_scan->second->vis(v,ESTIMATE);

		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};
		int j=0;
		for(iterLandmark it=landmarks.begin();it!=landmarks.end();it++,j++)
		{
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointcloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
			pushPoints(it->second,pointcloud,fuse);
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color(pointcloud,red[j],grn[j],blu[j]);
			v->addPointCloud(pointcloud,color,it->first);
			if(it->second->Type()==LINE)  v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, it->first);
		}
		v->spin();
	}

	void Map::pushPoints(Landmark *lm, ptrPointCloud points, bool fuse)
	{
//		lm->points()=pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
		for(iterObserv it=lm->beginObserv();it!=lm->endObserv();it++) 
		{
			for(int i=0;i<it->second.size();i++)
			{
				double time=it->first;
				std::string id=it->second[i];
				Transform Tgc=camera(time).inv();
				Feature *ft=scan(time)->findFeature(id);
				std::vector<int>* ptr_indices=ft->ptrIndices();
				ptrPointCloud ptr_points=ft->ptrPoints();
				pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGBA>);
				for(int j=0;j<ptr_indices->size();j++)
				{
					int idx=(*ptr_indices)[j];
					pcl::PointXYZRGBA pt;
					pt.x=ptr_points->at(idx).x;
					pt.y=ptr_points->at(idx).y;
					pt.z=ptr_points->at(idx).z;
					tmp->push_back(pt);
				}
				pcl::transformPointCloud(*tmp,*tmp,Tgc.getMatrix4f());
//				*lm->points()=*lm->points()+*tmp;
				*points=*points+*tmp;
			}
		}
		if(fuse)
		if(lm->Type()==PLANE)
		{
			Eigen::Vector3d n=lm->planelm()->pi.block<3,1>(0,0);
			double d=lm->planelm()->pi(3)/n.norm();
			n.normalize();
			for(int i=0;i<points->size();i++)
			{
				Eigen::Vector3d p;
				p(0)=points->at(i).x;
				p(1)=points->at(i).y;
				p(2)=points->at(i).z;
				p=p-(n.dot(p)+d)*n;
				points->at(i).x=p(0);
				points->at(i).y=p(1);
				points->at(i).z=p(2);
			}
		}
		else if(lm->Type()==LINE)
		{
			Eigen::Vector3d v=lm->linelm()->L.block<3,1>(3,0);
			Eigen::Vector3d u=lm->linelm()->L.block<3,1>(0,0)/v.norm();
			v.normalize();
			for(int i=0;i<points->size();i++)
			{
				Eigen::Vector3d p;
				p(0)=points->at(i).x;
				p(1)=points->at(i).y;
				p(2)=points->at(i).z;
				Eigen::Vector3d w=u-p.cross(v);
				p=p+v.cross(w)*(w.norm()/v.cross(w).norm());
				points->at(i).x=p(0);
				points->at(i).y=p(1);
				points->at(i).z=p(2);
			}
		}
	}

//	void Map::saveTrajSLAM(const std::string &filename) const 
//	{
//		std::ofstream fp;
//		fp.open(filename,std::ios::out);
//		fp.precision(6); fp<<std::fixed;
//		for(const_iterCamera it=cameras.begin();it!=cameras.end();it++)
//		{
//			fp<<it->first<<" "<<it->second.inv()<<std::endl;
//		}
//		fp.close();
//	}
//
//	void Map::saveTrajVO(const std::string &filename) const 
//	{
//		std::ofstream fp;
//		fp.open(filename,std::ios::out);
//		fp.precision(6); fp<<std::fixed;
//		for(const_iterScan it=scans.begin();it!=scans.end();it++)
//		{
//			fp<<it->first<<" "<<it->second->Tcg().inv()<<std::endl;
//		}
//		fp.close();
//	}

	void Map::saveTraj(const std::string &filename) const 
	{
		std::ofstream fp;
		fp.open(filename,std::ios::out);
		printCameras(fp);
		fp.close();
	}

	void Map::saveMap(const std::string &filename) const 
	{
		std::ofstream fp;
		fp.open(filename,std::ios::out);
		printLandmarks(fp);
		fp.close();
	}

	void Map::loadTraj(const std::string &filename)
	{
		std::ifstream fp;
		fp.open(filename,std::ios::in);
		while(!fp.eof())
		{
			std::string s; getline(fp,s);
			if(!s.empty())
			{
				std::stringstream ss; ss<<s;
				double t,tx,ty,tz,qx,qy,qz,qw;
				ss>>t>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
				Eigen::Quaterniond quat(qw,qx,qy,qz);
				Eigen::Vector3d trans(tx,ty,tz);
//					timestamps.push_back(t);
//					cameras.push_back(ulysses::Transform(quat,trans));
				cameras.insert(std::pair<double,Transform>(t,Transform(quat,trans)));
				scans.insert(std::pair<double,Scan*>(t,new Scan(t)));
			}
		}
		fp.close();
	}

	void Map::loadMap(const std::string &filename)
	{
		std::ifstream fp;
		fp.open(filename,std::ios::in);
		std::stringstream ss; 
		std::string id; 
		int numPoints; 
		while(!fp.eof())
		{
			std::string s; getline(fp,s);
			if(!s.empty())
			{
				ss.clear(); ss<<s; ss>>id;
				if(id.substr(0,10)=="map_plane_")
				{
					Eigen::Vector4d pi;
					ss>>pi(0)>>pi(1)>>pi(2)>>pi(3);
					landmarks.insert(std::pair<std::string,Landmark*>(id,new Landmark(id,pi)));
				}
				else if(id.substr(0,10)=="map_xline_")
				{
					Vector6d L;
					ss>>L(0)>>L(1)>>L(2)>>L(3)>>L(4)>>L(5);
					landmarks.insert(std::pair<std::string,Landmark*>(id,new Landmark(id,L)));
				}
				Landmark* lm=landmarks.find(id)->second;
//				cout<<lm<<endl;
				// num of observ.s;
				ss>>numPoints;
//				cout<<numPoints<<endl;
				for(int i=0;i<numPoints;i++)
				{
					double observ_time;
					std::string observ_id;
					ss>>observ_time>>observ_id;
					lm->pushObserv(observ_time,observ_id);
//					cout<<observ_time<<", "<<observ_id<<endl;
				}
			}
		}
		fp.close();
	}


	Scan::~Scan()
	{
		point_cloud.reset();
		normal_cloud.reset();
		pixel_cloud.reset();
		if(!img_rgb.empty()) img_rgb.release();
		if(!img_depth.empty()) img_depth.release();
		for(iterFeature it=features.begin();it!=features.end();it++) {delete it->second;}
		for(size_t i=0;i<edge_points.size();i++) delete edge_points[i];
		if(feature_association!=0) delete feature_association;
		if(landmark_association!=0) delete landmark_association;
	}

	void Scan::release()
	{
		point_cloud.reset();
		normal_cloud.reset();
		pixel_cloud.reset();
		for(iterFeature it=features.begin();it!=features.end();it++) {it->second->release();}
		for(size_t i=0;i<edge_points.size();i++) 
		{ delete edge_points[i]; }
		edge_points.resize(0);
		if(!img_rgb.empty()) img_rgb.release();
		if(!img_depth.empty()) img_depth.release();
		if(feature_association!=0) 
		{
			delete feature_association;
			feature_association=0;
		}
		if(landmark_association!=0) 
		{
			delete landmark_association;
			landmark_association=0;
		}
	}

	void Scan::addFeature(Feature *f)
	{
		f->setID(features.size());
		features.insert(std::pair<std::string,Feature*>(f->ID(),f));
	}

	Feature* Scan::findFeature(std::string id) 
	{
		iterFeature it=features.find(id);
		if(it==features.end()) 
		{
//			std::cout<<"cannot find "<<id<<std::endl;
			return 0;
		}
		return it->second;
	}

	iterFeature Scan::eraseFeature(std::string id)
	{
		iterFeature it=features.find(id);
		if(it!=features.end()) 
		{
			it=features.erase(it);
		}
		return it;
	}

	void Scan::printFeatures(std::ostream &os)
	{
//			printPlanes(os);
//			printLines(os);
		for(iterFeature it=features.begin();it!=features.end();it++)
		{
			os<<it->second<<std::endl;
		}
	}

	void Scan::saveFeatures(const std::string &folder)
	{
		std::string filename=folder+"/scans/"+std::to_string(time_stamp)+".txt";
		std::ofstream fp;
		fp.open(filename,std::ios::out);
		for(iterFeature it=features.begin();it!=features.end();it++)
		{
			it->second->print(fp);
	}
//		printFeatures(fp);
//			printFeatures(std::cout);
		fp.close();
	}

	void Scan::saveAssociations(const std::string &folder)
	{
		std::string filename=folder+"/scans/"+std::to_string(time_stamp)+"_association.txt";
		std::ofstream fp;
		fp.open(filename,std::ios::out);
		feature_association->print(fp);
		fp.close();
	}

//	void Scan::saveAssociationsMap(const std::string &folder)
//	{
//		std::string filename=folder+"/scans/"+std::to_string(time_stamp)+"_association_map.txt";
//		std::ofstream fp;
//		fp.open(filename,std::ios::out);
//		landmark_association->print(fp);
//		fp.close();
//	}

//	using namespace std;
	bool Scan::loadFeatures(const std::string &folder)
	{
//		ofstream ff; ff.open("loadFeatures.txt");
		std::string filename=folder+"/scans/"+std::to_string(time_stamp)+".txt";
		std::ifstream fp;
		std::stringstream ss;
		std::string id, s; 
		int numPoints; 
		fp.open(filename,std::ios::in);
//		cout<<fp.is_open()<<endl;
		if(!fp.is_open()) return false;
		while(!fp.eof())
		{
			getline(fp,s);
//			ff<<s<<endl;
//			cout<<s<<endl;
			if(!s.empty())
			{
				ss.clear(); ss<<s; ss>>id;
				if(id.substr(0,6)=="plane_")
				{
					Eigen::Vector3d n; double d;
					ss>>n(0)>>n(1)>>n(2)>>d;
					features.insert(std::pair<std::string,Feature*>(id,new Feature(id,n,d)));
				}
				else if(id.substr(0,6)=="xline_")
				{
					Eigen::Vector3d u,v;
					ss>>u(0)>>u(1)>>u(2)>>v(0)>>v(1)>>v(2);
					features.insert(std::pair<std::string,Feature*>(id,new Feature(id,u,v)));
				}
			}
			Feature* ft=features.find(id)->second;
			ft->setPtrPoints(point_cloud);
			if(ft->Type()==PLANE)
			{
				ft->plane()->ptr_pixels=pixel_cloud;
				ft->plane()->ptr_normals=normal_cloud;
			}
//			cout<<ft<<endl;
			// num of points;
			getline(fp,s);
//			ff<<s<<endl;
//			cout<<s<<endl;
//			ss.str(""); 
			ss.clear(); ss<<s; ss>>numPoints;
//			cout<<numPoints<<endl;
			for(int i=0;i<numPoints;i++)
			{
				getline(fp,s);
//				ff<<s<<endl;
				int index;
//				ss.str(""); 
				ss.clear(); ss<<s; ss>>index;
				ft->pushIndex(index);
			}
		}
		fp.close();
		return true;
	}

//	void Scan::savePRC(const std::string &folder, FeatureAssociation *fa)
//	{
//		std::string filename=folder+"/PRC.txt";
//		std::ofstream fp; 
//		fp.open(filename,std::ios::app);
//		fp.precision(6); fp<<std::fixed;
//		double precision, recall;
//		feature_association->evalulatePR(fa,precision,recall);
//		fp<<time_stamp<<" "<<precision<<" "<<recall<<std::endl;
//		fp.close();
//	}
//
//	void Scan::savePRCMap(const std::string &folder, LandmarkAssociation *fa)
//	{
//		std::string filename=folder+"/PRC_map.txt";
//		std::ofstream fp; 
//		fp.open(filename,std::ios::app);
//		fp.precision(6); fp<<std::fixed;
//		double precision, recall;
//		landmark_association->evalulatePR(fa,precision,recall);
//		fp<<time_stamp<<" "<<precision<<" "<<recall<<std::endl;
//		fp.close();
//	}

	void Scan::vis(boost::shared_ptr<pcl::visualization::PCLVisualizer> v, POSE_TYPE type)
	{
		Transform T;
		if(type==IDENTITY) T.setIdentity();
		else if(type==ESTIMATE) T=T_cg.inv(); // Tgc
		else if(type==GROUNDTRUTH) T=T_cw.inv(); // Twc

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::transformPointCloud(*point_cloud,*cloud,T.getMatrix4f());

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
		vg.setInputCloud(cloud);
		vg.setLeafSize(0.01f,0.01f,0.01f);
		vg.filter(*cloud_filtered);

//		for(int i=0;i<cloud_filtered->size();i++)
//		{
//			cloud_filtered->at(i).x=-cloud_filtered->at(i).x;
//		}

		T.vis(v,time_stamp);
		if (!v->updatePointCloud(cloud_filtered,std::to_string(time_stamp)))
			v->addPointCloud(cloud_filtered,std::to_string(time_stamp));
//		v->spinOnce();
	}

	void Scan::visScanFeatures(boost::shared_ptr<pcl::visualization::PCLVisualizer> v)
	{
		v->removeAllPointClouds();
		v->removeAllCoordinateSystems();
		vis(v,IDENTITY);

		unsigned char red [14] = {   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = { 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};

//		pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
//		vg.setLeafSize(0.02f,0.02f,0.02f);

//		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planes (new pcl::PointCloud<pcl::PointXYZRGBA>);
//		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGBA>);
		int j=0;
		for(iterFeature it=features.begin();it!=features.end();it++,j++)
		{
//			if(it->second->Type()!=PLANE) continue;
//			Plane* pln=it->second->plane();
//			pcl::copyPointCloud(*pln->ptr_points,pln->indices,*tmp);
//			*planes=*planes+*tmp;
			it->second->vis(v,red[j%14],grn[j%14],blu[j%14],it->first+std::to_string(time_stamp), Transform::Identity());
		}
//		pcl::transformPointCloud(*planes,*planes,T.getMatrix4f());
//		vg.setInputCloud(planes);
//		vg.filter(*planes);
//		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color(planes,0,0,255);
//		if (!v->updatePointCloud (planes, color, "plane"+std::to_string(time_stamp))) 
//			v->addPointCloud (planes, color, "plane"+std::to_string(time_stamp));
//		v->spin();

//		if(feature_association==0) return;
//
//		v->removeAllPointClouds();
//		v->removeAllCoordinateSystems();
//		vis(v); scan_ref->vis(v);
//
//		for(int i=0;i<feature_association->size();i++)
//		{
//			Feature* ft_cur=feature_association->getFeature(i);
//			Feature* ft_ref=feature_association->getFeatureRef(i);
//			ft_cur->vis(v,red[i%14],grn[i%14],blu[i%14],ft_cur->ID()+std::to_string(time_stamp));
////			v->spin();
//			ft_ref->vis(v,red[i%14],grn[i%14],blu[i%14],ft_ref->ID()+std::to_string(time_stamp));
////			v->spin();
//		}

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edge (new pcl::PointCloud<pcl::PointXYZRGBA>);
		edge->resize(sizeEdgePoint());
		for(size_t i=0;i<sizeEdgePoint();i++)
		{
			if(!edgePoint(i)->isEdge) continue;
			edge->at(i).x=edgePoint(i)->xyz(0);
			edge->at(i).y=edgePoint(i)->xyz(1);
			edge->at(i).z=edgePoint(i)->xyz(2);
			edge->at(i).r=255;
			edge->at(i).g=255;
			edge->at(i).b=255;
		}
		pcl::transformPointCloud(*edge,*edge,Transform::Identity().getMatrix4f());
		v->addPointCloud (edge, "edge"+std::to_string(time_stamp));
		v->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "edge"+std::to_string(time_stamp));
//		v->spin();
	}

	void Scan::visScanFeatures(boost::shared_ptr<pcl::visualization::PCLVisualizer> v, POSE_TYPE type)
	{
//		vis(v,type);

		Transform T;
		if(type==IDENTITY) 
		{
			T.setIdentity();
			vis(v,type);
		}
		else if(type==ESTIMATE) T=T_cg.inv(); // Tgc
		else if(type==GROUNDTRUTH) T=T_cw.inv(); // Twc

//		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
//		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
//		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};
//
		pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
		vg.setLeafSize(0.02f,0.02f,0.02f);

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planes (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGBA>);
		int j=0;
		for(iterFeature it=features.begin();it!=features.end();it++,j++)
		{
			if(it->second->Type()!=PLANE) continue;
			Plane* pln=it->second->plane();
			if(!pln->in_map) continue;
			pcl::copyPointCloud(*pln->ptr_points,pln->indices,*tmp);
			*planes=*planes+*tmp;
//			it->second->vis(v,red[j%14],grn[j%14],blu[j%14],it->first+std::to_string(time_stamp), T.inv());
		}
		pcl::transformPointCloud(*planes,*planes,T.getMatrix4f());
		vg.setInputCloud(planes);
		vg.filter(*planes);
//		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color(planes,0,0,255);
		if (!v->updatePointCloud (planes, "plane"+std::to_string(time_stamp))) 
			v->addPointCloud (planes, "plane"+std::to_string(time_stamp));
//		v->spin();

//		if(feature_association==0) return;
//
//		v->removeAllPointClouds();
//		v->removeAllCoordinateSystems();
//		vis(v); scan_ref->vis(v);
//
//		for(int i=0;i<feature_association->size();i++)
//		{
//			Feature* ft_cur=feature_association->getFeature(i);
//			Feature* ft_ref=feature_association->getFeatureRef(i);
//			ft_cur->vis(v,red[i%14],grn[i%14],blu[i%14],ft_cur->ID()+std::to_string(time_stamp));
////			v->spin();
//			ft_ref->vis(v,red[i%14],grn[i%14],blu[i%14],ft_ref->ID()+std::to_string(time_stamp));
////			v->spin();
//		}

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edge (new pcl::PointCloud<pcl::PointXYZRGBA>);
		edge->resize(sizeEdgePoint());
		for(size_t i=0;i<sizeEdgePoint();i++)
		{
			if(!edgePoint(i)->isEdge) continue;
			edge->at(i).x=edgePoint(i)->xyz(0);
			edge->at(i).y=edgePoint(i)->xyz(1);
			edge->at(i).z=edgePoint(i)->xyz(2);
			edge->at(i).r=255;
			edge->at(i).g=255;
			edge->at(i).b=255;
		}
		pcl::transformPointCloud(*edge,*edge,T.getMatrix4f());
		v->addPointCloud (edge, "edge"+std::to_string(time_stamp));
		v->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "edge"+std::to_string(time_stamp));
//		v->spin();
	}

	void Scan::vis2Scans(boost::shared_ptr<pcl::visualization::PCLVisualizer> v, POSE_TYPE type)
	{
		v->removeAllPointClouds();
		v->removeAllShapes();
		v->removeAllCoordinateSystems();

		Transform T=Transform::Identity();
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::transformPointCloud(*point_cloud,*cloud,T.getMatrix4f());

		Transform Tcr;
		if(type==IDENTITY) Tcr=Transform::Identity();
		else Tcr=T_cr;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ref (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::transformPointCloud(*ref()->points(),*cloud_ref,Tcr.getMatrix4f());

		T.vis(v,ref()->time());
		Tcr.vis(v,time_stamp);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color(cloud,255,0,0);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color_ref(cloud_ref,0,0,255);
		v->addPointCloud(cloud,color,std::to_string(time_stamp));
		v->addPointCloud(cloud_ref,color_ref,std::to_string(ref()->time()));

		v->spin();
	}


	void Scan::visPlaneLines(boost::shared_ptr<pcl::visualization::PCLVisualizer> v)
	{
		v->removeAllPointClouds();
		v->removeAllCoordinateSystems();
//		vis(v,IDENTITY);

		unsigned char red [14] = { 255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {   0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {   0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};
		std::string color[14]={ "red", "green", "blue", "yellow", "magenta", "cyan", "darkred", "darkgreen", "darkblue", "olive", "purple", "darkcyan", "gray", "white"};

				// 255,  0,  0 - red
				//   0,255,  0 - green 
				//   0,  0,255 - blue
				// 255,255,  0 - yellow
				// 255,  0,255 - magenta
				//   0,255,255 - cyan
				// 130,  0,  0 - darkred
				//   0,130,  0 - darkgreen
				//   0,  0,130 - darkblue
				// 130,130,  0 - olive
				// 130,  0,130 - purple
				//   0,130,130 - darkcyan
				// 130,130,130 - gray 
				// 255,255,255 - white    

		int j=0;
		for(iterFeature it=features.begin();it!=features.end();it++,j++)
		{
			it->second->vis(v,red[j%14],grn[j%14],blu[j%14],it->first+std::to_string(time_stamp), Transform::Identity());
			std::cout<<it->first<<" "<<color[j%14]<<std::endl;
		}
	}

	void Scan::visEdges(boost::shared_ptr<pcl::visualization::PCLVisualizer> v)
	{
		v->removeAllPointClouds();
		v->removeAllCoordinateSystems();
		vis(v,IDENTITY);

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edge (new pcl::PointCloud<pcl::PointXYZRGBA>);
		edge->resize(sizeEdgePoint());
		for(size_t i=0;i<sizeEdgePoint();i++)
		{
			if(!edgePoint(i)->isEdge) continue;
			edge->at(i).x=edgePoint(i)->xyz(0);
			edge->at(i).y=edgePoint(i)->xyz(1);
			edge->at(i).z=edgePoint(i)->xyz(2);
			edge->at(i).r=255;
			edge->at(i).g=0;
			edge->at(i).b=0;
		}
		pcl::transformPointCloud(*edge,*edge,Transform::Identity().getMatrix4f());
		v->addPointCloud (edge, "edge"+std::to_string(time_stamp));
		v->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "edge"+std::to_string(time_stamp));
	}

	void Scan::visKeypoints(boost::shared_ptr<pcl::visualization::PCLVisualizer> v)
	{
		v->removeAllPointClouds();
		v->removeAllCoordinateSystems();
		vis(v,IDENTITY);

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZRGBA>);
		keypoints->resize(sizeKeyPoint()*2);
		for(size_t i=0;i<sizeKeyPoint();i++)
		{
			keypoints->at(i).x=keyPoint(i)->point_cur(0);
			keypoints->at(i).y=keyPoint(i)->point_cur(1);
			keypoints->at(i).z=keyPoint(i)->point_cur(2);
			keypoints->at(i).r=255;
			keypoints->at(i).g=0;
			keypoints->at(i).b=0;
			keypoints->at(sizeKeyPoint()+i).x=keyPoint(i)->point_ref(0);
			keypoints->at(sizeKeyPoint()+i).y=keyPoint(i)->point_ref(1);
			keypoints->at(sizeKeyPoint()+i).z=keyPoint(i)->point_ref(2);
			keypoints->at(sizeKeyPoint()+i).r=0;
			keypoints->at(sizeKeyPoint()+i).g=0;
			keypoints->at(sizeKeyPoint()+i).b=255;
		}
		pcl::transformPointCloud(*keypoints,*keypoints,Transform::Identity().getMatrix4f());
		v->addPointCloud (keypoints, "keypoints"+std::to_string(time_stamp));
		v->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoints"+std::to_string(time_stamp));
	}

	void Scan::loadScan(const double &time, const std::string &file_depth, const std::string &file_rgb)
	{
		time_stamp=time;
		point_cloud=pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
		normal_cloud=pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
		pixel_cloud=pcl::PointCloud<pcl::PointXY>::Ptr (new pcl::PointCloud<pcl::PointXY>);

		cv::Mat rgb_image,depth_image;
		uint8_t *depth_ptr,*rgb_ptr;
		pcl::PointXYZRGBA point_tmp;
		unsigned short *depth_tmp_ptr=new unsigned short;
		pcl::PointXY tmp_pointxy;
		// full path of the current rgb and depth image;
//		std::string filename_rgb_full=path+"/"+filename_rgb;
//		std::string filename_depth_full=path+"/"+filename_depth;
		// load the rgb and depth image to cv::Mat;
		// the depth_image is stored as CV_8UC2;
		img_depth=cv::imread(file_depth,cv::IMREAD_UNCHANGED);
		img_rgb=cv::imread(file_rgb,cv::IMREAD_UNCHANGED);
		filename_rgb=file_rgb;
		//cv::imshow("rgb",img_rgb);
		//cv::waitKey(0);
		//cv::imshow("dep",img_depth);
		//cv::waitKey(0);

		// pointer to the Mat data;
		rgb_ptr=img_rgb.data;
		depth_ptr=img_depth.data;
		// clear the pointcloud;
		// the allocated memory does not release;
		// the newly pushed elements cover the old ones;
		point_cloud->clear();
		normal_cloud->clear();
		pixel_cloud->clear();
		// generate the point_cloud;
		for(int i=0;i<img_depth.rows;i++)
		{
			for(int j=0;j<img_depth.cols;j++)
			{
				// 3 channels for one pixel in rgb image;
				point_tmp.b=*rgb_ptr;
				rgb_ptr++;
				point_tmp.g=*rgb_ptr;
				rgb_ptr++;
				point_tmp.r=*rgb_ptr;
				rgb_ptr++;
				// 2 channels for one pixel in depth image;
				memcpy(depth_tmp_ptr,depth_ptr,2);
				depth_ptr+=2;
				point_tmp.z=*depth_tmp_ptr/camera_intrinsic.factor;
				// transformation from pixel coordinate to the camera coordinate;
				// wrong results if considering length of the pixel;
				point_tmp.x=(j-camera_intrinsic.cx)*point_tmp.z/camera_intrinsic.fx;
				point_tmp.y=(i-camera_intrinsic.cy)*point_tmp.z/camera_intrinsic.fy;
//				if(j<20 || j>=490 || i<50 || i>=370 || point_tmp.z>8.0)
//				{
//					point_tmp.z=0;
//					point_tmp.x=0;
//					point_tmp.y=0;
//				}
				point_cloud->push_back(point_tmp);
			}
		}
		delete depth_tmp_ptr;
		// organize the point_cloud for the normal estimation;
		point_cloud->width=camera_intrinsic.width;//500;//
		point_cloud->height=camera_intrinsic.height;//420;//
		// generate the normal_cloud;
		pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normal_estimate_integral;
		normal_estimate_integral.setNormalEstimationMethod(pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA,pcl::Normal>::AVERAGE_DEPTH_CHANGE);
//		AVERAGE_3D_GRADIENT
//		AVERAGE_DEPTH_CHANGE
//		COVARIANCE_MATRIX
		normal_estimate_integral.setDepthDependentSmoothing(true);
		normal_estimate_integral.setNormalSmoothingSize(40.0);
		normal_estimate_integral.setInputCloud(point_cloud);
		normal_estimate_integral.compute (*normal_cloud);
		// generate the pixel_cloud;
		for(int v=0;v<point_cloud->height;v++)
		{
			for(int u=0;u<point_cloud->width;u++)
			{
				tmp_pointxy.x=u;
				tmp_pointxy.y=v;
				pixel_cloud->push_back(tmp_pointxy);
			}
		}
	}

	void Scan::addEdgePoint(int i)
	{
		if(fabs(point_cloud->at(i).z)<1e-4) return;
		EdgePoint* ep=new EdgePoint;
		ep->xyz(0)=point_cloud->at(i).x;
		ep->xyz(1)=point_cloud->at(i).y;
		ep->xyz(2)=point_cloud->at(i).z;
		ep->pixel(0)=pixel_cloud->at(i).x;
		ep->pixel(1)=pixel_cloud->at(i).y;
		ep->idx=i;
		edge_points.push_back(ep);
	}

}
