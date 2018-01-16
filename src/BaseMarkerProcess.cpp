#include <BaseMarkerProcess.h>

//-----------------OPTI MASK---------------------------------

OptiMask::OptiMask(Size im_size)
	:OptiMask()
{
	staticMask=Mat(im_size, CV_8UC3, Scalar::all(0));
};

void OptiMask::cleanOldMask(){
	list<ros::Time>::iterator time = sliding_timestamp.end();
	while(sliding_timestamp.size()!=0 && time!=sliding_timestamp.begin()){
		time--;//pour choisir le temps précedent
		ros::Duration delta=ros::Time::now()-(*time);
		if(delta >ros::Duration(OLDEST_OPTI_MASK )){
			sliding_mask.pop_back();
			sliding_timestamp.pop_back();
		}
		else
			return;
	}
}

Rect2d OptiMask::
watchingBindingBox(Marker marker,Size im_size){
	float cote=marker.getPerimeter()/WATCHING_BOX_DIVIDER;
	Point2f centre=marker.getCenter();
	int x=max((int)(centre.x-cote),0);
	int y=max((int)(centre.y-cote),0);
	Rect2d res = Rect2d(x,
			  	  	    y,
			            min((int)(cote*2),im_size.width-x),
			            min((int)(cote*2),im_size.height-y));
	return res;
}

void OptiMask::
updateOptiMask(vector<Marker>markers){
	Mat new_mask;

	(*r_mutex).lock();

	staticMask.copyTo(new_mask);
	for(int i=0;i<markers.size();i++){
		Rect2d box=watchingBindingBox(markers[i],
									staticMask.size());
		new_mask(box).setTo(Scalar::all(255));
	}
	sliding_mask.push_front(new_mask);
	sliding_timestamp.push_front(ros::Time::now());

	cleanOldMask();

	(*r_mutex).unlock();
};

Mat OptiMask::getOptiMask(){
	Mat res_mat;

	(*r_mutex).lock();

	staticMask.copyTo(res_mat);
	Mat WhiteMask(staticMask.size(), CV_8UC3, Scalar::all(255));

	for(list<Mat>::iterator i=sliding_mask.begin();i!=sliding_mask.end();i++){
		WhiteMask.copyTo(res_mat,(*i));
	}

	(*r_mutex).unlock();

	return res_mat;
}

//-----------------MARKER PROCESSER---------------------------------

MarkerProcesser::
MarkerProcesser(string yaml,ImageHandler* i,PublisherHandler* p)
	:MarkerProcesser(
			CameraParameters(),
		    YAML::LoadFile(yaml)["marker_size"].as<float>(),
		    i,p)
{

	CameraParameters cam_params;
	cam_params.readFromXMLFile(yaml);
	//wait a image
	Mat current_image;
	while (current_image.empty()) {
		ros::spinOnce();
		ImConv->getCurrentImage(&current_image);
		usleep(1000);
	}

	//Config Camera Params & Optimask
	cam_params.resize(current_image.size());
	TheCameraParameters=cam_params;
	OptimisationMask=OptiMask(current_image.size());
}

MarkerProcesser::
MarkerProcesser(CameraParameters cam_params,float mark_size,ImageHandler* i,PublisherHandler* p)

{
	ImConv=i;

	r_save=new std::recursive_mutex ();

	//detect marker
	MDetector.setCornerRefinementMethod(MarkerDetector::LINES);
	//Minimum parameters
	TheMarkerSize=mark_size;

	//Output Publisher
	publisher=p;

	if( cam_params.CamSize.height <= 0 || cam_params.CamSize.width <= 0  )
		return ;

	//wait a image
	Mat current_image;
	while (current_image.empty()) {
		ros::spinOnce();
		ImConv->getCurrentImage(&current_image);
		usleep(1000);
	}

	//Config Camera Params & Optimask
	cam_params.resize(current_image.size());
	TheCameraParameters=cam_params;
	OptimisationMask=OptiMask(current_image.size());

}

void MarkerProcesser::
DetectUpdateMaskPublish(bool Opti,Mat* plot){

	(*r_save).lock();
	ros::spinOnce();
	(*r_save).unlock();

	//get im
	Mat current_im,trait_im;
	if(Opti)
		ImConv->getCurrentImage(&current_im);
	else
		ImConv->getLastImage(&current_im);

	ros::Time mesure_temps=ros::Time::now();
	//apply mask if needed
	if(Opti){
		current_im.copyTo(trait_im,OptimisationMask.getOptiMask());
	}
	else{
		current_im.copyTo(trait_im);
	}
	//Detection
	vector<Marker>markers;
	(*r_save).lock();
	MDetector.detect(trait_im, markers, TheCameraParameters, TheMarkerSize);
	(*r_save).unlock();

	//update optimask
	OptimisationMask.updateOptiMask(markers);
	//plot if needed
	if(plot!=NULL){
		trait_im.copyTo(*plot);
		aff_markers(markers,plot);
	}

	//publish
	(*r_save).lock();
	publisher->publishMarckersPose(markers);
	cout<<"Time:"<<(ros::Time::now()-mesure_temps)*1000<<" ms"<<endl;
	(*r_save).unlock();
}

void MarkerProcesser::
aff_markers(vector<Marker>markers,Mat *plot){
	if(plot==NULL)
		return;
	char id_str[3];
	Point2d coin_bas_gauche_text;
	for(int i=0;i<markers.size();i++){
		//axis
		CvDrawingUtils::draw3dAxis(*plot,
								TheCameraParameters,
								markers[i].Rvec,
								markers[i].Tvec,
								PLOT_AXIS_LENGHT);
		//id of marker
		sprintf(id_str,"%d",markers[i].id);
		Point2d coin_bas_droit_text=markers[i].getCenter()-Point2f(markers[i].getPerimeter()/10.,
																  -markers[i].getPerimeter()/10.);
		putText(*plot,id_str,coin_bas_droit_text,
		    		FONT_HERSHEY_SCRIPT_SIMPLEX,markers[i].getPerimeter()/4./50,
					Scalar(0,0,255),3);
	}
}


//optimisation thread => use the opti mask
void threadUseMaskOptimisation(MarkerProcesser *mark_process){
	while(true){
		ros::Time test=ros::Time::now();
		mark_process->DetectUpdateMaskPublish(true);
		cout<<"IN :"<<ros::Time::now()-test<<" ms"<<endl;
	}
}
