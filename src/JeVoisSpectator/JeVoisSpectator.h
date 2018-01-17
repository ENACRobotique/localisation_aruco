/*
 * JeVoisSpectator.h
 *
 *  Created on: 17 janv. 2018
 *      Author: liehnfl
 */

#ifndef SRC_JEVOISSPECTATOR_JEVOISSPECTATOR_H_
#define SRC_JEVOISSPECTATOR_JEVOISSPECTATOR_H_

#include <jevois/Core/Module.H>
#include <jevois/Image/RawImageOps.H>
#include <BaseMarkerProcess.h>

class JeVoisImageHandler:public ImageHandler{
	//jevois image interface
	jevois::InputFrame * inframe;
	//minimal properties
	std::recursive_mutex * r_mutex;
	Mat current_im;
	long int current_time=0;

public:

	//Constructor, Destructor
	JeVoisImageHandler(jevois::InputFrame frame_in);
	~JeVoisImageHandler(){}

	//override needed functions
	void getCurrentImage(cv::Mat *input_image);
	void getLastImage(cv::Mat *input_image);
	void getCurrentTime(long int* t);//en us
};

class JeVoisUartPublisher:public PublisherHandler{
private:
	//debug output
	jevois::OutputFrame * outframe;

	int Cam_id=0;
	void publishMarker(Marker);
public:
	JeVoisUartPublisher(jevois::OutputFrame frame_out,int id_cam);
	void publishMarkersPose(vector<Marker>markers,Mat *plot =NULL);
};



#endif /* SRC_JEVOISSPECTATOR_JEVOISSPECTATOR_H_ */
