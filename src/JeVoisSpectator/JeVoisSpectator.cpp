/*
 * JeVoisSpectator.cpp
 *
 *  Created on: 17 janv. 2018
 *      Author: liehnfl
 */

#include "JeVoisSpectator.h"

//-----------------JeVoisImageHandler---------------------------------
JeVoisImageHandler::
JeVoisImageHandler(jevois::InputFrame frame_in):
		r_mutex(new std::recursive_mutex ()),inframe(&frame_in) {}

void JeVoisImageHandler::
getCurrentImage(cv::Mat *input_image){
	(*r_mutex).lock();
	*input_image=current_im;
	(*r_mutex).unlock();
}

void JeVoisImageHandler::
getCurrentTime(long int* t){
	(*r_mutex).lock();
	*t=current_time;
	(*r_mutex).unlock();
}

void JeVoisImageHandler::
getLastImage(cv::Mat *input_image){
	//image
	jevois::RawImage const inimg = (*inframe).get(true);
	(*inframe).done();//free inframe

	//time
	struct timeval tp;
	gettimeofday(&tp, NULL);

	(*r_mutex).lock();
	current_im = jevois::rawimage::convertToCvBGR(inimg);
	current_time=tp.tv_sec * 1e6 + tp.tv_usec;
	*input_image=current_im;
	(*r_mutex).unlock();
}

//-----------------JeVoisUartPublisher---------------------------------
JeVoisUartPublisher::
JeVoisUartPublisher(jevois::OutputFrame frame_out,int id_cam)
			:outframe(&frame_out),Cam_id(id_cam) {}

void JeVoisUartPublisher::publishMarker(Marker){
	//TODO use UART
}

void JeVoisUartPublisher::
publishMarkersPose(vector<Marker>markers,Mat *plot){
	//pass through image
	if(plot!=NULL){
		//convert to jevois::RawImage
		jevois::RawImage outimg = (*outframe).get();
		jevois::rawimage::convertCvBGRtoRawImage(*plot, outimg,50);
		jevois::rawimage::writeText(outimg, "Hello JeVois!",
				100, 230, jevois::yuyv::White, jevois::rawimage::Font20x38);
		//require good dimension
		outimg.require("output", outimg.width, outimg.height, outimg.fmt);
		//envoie de l'image
		(*outframe).send();
	}
	//publish data on UART
	for(int i=0;i<markers.size();i++)
		publishMarker(markers[i]);
}


 // Simple module that just passes the captured camera frames through to USB host
 class JeVoisSpectator : public jevois::Module
 {
   public:
     // Default base class constructor ok
     using jevois::Module::Module;

     // Virtual destructor for safe inheritance
     virtual ~JeVoisSpectator() { }

     // Processing function
     virtual void process(jevois::InputFrame && inframe, jevois::OutputFrame && outframe) override
     {
       // Wait for next available camera image:
       jevois::RawImage const inimg = inframe.get(true);

       // Wait for an image from our gadget driver into which we will put our results:
       jevois::RawImage outimg = outframe.get();

       // Enforce that the input and output formats and image sizes match:
       outimg.require("output", inimg.width, inimg.height, inimg.fmt);

       // Just copy the pixel data over:
       memcpy(outimg.pixelsw<void>(), inimg.pixels<void>(), outimg.bytesize());

       jevois::rawimage::writeText(outimg, "Hello JeVois!", 100, 230,
    		   jevois::yuyv::White, jevois::rawimage::Font20x38);
       // Let camera know we are done processing the input image:
       inframe.done(); // NOTE: optional here, inframe destructor would call it anyway

       // Send the output image with our processing results to the host over USB:
       outframe.send(); // NOTE: optional here, outframe destructor would call it anyway
     }
 };

 // Allow the module to be loaded as a shared object (.so) file:
 JEVOIS_REGISTER_MODULE(JeVoisSpectator);


