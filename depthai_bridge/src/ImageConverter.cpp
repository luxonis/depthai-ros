#include <depthai_bridge/ImageConverter.hpp>


// FIXME(Sachin): Do I need to convert the encodings that are available in dai to only that ros support ? 
// I mean we can publish whatever it is and decode it on the other side but howver maybe we should have option to convert planar to interleaved before publishing ???


// By default everthing form dai is changed to interleaved when publishing over ros.
// and if we subscribe to a previously published ros msg as input to xlinkin node 
// then we need to convert it back to planar if xlinkin node needs it planar 
namespace dai::ros {

    ImageConverter::ImageConverter(bool interleaved):_daiInterleaved(interleaved){}

    ImageConverter::toRosMsg(std::shared_ptr<dai::ImgFrame> inData, sensor_msgs::Image& opMsg){
    
        if (encoding_enum_map.find(inData->getType()) == encoding_enum_map.end())
            throw std::runtime_error("Encoding value node found ");

        auto tstamp = inData->getTimestamp();
        // std::cout << "time ->  " << encoding_enum_map[inData->getType()] << "  " << (int)inData->getType() << std::endl;
        int32_t sec = std::chrono::duration_cast<std::chrono::seconds>(tstamp.time_since_epoch()).count();
        int32_t nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(tstamp.time_since_epoch()).count() % 1000000000UL;

        // setting the header
        // std_msgs::Header imgHeader; 
        opMsg.seq      = inData->getSequenceNum();
        opMsg.stamp    = ros::Time(sec, nsec);;
        opMsg.frame_id = _frameName;

        // if(encoding_enum_map[inData->getType()] == "NV12"){
        //     //TODO(sachin): Replace cv_mat->cvbridge with handling raw convertions
        //     // ------------------------------------------------------------------------------------ //
        //     cv::Mat nv_frame(inData->getHeight() * 3 / 2, inData->getWidth(), CV_8UC1, inData->getData().data());
        //     cv::Mat rgb(inData->getHeight(), inData->getWidth(), CV_8UC3);
        //     cv::cvtColor(nv_frame, rgb, cv::COLOR_YUV2BGR_NV12);
        //     cv_bridge::CvImage cvRgbImg(imgHeader, sensor_msgs::image_encodings::BGR8, rgb);
        //     cvRgbImg.toImageMsg(outImageMsg);
        // }
        // else 
        if(encoding_enum_map[inData->getType()].find("planar") != std::string::npos){
            
            std::istringstream f(encoding_enum_map[inData->getType()]);
            std::vector<std::string> encoding_info;    
            std::string s;
            
            while (getline(f, s, '_')) 
                encoding_info.push_back(s);
            // outImageMsg.header        = imgHeader;
            outImageMsg.encoding      = encoding_info[3];
            outImageMsg.height       = inData->getHeight();
            outImageMsg.width        = inData->getWidth();
            outImageMsg.step         = inData->getData().size() / inData->getHeight();
            outImageMsg.is_bigendian = true;
            size_t size = inData->getData().size();
            outImageMsg.data.resize(size);
            planarToInterleaved(inData->getData(), outImageMsg.data, outImageMsg.width, outImageMsg.height , std::stoi(encoding_info[1]), std::stoi(encoding_info[2]));
        }
        else{
            // copying the data to ros msg
            // outImageMsg.header       = imgHeader;
            outImageMsg.encoding     = encoding_enum_map[inData->getType()];
            outImageMsg.height       = inData->getHeight();
            outImageMsg.width        = inData->getWidth();
            outImageMsg.step         = inData->getData().size() / inData->getHeight();
            // std::cout << inData->getData().size() << "..." << inData->getHeight() << std::endl;
            if (outImageMsg.encoding == "mono16")
                outImageMsg.is_bigendian = false;
            else
                outImageMsg.is_bigendian = true;

            size_t size = inData->getData().size();
            outImageMsg.data.resize(size);
            unsigned char* imageMsgDataPtr = reinterpret_cast<unsigned char*>(&outImageMsg.data[0]);
            unsigned char* daiImgData = reinterpret_cast<unsigned char*>(inData->getData().data());

            // TODO(Sachin): Try using assign since it is a vector img->data.assign(packet.data->cbegin(), packet.data->cend());
            memcpy(imageMsgDataPtr, daiImgData, size);
        }
        return;
    }

    // TODO(sachin): Not tested
    void ImageConverter::toDaiMsg(sensor_msgs::Image& inMsg, std::shared_ptr<dai::ImgFrame> outData){

        std::unordered_map<dai::RawImgFrame::Type, std::string>::iterator revEncodingIter;
        if(_daiInterleaved){
            revEncodingIter = std::find_if( encodingEnumMap.begin(),
                                            encodingEnumMap.end(), 
                                            [&](const std::pair<dai::RawImgFrame::Type, std::string> &pair)
                                            {
                                                return pair.second == inMsg.encoding;
                                            });
            if(revEncodingIter == encodingEnumMap.end()) std::runtime_error("Unable to find DAI encoding for the corresponding sensor_msgs::image.encoding stream")
            
            outData->setData(inMsg.data);
        }
        else{
            revEncodingIter = std::find_if( encodingEnumMap.begin(),
                                encodingEnumMap.end(), 
                                [&](const std::pair<dai::RawImgFrame::Type, std::string> &pair)
                                {   
                                   return pair.second.find(inMsg.encoding) != pair.second.end();
                                    
                                });

            std::istringstream f(revEncodingIter->second);
            std::vector<std::string> encoding_info;    
            std::string s;
            
            while (getline(f, s, '_')) 
                encoding_info.push_back(s);

            std::vector<std::uint8_t> opData(inMsg.data.size());
            interleavedToPlanar(inMsg.data, opData, inMsg.height, inMsg.width,std::stoi(encoding_info[1]), std::stoi(encoding_info[2]));
            outData->setData(opData);
        }
        
        /** FIXME(sachin) : is this time convertion correct ??? 
          * Print the original time and ros time in seconds in 
          * ImageFrame::toRosMsg(std::shared_ptr<dai::ImgFrame> inData, sensor_msgs::Image& opMsg)
          * to cross verify..
          **/
        timePoint ts(std::chrono::seconds(inMsg.header.stamp.toSec()) + std::chrono::nanoseconds(inMsg.header.stamp.toSec()));
        outData->setTimestamp(ts);
        outData->setSequenceNum(inMsg.header.seq)
        outData->setWidth(inMsg.width)
        outData->setHeight(inMsg.height);
        outData->setType(revEncodingIter->first);
    }

    sensor_msgs::ImagePtr ImageConverter::toRosMsgPtr(std::shared_ptr<dai::ImgFrame> inData){
      sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
      toRosMsg(inData, *ptr);
      return ptr;
    }

    void ImageConverter::planarToInterleaved(const std::vector<uint8_t>& srcData, std::vector<uint8_t>& destData, int w, int h , int numPlanes, int bpp){
        
        if(numPlanes == 3){
            // optimization (cache)
            for(int i = 0; i < w*h; i++) {
                uint8_t b = srcData.data()[i + w*h * 0];
                destData[i*3+0] = b;
            }
            for(int i = 0; i < w*h; i++) {                
                uint8_t g = srcData.data()[i + w*h * 1];    
                destData[i*3+1] = g;
            }
            for(int i = 0; i < w*h; i++) {
                uint8_t r = srcData.data()[i + w*h * 2];
                destData[i*3+2] = r;
            }
        } 
        else{
            std::runtime_error("If you encounter the scenario where you need this please create an issue on github");
        }
        return;
    }

    void ImageConverter::interleavedToPlanar(const std::vector<uint8_t>& srcData, std::vector<uint8_t>& destData, int w, int h , int numPlanes, int bpp){
        if(numPlanes == 3){
            // optimization (cache)
            for(int i = 0; i < w * h; i++) {


                uint8_t b = srcData[i*3+0];
                uint8_t g = srcData[i*3+1];
                uint8_t r = srcData[i*3+2];
                
                destData[i + w * h * 0] = b;
                destData[i + w * h * 1] = g;
                destData[i + w * h * 2] = r;
            }
            // for(int i = 0; i < w*h; i++) {                
            //     uint8_t g = srcData.data()[i + w*h * 1];    
            //     destData[i*3+1] = g;
            // }
            // for(int i = 0; i < w*h; i++) {
            //     uint8_t r = srcData.data()[i + w*h * 2];
            //     destData[i*3+2] = r;
            // }
        } 
        else{
            std::runtime_error("If you encounter the scenario where you need this please create an issue on github");
        }
        return;
    }


    void ImageConverter::rosMsgtoCvMat(sensor_msgs::Image& inMsg, cv::Mat dest){
        std::runtime_error("THis frature is still WIP")
    }


}   // namespace dai::ros
