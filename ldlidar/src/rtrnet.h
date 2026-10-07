#ifndef _R_TR_NET_H
#define _R_TR_NET_H

#include <stdint.h>
#include <vector>
#include <iostream>
#include <array>
#include "cmd_interface_linux.h"
#include "trnet.h"

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef enum 
{
	PACK_GET_DISTANCE = 0x02,  /*Frame ID of distance data*/
	PACK_STOP = 0x0F,		   /*Frame ID of stop distance data transmission*/
	PACK_GET_COE = 0x12,	   /*Frame ID of Get parameters*/
	PACK_VIDEO_SIZE = 0x15,	   /*Frame ID of Get camera resolution*/
	PACK_CONFIG_ADDRESS = 0x16 /*Frame ID of Configure address*/
}PackageIDTypeDef;

class RTRNet 
{
public:
	RTRNet();
	void SendCmd(CmdInterfaceLinux &port, uint8_t address, uint8_t id);
	bool UnpackData(const uint8_t *data, uint32_t len);
	void ResetFrameReady() {frame_ready_ = false;}
	sensor_msgs::PointCloud2 GetPointCloud2() {return output;}
	bool IsParametersReady(void) { return parameters_ready_; }
	bool IsFrameReady(void) { return frame_ready_; } /*Lidar data frame is ready*/



private:
	bool Transform(const TRData *tr_data);			   /*transform raw data to stantard data */
	void TransformSignlePoint(uint16_t dist, int n, uint8_t confidence);
    std::vector<uint8_t> data_tmp_;
	uint32_t coe_[4];
	uint16_t coe_u_;
	uint16_t coe_v_;
	bool parameters_ready_;
	bool frame_ready_;
	pcl::PointCloud<pcl::PointXYZI> input;
	sensor_msgs::PointCloud2 output;

};

#endif // _TR_NET_H
