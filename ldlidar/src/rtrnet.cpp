#include <cstring>
#include <stdint.h>
#include <vector>
#include <array>
#include <iostream>
#include <math.h>
#include <algorithm>
#include "trnet.h"
#include "rtrnet.h"

RTRNet::RTRNet():frame_ready_(false),
    parameters_ready_(false),
    coe_u_(0),
    coe_v_(0)
{
    input.resize(160);
}

bool RTRNet::UnpackData(const uint8_t *data, uint32_t len)
{
    int pos = 0;
    TRNet t;

    for (int i = 0; i < len; i++)
    {
        data_tmp_.push_back(*(data + i));
    }

    if (data_tmp_.size() < 4)
    {
        return false;
    }

    for (int i = 0; i < data_tmp_.size() - 4;)
    {
        bool is_find = t.FindLeadingCode(data_tmp_.data() + i);
        if (is_find)
        {
            const TRData *tr_data = nullptr;
            tr_data = t.Unpack(data_tmp_.data() + i, data_tmp_.size() - i - 4);
            if (tr_data != nullptr)
            {
                switch (tr_data->pack_id)
                {
                case PACK_GET_DISTANCE:
                    Transform(tr_data);
                    break;

                case PACK_GET_COE:
                    coe_[0] = *(uint32_t *)(tr_data->data.data());
                    coe_[1] = *(uint32_t *)(tr_data->data.data() + 4);
                    coe_[2] = *(uint32_t *)(tr_data->data.data() + 8);
                    coe_[3] = *(uint32_t *)(tr_data->data.data() + 12);
                    coe_u_ = *(uint16_t *)(tr_data->data.data() + 16);
                    std::cout << "get param successfull!!" << std::endl;
                    std::cout << "k0=" << coe_[0] << "  "
                              << "k1=" << coe_[1] << "  "
                              << "b0=" << coe_[2] << "  "
                              << "b1=" << coe_[3] << "  " << std::endl;
                    break;

                case PACK_VIDEO_SIZE:
                    coe_u_ = *(uint16_t *)(tr_data->data.data());
                    coe_v_ = *(uint16_t *)(tr_data->data.data() + 2);
                    std::cout << "Picture pixels: " << coe_u_ << "*" << coe_v_ << std::endl;
                    parameters_ready_ = true;
                    break;

                default:
                    break;
                }

                i += t.GetParseDataLen();
                pos = i;
            }
            else
            {
                i++;
            }
        }
        else
        {
            i++;
        }
    }

    if (pos > 0)
    {
        data_tmp_.erase(data_tmp_.begin(), data_tmp_.begin() + pos);
    }
}

bool RTRNet::Transform(const TRData *tr_data)
{
    /*Packet length minus 4-byte timestamp*/
    int data_amount = tr_data->data.size() - 4; 
    int n = 0;
    /*Pixel height*/
    int height = 160; 

    for (uint i = 0; i < data_amount; i += 2, n++)
    {
        /*Acquired distance information data*/
        uint16_t value = *(uint16_t *)(tr_data->data.data() + i + 4);
        uint8_t confidence = (value >> 9) << 1;
        value &= 0x1ff;
        int center_dis = value;
        double d = center_dis;
        if (center_dis > 0)
        {
            TransformSignlePoint(center_dis, n, confidence);
        }else
        {
            input[n].x = 0;
            input[n].y = 0;
            input[n].intensity=0;
        }
        
    }

    pcl::toROSMsg(input,output);
    output.header.frame_id = "lidar_frame";
	output.header.stamp = ros::Time::now();
    frame_ready_ = true;
    return true;
}

void RTRNet::TransformSignlePoint(uint16_t dist, int n, uint8_t confidence)
{
    /*Parameters previously obtained by PackID::PACK_GET_COE */
    double k0 = (double)(coe_[0]) / 10000.0;
    double k1 = (double)(coe_[1]) / 10000.0;
    double b0 = (double)(coe_[2]) / 10000.0;
    double b1 = (double)(coe_[3]) / 10000.0;
    const double pi = 3.14159265;
    double pixel_u = n, tmp_theta, tmp_dist, tmp_x, tmp_y;

    if (pixel_u > 80)
    {
        pixel_u = pixel_u - 80;
        pixel_u = 80 - pixel_u;
        if (b0 > 1) //217之前的版本计算的b值在20-30之间，217之后的版本计算的b值小于1;
        {
            tmp_theta = k0 * pixel_u - b0;
        }
        else
        {
            tmp_theta = atan(k0 * pixel_u - b0) * 180 / pi;
        }
        tmp_dist = (dist - 1.22) / cos((22.5 - (tmp_theta)) * pi / 180);
        tmp_theta = tmp_theta * pi / 180;
        tmp_x = cos(22.5 * pi / 180) * tmp_dist * cos(tmp_theta) + sin(22.5 * pi / 180) * (tmp_dist * sin(tmp_theta));
        tmp_y = -sin(22.5 * pi / 180) * tmp_dist * cos(tmp_theta) + cos(22.5 * pi / 180) * (tmp_dist * sin(tmp_theta));
        tmp_x = tmp_x + 1.22;
        tmp_y = tmp_y - 5.315;
    }
    else
    {
        pixel_u = 80 - pixel_u;
        if (b1 > 1)
        {
            tmp_theta = k1 * pixel_u - b1;
        }
        else
        {
            tmp_theta = atan(k1 * pixel_u - b1) * 180 / pi;
        }
        tmp_dist = (dist - 1.22) / cos((22.5 + (tmp_theta)) * pi / 180);
        tmp_theta = tmp_theta * pi / 180;
        tmp_x = cos(-22.5 * pi / 180) * tmp_dist * cos(tmp_theta) + sin(-22.5 * pi / 180) * (tmp_dist * sin(tmp_theta));
        tmp_y = -sin(-22.5 * pi / 180) * tmp_dist * cos(tmp_theta) + cos(-22.5 * pi / 180) * (tmp_dist * sin(tmp_theta));
        tmp_x = tmp_x + 1.22;
        tmp_y = tmp_y + 5.315;
    }
    input[n].x = tmp_x;
    input[n].y = tmp_y;
    input[n].intensity=confidence;
}

void RTRNet::SendCmd(CmdInterfaceLinux &port, uint8_t address, uint8_t id)
{
	TRNet pkg;
	std::vector<uint8_t> out;
	TRData out_data;
	uint32_t len = 0;
	out_data.device_address = address;
	out_data.pack_id = id;
	out_data.chunk_offset = 0;
	pkg.Pack(out_data, out);
	port.WriteToIo((const uint8_t *)out.data(), out.size(), &len);
}
