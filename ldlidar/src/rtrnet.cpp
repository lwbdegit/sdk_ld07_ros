#include <cstring>
#include <stdint.h>
#include <vector>
#include <array>
#include <iostream>
#include <limits>
#include <math.h>
#include <algorithm>
#include "trnet.h"
#include "rtrnet.h"

RTRNet::RTRNet():coe_u_(0),
    coe_v_(0),
    parameters_ready_(false),
    frame_ready_(false),
    rx_bytes_(0),
    frames_parsed_(0),
    logged_first_frame_(false)
{
    input.resize(160);
}

bool RTRNet::UnpackData(const uint8_t *data, uint32_t len)
{
    const uint32_t kExtraLen = 11; /*leading + header + checksum*/
    const uint32_t kMaxPayload = 4096;
    TRNet t;

    if (data != nullptr && len > 0)
    {
        data_tmp_.insert(data_tmp_.end(), data, data + len);
        rx_bytes_.fetch_add(len);
    }

    size_t i = 0;
    while (i + 4 <= data_tmp_.size())
    {
        if (!t.FindLeadingCode(data_tmp_.data() + i))
        {
            i++;
            continue;
        }

        /*Length field sits 8 bytes after the leading code. Wait until it arrives.*/
        if (data_tmp_.size() - i < 10)
        {
            break;
        }

        uint16_t payload = *(uint16_t *)(data_tmp_.data() + i + 8);
        if (payload > kMaxPayload)
        {
            i++;
            continue;
        }

        /*Incomplete frame: keep the header and wait for the rest of the packet.*/
        if (data_tmp_.size() - i < payload + kExtraLen)
        {
            break;
        }

        const TRData *tr_data = t.Unpack(data_tmp_.data() + i, data_tmp_.size() - i);
        if (tr_data == nullptr)
        {
            i++;
            continue;
        }

        switch (tr_data->pack_id)
        {
        case PACK_GET_DISTANCE:
            Transform(tr_data);
            break;

        case PACK_GET_COE:
            if (tr_data->data.size() >= 18)
            {
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
            }
            break;

        case PACK_VIDEO_SIZE:
            if (tr_data->data.size() >= 4)
            {
                coe_u_ = *(uint16_t *)(tr_data->data.data());
                coe_v_ = *(uint16_t *)(tr_data->data.data() + 2);
                std::cout << "Picture pixels: " << coe_u_ << "*" << coe_v_ << std::endl;
                parameters_ready_.store(true);
            }
            break;

        default:
            break;
        }

        i += t.GetParseDataLen();
    }

    if (i > 0)
    {
        data_tmp_.erase(data_tmp_.begin(), data_tmp_.begin() + i);
    }

    /*Drop a stuck prefix so a bad header cannot grow the buffer without bound.*/
    if (data_tmp_.size() > 65536)
    {
        data_tmp_.erase(data_tmp_.begin(), data_tmp_.end() - 4096);
    }

    return true;
}

bool RTRNet::Transform(const TRData *tr_data)
{
    /*Packet length minus 4-byte timestamp*/
    if (tr_data->data.size() < 6)
    {
        return false;
    }
    int data_amount = tr_data->data.size() - 4;
    int npts = data_amount / 2;
    if (npts < 1 || npts > 4096)
    {
        return false;
    }
    input.resize(npts);
    input.is_dense = false;
    int n = 0;

    for (int i = 0; i + 1 < data_amount && n < npts; i += 2, n++)
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
            /*No return. NaN is omitted by RViz instead of stacking on the origin.*/
            input[n].x = std::numeric_limits<float>::quiet_NaN();
            input[n].y = std::numeric_limits<float>::quiet_NaN();
            input[n].z = std::numeric_limits<float>::quiet_NaN();
            input[n].intensity = 0;
        }
        
    }

    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        pcl::toROSMsg(input, output);
        output.header.frame_id = "lidar_frame";
        output.header.stamp = ros::Time::now();
        frame_ready_.store(true);
    }
    frames_parsed_.fetch_add(1);
    if (!logged_first_frame_)
    {
        logged_first_frame_ = true;
        std::cout << "distance frame, points=" << npts << std::endl;
    }
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
    /*Calibration output is millimeters. RViz uses meters.*/
    input[n].x = static_cast<float>(tmp_x / 1000.0);
    input[n].y = static_cast<float>(tmp_y / 1000.0);
    input[n].z = 0.f;
    input[n].intensity = confidence;
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
