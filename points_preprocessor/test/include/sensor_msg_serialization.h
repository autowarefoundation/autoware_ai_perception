/*
 * Copyright 2020 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

bool serialize_sensor_msg(const sensor_msgs::PointCloud2::ConstPtr& cloud,
                          const std::string text_file_name,
                          const std::string binary_blob_file_name)
{
  std::ofstream blob_file, text_file;

  blob_file.open (binary_blob_file_name.c_str(),
                  std::ios::out | std::ios::binary | std::ios::trunc);
  if ( !blob_file.is_open() ){
    return false;
  }
  blob_file.write(reinterpret_cast<const char*>(cloud->data.data()),cloud->data.size());
  blob_file.close();

  text_file.open (text_file_name.c_str(), std::ios::out | std::ios::trunc);
  if ( !text_file.is_open() ){
    return false;
  }
  //header
  text_file << cloud->header.seq << std::endl;
  text_file << cloud->header.stamp.sec << std::endl;
  text_file << cloud->header.stamp.nsec << std::endl;
  text_file << cloud->header.frame_id << std::endl;

  //sensor_msg
  text_file << cloud->height << std::endl;
  text_file << cloud->width << std::endl;
  text_file << +(cloud->is_bigendian) << std::endl;
  text_file << cloud->point_step << std::endl;
  text_file << cloud->row_step << std::endl;
  text_file << +(cloud->is_dense);

  //fields
  for (uint i=0; i < cloud->fields.size(); i++)
  {
    text_file << std::endl;
    text_file << cloud->fields[i].name << std::endl;
    text_file << cloud->fields[i].offset << std::endl;
    text_file << +(cloud->fields[i].datatype) << std::endl;
    text_file << cloud->fields[i].count;
  }

  text_file.close();
  return true;
}

bool unserialize_sensor_msg(const sensor_msgs::PointCloud2::Ptr& cloud,
                            const std::string text_file_name,
                            const  std::string binary_blob_file_name)
{
  std::ifstream blob_file, text_file;

  text_file.open (text_file_name.c_str(), std::ios::in);
  if ( !text_file.is_open() ){
    return false;
  }
  //header
  text_file >> cloud->header.seq;
  text_file >> cloud->header.stamp.sec;
  text_file >> cloud->header.stamp.nsec;
  text_file >> cloud->header.frame_id;

  //sensor_msg
  text_file >> cloud->height;
  text_file >> cloud->width;
  int int_buffer = 0;
  text_file >> int_buffer;
  cloud->is_bigendian = int_buffer;
  text_file >> cloud->point_step;
  text_file >> cloud->row_step;
  int_buffer = 0;
  text_file >> int_buffer;
  cloud->is_dense = int_buffer;

  //fields
  uint i = 0;
  while(text_file.peek() != EOF )
  {
    cloud->fields.resize(i+1);
    int int_buffer;
    text_file >> cloud->fields[i].name >> cloud->fields[i].offset \
      >> int_buffer >> cloud->fields[i].count;
    cloud->fields[i].datatype = static_cast<uint8_t>(int_buffer);
    i++;
  }

  text_file.close();

  size_t point_size = cloud->row_step/cloud->width;  // in Byte
  size_t cloud_count = cloud->width*cloud->height;

  blob_file.open (binary_blob_file_name.c_str(), std::ios::in | std::ios::binary);
  if ( !blob_file.is_open() ){
    return false;
  }
  cloud->data.resize(cloud_count*point_size);
  blob_file.read(reinterpret_cast<char*>(cloud->data.data()),cloud_count*point_size);
  blob_file.close();
  return true;
}