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

#include <algorithm>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <vector>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

bool equals(const sensor_msgs::PointCloud2& lhs, const sensor_msgs::PointCloud2& rhs)
{
  //header
  bool are_header_equals = (lhs.header.seq        == rhs.header.seq) &&
                           (lhs.header.stamp.sec  == rhs.header.stamp.sec) &&
                           (lhs.header.stamp.nsec == rhs.header.stamp.nsec) &&
                           (lhs.header.frame_id   == rhs.header.frame_id);
  if (!are_header_equals)
  {
    return false;
  }

  //sensor_msg
  bool are_parameters_equals = (lhs.height       == rhs.height) &&
                               (lhs.width        == rhs.width) &&
                               (lhs.is_bigendian == rhs.is_bigendian) &&
                               (lhs.point_step   == rhs.point_step) &&
                               (lhs.row_step     == rhs.row_step) &&
                               (lhs.is_dense     == rhs.is_dense);
  if (!are_parameters_equals)
  {
    return false;
  }

  //fields
  bool are_all_fields_equals = true;
  if (lhs.fields.size() == rhs.fields.size()){
    for (uint i=0; i < lhs.fields.size(); i++)
    {
      if (!((lhs.fields[i].name     == rhs.fields[i].name) &&
            (lhs.fields[i].offset   == rhs.fields[i].offset) &&
            (lhs.fields[i].datatype == rhs.fields[i].datatype) &&
            (lhs.fields[i].count    == rhs.fields[i].count)))
      {
        are_all_fields_equals = false;
        break;
      }
    }
  }
  else
  {
    are_all_fields_equals = false;
  }
  if (!are_all_fields_equals)
  {
    return false;
  }

  //binary blob
  bool is_data_equal = (lhs.data.size() == rhs.data.size()) &&
                       (memcmp(lhs.data.data(),rhs.data.data(),lhs.data.size())) == 0;
  if (!is_data_equal)
  {
    return false;
  }

  return true;
}

std::string decode_chunk(const sensor_msgs::PointCloud2& sensor_msg, uint index)
{
  std::stringstream output_buffer;

  size_t point_size = sensor_msg.row_step/sensor_msg.width;  // in Byte

  const uint8_t* chunk_start_ptr = reinterpret_cast<const uint8_t*>(sensor_msg.data.data()) + (index*point_size);

  output_buffer << "chunk#" << index << " : ";

  for (uint i = 0; i < sensor_msg.fields.size(); i++)
  {
    uint offset = sensor_msg.fields[i].offset;  // In Byte from the chunk's start
    for (uint current_count = 0; current_count < sensor_msg.fields[i].count ; current_count++)
    {
        uint read_size;  // In byte
        switch(sensor_msg.fields[i].datatype)
        {
            case sensor_msgs::PointField::INT8 :
            read_size = 1;
            int8_t val8;
            memcpy(&val8, chunk_start_ptr+offset+(current_count*read_size), read_size);
            output_buffer << sensor_msg.fields[i].name << " : " << val8 << "; ";
            break;
            case sensor_msgs::PointField::UINT8 :
            read_size = 1;
            uint8_t valu8;
            memcpy(&valu8, chunk_start_ptr+offset+(current_count*read_size), read_size);
            output_buffer << sensor_msg.fields[i].name << " : " << valu8 << "; ";
            break;
            case sensor_msgs::PointField::INT16 :
            read_size = 2;
            int16_t val16;
            memcpy(&val16, chunk_start_ptr+offset+(current_count*read_size), read_size);
            output_buffer << sensor_msg.fields[i].name << " : " << val16 << "; ";
            break;
            case sensor_msgs::PointField::UINT16 :
            read_size = 2;
            int16_t valu16;
            memcpy(&valu16, chunk_start_ptr+offset+(current_count*read_size), read_size);
            output_buffer << sensor_msg.fields[i].name << " : " << valu16 << "; ";
            break;
            case sensor_msgs::PointField::INT32 :
            read_size = 4;
            int32_t val32;
            memcpy(&val32, chunk_start_ptr+offset+(current_count*read_size), read_size);
            output_buffer << sensor_msg.fields[i].name << " : " << val32 << "; ";
            break;
            case sensor_msgs::PointField::UINT32 :
            read_size = 4;
            uint32_t valu32;
            memcpy(&valu32, chunk_start_ptr+offset+(current_count*read_size), read_size);
            output_buffer << sensor_msg.fields[i].name << " : " << valu32 << "; ";
            break;
            case sensor_msgs::PointField::FLOAT32 :
            read_size = 4;
            float valf32;
            memcpy(&valf32, chunk_start_ptr+offset+(current_count*read_size), read_size);
            output_buffer << sensor_msg.fields[i].name << " : " << valf32 << "; ";
            break;
            case sensor_msgs::PointField::FLOAT64 :
            read_size = 8;
            double valf64;
            memcpy(&valf64, chunk_start_ptr+offset+(current_count*read_size), read_size);
            output_buffer << sensor_msg.fields[i].name << " : " << valf64 << "; ";
            break;
        }
    }
  }
  return output_buffer.str();
}

// Get a hex string of the first mismatch between two uint8_t vectors. They are considered containing data aligned on chunk_size
std::string get_first_mismatch(const std::vector<uint8_t>& expected,
                                const std::vector<uint8_t>& tested,
                                uint chunk_size, uint* out_index = nullptr)
{
  uint min_size = std::min(expected.size(),tested.size());
  for (uint i = 0; i < min_size; i++)
  {
    if (expected[i]!=tested[i])
    {
      // We have a mismatch. We make the string for up to chunk_size bytes
      uint chunk_id = i / chunk_size;  // Use the rounding down
      uint chunk_start_index = chunk_id * chunk_size;
      std::stringstream output_buffer;
      output_buffer << "first mismatch at index " << i << " (chunk #" << chunk_id << ") : ";
      for (uint j = chunk_start_index; j < std::min(chunk_start_index+chunk_size, min_size); j++)
      {
        output_buffer << std::hex << static_cast<int>(expected[j]);
      }
      output_buffer << " | ";
      for (uint j = chunk_start_index; j < std::min(chunk_start_index+chunk_size, min_size); j++)
      {
        output_buffer << std::hex << static_cast<int>(tested[j]);
      }
      if (out_index != nullptr)
      {
        *out_index = chunk_id;
      }
      return output_buffer.str();
    }
  }
  if (out_index != nullptr)
  {
    *out_index = ~0;
  }
  return "";
}

std::string find_mismatches(const sensor_msgs::PointCloud2& expected,
                            const sensor_msgs::PointCloud2& tested,
                            bool locate_first_data_disparity = false)
{
  std::stringstream output_buffer;

  //header
  if (expected.header.seq != tested.header.seq)
  {
    output_buffer << "header.seq are differents : expected : " << expected.header.seq
      << "; got : " << tested.header.seq << std::endl;
  }
  if (expected.header.stamp.sec != tested.header.stamp.sec)
  {
    output_buffer << "header.stamp.sec are differents : expected : " << expected.header.stamp.sec
      << "; got : " << tested.header.stamp.sec << std::endl;
  }
  if (expected.header.stamp.nsec != tested.header.stamp.nsec)
  {
    output_buffer << "header.stamp.nsec are differents : expected : " << expected.header.stamp.nsec
      << "; got : " << tested.header.stamp.nsec << std::endl;
  }
  if (expected.header.frame_id != tested.header.frame_id)
  {
    output_buffer << "header.frame_id are differents : expected : " << expected.header.frame_id
      << "; got : " << tested.header.frame_id << std::endl;
  }

  //sensor_msg
  if (expected.height != tested.height)
  {
    output_buffer << "height are differents : expected : " << expected.height
      << "; got : " << tested.height << std::endl;
  }
  if (expected.width != tested.width)
  {
    output_buffer << "width are differents : expected : " << expected.width <<
      "; got : " << tested.width << std::endl;
  }
  if (expected.is_bigendian != tested.is_bigendian)
  {
    output_buffer << "is_bigendian are differents : expected : " << expected.is_bigendian
      << "; got : " << tested.is_bigendian << std::endl;
  }
  if (expected.point_step != tested.point_step)
  {
    output_buffer << "point_step are differents : expected : " << expected.point_step
      << "; got : " << tested.point_step << std::endl;
  }
  if (expected.row_step != tested.row_step)
  {
    output_buffer << "row_step are differents : expected : " << expected.row_step
      << "; got : " << tested.row_step << std::endl;
  }
  if (expected.is_dense != tested.is_dense)
  {
    output_buffer << "is_dense are differents : expected : " << expected.is_dense
      << "; got : " << tested.is_dense << std::endl;
  }

  //fields
  if (expected.fields.size() != tested.fields.size())
  {
    output_buffer << "field sizes are differents : expected : " << expected.fields.size()
      << "; got : " << tested.fields.size() << std::endl;
  }

  uint min_field_size = std::min(expected.fields.size(),tested.fields.size());
  for (uint i=0; i < min_field_size; i++)
  {
    if (expected.fields[i].name != tested.fields[i].name)
    {
      output_buffer << "fields[" << i << "].name are differents : expected : "
        << expected.fields[i].name << "; got : " << tested.fields[i].name << std::endl;
    }
    if (expected.fields[i].offset != tested.fields[i].offset)
    {
      output_buffer << "fields[" << i << "].offset are differents : expected : "
        << expected.fields[i].offset << "; got : " << tested.fields[i].offset << std::endl;
    }
    if (expected.fields[i].datatype != tested.fields[i].datatype)
    {
      output_buffer << "fields[" << i << "].datatype are differents : expected : "
        << expected.fields[i].datatype << "; got : " << tested.fields[i].datatype << std::endl;
    }
    if (expected.fields[i].count != tested.fields[i].count)
    {
      output_buffer << "fields[" << i << "].count are differents : expected : "
        << expected.fields[i].count << "; got : " << tested.fields[i].count << std::endl;
    }
  }

  //binary blob
  if(expected.data.size() != tested.data.size())
  {
    output_buffer << "data sizes are differents : expected : " << expected.data.size()
      << "; got : " << tested.data.size() << std::endl;
  }
  if (memcmp(expected.data.data(),tested.data.data(),expected.data.size()) != 0)
  {
    output_buffer << "data are differents";
    if (locate_first_data_disparity)
    {
      uint point_size = expected.row_step/expected.width;  // In Byte
      uint mismatch_index = 0;
      output_buffer << " : " << get_first_mismatch(expected.data, tested.data, point_size, &mismatch_index);
      output_buffer << std::endl << "expected : " << decode_chunk(expected, mismatch_index);
      output_buffer << std::endl << "     got : " << decode_chunk(tested, mismatch_index);
    }
    output_buffer << std::endl;
  }

  return output_buffer.str();
}



bool count_data_mismatch(const sensor_msgs::PointCloud2& expected_ground,
                          const sensor_msgs::PointCloud2& expected_no_ground,
                          const sensor_msgs::PointCloud2& tested_ground,
                          const sensor_msgs::PointCloud2& tested_no_ground,
                          uint* out_false_ground_count,
                          uint* out_false_no_ground_count)
{
  uint point_size = expected_no_ground.row_step/expected_no_ground.width;  // In Byte
  if (((expected_ground.data.size()%point_size) != 0) ||
      ((expected_no_ground.data.size()%point_size) != 0) ||
      ((tested_ground.data.size()%point_size) != 0) ||
      ((tested_no_ground.data.size()%point_size) != 0))
  {
    return 0;
  }

  uint expected_ground_point_count = expected_ground.data.size()/point_size;
  uint expected_no_ground_point_count = expected_no_ground.data.size()/point_size;
  uint tested_ground_point_count = tested_ground.data.size()/point_size;
  uint tested_no_ground_point_count = tested_no_ground.data.size()/point_size;

  uint tested_no_ground_index = 0;
  uint expected_no_ground_index = 0;

  *out_false_ground_count = 0;
  *out_false_no_ground_count = 0;

  bool all_mismatches_are_legit = true;

  while ((tested_no_ground_index < tested_no_ground_point_count) &&
         (expected_no_ground_index < expected_no_ground_point_count))
  {
    // Progress on the no_ground point until we hit a mismatch
    bool is_equal;
    do{
      is_equal = memcmp(tested_no_ground.data.data() + tested_no_ground_index*point_size,
                        expected_no_ground.data.data() + (expected_no_ground_index)*point_size,
                        point_size) == 0;
      tested_no_ground_index++;
      expected_no_ground_index++;
    }while (is_equal && (tested_no_ground_index < tested_no_ground_point_count) &&
         (expected_no_ground_index < expected_no_ground_point_count));

    if (!is_equal)
    {
      // ---- Test for a false no_ground (a true ground point in the no_ground vector) ----
      bool is_false_no_ground = false;
      for (uint i=0; i<expected_ground_point_count; i++)
      {
        if (0 == memcmp(tested_no_ground.data.data() + (tested_no_ground_index-1)*point_size,
                        expected_ground.data.data() + i*point_size,point_size))
        {
          is_false_no_ground = true;
          break;
        }
      }
      // ---- Test for a false ground (a true no_ground point in the ground vector) ----
      bool is_false_ground = false;
      for (uint i=0; i<tested_ground_point_count; i++)
      {
        if (0 == memcmp(expected_no_ground.data.data() + (expected_no_ground_index-1)*point_size,
                        tested_ground.data.data() + i*point_size,point_size))
        {
          is_false_ground = true;
          break;
        }
      }

      // ---- Now we update the two indexes ----
      if (is_false_no_ground)
      {
        if (is_false_ground)
        {
          //Two values are swapped. We don't change any offset.
          *out_false_ground_count += 1;
          *out_false_no_ground_count += 1;
        }
        else
        {
          //revert the expected index but keep the tested index the same
          expected_no_ground_index--;
          *out_false_no_ground_count += 1;
        }
      }
      else
      {
        if (is_false_ground)
        {
          //revert the tested index but keep the expected index the same
          tested_no_ground_index--;
          *out_false_ground_count += 1;
        }
        else
        {
          //Nothing ? We have a problem... It means some point is either lost or a new one created
          all_mismatches_are_legit = false;
        }
      }
    }
  }
  return all_mismatches_are_legit;
}