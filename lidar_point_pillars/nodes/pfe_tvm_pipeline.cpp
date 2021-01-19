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

/**
* @file pfe_tvm_pipeline.cpp
* @brief TVM pipeline definition for the RPN model
* @author Luca Fancellu
* @date 2020/09/04
*/

#include "point_pillars_pfe/pfe_tvm_pipeline.h"


PreProcessorPFE::PreProcessorPFE(
  tvm_utility::pipeline::InferenceEngineTVMConfig config,
  cudaStream_t* stream
  ) : stream_(stream)
{
  // allocate input variables
  uint64_t net_input_size;
  for (uint64_t i = 0; i < config.network_inputs.size(); i++) {
    net_input_size = 1;
    for (uint64_t j = 0; j < config.network_inputs[i].second.size(); j++)
    {
      net_input_size *= config.network_inputs[i].second[j];
    }
    net_input_size *= (config.tvm_dtype_bits / 8);
    tvm_utility::pipeline::TVMArrayContainer input_var {
      config.network_inputs[i].second,
      config.tvm_dtype_code,
      config.tvm_dtype_bits,
      config.tvm_dtype_lanes,
      config.tvm_device_type,
      config.tvm_device_id
    };

    pfe_buffers_.push_back(input_var);
    net_input_size_byte_.push_back(net_input_size);
  }
}

tvm_utility::pipeline::TVMArrayContainerVector
  PreProcessorPFE::schedule(const std::vector<float*> &net_input)
{
  for (uint64_t i = 0; i < 8; i++) {

    GPU_CHECK(cudaMemcpyAsync(pfe_buffers_[i].getArray()->data,
                              net_input[i],
                              net_input_size_byte_[i],
                              cudaMemcpyDeviceToHost,
                              *stream_));
  }

  return pfe_buffers_;
}

PostProcessorPFE::PostProcessorPFE(
  tvm_utility::pipeline::InferenceEngineTVMConfig config)
{
  net_output_size_ = config.network_outputs[0].second[1] *
                     config.network_outputs[0].second[2] *
                     config.network_outputs[0].second[3];
}

std::vector<float> PostProcessorPFE::schedule(
  const tvm_utility::pipeline::TVMArrayContainerVector &net_output)
{
  std::vector<float> output;

  // get a pointer to the output data
  float *data_ptr = (float *)((uint8_t *)net_output[0].getArray()->data +
                              net_output[0].getArray()->byte_offset);

  output.insert(output.end(), data_ptr, data_ptr + net_output_size_);

  return output;
}
