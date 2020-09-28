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
* @file rpn_tvm_pipeline.cpp
* @brief TVM pipeline definition for the RPN model
* @author Luca Fancellu
* @date 2020/09/04
*/

#include "point_pillars_rpn/rpn_tvm_pipeline.h"


PreProcessorRPN::PreProcessorRPN(
  tvm_utility::pipeline::InferenceEngineTVMConfig config,
  cudaStream_t* stream
  ) : stream_(stream)
{
  // allocate input variable
  tvm_utility::pipeline::TVMArrayContainer input_var {
      config.network_inputs[0].second,
      config.tvm_dtype_code,
      config.tvm_dtype_bits,
      config.tvm_dtype_lanes,
      config.tvm_device_type,
      config.tvm_device_id
  };

  net_input_size_byte_ = config.network_inputs[0].second[1] *
                         config.network_inputs[0].second[2] *
                         config.network_inputs[0].second[3] *
                         (config.tvm_dtype_bits / 8);

  rpn_net_input_ = input_var;
}

tvm_utility::pipeline::TVMArrayContainerVector
  PreProcessorRPN::schedule(const std::vector<float*> &net_input)
{
  GPU_CHECK(cudaMemcpyAsync(rpn_net_input_.getArray()->data,
                            net_input[0],
                            net_input_size_byte_,
                            cudaMemcpyDeviceToHost,
                            *stream_));

  return {rpn_net_input_};
}

PostProcessorRPN::PostProcessorRPN(
  tvm_utility::pipeline::InferenceEngineTVMConfig config)
{
  for (uint64_t i = 0; i < config.network_outputs.size(); i++) {
    int64_t output_size = config.network_outputs[i].second[1] *
                          config.network_outputs[i].second[2] *
                          config.network_outputs[i].second[3];
    net_output_size_.push_back(output_size);
  }
}

RPN_Net_Output PostProcessorRPN::schedule(
  const tvm_utility::pipeline::TVMArrayContainerVector &net_output)
{
  RPN_Net_Output output;
  std::vector<float>* ptr_output[3] = {&output.box_output,
                                      &output.cls_output, &output.dir_output};
  for (uint64_t i = 0; i < 3; i++) {

      // get a pointer to the output data
      float *data_ptr = (float *)((uint8_t *)net_output[i].getArray()->data +
                                  net_output[i].getArray()->byte_offset);

      ptr_output[i]->insert(ptr_output[i]->end(), data_ptr, data_ptr +
                                                        net_output_size_[i]);
  }

  return output;
}
