/*
 * Copyright 2021 Autoware Foundation. All rights reserved.
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
* @file rpn_tvm_pipeline.h
* @brief TVM pipeline definition for the RPN model
* @author Luca Fancellu
* @date 2020/09/04
*/

#ifndef RPNTVM_PIPELINE_H
#define RPNTVM_PIPELINE_H

#include <tvm_utility/pipeline.h>
#include "lidar_point_pillars/common.h"


typedef struct RPN_Net_Output_ {
  std::vector<float> box_output;
  std::vector<float> cls_output;
  std::vector<float> dir_output;
} RPN_Net_Output;


class PreProcessorRPN
    : public tvm_utility::pipeline::PreProcessor<std::vector<float*>> {
public:

  /**
  * @brief Constructor
  * @param[in] config The configuration of the TVM network
  * @param[in] stream Pointer to the cuda stream to be used during operations
  * @details Constructor for RPN pre-processor stage of the pipeline
  */
  PreProcessorRPN(tvm_utility::pipeline::InferenceEngineTVMConfig config,
  cudaStream_t* stream);

  /**
  * @brief Pipeline stage schedule function
  * @param[in] net_input Array of pointers to cuda memory array
  * @return TVMArrayContainerVector for the TVM inference engine
  * @details Pre-process input data to convert it to TVM structure
  */
  tvm_utility::pipeline::TVMArrayContainerVector
    schedule(const std::vector<float*> &net_input);

private:
  tvm_utility::pipeline::TVMArrayContainer  rpn_net_input_;
  uint64_t                                  net_input_size_byte_;
  cudaStream_t*                             stream_;
};


class PostProcessorRPN
    : public tvm_utility::pipeline::PostProcessor<RPN_Net_Output> {
public:

  /**
  * @brief Constructor
  * @param[in] config The configuration of the TVM network
  * @details Constructor for RPN post-processor stage of the pipeline
  */
  PostProcessorRPN(
      tvm_utility::pipeline::InferenceEngineTVMConfig config);

  /**
  * @brief Pipeline stage schedule function
  * @param[in] net_output TVMArrayContainerVector from the inference engine
  * @return structure with vectors of float containing the network results
  * @details Post-process the output of the inference engine
  */
  RPN_Net_Output
  schedule(const tvm_utility::pipeline::TVMArrayContainerVector &net_output);

private:
  std::vector<uint64_t> net_output_size_;
};

#endif // RPNTVM_PIPELINE_H