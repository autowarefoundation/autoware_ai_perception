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
* @file pfe_tvm_pipeline.h
* @brief TVM pipeline definition for the PointPillars Feature Extractor
* @author Luca Fancellu
* @date 2020/09/04
*/

#ifndef PFE_TVM_PIPELINE_H
#define PFE_TVM_PIPELINE_H

#include <tvm_utility/pipeline.h>
#include "lidar_point_pillars/common.h"


class PreProcessorPFE
    : public tvm_utility::pipeline::PreProcessor<std::vector<float*>> {
public:

  /**
  * @brief Constructor
  * @param[in] config The configuration of the TVM network
  * @param[in] Pointer to the cuda stream to be used during operations
  * @details Constructor for PFE pre-processor stage of the pipeline
  */
  PreProcessorPFE(tvm_utility::pipeline::InferenceEngineTVMConfig config,
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
  std::vector<uint64_t>                          net_input_size_byte_;
  tvm_utility::pipeline::TVMArrayContainerVector pfe_buffers_;
  cudaStream_t*                                  stream_;
};


class PostProcessorPFE
    : public tvm_utility::pipeline::PostProcessor<std::vector<float>> {
public:

  /**
  * @brief Constructor
  * @param[in] config The configuration of the TVM network
  * @details Constructor for PFE post-processor stage of the pipeline
  */
  PostProcessorPFE(
      tvm_utility::pipeline::InferenceEngineTVMConfig config);

  /**
  * @brief Pipeline stage schedule function
  * @param[in] net_output TVMArrayContainerVector from the inference engine
  * @return vector of float containing the network results
  * @details Post-process the output of the inference engine
  */
  std::vector<float>
  schedule(const tvm_utility::pipeline::TVMArrayContainerVector &net_output);

private:
  int64_t net_output_size_;
};

#endif // PFE_TVM_PIPELINE_H