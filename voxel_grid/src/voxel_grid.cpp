/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <voxel_grid/voxel_grid.h>
#include <sys/time.h>
#include <bitset>
#include <ros/console.h>

namespace voxel_grid {
  VoxelGrid::VoxelGrid(unsigned int size_x, unsigned int size_y, unsigned int size_z)
  {
    size_x_ = size_x; 
    size_y_ = size_y; 
    size_z_ = size_z; 

    if(size_z_ > 16){
      ROS_INFO("Error, this implementation can only support up to 16 z values (%d)", size_z_); 
      size_z_ = 16;
    }

    data_ = new uint32_t[size_x_ * size_y_];
    uint32_t unknown_col = ~((uint32_t)0)>>16;
    uint32_t* col = data_;
    for(unsigned int i = 0; i < size_x_ * size_y_; ++i){
      *col = unknown_col;
      ++col;
    }
  }

  void VoxelGrid::resize(unsigned int size_x, unsigned int size_y, unsigned int size_z)
  {
    //if we're not actually changing the size, we can just reset things
    if(size_x == size_x_ && size_y == size_y_ && size_z == size_z_){
      reset();
      return;
    }

    delete[] data_;
    size_x_ = size_x; 
    size_y_ = size_y; 
    size_z_ = size_z; 

    if(size_z_ > 16){
      ROS_INFO("Error, this implementation can only support up to 16 z values (%d)", size_z); 
      size_z_ = 16;
    }

    data_ = new uint32_t[size_x_ * size_y_];
    uint32_t unknown_col = ~((uint32_t)0)>>16;
    uint32_t* col = data_;
    for(unsigned int i = 0; i < size_x_ * size_y_; ++i){
      *col = unknown_col;
      ++col;
    }
  }

  VoxelGrid::~VoxelGrid()
  {
    delete [] data_;
  }

  void VoxelGrid::reset(){
    uint32_t unknown_col = ~((uint32_t)0)>>16;
    uint32_t* col = data_;
    for(unsigned int i = 0; i < size_x_ * size_y_; ++i){
      *col = unknown_col;
      ++col;
    }
  }

  void VoxelGrid::markVoxelLine(double x0, double y0, double z0, double x1, double y1, double z1, unsigned int max_length){
    if(x0 >= size_x_ || y0 >= size_y_ || z0 >= size_z_ || x1>=size_x_ || y1>=size_y_ || z1>=size_z_){
      ROS_DEBUG("Error, line endpoint out of bounds. (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f),  size: (%d, %d, %d)", x0, y0, z0, x1, y1, z1, 
          size_x_, size_y_, size_z_);
      return;
    }

    MarkVoxel mv(data_);
    raytraceLine(mv, x0, y0, z0, x1, y1, z1, max_length);
  }

  void VoxelGrid::clearVoxelLine(double x0, double y0, double z0, double x1, double y1, double z1, unsigned int max_length){
    if(x0 >= size_x_ || y0 >= size_y_ || z0 >= size_z_ || x1>=size_x_ || y1>=size_y_ || z1>=size_z_){
      ROS_DEBUG("Error, line endpoint out of bounds. (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f),  size: (%d, %d, %d)", x0, y0, z0, x1, y1, z1, 
          size_x_, size_y_, size_z_);
      return;
    }

    ClearVoxel cv(data_);
    raytraceLine(cv, x0, y0, z0, x1, y1, z1, max_length);
  }

  void VoxelGrid::clearVoxelLineInMap(double x0, double y0, double z0, double x1, double y1, double z1, unsigned char *map_2d, 
      unsigned int unknown_threshold, unsigned int mark_threshold, unsigned char free_cost, unsigned char unknown_cost, unsigned int max_length){
    costmap = map_2d;
    if(map_2d == NULL){
      clearVoxelLine(x0, y0, z0, x1, y1, z1, max_length);
      return;
    }

    if(x0 >= size_x_ || y0 >= size_y_ || z0 >= size_z_ || x1>=size_x_ || y1>=size_y_ || z1>=size_z_){
      ROS_DEBUG("Error, line endpoint out of bounds. (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f),  size: (%d, %d, %d)", x0, y0, z0, x1, y1, z1, 
          size_x_, size_y_, size_z_);
      return;
    }

    ClearVoxelInMap cvm(data_, costmap, unknown_threshold, mark_threshold, free_cost, unknown_cost);
    raytraceLine(cvm, x0, y0, z0, x1, y1, z1, max_length);
  }

  void VoxelGrid::updateClearingMask(boost::shared_ptr<uint32_t[]>& padded_grid_mask, boost::shared_ptr<bool[]>& updated_columns, double x0, double y0, double z0,
                                     double x1, double y1, double z1, unsigned int max_length, bool raytrace_corner_cases, bool padded_raytracing)
  {
    if (x0 >= size_x_ || y0 >= size_y_ || z0 >= size_z_ || x1 >= size_x_ || y1 >= size_y_ || z1 >= size_z_)
    {
      ROS_DEBUG("Error, line endpoint out of bounds. (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f),  size: (%d, %d, %d)", x0,
                y0, z0, x1, y1, z1, size_x_, size_y_, size_z_);
      return;
    }

    unsigned int padding_size = 0;

    if(padded_raytracing)
      padding_size = 1;

    unsigned int y_offset = size_x_ + (padding_size * 2);

    int dx = int(x1) - int(x0);
    int dy = int(y1) - int(y0);
    int dz = int(z1) - int(z0);

    unsigned int abs_dx = abs(dx);
    unsigned int abs_dy = abs(dy);
    unsigned int abs_dz = abs(dz);

    int offset_dx = sign(dx);
    int offset_dy = sign(dy) * y_offset;
    int offset_dz = sign(dz);

    unsigned int z_mask = ((1 << 16) | 1) << (unsigned int)z0;
    unsigned int offset = (unsigned int)y0 * y_offset + (unsigned int)x0;

    GridOffset grid_off(offset);
    ZOffset z_off(z_mask);

    //we need to chose how much to scale our dominant dimension, based on the maximum length of the line
    double dist = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1) + (z0 - z1) * (z0 - z1));
    double scale = std::min(1.0, max_length / dist);

    //is x dominant
    if (abs_dx >= max(abs_dy, abs_dz))
    {
      int error_y = abs_dx / 2;
      int error_z = abs_dx / 2;

      if (!raytrace_corner_cases)
      {
        if (padded_raytracing)
        {
          bresenham3DOriginal(GridMaskUpdaterX(padded_grid_mask, updated_columns, y_offset), grid_off, grid_off, z_off,
                              abs_dx, abs_dy, abs_dz, error_y, error_z, offset_dx, offset_dy, offset_dz, offset, z_mask,
                              (unsigned int)(scale * abs_dx));
          return;
        }

        bresenham3DOriginal(GridMaskUpdater(padded_grid_mask, updated_columns), grid_off, grid_off, z_off, abs_dx, abs_dy,
                            abs_dz, error_y, error_z, offset_dx, offset_dy, offset_dz, offset, z_mask,
                            (unsigned int)(scale * abs_dx));
        return;
      }

      if (padded_raytracing)
      {
        bresenham3D(GridMaskUpdaterX(padded_grid_mask, updated_columns, y_offset), grid_off, grid_off, z_off, abs_dx,
                    abs_dy, abs_dz, error_y, error_z, offset_dx, offset_dy, offset_dz, offset, z_mask,
                    (unsigned int)(scale * abs_dx));
        return;
      }

      bresenham3D(GridMaskUpdater(padded_grid_mask, updated_columns), grid_off, grid_off, z_off, abs_dx, abs_dy, abs_dz,
                  error_y, error_z, offset_dx, offset_dy, offset_dz, offset, z_mask, (unsigned int)(scale * abs_dx));
      return;
    }

    //y is dominant
    if (abs_dy >= abs_dz)
    {
      int error_x = abs_dy / 2;
      int error_z = abs_dy / 2;

      if (!raytrace_corner_cases)
      {
        if (padded_raytracing)
        {
          bresenham3DOriginal(GridMaskUpdaterY(padded_grid_mask, updated_columns), grid_off, grid_off, z_off, abs_dy,
                              abs_dx, abs_dz, error_x, error_z, offset_dy, offset_dx, offset_dz, offset, z_mask,
                              (unsigned int)(scale * abs_dy));
          return;
        }

        bresenham3DOriginal(GridMaskUpdater(padded_grid_mask, updated_columns), grid_off, grid_off, z_off, abs_dy, abs_dx,
                            abs_dz, error_x, error_z, offset_dy, offset_dx, offset_dz, offset, z_mask,
                            (unsigned int)(scale * abs_dy));
        return;
      }

      if (padded_raytracing)
      {
        bresenham3D(GridMaskUpdaterY(padded_grid_mask, updated_columns), grid_off, grid_off, z_off, abs_dy, abs_dx,
                    abs_dz, error_x, error_z, offset_dy, offset_dx, offset_dz, offset, z_mask,
                    (unsigned int)(scale * abs_dy));
        return;
      }

      bresenham3D(GridMaskUpdater(padded_grid_mask, updated_columns), grid_off, grid_off, z_off, abs_dy, abs_dx, abs_dz,
                  error_x, error_z, offset_dy, offset_dx, offset_dz, offset, z_mask, (unsigned int)(scale * abs_dy));
      return;
    }

    //otherwise, z is dominant
    int error_x = abs_dz / 2;
    int error_y = abs_dz / 2;

    if (!raytrace_corner_cases)
    {
      if (padded_raytracing)
      {
        bresenham3DOriginal(GridMaskUpdaterZ(padded_grid_mask, updated_columns, y_offset), z_off, grid_off, grid_off,
                            abs_dz, abs_dx, abs_dy, error_x, error_y, offset_dz, offset_dx, offset_dy, offset, z_mask,
                            (unsigned int)(scale * abs_dz));
        return;
      }

      bresenham3DOriginal(GridMaskUpdater(padded_grid_mask, updated_columns), z_off, grid_off, grid_off, abs_dz, abs_dx,
                          abs_dy, error_x, error_y, offset_dz, offset_dx, offset_dy, offset, z_mask,
                          (unsigned int)(scale * abs_dz));
      return;
    }

    if (padded_raytracing)
    {
      bresenham3D(GridMaskUpdaterZ(padded_grid_mask, updated_columns, y_offset), z_off, grid_off, grid_off, abs_dz,
                  abs_dx, abs_dy, error_x, error_y, offset_dz, offset_dx, offset_dy, offset, z_mask,
                  (unsigned int)(scale * abs_dz));
      return;
    }

    bresenham3D(GridMaskUpdater(padded_grid_mask, updated_columns), z_off, grid_off, grid_off, abs_dz, abs_dx, abs_dy,
                error_x, error_y, offset_dz, offset_dx, offset_dy, offset, z_mask, (unsigned int)(scale * abs_dz));
    return;
  }

  void VoxelGrid::updateGrid(boost::shared_ptr<uint32_t[]>& padded_grid_mask, bool padded_raytracing)
  {
    uint32_t mask;

    unsigned int padding_size = 0;

    if(padded_raytracing)
      padding_size = 1;

    //if only one of this bits is set, this means we are not clearing it
    //so this means underflow or overflow happened
    uint32_t mask_overflow_test = ((unsigned int)1 << 16) | (unsigned int)1;
    uint32_t mask_underflow_test = ((unsigned int)1 << 31) | ((unsigned int)1 << 15);

    uint32_t mask_lower_bits = ~((uint32_t)0) >> 16;

    unsigned int linear_index = 0;
    unsigned int linear_index_padded = 0;
    unsigned int padding_offset_y = size_x_ + 2 * padding_size;

    for (unsigned int row_index = 0; row_index < size_y_; ++row_index)
    {
      linear_index = row_index * size_x_;
      linear_index_padded = (row_index + padding_size) * padding_offset_y + padding_size;

      for (unsigned int column_index = 0; column_index < size_x_; ++column_index)
      {
        mask = padded_grid_mask[linear_index_padded];

        //if only one of mask_overflow_test bits is set we are not clearing the cell
        //so we had an overflow
        if (((mask & mask_overflow_test) ^ mask_overflow_test) == (unsigned int)1)
        {
          mask &= ~(mask_overflow_test); //remove overflow
        }

        //if only one of mask_underflow_test bits is set we are not clearing the cell
        //so we had an underflow
        if (((mask & mask_underflow_test) ^ mask_underflow_test) == ((unsigned int)1 << 31))
        {
          mask &= ~(mask_underflow_test); //remove underflow
        }

        ///TODO: Remove this after thorough testing
        if ((mask >> 16) ^ (mask & mask_lower_bits))
        {
          ROS_ERROR("Error in voxel grid: Clearing ended up wrong. Seems like there is an bug in the raytracing with thick rays algorithm!");

          std::bitset<32> pretty_mask(mask);
          std::cout << "Clearing Mask:" << pretty_mask << std::endl;

          linear_index++;
          linear_index_padded++;
          continue;
        }

        data_[linear_index] &= ~(mask);

        linear_index++;
        linear_index_padded++;
      }
    }
  }

  void VoxelGrid::updateCostmap(unsigned char* costmap, boost::shared_ptr<bool[]>& updated_columns, unsigned int unknown_clear_threshold,
                   unsigned int marked_clear_threshold, unsigned char free_cost, unsigned char unknown_cost, bool padded_raytracing)
  {
    unsigned int padding_size = 0;

    if(padded_raytracing)
      padding_size = 1;

    unsigned int linear_index = 0;
    unsigned int linear_index_padded = 0;
    unsigned int padding_offset_y = size_x_ + (2 * padding_size);

    for (unsigned int row_index = 0; row_index < size_y_; ++row_index)
    {
      linear_index = row_index * size_x_;
      linear_index_padded = (row_index + padding_size) * padding_offset_y + padding_size;

      for (unsigned int column_index = 0; column_index < size_x_; ++column_index)
      {
        if (!updated_columns[linear_index_padded])
        {
          linear_index++;
          linear_index_padded++;
          continue;
        }

        uint32_t column_occupation = data_[linear_index];

        unsigned int unknown_bits = uint16_t(column_occupation >> 16) ^ uint16_t(column_occupation);
        unsigned int marked_bits = column_occupation >> 16;

        if (bitsBelowThreshold(marked_bits, marked_clear_threshold))
        {
          if (bitsBelowThreshold(unknown_bits, unknown_clear_threshold))
          {
            costmap[linear_index] = free_cost;
          }
          else
          {
            costmap[linear_index] = unknown_cost;
          }
        }

        linear_index++;
        linear_index_padded++;
      }
    }
  }

  VoxelStatus VoxelGrid::getVoxel(unsigned int x, unsigned int y, unsigned int z)
  {
    if(x >= size_x_ || y >= size_y_ || z >= size_z_){
      ROS_DEBUG("Error, voxel out of bounds. (%d, %d, %d)\n", x, y, z);
      return UNKNOWN;
    }
    uint32_t full_mask = ((uint32_t)1<<z<<16) | (1<<z);
    uint32_t result = data_[y * size_x_ + x] & full_mask; 
    unsigned int bits = numBits(result);

    // known marked: 11 = 2 bits, unknown: 01 = 1 bit, known free: 00 = 0 bits
    if(bits < 2){
      if(bits < 1)
        return FREE;

      return UNKNOWN;
    }

    return MARKED;
  }

  VoxelStatus VoxelGrid::getVoxelColumn(unsigned int x, unsigned int y, unsigned int unknown_threshold, unsigned int marked_threshold)
  {
    if(x >= size_x_ || y >= size_y_){
      ROS_DEBUG("Error, voxel out of bounds. (%d, %d)\n", x, y);
      return UNKNOWN;
    }
    
    uint32_t* col = &data_[y * size_x_ + x];

    unsigned int unknown_bits = uint16_t(*col>>16) ^ uint16_t(*col);
    unsigned int marked_bits = *col>>16;

    //check if the number of marked bits qualifies the col as marked
    if(!bitsBelowThreshold(marked_bits, marked_threshold)){
      return MARKED;
    }

    //check if the number of unkown bits qualifies the col as unknown
    if(!bitsBelowThreshold(unknown_bits, unknown_threshold))
      return UNKNOWN;

    return FREE;
  }

  unsigned int VoxelGrid::sizeX(){
    return size_x_;
  }

  unsigned int VoxelGrid::sizeY(){
    return size_y_;
  }

  unsigned int VoxelGrid::sizeZ(){
    return size_z_;
  }

  void VoxelGrid::printVoxelGrid(){
    for(unsigned int z = 0; z < size_z_; z++){
      printf("Layer z = %u:\n",z);
      for(unsigned int y = 0; y < size_y_; y++){
        for(unsigned int x = 0 ; x < size_x_; x++){
          printf((getVoxel(x, y, z)) == voxel_grid::MARKED? "#" : " ");
        }
        printf("|\n");
      } 
    }
  }

  void VoxelGrid::printColumnGrid(){
    printf("Column view:\n");
    for(unsigned int y = 0; y < size_y_; y++){
      for(unsigned int x = 0 ; x < size_x_; x++){
        printf((getVoxelColumn(x, y, 16, 0) == voxel_grid::MARKED)? "#" : " ");
      }
      printf("|\n");
    } 
  }
};
