#ifndef VOXEL_GRID_GRID_MASK_UPDATER_
#define VOXEL_GRID_GRID_MASK_UPDATER_

namespace voxel_grid
{

class GridMaskUpdater
{
public:
  GridMaskUpdater(boost::shared_ptr<uint32_t[]>& grid_mask, boost::shared_ptr<bool[]>& updated_columns) :
      grid_mask_(grid_mask), updated_columns_(updated_columns)
  {
  }

  inline void operator()(unsigned int offset, unsigned int z_mask)
  {
    grid_mask_[offset] |= z_mask;
    updated_columns_[offset] = true;
  }

protected:
  boost::shared_ptr<uint32_t[]> grid_mask_;
  boost::shared_ptr<bool[]> updated_columns_;
};

class GridMaskUpdaterX
{
public:
  GridMaskUpdaterX(boost::shared_ptr<uint32_t[]>& grid_mask, boost::shared_ptr<bool[]>& updated_columns, unsigned int offset_y) :
      grid_mask_(grid_mask), updated_columns_(updated_columns), offset_y_(offset_y)
  {
  }

  inline void operator()(unsigned int offset, unsigned int z_mask)
  {
    grid_mask_[offset] |= z_mask;
    updated_columns_[offset] = true;

    //padding z
    grid_mask_[offset] |= z_mask << 1; //overflow can happen, is treated later
    grid_mask_[offset] |= z_mask >> 1; //underflow can happen, is treated later

    //padding y, data is padded so no under / overflow can happen
    grid_mask_[offset + offset_y_] |= z_mask;
    updated_columns_[offset + offset_y_] = true;
    grid_mask_[offset - offset_y_] |= z_mask;
    updated_columns_[offset - offset_y_] = true;
  }

protected:
  boost::shared_ptr<uint32_t[]> grid_mask_;
  boost::shared_ptr<bool[]> updated_columns_;
  unsigned int offset_y_;
};

class GridMaskUpdaterY
{
public:
  GridMaskUpdaterY(boost::shared_ptr<uint32_t[]>& grid_mask, boost::shared_ptr<bool[]>& updated_columns) :
      grid_mask_(grid_mask), updated_columns_(updated_columns)
  {
  }

  inline void operator()(unsigned int offset, unsigned int z_mask)
  {
    grid_mask_[offset] |= z_mask;
    updated_columns_[offset] = true;

    //padding z
    grid_mask_[offset] |= z_mask << 1; //overflow can happen, is treated later
    grid_mask_[offset] |= z_mask >> 1; //underflow can happen, is treated later

    //padding x, data is padded so no under / overflow can happen
    grid_mask_[offset + 1] |= z_mask;
    updated_columns_[offset + 1] = true;
    grid_mask_[offset - 1] |= z_mask;
    updated_columns_[offset - 1] = true;
  }

protected:
  boost::shared_ptr<uint32_t[]>grid_mask_;
  boost::shared_ptr<bool[]> updated_columns_;
};

class GridMaskUpdaterZ
{
public:
  GridMaskUpdaterZ(boost::shared_ptr<uint32_t[]>& grid_mask, boost::shared_ptr<bool[]>& updated_columns, unsigned int offset_y) :
      grid_mask_(grid_mask), updated_columns_(updated_columns), offset_y_(offset_y)
  {
  }

  inline void operator()(unsigned int offset, unsigned int z_mask)
  {
    grid_mask_[offset] |= z_mask;
    updated_columns_[offset] = true;

    //padding x, data is padded so no under / overflow should happen
    grid_mask_[offset + 1] |= z_mask;
    updated_columns_[offset + 1] = true;
    grid_mask_[offset - 1] |= z_mask;
    updated_columns_[offset - 1] = true;

    //padding y, data is padded so no under / overflow should happen
    grid_mask_[offset + offset_y_] |= z_mask;
    updated_columns_[offset + offset_y_] = true;
    grid_mask_[offset - offset_y_] |= z_mask;
    updated_columns_[offset - offset_y_] = true;
  }

protected:
  boost::shared_ptr<uint32_t[]> grid_mask_;
  boost::shared_ptr<bool[]> updated_columns_;
  unsigned int offset_y_;
};

}
;

#endif

