#pragma once
#include <vector>
#include <math.h>
#include <ostream>
#include <memory>
#ifdef DEBUG
  #include "geometry_utils.h"
#else
  #include "../../core/geometry_utils.h"
#endif
#include "raster.h"
#include "extra_util_functions.h"

struct PointD {
  double x;
  double y;
};


class HoughTransform{
public: //typedefs
  using DOUBLE     = double;
  using Cells_type = int;
  using Cov_type   = long long;
  //using size_t     = std::size_t;

  using Array_cells  = std::vector<Cells_type>;
  using Array_cov    = std::vector<Cov_type>;
  using Array2d      = std::vector<Array_cells>;
  using Array_points = std::vector<DiscretePoint2D>;
public: //methods
  HoughTransform(int theta_partition, double delta_ro);

  void transform(const PointD& p);
  std::shared_ptr<Array_cov> spectrum();
  //std::shared_ptr<Array_cells> spectrumRO();

  size_t height() const;
  size_t width() const;

  Array2d& getCells();
  const Array2d& getCells() const;

  Cells_type& getCell(int x, int y);
  const Cells_type& getCell(int x, int y) const;

  DOUBLE delta_ro() const;
  DOUBLE delta_theta () const;

  friend std::ostream& operator<<(std::ostream& ostr, const HoughTransform& h);
  int printOpenGL() const;

  //DiscretePoint2D convert_to_cell(const double x, const double y);
private:
  Cov_type invariant_function(int x);
  void update_size(const std::size_t new_height);
  Array2d _cells;
  RasterGrid<DiscretePoint2D> rg;
};
