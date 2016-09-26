#include "hough_scan_matcher.h"

#include <math.h>
#include <random>
#include <memory>
#include <vector>
#include <set>
#include <functional>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include "walker.h"

#include "extra_util_functions.h"
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>

using namespace std;
using HT = HoughTransform;

struct PointMax{
  long long value;
  int sigma;
  long ind;
};

using _set_ = set<PointMax, function<bool(PointMax,PointMax)>>;
template <typename T>
inline shared_ptr<_set_> find_maxs(const vector<T>& array) {

  shared_ptr<_set_> out(new _set_([](PointMax p1, PointMax p2)-> bool {
                                    return p2.sigma <= p1.sigma;
                                  }));
  const int division_size = 2;
  const int max_iter = array.size()/division_size;
  for (long  i = -(array.size()/division_size); i < (long) array.size()/division_size; i++) {
    int j = 0;
    while ((get<T>(array,i-(j+1)) <= get<T>(array,i)) && (get<T>(array,i+(j+1)) <= get<T>(array,i))
           && j < max_iter) {j++;}
    if (3 < j)
      out->insert({get<T>(array,i),j,i});
  }
  return out;
}

int HoughScanMatcher::count = 0;

bool isNear(const ScanPoint& p1, const ScanPoint& p2, bool new_iter) {
  static double avg_distance = 0;
  static int count = 0;
  static int droped = 0;
  if (new_iter) {
    avg_distance = 0;
    count = 0;
    droped = 0;
  }
  if (3 < droped) {
    droped = 0;
    return false;
  }
  double distance = abs(p1.range - p2.range) + abs(p1.angle - p2.angle);
  if (avg_distance == 0) {
    avg_distance = distance;
    count++;
    return false;
  }
  if (distance < 3*avg_distance/(float)count) {
    avg_distance += distance;
    count++;
    droped++;
    return true;
  }
  return false;
}

int generate_map(const RobotState& init_pose, const TransformedLaserScan& scan,
                 const GridMap& map, int window_scan, int window_map,
    /*out vars*/ HoughTransform& local_map_HT, HoughTransform& scan_map_HT) {
  DiscretePoint2D robot_on_map = map.world_to_cell(init_pose.x,init_pose.y);
  int x_max = 0, y_max = 0, ii = 0, scan_count = 0;
  glutSetWindow(window_scan);
  glClearColor (1.0, 1.0, 1.0, 0.0);
  glClear (GL_COLOR_BUFFER_BIT);
  for (auto& sp : scan.points) {
    if (ii%3 == 0) {
      double x = sp.range * cos(sp.angle);
      double y = sp.range * sin(sp.angle);
      PointD dsp{x,y};
      printRect(150+dsp.y,150+dsp.y+1,150+dsp.x,150+dsp.x+1,0.0);
      scan_map_HT.transform(dsp);
      if (x_max < dsp.x) {
        x_max = dsp.x;
      }
      if (y_max < dsp.y) {
        y_max = dsp.y;
      }
      scan_count++;
    }
    ii++;
  }
  glFlush();
  //cout << "точек из скана: " << scan_count << endl;
  //cout << "(" << x_max << ";" << y_max << ")" << endl;
  float expand_koef = 1.2;
  float min_sensity = 0.7;
  int map_count = 0;
  glutSetWindow(window_map);
  glClearColor (1.0, 1.0, 1.0, 0.0);
  glClear (GL_COLOR_BUFFER_BIT);
  for (int x = -x_max*expand_koef; x <= x_max*expand_koef; x++) {
    for (int y = -y_max*expand_koef; y <= y_max*expand_koef; y++) {
      if (map.cell_value({x+robot_on_map.x, y+robot_on_map.y}) > min_sensity) {
        local_map_HT.transform({x, y});
        printRect(150+y,150+y+1,150+x,150+x+1,0.0);
        map_count++;
      }
    }
  }
  glFlush();
  //cout << "точек на локальной карте: " << map_count << endl << endl;
  return 0;
  glutMainLoop();
}

double HoughScanMatcher::process_scan(const RobotState &init_pose,
                                      const TransformedLaserScan &scan,
                                      const GridMap &map,
                                      RobotState &pose_delta) {

  shared_ptr<HoughTransform> scan_HT{new HoughTransform{720, 0.1}};

  //HoughTransform local_map_HT(720,0.1);
  //HoughTransform mapHT(scan.points.size(), map.scale()/2);
  bool new_iter = true;
  for (size_t i = 0; i < scan.points.size(); i++) {
    if (!scan.points[i].is_occupied)
      continue;
    /*if (isNear(scan.points[i-1],scan.points[i], new_iter)) {
      new_iter = false;
      continue;
    }
    new_iter = false;*/
    //double x = sp.range * cos(init_pose.theta + sp.angle);
    //double y = sp.range * sin(init_pose.theta + sp.angle);

    double x = scan.points[i].range * cos(init_pose.theta + scan.points[i].angle);
    double y = scan.points[i].range * sin(init_pose.theta + scan.points[i].angle);
    scan_HT->transform({x,y});
    //cout << "transform happens\n";
    //i++;
  }
  //cout << endl << endl;

  //scan_HT.printOpenGL();

  //cout << scan_HT << endl;
  /*generate_map(init_pose,scan,map,window_scan,window_local_map,local_map_HT,scan_HT);

  for (const auto& psp : prev_scan.points) {
    double x = psp.range * cos(init_pose.theta + psp.angle);
    double y = psp.range * sin(init_pose.theta + psp.angle);
    //double x = psp.range * cos(psp.angle);
    //double y = psp.range * sin(psp.angle);
    mapHT.transform(map.world_to_cell(x,y));
  }*/
  /*vector<vector<GridMap::Cell>> local_map = map.cells();
  for (size_t i = 0; i < local_map.size(); i++) {
    for (size_t j = 0; j < local_map[0].size(); j++) {
      if (min_sensity < local_map[i][j]->value())
        mapHT.transform({(int)(j-map.map_center_x()),(int)(i-map.map_center_y())});
    }
  }*/
  shared_ptr<HT::Array_cov> scanSpectr = scan_HT->spectrum();
  //shared_ptr<HT::Array_cov> mapSpectr  = local_map_HT.spectrum();


  if(count == 0) {
    prev_spectr = scanSpectr;
    prev_HT = scan_HT;
    count++;
    pose_delta.theta = 0;
    return 0;
  }

  HT::Array_cov covariance(scanSpectr->size());
  for (size_t i = 0; i < covariance.size(); i++) {
    covariance[i] = scalar_mul(*prev_spectr,*scanSpectr,i,0);
  }
  /*cout << "cov[-3] = " << get(covariance,-3) << endl;
  cout << "cov[-2] = " << get(covariance,-2) << endl;
  cout << "cov[-1] = " << get(covariance,-1) << endl;
  cout << "cov[ 0] = " << get(covariance, 0) << endl;
  cout << "cov[ 1] = " << get(covariance, 1) << endl;*/

  print(*prev_spectr, "prev_spectr");
  print(*scanSpectr, "curr_spectr");
  print(covariance,"cov");

  shared_ptr<_set_> better_angle_array = find_maxs(covariance);
  auto iterator = better_angle_array->begin();
  int offset = iterator->ind;

  /*HT::Array_cov covarianceRO(scanHT.height());
  for (size_t i = 0; i < covarianceRO.size(); i++) {
    covarianceRO[i] = scalar_mul(mapHT.getCells()[offset],scanHT.getCells()[0],i,0);
  }
  shared_ptr<_set_> better_range_array = find_maxs(covarianceRO);*/
  cout << "offset found: " << offset << ", with value " << iterator->value << endl;
  pose_delta.theta = offset*scan_HT->delta_theta();
  cout << pose_delta.theta << endl;
  pose_delta.x = 0;
  pose_delta.y = 0;

  iterator++;
  /*if (iterator != better_angle_array->end()) {
    cout << "second offset found: " << iterator->ind << ", with value " << iterator->value << endl;
  }*/

  /*RobotState posiblePose {init_pose.x, init_pose.y, init_pose.theta + pose_delta.theta};
  double cost = cost_estimator()->estimate_scan_cost(posiblePose,scan,map,std::numeric_limits<double>::max());
  iterator++;
  if(iterator != better_angle_array->end()) {
    offset = iterator->ind;
    double cost2 = cost_estimator()->estimate_scan_cost(posiblePose,scan,map,std::numeric_limits<double>::max());
    if (cost2 < cost) {
      pose_delta.theta = -offset*scan_HT.delta_theta();
      cout << "а второе-то лучше пошло!" << endl;
    }
  }
  for (double theta = -0.2; theta < 0.2; theta += 0.05) {
    RobotState posiblePose2 {init_pose.x, init_pose.y, init_pose.theta + pose_delta.theta + theta};
    double cost3 = cost_estimator()->estimate_scan_cost(posiblePose2,scan,map,std::numeric_limits<double>::max());
    if (cost3 < cost) {
      pose_delta.theta += theta;
      cout << "что-то лучше получилось!" << endl;
    }
  }*/
  /*if(count == 2) {
    print(*scanSpectr, "scan");
    print(*prev_spectr,"prev");
    print(covariance,"cov");
  }*/
  prev_spectr = scanSpectr;
  HoughScanMatcher::count++;

  //<!-----------------!>
  //to find Δx

  //to get columns from a curr_specrt & a prev_spectr which are the same one from the correlation function
  //to find the offset of the most correlation of these vectors
  //the output Δρ=ρ_num*offset will built the equation: Δρ=cos(θ)*Δx+sin(θ)*Δy | θ=0
  //whitch means that Δx = ρ_num*offset;

  HT::Array_cov covarianceRO(scan_HT->height());
  print(get(prev_HT->getCells(),offset+1),"prev_ro");
  print(get(scan_HT->getCells(),1),"cur_ro");
  for (size_t i = 0; i < covarianceRO.size(); i++) {
    covarianceRO[i] = scalar_mul(get(prev_HT->getCells(),offset+180),get(scan_HT->getCells(),180),i,0);
  }
  print(covarianceRO, "covRO");
  auto better_rho_array = find_maxs(covarianceRO);
  //pose_delta.y = better_rho_array->begin()->ind * scan_HT->delta_ro();

  //double min_cost = std::numeric_limits<double>::max();
  //for (double theta = -0.5; theta < 0.5; theta += 0.001) {
  //  RobotState posiblePose2 {init_pose.x, init_pose.y, init_pose.theta + theta};
  //  double current_cost = cost_estimator()->estimate_scan_cost(posiblePose2,scan,map,std::numeric_limits<double>::max());
  //  if (current_cost < min_cost) {
  //    min_cost = current_cost;
  //    pose_delta.theta = theta;
  //  }
  //}
  //return 0;
}

inline double scalar_mul(const std::vector<ScanPoint>& v1,
                    const std::vector<ScanPoint>& v2,
                    long ind1 = 0, long ind2 = 0) {
  size_t v1size = v1.size();
  size_t v2size = v2.size();
  if (v1size != v2size)
    v1size = std::min(v1size,v2size);
  double out = 0;
  for (size_t i = 0; i < v1size; i++)
    out += get(v1,(long)(ind1+i)).range*get(v2,(long)(ind2+i)).range;
  return out;
}

//double HoughScanMatcher::process_scan(const RobotState &init_pose,
//                                      const TransformedLaserScan &scan,
//                                      const GridMap &map,
//                                      RobotState &pose_delta) {
//  cout << "начался process scan\n";
//  vector<double> covariance(scan.points.size(),0);
//  cout << "создали массив для ковариации размером" << scan.points.size() << endl;
//  cout << "начали считать ковариации:\n";
//  for (long i = 0; i < covariance.size(); i++) {
//    covariance[i] = scalar_mul(scan.points,prev_scan.points,0,i);
//    //cout << "covariance[" << i << "] = " << covariance[i] << endl;
//  }
//  cout << "посчитали значение ковариаций\n";
//  auto sorted = find_maxs(covariance);
//  cout << "отсортировали значения\n";
//  long offset = sorted->begin()->ind;
//  cout << "длина всего массива равна " << covariance.size() << ", а смещение получилось " << offset <<endl;
//  prev_scan = scan;
//  return 0;
//}
DiscretePoint2D rotate(double fi, DiscretePoint2D pt) {
  auto res_pt = pt;
  res_pt.x = cos(fi)*pt.x - sin(fi)*pt.y;
  res_pt.y = sin(fi)*pt.x + cos(fi)*pt.y;
  return res_pt;
}

template<class T>
inline bool InvertMatrix(const boost::numeric::ublas::matrix<T>& input, boost::numeric::ublas::matrix<T>& inverse)
{
  typedef boost::numeric::ublas::permutation_matrix<std::size_t> pmatrix;

  // create a working copy of the input
  boost::numeric::ublas::matrix<T> A(input);

  // create a permutation matrix for the LU-factorization
  pmatrix pm(A.size1());

  // perform LU-factorization
  int res = lu_factorize(A, pm);
  if (res != 0)
    return false;

  // create identity matrix of "inverse"
  inverse.assign(boost::numeric::ublas::identity_matrix<T> (A.size1()));

  // backsubstitute to get the inverse
  lu_substitute(A, pm, inverse);

  return true;
}

//#ifdef DEBUG
//  int main(int argc, char **argv) {
//#else
//  int main1(int argc, char **argv) {
//#endif
//  glutInit(&argc, argv);
//
//  const int theta_partition = 720;
//  const double delta_ro = 0.1;
//  const int min_xy_coord = 0;
//  const int max_xy_coord = 149;
//  const int points_count = 5000;
//  const int top = 100;
//  const int bot = 20;
//  const int left = 40;
//  const int right = 80;
//  const int x_center_sub = (right - left)/2;
//  const int y_center_sub = (top - bot)/2;
//
//  const int delta_x = 0;
//  const int delta_y = 0;
//  const int top_origin = min(top + delta_y, max_xy_coord);
//  const int bot_origin = max(bot - delta_y, 0);
//  const int left_origin = max(left - delta_x,0);
//  const int right_origin = min(right + delta_x, max_xy_coord);
//  const int x_center_sub_origin = (right_origin - left_origin)/2;
//  const int y_center_sub_origin = (top_origin - bot_origin)/2;
//
//  if (left < min_xy_coord || left > max_xy_coord ||
//      right < min_xy_coord || right > max_xy_coord ||
//      left > right ||
//      bot < min_xy_coord || bot > max_xy_coord ||
//      top < min_xy_coord || top > max_xy_coord ||
//      bot > top ) {
//    cout << "введены не те границы подквадра\n";
//    return 0;
//  }
//
//  HoughTransform h_origin(theta_partition,delta_ro);
//  HoughTransform h_sub(theta_partition,delta_ro);
//  HoughTransform h_sub_origin(theta_partition,delta_ro);
//
//  Walker w(min_xy_coord,max_xy_coord,min_xy_coord,max_xy_coord);
//  HT::Array_points walk = w.go(3,0,points_count,Walker::DOWN);
//  //HT::Array_points walk = {{45,25},{48,80},{65,27},{65,65}};
//  HoughTransform::Array2d points(max_xy_coord - min_xy_coord + 1);
//  for(auto& elem : points){
//    elem = HoughTransform::Array_cells(max_xy_coord - min_xy_coord + 1);
//  }
//  HoughTransform::Array2d points_sub(right - left + 1);
//  for(auto& elem : points_sub) {
//    elem = HoughTransform::Array_cells(top - bot +1);
//  }
//  HoughTransform::Array2d points_sub_origin(right_origin - left_origin + 1);
//  for (auto& elem : points_sub_origin) {
//    elem = HoughTransform::Array_cells(top_origin - bot_origin + 1);
//  }
//  for(size_t i = 0; i < walk.size(); i++){
//    int rand_x = walk[i].x, rand_y = walk[i].y;
//    if(points[rand_x][rand_y] == 0) {
//      points[rand_x][rand_y]++;
//    }
//    if (left <= rand_x && rand_x < right &&
//        bot <= rand_y && rand_y < top) {
//      points_sub[rand_x-left][rand_y-bot]++;
//    }
//    if (left_origin <= rand_x && rand_x < right_origin &&
//        bot_origin <= rand_y && rand_y < top_origin) {
//      points_sub_origin[rand_x-left_origin][rand_y-bot_origin]++;
//    }
//  }
//
//  create_window(max_xy_coord - min_xy_coord + 1 + 2,max_xy_coord - min_xy_coord + 1 + 2,"proimage");
//
//  for (size_t x = 0; x < points.size(); x++)
//    for (size_t y = 0; y < points[0].size(); y++)
//      printRect(y+1,y+2,x+1,x+2,1.0-points[x][y]);
//  glPolygonMode(GL_FRONT, GL_LINE);
//  printRect(bot+1,top+2,left+1,right+2,0.5);
//
//  glFlush();
//  printRect(bot_origin+1,top_origin+2,left_origin+1,right_origin+2,0.5);
//  glFlush();
//  glPolygonMode(GL_FRONT, GL_FILL);
//
//  create_window(right - left + 1 + 2,top - bot + 1 + 2,"proimage_sub");
//  for (size_t x = 0; x < points_sub.size(); x++)
//    for (size_t y = 0; y < points_sub[0].size(); y++)
//      printRect(y+1,y+2,x+1,x+2,1-points_sub[x][y]);
//  glFlush();
//
//  create_window(right_origin - left_origin + 1 + 2,top_origin - bot_origin + 1 + 2,"proimage_sub_origin");
//  for (size_t x = 0; x < points_sub_origin.size(); x++)
//    for (size_t y = 0; y < points_sub_origin[0].size(); y++)
//      printRect(y+1,y+2,x+1,x+2,1-points_sub_origin[x][y]);
//  glFlush();
//
//  //for (int x = 0; x < (int) points.size(); x++)
//  //  for (int y = 0; y < (int) points[0].size(); y++)
//  //    if(points[x][y])
//  //      h_origin.transform({x,y});
//  for (int x = 0; x < (int) points_sub.size(); x++)
//    for (int y = 0; y < (int) points_sub[0].size(); y++)
//      if(points_sub[x][y])
//        h_sub.transform(rotate(M_PI/2,{x-x_center_sub,y-y_center_sub})+DiscretePoint2D{5,-5});
//  for (int x = 0; x < (int) points_sub_origin.size(); x++)
//    for (int y = 0; y < (int) points_sub_origin[0].size(); y++)
//      if(points_sub_origin[x][y])
//        h_sub_origin.transform({x-x_center_sub_origin,y-y_center_sub_origin});
//  //h_origin.printOpenGL();
//  h_sub.printOpenGL();
//  int h_id = h_sub_origin.printOpenGL();
//
//
//  shared_ptr<HT::Array_cov> spectrum_origin = h_origin.spectrum();
//  shared_ptr<HT::Array_cov> spectrum_sub = h_sub.spectrum();
//  print(*spectrum_sub,"spectrum sub");
//  shared_ptr<HT::Array_cov> spectrum_sub_origin = h_sub_origin.spectrum();
//  print(*spectrum_sub_origin, "spectrum sub origin");
//
//  vector<long long> covariance(spectrum_origin->size());
//  for (size_t i = 0; i < covariance.size(); i++) {
//    covariance[i] = scalar_mul(*spectrum_sub_origin, *spectrum_sub, i, 0);
//  }
//  double max = my_max(covariance);
//  for(auto elem: covariance) {
//    cout << (double)elem/max << " ";
//  }
//  cout << endl;
//  print(covariance,"covariance");
//  shared_ptr<_set_> sortedPoints = find_maxs(covariance);
//  for(auto elem : *sortedPoints) {
//    cout << "a[" << elem.ind << "] = " <<  elem.value << ", whith max = " << elem.sigma << endl;
//  }
//  glutSetWindow(h_id);
//  auto iterator = sortedPoints->begin();
//  for(int k = 0; k < 3 && iterator != sortedPoints->end(); k++) {
//    PointMax current = *iterator;
//    double r = 0, g = 0, b = 0;
//    if (k == 0) r = 1.0;
//    if (k == 1) g = 1.0;
//    if (k == 2) b = 1.0;
//    //printRect(h_origin.width()+current.ind+1, h_origin.width()+current.ind+2,h_originmin_xy_coord+1,max_xy_coord+1,r,g,b);
//    printRect(h_sub_origin.height()/2, h_sub_origin.height()*3/2,h_sub_origin.width()/2+current.ind +1,h_sub_origin.width()/2+current.ind +2,r,g,b);
//    iterator++;
//  }
//  glFlush();
//
//  int offset_on_origin = sortedPoints->begin()->ind;
//  int fi_count = /*h_sub.width() - */offset_on_origin;
//  cout << "offset found " << offset_on_origin << endl;
//  double fi_out = fi_count * 360/theta_partition;
//  cout << "found rotate angle: " << fi_out << endl;
//  int ro_counts = 360;
//  boost::numeric::ublas::matrix<double> A(360,2);
//  boost::numeric::ublas::matrix<double> b(360,1);
//  for(int j = 0; j < ro_counts; j++) {
//    if(covariance[j] == 0)
//      continue;
//    HT::Array_cov covarianceRO(h_sub.height());
//    //print(h_sub_origin.getCells().at(offset_on_origin+j), "ro on sub_origin");
//    //print(h_sub.getCells().at(0+j), "ro on sub");
//    for (size_t i = 0; i < covarianceRO.size(); i++) {
//      covarianceRO[i] = scalar_mul(get(h_sub_origin.getCells(),offset_on_origin+j),get(h_sub.getCells(),0+j),i,0);
//    }
//    //print(covarianceRO, "covarianseRO");
//    auto sortedRo = find_maxs(covarianceRO);
//    double ro_out = delta_ro * ((h_sub_origin.height()-sortedRo->begin()->ind)%h_sub_origin.height());
//    //cout << "on algle " << j*360/theta_partition <<" found delta rho: " << ro_out << endl;
//    A.insert_element(j,0,cos(j*M_PI/180));
//    A.insert_element(j,1,sin(j*M_PI/180));
//    b.insert_element(j,0,ro_out);
//  }
//  boost::numeric::ublas::matrix<double> A_tr = boost::numeric::ublas::trans(A);
//  boost::numeric::ublas::matrix<double> A_tr__A = boost::numeric::ublas::prod(A_tr,A);
//  boost::numeric::ublas::matrix<double> A_tr__A_1(2,2);
//  boost::numeric::ublas::matrix<double> X;
//  bool invertSucces = InvertMatrix(A_tr__A, A_tr__A_1);
//  if(invertSucces)
//    X = boost::numeric::ublas::prod(boost::numeric::ublas::matrix<double>(boost::numeric::ublas::prod(A_tr__A_1,A_tr)),b);
//  else
//    X = boost::numeric::ublas::matrix<double>(2,1,0);
//  cout << "итого получено: delta_x = " << X(0,0) << ", delta_y = " << X(1,0) << endl;
//  /*for(int k = 0; k < 1; k++){
//    int offset = iterator->ind;
//    vector<long long> covarianceRo(h_origin.height());
//    for (int i = 0; i < covarianceRo.size(); i++) {
//      covarianceRo[i] = scalar_mul(h_origin.getCells()[offset], h_sub.getCells()[0],i,0);
//    }
//    shared_ptr<_set_> sortedPointsRO = find_maxs(covarianceRo);
//    for(auto elem : *sortedPointsRO) {
//      cout << "a[" << elem.ind << "] = " <<  elem.value << ", whith max = " << elem.sigma << endl;
//    }
//    int offsetRo = sortedPointsRO->begin()->ind;
//
//    RasterGrid<DiscretePoint2D> r(1,1);
//
//    double angle = 2*M_PI/theta_partition*offset;
//    double range = delta_ro*offsetRo;
//
//    cout << angle << " " << range << endl;
//    vector<DiscretePoint2D> ppp = r.raster_build(min_xy_coord, max_xy_coord,[angle,range](double x)->double {return -cos(angle)/sin(angle)*x + range/sin(angle);});
//    for (auto p : ppp) {
//      cout << "(" << p.x << "; " << p.y << ") ";
//      if(p.y >= 0 && p.y <= max_xy_coord && p.x >=min_xy_coord && p.x <= max_xy_coord) {
//        printRect(p.x+1,p.x+2,p.y+1,p.y+2,0.8);
//      }
//    }
//    iterator++;
//  }*/
//  cout << "тут всё хорошо!" <<endl;
//  glutMainLoop();
//  return 0;
//}
