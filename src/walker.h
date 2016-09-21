#pragma once
#ifdef DEBUG
  #include "geometry_utils.h"
#else
  #include "../../core/geometry_utils.h"
#endif
#include <vector>
#include <random>
#include <iostream>
#include <cmath>

class Walker{
public:
  using Array = std::vector<DiscretePoint2D>;

  static const int UP = 0;
  static const int UPRIGHT = 1;
  static const int RIGHT = 2;
  static const int DOWNRIGHT = 3;
  static const int DOWN = 4;
  static const int DOWNLEFT = 5;
  static const int LEFT = 6;
  static const int UPLEFT = 7;
private:
  int bot;
  int top;
  int left;
  int right;

  std::random_device rd;
  std::mt19937 mt;

  int cur_x = 0;
  int cur_y = 0;

  int view = RIGHT;

  /*std::vector<direction> allDir = {direction::up        = 0,
                                   direction::upright   = 1,
                                   direction::right     = 2,
                                   direction::rightdown = 3,
                                   direction::down      = 4,
                                   direction::downleft  = 5,
                                   direction::left      = 6,
                                   direction::upleft    = 7};*/
  int getDir(int i){
    if (0 <= i && i < 8)
      return i;
    i = i % 8;
    if (i < 0)
      i += 8;
    return i;
  }

  DiscretePoint2D step(){
    int delta_x = 0, delta_y = 0;
    int newView;
    do {
      delta_x = 0;
      delta_y = 0;
      std::uniform_int_distribution<int> ud(view-2,view+2);
      newView = getDir(ud(mt));
      view = newView;
      if(newView == UPRIGHT || newView == RIGHT || newView == DOWNRIGHT)
        delta_x = 1;
      if(newView == DOWNLEFT || newView == LEFT || newView == UPLEFT)
        delta_x = -1;
      if(newView == UPLEFT || newView == UP || newView == UPRIGHT)
        delta_y = 1;
      if(newView == DOWNRIGHT || newView == DOWN || newView == DOWNLEFT)
        delta_y = -1;
    } while (cur_x + delta_x < left || cur_x + delta_x > right ||
             cur_y + delta_y < bot || cur_y + delta_y > top);
    cur_x += delta_x;
    cur_y += delta_y;
    view = newView;
    std::cout << "next step: (" << cur_x << "; " << cur_y << ")" << std::endl;
    return {cur_x,cur_y};

  }
public:
  Walker(int bot,int top, int left, int right) : bot(bot), top(top),left(left),right(right){
    mt = std::mt19937(rd());
  }
  Array go(int x_start, int y_start, int step_counts, int view = RIGHT) {
    cur_x = x_start;
    cur_y = y_start;
    this->view = view;
    Array out;
    for (int i = 0; i < step_counts; i++)
      out.push_back(step());
    return out;
  }


};
